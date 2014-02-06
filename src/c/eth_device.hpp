/*
 * Header file for the class implementing an emulated Ethernet interface
 * device
 */

/*
 * Based on the OpenCores Ethernet MAC driver (drivers/net/ethernet/ethoc.c)
 */

// (list below from the original JavaScript version)
//TODO: MODER.LOOPBCK - loopback support
//TODO: Carrier Sense?
//TODO: Huge frames
//TODO: IAM mode
//TODO: MODER.BRO

#ifndef OR_EMULATOR_ETHERNET_DEVICE_HPP_
#define OR_EMULATOR_ETHERNET_DEVICE_HPP_

#include "device.hpp"
#include "cpu.hpp"

class EthernetDevice : public Device {
private:
  // Address map for device registers
  enum DevAddr {
    ADDR_MODER = 0x0,
    ADDR_INT_SOURCE = 0x4,
    ADDR_INT_MASK = 0x8,
    ADDR_IPGT = 0xC,
    ADDR_IPGR1 = 0x10,
    ADDR_IPGR2 = 0x14,
    ADDR_PACKETLEN = 0x18,
    ADDR_COLLCONF = 0x1C,
    ADDR_TX_BD_NUM = 0x20,
    ADDR_CTRLMODER = 0x24,
    ADDR_MIIMODER = 0x28,
    ADDR_MIICOMMAND = 0x2C,
    ADDR_MIIADDRESS = 0x30,
    ADDR_MIITX_DATA = 0x34,
    ADDR_MIIRX_DATA = 0x38,
    ADDR_MIISTATUS = 0x3C,
    ADDR_MAC_ADDR0 = 0x40,
    ADDR_MAC_ADDR1 = 0x44,
    ADDR_ETH_HASH0_ADR = 0x48,
    ADDR_ETH_HASH1_ADR = 0x4C,
    ADDR_ETH_TXCTRL = 0x50,

    ADDR_BD_START = 0x400,
    ADDR_BD_END = 0x7FF
  };

  // Bit-masks for accessing MODER register (moderReg) bits
  static const uint32_t MODER_RXEN  = (1 <<  0); /* receive enable */
  static const uint32_t MODER_TXEN  = (1 <<  1); /* transmit enable */
  static const uint32_t MODER_NOPRE = (1 <<  2); /* no preamble */
  static const uint32_t MODER_BRO   = (1 <<  3); /* broadcast address */
  static const uint32_t MODER_IAM   = (1 <<  4); /* individual address mode */
  static const uint32_t MODER_PRO   = (1 <<  5); /* promiscuous mode */
  static const uint32_t MODER_IFG   = (1 <<  6); /* interframe gap for incoming frames */
  static const uint32_t MODER_LOOP  = (1 <<  7); /* loopback */
  static const uint32_t MODER_NBO   = (1 <<  8); /* no back-off */
  static const uint32_t MODER_EDE   = (1 <<  9); /* excess defer enable */
  static const uint32_t MODER_FULLD = (1 << 10); /* full duplex */
  static const uint32_t MODER_RESET = (1 << 11); /* FIXME: reset (undocumented) */
  static const uint32_t MODER_DCRC  = (1 << 12); /* delayed CRC enable */
  static const uint32_t MODER_CRC   = (1 << 13); /* CRC enable */
  static const uint32_t MODER_HUGE  = (1 << 14); /* huge packets enable */
  static const uint32_t MODER_PAD   = (1 << 15); /* padding enabled */
  static const uint32_t MODER_RSM   = (1 << 16); /* receive small packets */

  // Bit-masks for accessing INT_MASK register (intMaskReg) bits
  static const uint32_t INT_MASK_TXF  = (1 << 0); /* transmit frame */
  static const uint32_t INT_MASK_TXE  = (1 << 1); /* transmit error */
  static const uint32_t INT_MASK_RXF  = (1 << 2); /* receive frame */
  static const uint32_t INT_MASK_RXE  = (1 << 3); /* receive error */
  static const uint32_t INT_MASK_BUSY = (1 << 4);
  static const uint32_t INT_MASK_TXC  = (1 << 5); /* transmit control frame */
  static const uint32_t INT_MASK_RXC  = (1 << 6); /* receive control frame */
  
  static const uint32_t INT_MASK_TX = (INT_MASK_TXF | INT_MASK_TXE);
  static const uint32_t INT_MASK_RX = (INT_MASK_RXF | INT_MASK_RXE);

  static const uint32_t INT_MASK_ALL = (
                                        INT_MASK_TXF | INT_MASK_TXE |
                                        INT_MASK_RXF | INT_MASK_RXE |
                                        INT_MASK_TXC | INT_MASK_RXC |
                                        INT_MASK_BUSY
                                        );

  // Bit-masks for accessing the transmission status word bits of a buffer
  // descriptor
  static const uint32_t TX_BD_CS       = (1 <<  0); /* carrier sense lost */
  static const uint32_t TX_BD_DF       = (1 <<  1); /* defer indication */
  static const uint32_t TX_BD_LC       = (1 <<  2); /* late collision */
  static const uint32_t TX_BD_RL       = (1 <<  3); /* retransmission limit */
  static const uint32_t TX_BD_RETRY_MASK = 0x00f0;
  static const uint32_t TX_BD_RETRY_POS  = 4;
  static const uint32_t TX_BD_UR       = (1 <<  8); /* transmitter underrun */
  static const uint32_t TX_BD_CRC      = (1 << 11); /* TX CRC enable */
  static const uint32_t TX_BD_PAD      = (1 << 12); /* pad enable for short packets */
  static const uint32_t TX_BD_WRAP     = (1 << 13);
  static const uint32_t TX_BD_IRQ      = (1 << 14); /* interrupt request enable */
  static const uint32_t TX_BD_READY    = (1 << 15); /* TX buffer ready */
  static const uint32_t TX_BD_LEN_MASK = 0xffff0000;
  static const uint32_t TX_BD_LEN_POS  = 16;

  static const uint32_t TX_BD_STATS   = (TX_BD_CS | TX_BD_DF | TX_BD_LC |
                                         TX_BD_RL | TX_BD_RETRY_MASK | TX_BD_UR);

  // Bit-masks for accessing the receive status word bits of a buffer
  // descriptor
  static const uint32_t RX_BD_LC       = (1 <<  0); /* late collision */
  static const uint32_t RX_BD_CRC      = (1 <<  1); /* RX CRC error */
  static const uint32_t RX_BD_SF       = (1 <<  2); /* short frame */
  static const uint32_t RX_BD_TL       = (1 <<  3); /* too long */
  static const uint32_t RX_BD_DN       = (1 <<  4); /* dribble nibble */
  static const uint32_t RX_BD_IS       = (1 <<  5); /* invalid symbol */
  static const uint32_t RX_BD_OR       = (1 <<  6); /* receiver overrun */
  static const uint32_t RX_BD_MISS     = (1 <<  7);
  static const uint32_t RX_BD_CF       = (1 <<  8); /* control frame */
  static const uint32_t RX_BD_WRAP     = (1 << 13);
  static const uint32_t RX_BD_IRQ      = (1 << 14); /* interrupt request enable */
  static const uint32_t RX_BD_EMPTY    = (1 << 15);
  static const uint32_t RX_BD_LEN_MASK = 0xffff0000;
  static const uint32_t RX_BD_LEN_POS  = 16;

  static const uint32_t RX_BD_STATS = (RX_BD_LC | RX_BD_CRC | RX_BD_SF | RX_BD_TL |
                                       RX_BD_DN | RX_BD_IS | RX_BD_OR | RX_BD_MISS);

private:
  CPU* cpu;
  uint32_t* ram;

  // Memory-mapped device registers
  uint32_t moderReg;
  uint32_t intSourceReg;
  uint32_t intMaskReg;
  uint32_t ipgtReg;
  uint32_t ipgr1Reg;
  uint32_t ipgr2Reg;
  uint32_t packetLenReg;
  uint32_t collConfReg;
  uint32_t txBDNumReg;
  uint32_t ctrlModerReg;
  uint32_t miiModerReg;
  uint32_t miiCommandReg;
  uint32_t miiAddressReg;
  uint32_t miiTxDataReg;
  uint32_t miiRxDataReg;
  uint32_t miiStatusReg;
  uint32_t macAddr0Reg;
  uint32_t macAddr1Reg;
  uint32_t ethHash0AdrReg;
  uint32_t ethHash1AdrReg;
  uint32_t ethTxCtrlReg;

  // 
  uint32_t curRX;

  // Buffer descriptor array (each buffer descriptor consists of 2 x 32-bit
  // values)
  uint32_t bd[256];
  
public:
  EthernetDevice(CPU* cpu, uint32_t* ram);
  virtual ~EthernetDevice();

  virtual void Reset();

  /*
   * Device memory access methods.  The passed address offset is specified in
   * bytes and must be aligned to the access size (8/16/32 bits)
   */
  virtual uint8_t Read8(uint32_t offset);
  virtual uint16_t Read16(uint32_t offset);
  virtual uint32_t Read32(uint32_t offset);

  virtual void Write8(uint32_t offset, uint8_t data);
  virtual void Write16(uint32_t offset, uint16_t data);
  virtual void Write32(uint32_t offset, uint32_t data);

protected:
  /*
   * Transmit data based on the contents of the indicated buffer descriptor.
   * The value of bdNum must be of even parity (as each buffer descriptor is
   * 64-bits but is stored in an array of 32-bit values)
   */
  virtual void Transmit(uint32_t bdNum);

};

#endif
