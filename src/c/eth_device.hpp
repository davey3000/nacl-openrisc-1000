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

private:
  CPU* cpu;

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
  
public:
  EthernetDevice(CPU* cpu);
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

};

#endif
