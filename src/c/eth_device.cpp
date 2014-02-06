/*
 * Class implementing an emulated Ethernet interface device
 */

#include <stdlib.h>

#include "eth_device.hpp"

EthernetDevice::EthernetDevice(CPU* cpu, uint32_t* ram) :
  cpu(cpu),
  ram(ram)
{
  Reset();
}

EthernetDevice::~EthernetDevice() {
}

void EthernetDevice::Reset() {
  moderReg = 0xa000;
  intSourceReg = 0x0;
  intMaskReg = 0x0;
  ipgtReg = 0x12;
  ipgr1Reg = 0xc;
  ipgr2Reg = 0x12;
  packetLenReg = 0x400600;
  collConfReg = 0xf003f;
  txBDNumReg = 0x40;
  ctrlModerReg = 0x0;
  miiModerReg = 0x64;
  miiCommandReg = 0x0;
  miiAddressReg = 0x0;
  miiTxDataReg = 0x0;
  miiRxDataReg = 0x22;
  miiStatusReg = 0x0;

  macAddr0Reg = (rand() % 256);
  macAddr0Reg |= ((rand() % 256) << 8);
  macAddr0Reg |= ((rand() % 256) << 16);
  macAddr0Reg |= ((rand() % 256) << 24);
  macAddr1Reg = (rand() % 256);
  macAddr1Reg |= (((rand() % 256) & 0x7f) << 8);

  ethHash0AdrReg = 0x0;
  ethHash1AdrReg = 0x0;
  ethTxCtrlReg = 0x0;

  memset(bd, 0, sizeof(bd));

  curRX = (txBDNumReg << 1);
}

uint8_t EthernetDevice::Read8(uint32_t offset) {
  cpu->DebugMessage("INFO (eth): 8-bit reads not supported -- 0x%08x", offset);
  return 0;
}


uint16_t EthernetDevice::Read16(uint32_t offset) {
  cpu->DebugMessage("INFO (eth): 16-bit reads not supported -- 0x%08x", offset);
  return 0;
}

uint32_t EthernetDevice::Read32(uint32_t offset) {
  uint32_t retVal;

  //fprintf(stderr, "DEBUG (eth): 32-bit read of offset 0x%08x\n", offset);
  
  switch (offset) {
  case ADDR_MODER:
    retVal = moderReg;
    break;

  case ADDR_INT_SOURCE:
    retVal = intSourceReg;
    break;

  case ADDR_INT_MASK:
    retVal = intMaskReg;
    break;

  case ADDR_IPGT:
    retVal = ipgtReg;
    break;

  case ADDR_IPGR1:
    retVal = ipgr1Reg;
    break;

  case ADDR_IPGR2:
    retVal = ipgr2Reg;
    break;

  case ADDR_PACKETLEN:
    retVal = packetLenReg;
    break;

  case ADDR_COLLCONF:
    retVal = collConfReg;
    break;

  case ADDR_TX_BD_NUM:
    retVal = txBDNumReg;
    break;

  case ADDR_CTRLMODER:
    retVal = ctrlModerReg;
    break;

  case ADDR_MIIMODER:
    retVal = miiModerReg;
    break;

  case ADDR_MIICOMMAND:
    retVal = miiCommandReg;
    break;

  case ADDR_MIIADDRESS:
    retVal = miiAddressReg;
    break;

  case ADDR_MIITX_DATA:
    retVal = miiTxDataReg;
    break;

  case ADDR_MIIRX_DATA:
    retVal = miiRxDataReg;
    if (miiRxDataReg == 0x1613) {
      miiRxDataReg = 0xffff;
    }
    if (miiRxDataReg == 0x0022) {
      miiRxDataReg = 0x1613;
    }
    break;

  case ADDR_MIISTATUS:
    retVal = miiStatusReg;
    break;

  case ADDR_MAC_ADDR0:
    retVal = macAddr0Reg;
    break;

  case ADDR_MAC_ADDR1:
    retVal = macAddr1Reg;
    break;

  case ADDR_ETH_HASH0_ADR:
    retVal = ethHash0AdrReg;
    break;

  case ADDR_ETH_HASH1_ADR:
    retVal = ethHash1AdrReg;
    break;

  case ADDR_ETH_TXCTRL:
    retVal = ethTxCtrlReg;
    break;

  default:
    if (offset >= ADDR_BD_START &&
        offset <= ADDR_BD_END) {

      retVal = bd[(offset - ADDR_BD_START) >> 2];
    } else {
      cpu->DebugMessage("INFO (eth): 32-bit read of offset 0x%08x not supported", offset);
      retVal = 0;
    }
    break;
  }

  return retVal;
}

void EthernetDevice::Write8(uint32_t offset, uint8_t data) {
  cpu->DebugMessage("INFO (eth): 8-bit writes not supported -- 0x%08x, data 0x%02x", offset, data);
}

void EthernetDevice::Write16(uint32_t offset, uint16_t data) {
  cpu->DebugMessage("INFO (eth): 16-bit writes not supported -- 0x%08x, data 0x%04x", offset, data);
}

void EthernetDevice::Write32(uint32_t offset, uint32_t data) {
  //fprintf(stderr, "DEBUG (eth): 32-bit write to offset 0x%08x -- data 0x%08x\n", offset, data);

  switch (offset) {
  case ADDR_MODER:
    moderReg = data;
    break;

  case ADDR_INT_SOURCE:
    // Clear interrupt bit that are set in the written data value
    intSourceReg &= ~data;

    if (intMaskReg & intSourceReg) {
      cpu->RaiseInterrupt(0x4);
    } else {
      cpu->ClearInterrupt(0x4);
    }

    break;

  case ADDR_INT_MASK:
    intMaskReg = data;

    if (intMaskReg & intSourceReg) {
      cpu->RaiseInterrupt(0x4);
    } else {
      cpu->ClearInterrupt(0x4);
    }

    break;

  case ADDR_IPGT:
    ipgtReg = data;
    break;

  case ADDR_IPGR1:
    ipgr1Reg = data;
    break;

  case ADDR_IPGR2:
    ipgr2Reg = data;
    break;

  case ADDR_PACKETLEN:
    packetLenReg = data;
    break;

  case ADDR_COLLCONF:
    collConfReg = data;
    break;

  case ADDR_TX_BD_NUM:
    txBDNumReg = data;
    curRX = (data << 1);
    break;

  case ADDR_CTRLMODER:
    ctrlModerReg = data;
    break;

  case ADDR_MIIMODER:
    miiModerReg = data;
    break;

  case ADDR_MIICOMMAND:
    miiCommandReg = data;
    break;

  case ADDR_MIIADDRESS:
    miiAddressReg = data;
    break;

  case ADDR_MIITX_DATA:
    miiTxDataReg = data;
    break;

  case ADDR_MIIRX_DATA:
    miiRxDataReg = data;

  case ADDR_MIISTATUS:
    miiStatusReg = data;
    break;

  case ADDR_MAC_ADDR0:
    macAddr0Reg = data;
    break;

  case ADDR_MAC_ADDR1:
    macAddr1Reg = data;
    break;

  case ADDR_ETH_HASH0_ADR:
    ethHash0AdrReg = data;
    break;

  case ADDR_ETH_HASH1_ADR:
    ethHash1AdrReg = data;
    break;

  case ADDR_ETH_TXCTRL:
    ethTxCtrlReg = data;
    break;

  default:
    // Check for buffer descriptor write
    if (offset >= ADDR_BD_START &&
        offset <= ADDR_BD_END) {

      uint32_t bdNum = ((offset - ADDR_BD_START) >> 3);

      bd[(offset - ADDR_BD_START) >> 2] = data;

      // Check for packet transmission request
      if ((offset == ((bdNum << 3) + ADDR_BD_START))
          && ((data & (1 << 15)) != 0)
          && (bdNum < txBDNumReg)) {
        Transmit(bdNum);
      }

    } else {
      cpu->DebugMessage("INFO (eth): 32-bit write to offset 0x%08x not supported -- data 0x%08x", offset, data);
    }
    break;
  }
}

void EthernetDevice::Transmit(uint32_t bdNum) {
  fprintf(stderr, "DEBUG(eth): Packet transmission -- buffer descriptor: 0x%08x, 0x%08x\n", bd[0], bd[1]);

  // Check that transmitting is enabled
  if (!(moderReg & MODER_TXEN)) {
    return;
  }

  uint32_t status = bd[bdNum << 1];
  uint32_t ramPtr = bd[(bdNum << 1) + 1];

  // Check descriptor is ready for transmission
  if (!(status & TX_BD_READY)) {
    return;
  }

  uint32_t frameSize = ((status & TX_BD_LEN_MASK) >> TX_BD_LEN_POS);
  
  // Pad frame to minimum size if necessary
  if (((status & TX_BD_PAD) || (moderReg & MODER_PAD))
      && ((packetLenReg >> 16) > frameSize)) {
    frameSize = (packetLenReg >> 16);
  }

  // REVISIT: actually transmit the frame
  //fprintf(stderr, "DEBUG(eth): Sending frame -- length: %d bytes, first bytes: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", frameSize, ram[ramPtr], ram[ramPtr+1], ram[ramPtr+2], ram[ramPtr+3], ram[ramPtr+4], ram[ramPtr+5], ram[ramPtr+6], ram[ramPtr+7], ram[ramPtr+8], ram[ramPtr+9], ram[ramPtr+10], ram[ramPtr+11], ram[ramPtr+12], ram[ramPtr+13], ram[ramPtr+14], ram[ramPtr+15], ram[ramPtr+16], ram[ramPtr+17], ram[ramPtr+18], ram[ramPtr+19], ram[ramPtr+20], ram[ramPtr+21], ram[ramPtr+22], ram[ramPtr+23], ram[ramPtr+24], ram[ramPtr+25], ram[ramPtr+26], ram[ramPtr+27], ram[ramPtr+28], ram[ramPtr+29], ram[ramPtr+30], ram[ramPtr+31]);
  fprintf(stderr, "DEBUG(eth): Sending frame -- length: %d bytes, first bytes: 0x%08x%08x%08x%08x%08x%08x%08x%08x\n", frameSize, ram[ramPtr>>2], ram[(ramPtr>>2)+1], ram[(ramPtr>>2)+2], ram[(ramPtr>>2)+3], ram[(ramPtr>>2)+4], ram[(ramPtr>>2)+5], ram[(ramPtr>>2)+6], ram[(ramPtr>>2)+7]);

  

  // Update the status bits to indicate the frame was sent successfully and
  // clear its "ready" status
  status &= ~(TX_BD_CS);
  status &= ~(TX_BD_DF);
  status &= ~(TX_BD_LC);
  status &= ~(TX_BD_RL);
  status &= ~(TX_BD_RETRY_MASK);
  status &= ~(TX_BD_UR);
  status &= ~(TX_BD_READY);
  
  bd[bdNum << 1] = status;

  // Raise/clear interrupt
  intSourceReg |= 1;
  if (intMaskReg & intSourceReg) {
    cpu->RaiseInterrupt(4);
  } else {
    cpu->ClearInterrupt(4);
  }
}
