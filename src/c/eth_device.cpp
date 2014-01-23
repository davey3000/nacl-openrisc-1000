/*
 * Class implementing an emulated Ethernet interface device
 */

#include <stdlib.h>

#include "eth_device.hpp"

EthernetDevice::EthernetDevice(CPU* cpu) :
  cpu(cpu)
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
      //retVal = BD[(offset-ADDR_BD_START)>>>2];
      retVal = 0;
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
    if (offset >= ADDR_BD_START &&
        offset <= ADDR_BD_END) {

      /*BD[(addr-ADDR_BD_START)>>>2] = data;

      //which buffer descriptor?
      var BD_NUM = (addr - ADDR_BD_START)>>>3;
                    
      //make sure this isn't the pointer portion
      if (((BD_NUM << 3) + ADDR_BD_START) == addr) {
        //did we just set the ready/empty bit?
        if ((data & (1 << 15)) != 0) {
          //TX, or RX?
          if (BD_NUM < TX_BD_NUM) {
            //TX BD
            Transmit(BD_NUM);
          }
        }
      }*/
    } else {
      cpu->DebugMessage("INFO (eth): 32-bit write to offset 0x%08x not supported -- data 0x%08x", offset, data);
    }
    break;
  }
}
