/*
 * Class implementing an emulated ATA device
 */

#include <stdlib.h>

#include "util.hpp"

#include "ata_device.hpp"

ATADevice::ATADevice(CPU* cpu) :
  cpu(cpu),
  heads(0),
  sectors(0),
  cylinders(0),
  totalSectors(0),
  diskBuffer(NULL),
  diskBufferSize(0)
{
  Reset();
}

ATADevice::~ATADevice() {
  free(diskBuffer);
}

void ATADevice::Reset() {
  curBuffer                = infoBuffer;
  curBufferIndex           = 0;
  curBufferSize            = 256;
  curBufferNextSectorIndex = 256;

  deviceCtrlReg   = 0x08;
  driveHeadReg    = 0xa0;
  sectorNumberReg = 0x01;
  sectorCountReg  = 0x01;
  statusReg       = ST_DRDY;
  featureReg      = 0x00;
  errorReg        = 0x01;
  commandReg      = 0x00;
  loCylinderReg   = 0x00;
  hiCylinderReg   = 0x00;
  driveSelected   = true;

  SetupInfoBuffer();
}

void ATADevice::SetDiskImage(void* buffer, uint32_t sizeBytes) {
  diskBuffer = (uint16_t*)buffer;
  diskBufferSize = (sizeBytes >> 1);
  heads = 16;
  sectors = 64;
  cylinders = (uint16_t)(sizeBytes
                         / (uint32_t)((uint32_t)heads * (uint32_t)sectors * 512));
  totalSectors = (uint32_t)heads * (uint32_t)sectors * (uint32_t)cylinders;
  //fprintf(stderr, "INFO (ATA): Setting disk image: sizeBytes=%u, heads=%u, sectors=%u, cylinders=%u, totalSectors=%u\n", sizeBytes, heads, sectors, cylinders, totalSectors);
  SetupInfoBuffer();
}

uint8_t ATADevice::Read8(uint32_t offset) {
  if (!driveSelected) {
    return 0xff;
  }
  switch(offset) {
  case REG_ERR:
    return errorReg;
    
  case REG_SECTC:
    return sectorCountReg;
    
  case REG_SECTN:
    return sectorNumberReg;
    
  case REG_LCYL:
    return loCylinderReg;
    
  case REG_HCYL:
    return hiCylinderReg;
    
  case REG_DRIVE_HEAD:
    return driveHeadReg;
    
  case REG_STATUS:	
    cpu->ClearInterrupt(15);
    return statusReg;
    
  case REG_ALT_STATUS:
    return statusReg;
    
  default:
    cpu->DebugMessage("INFO (ATA): 8-bit reads of offset 0x%08x not supported", offset);
    return 0;
  }
}

uint16_t ATADevice::Read16(uint32_t offset) {
  uint16_t val;

  if (offset != 0) { // data register
    cpu->DebugMessage("INFO (ATA): 16-bit reads of offset 0x%08x not supported", offset);
    return 0;
  }

  if (curBufferIndex > curBufferSize) {
    cpu->DebugMessage("INFO (ATA): attempted to read beyond the end of the disk image buffer");
    return 0;
  }

  //fprintf(stderr, "%lld: ATA rd: 0x%08x: 0x%04x\n", cpu->debug_instcount, curBufferIndex << 1, util::ByteSwap16(curBuffer[curBufferIndex]));
  val = curBuffer[curBufferIndex++];

  if (curBufferIndex >= curBufferNextSectorIndex) {
    statusReg = ST_DRDY | ST_DSC; // maybe no DSC for identify command but it works
        
    if ((commandReg == 0x20) && (sectorCountReg != 1)) {
      sectorCountReg--;
      SetSector(GetSector() + 1);
      curBufferNextSectorIndex += 256;
      statusReg = ST_DRDY | ST_DSC | ST_DRQ;
      if (!(deviceCtrlReg & DC_nIEN)) {
        cpu->RaiseInterrupt(15);
      }
    }

  }
  return val;
}

uint32_t ATADevice::Read32(uint32_t offset) {
  cpu->DebugMessage("INFO (ATA): 32-bit reads not supported -- 0x%08x", offset);
  return 0;
}

void ATADevice::Write8(uint32_t offset, uint8_t data) {
  if (offset == REG_DRIVE_HEAD) {
    driveHeadReg = data;
    driveSelected = ((data >> 4) & 1) ? false : true;
    return;
  }

  if (offset == 0x100) { //device control register

    if (!(data & DC_RST)
        && (deviceCtrlReg & DC_RST)) { // reset done

      driveHeadReg &= 0xf0; // reset head
      statusReg = ST_DRDY | ST_DSC;
      sectorNumberReg = 0x1;
      sectorCountReg = 0x1;
      loCylinderReg = 0x0;
      hiCylinderReg = 0x0;
      errorReg = 0x1;
      commandReg = 0x0;
    } else if ((data & DC_RST)
               && !(deviceCtrlReg & DC_RST)) { // reset

      errorReg = 0x1; // set diagnostics message
      statusReg = ST_BSY | ST_DSC;
    }
    
    deviceCtrlReg = data;
    return;
  }

  if (!driveSelected) {
    return;
  }

  switch(offset)
    {
    case REG_FEATURE:
      featureReg = data;
      break;

    case REG_SECTC:
      sectorCountReg = data;
      break;

    case REG_SECTN:
      sectorNumberReg = data;
      break;

    case REG_LCYL:
      loCylinderReg = data;
      break;

    case REG_HCYL:
      hiCylinderReg = data;
      break;

    case REG_CMD:
      commandReg = data;
      ExecuteCommand();
      break;

    default:
      cpu->DebugMessage("INFO (ATA): 8-bit write to offset 0x%08x not supported, data 0x%02x", offset, data);
      break;
    }
}

void ATADevice::Write16(uint32_t offset, uint16_t data) {
  if (offset != 0) { // data register
    cpu->DebugMessage("INFO (ATA): 16-bit writes to offset 0x%08x not supported, data 0x%08x", offset, data);
    return;
  }

  if (curBufferIndex > curBufferSize) {
    cpu->DebugMessage("INFO (ATA): attempted to write beyond the end of the disk image buffer");
    return;
  }

  //fprintf(stderr, "ATA wr: 0x%08x: 0x%04x\n", curBufferIndex << 1, data);
  curBuffer[curBufferIndex++] = data;

  if (curBufferIndex >= curBufferNextSectorIndex) {
    statusReg = ST_DRDY | ST_DSC;
    if (!(deviceCtrlReg & DC_nIEN)) {
      cpu->RaiseInterrupt(15);
    }
    if ((commandReg == 0x30) && (sectorCountReg != 1)) {
      sectorCountReg--;
      SetSector(GetSector() + 1);
      curBufferNextSectorIndex += 256;
      statusReg = ST_DRDY | ST_DSC | ST_DRQ;
    }
  }
}

void ATADevice::Write32(uint32_t offset, uint32_t data) {
  cpu->DebugMessage("INFO (ATA): 32-bit writes not supported -- 0x%08x, data 0x%08x", offset, data);
}

void ATADevice::SetupInfoBuffer() {
  uint32_t i;

  for (i = 0; i < 256; ++i) {
    infoBuffer[i] = 0;
  }

  infoBuffer[0] = 0x0444;
  infoBuffer[1] = cylinders;
  infoBuffer[3] = heads;
  infoBuffer[4] = 512 * sectors; // Number of unformatted bytes per track (sectors*512)
  infoBuffer[5] = 512;     // Number of unformatted bytes per sector
  infoBuffer[6] = sectors; // sectors per track

  infoBuffer[20] = 0x0003; // buffer type
  infoBuffer[21] = 512;    // buffer size in 512 byte increments
  infoBuffer[22] = 4;      // number of ECC bytes available

  infoBuffer[27] = 0x6f72; // or (model string)
  infoBuffer[28] = 0x656d; // em
  infoBuffer[29] = 0x752d; // u-
  infoBuffer[30] = 0x6469; // di
  infoBuffer[31] = 0x736b; // sk

  for(i = 32; i <= 46; ++i) {
    infoBuffer[i] = 0x2020; // spaces (model string)
  }
    
  infoBuffer[47] = 0x8000 | 256;
  infoBuffer[48] = 0x0000;
  infoBuffer[49] = (1 << 9);
  infoBuffer[51] = 0x0200; // PIO data transfer cycle timing mode
  infoBuffer[52] = 0x0200; // DMA data transfer cycle timing mode

  infoBuffer[54] = cylinders;
  infoBuffer[55] = heads;
  infoBuffer[56] = sectors; // sectors per track

  infoBuffer[57] = totalSectors         & 0xffff; // number of sectors
  infoBuffer[58] = (totalSectors >> 16) & 0xffff;

  infoBuffer[59] = 0x0100; // multiple sector settings

  // Total number of user-addressable sectors (low and high half-words)
  infoBuffer[60] = totalSectors         & 0xffff; 
  infoBuffer[61] = (totalSectors >> 16) & 0xffff;

  // Values that are not part of the basic ATA standard
  // REVISIT: what standard are they from?  Linux seems to want them...
  infoBuffer[80] = (1 << 1) | (1 << 2); // version, support ATA-1 and ATA-2
  infoBuffer[82] = (1 << 14); // Command sets supported. (NOP supported)
  infoBuffer[83] = (1 << 14);
  infoBuffer[84] = (1 << 14);
  infoBuffer[85] = (1 << 14); // Command set/feature enabled (NOP)
  infoBuffer[87] = (1 << 14);

  // Convert from big to little endian (on half-words)
  for (uint32_t i = 0; i < (sizeof(infoBuffer) / sizeof(infoBuffer[0])); ++i) {
    infoBuffer[i] = util::ByteSwap16(infoBuffer[i]);
  }
}

uint32_t ATADevice::GetSector() {
  if (!(driveHeadReg & 0x40)) {
    cpu->DebugMessage("INFO (ATA): CHS mode not supported");
    return 0;
  }
  return (uint32_t)(((uint32_t)(driveHeadReg & 0x0f) << 24)
                    | ((uint32_t)hiCylinderReg << 16)
                    | ((uint32_t)loCylinderReg << 8)
                    | (uint32_t)sectorNumberReg);
}

void ATADevice::SetSector(uint32_t sector) {
  if (!(driveHeadReg & 0x40)) {
    cpu->DebugMessage("INFO (ATA): CHS mode not supported");
  }
  sectorNumberReg = sector & 0xff;
  loCylinderReg = (sector >> 8) & 0xff;
  hiCylinderReg = (sector >> 16) & 0xff;
  driveHeadReg = (driveHeadReg & 0xf0) | ((sector >> 24) & 0x0f);
}

void ATADevice::ExecuteCommand() {
  uint32_t sector;

  switch(commandReg) {
  case 0xEC: // identify device
    curBuffer = infoBuffer;
    curBufferIndex = 0;
    curBufferSize = 256;
    curBufferNextSectorIndex = 256;
    statusReg = ST_DRDY | ST_DSC | ST_DRQ;
    if (!(deviceCtrlReg & DC_nIEN)) {
      cpu->RaiseInterrupt(15);
    }
    break;

  case 0x91: // initialize drive parameters
    statusReg = ST_DRDY | ST_DSC;
    errorReg = 0x0;
    if (!(deviceCtrlReg & DC_nIEN)) {
      cpu->RaiseInterrupt(15);
    }
    break;

  case 0x20: // read sector (with retry)
  case 0x21: // read sector (no retry)
  case 0x30: // write sector (with retry)
  case 0x31: // write sector (no retry)
    sector = GetSector();
    curBuffer = diskBuffer;
    curBufferIndex = sector << 8;
    curBufferSize = diskBufferSize;
    curBufferNextSectorIndex = curBufferIndex + 256;
    statusReg = ST_DRDY | ST_DSC | ST_DRQ;
    errorReg = 0x0;
    if (commandReg == 0x20 || commandReg == 0x21) {
      if (!(deviceCtrlReg & DC_nIEN)) {
        cpu->RaiseInterrupt(15);
      }
    }
    break;

  case 0xC4: // read multiple sectors
  case 0xC5: // write multiple sectors
    sector = GetSector();
    curBuffer = diskBuffer;
    curBufferIndex = sector << 8;
    curBufferSize = diskBufferSize;
    if (sectorCountReg == 0) {
      curBufferNextSectorIndex = curBufferIndex + 256 * 256;
    } else {
      curBufferNextSectorIndex = curBufferIndex + 256 * (uint32_t)sectorCountReg;
    }
    statusReg = ST_DRDY | ST_DSC | ST_DRQ;
    errorReg = 0x0;
    if (commandReg == 0xC4) {
      if (!(deviceCtrlReg & DC_nIEN)) {
        cpu->RaiseInterrupt(15);
      }
    }
    break;

  default:
    cpu->DebugMessage("INFO (ATA): Command 0x%02x not supported", commandReg);
    break;
  }
}
