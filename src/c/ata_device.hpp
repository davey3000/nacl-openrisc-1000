/*
 * Header file for the class implementing an emulated ATA device
 */

/* 
 * ATA generic implementation
 *
 * See spec: ftp://ftp.seagate.com/pub/acrobat/reference/111-1c.pdf
 *
 * Notes:
 *
 * - Only one "master" drive is supported.
 * - Entire hard disk image is loaded into RAM.
 * - Image size must be a multiple of 512KiB.
 * - Max image size supported is 4GB (although will probably be less in practice).
 * - Image will be freed when the class instance is destroyed.
 */

/* Use these dts lines (Linux)
 ata@9e000000  {
                compatible = "ata-generic";
                reg = <0x9e000000 0x100
                       0x9e000100 0xf00>;
                pio-mode = <4>;
                reg-shift = <2>;
                interrupts = <15>;
        };
*/

// REVISIT: consider performing the 16-bit endian byte swap on image load rather than for every data access

#ifndef OR_EMULATOR_ATA_DEVICE_HPP_
#define OR_EMULATOR_ATA_DEVICE_HPP_

#include "cpu.hpp"

class ATADevice : public Device {
private:
  enum ATAReg {
    // ATA command block registers (register reads)
    REG_DATA       = 0x00 << 2, // data register
    REG_ERR        = 0x01 << 2, // error register
    REG_SECTC      = 0x02 << 2, // sector count register
    REG_SECTN      = 0x03 << 2, // sector number register
    REG_LCYL       = 0x04 << 2, // cylinder low register
    REG_HCYL       = 0x05 << 2, // cylinder high register
    REG_DRIVE_HEAD = 0x06 << 2, // drive/head register
    REG_STATUS     = 0x07 << 2, // status register

    // ATA command block registers (register writes)
    REG_FEATURE    = 0x01 << 2, // feature register
    REG_CMD        = 0x07 << 2, // command register
    
    // ATA control block register (reads)
    REG_CTRL       = 0x100,     // device control register
    
    // ATA control block register (writes)
    REG_ALT_STATUS = 0x100      // alternate status register
  };

  // Device control register values
  enum DevCtrl {
    DC_RST = 0x04,	// Software reset   (RST=1, reset)
    DC_nIEN = 0x02	// Interrupt Enable (nIEN=0, enabled)
  };

  
  // Status register values
  enum Status {
    ST_BSY  = 0x80,  // Busy
    ST_DRDY = 0x40,  // Device Ready
    ST_DF   = 0x20,  // Device Fault
    ST_DSC  = 0x10,  // Device Seek Complete
    ST_DRQ  = 0x08,  // Data Request
    ST_COR  = 0x04,  // Corrected data (obsolete)
    ST_IDX  = 0x02,  //                (obsolete)
    ST_ERR  = 0x01   // Error
  };

private:
  CPU* cpu;

  /*
   * Device registers
   */
  uint8_t deviceCtrlReg;
  uint8_t driveHeadReg;
  uint8_t sectorNumberReg;
  uint8_t sectorCountReg;
  uint8_t statusReg;
  uint8_t featureReg;
  uint8_t errorReg;
  uint8_t commandReg;
  uint8_t loCylinderReg;
  uint8_t hiCylinderReg;
  bool driveSelected;

  /*
   * Drive characteristics
   */
  uint16_t heads;
  uint16_t sectors;
  uint16_t cylinders;
  uint32_t totalSectors;

  // Pointer to and info about the currently selected device buffer.  The index
  // position, next sector index position and size are in multiples of half-words
  // (2 bytes)
  uint16_t* curBuffer;
  uint32_t curBufferIndex;
  uint32_t curBufferSize;
  uint32_t curBufferNextSectorIndex;

  // The buffers that can be accessed through the device.  Only one can be
  // selected at a time.  Size is in multiples of hard-words (2 bytes)
  uint16_t infoBuffer[256];
  uint16_t* diskBuffer;
  uint32_t  diskBufferSize;

public:
  ATADevice(CPU* cpu);
  virtual ~ATADevice();

  virtual void Reset();

  /*
   * Sets the disk image to be used by the emulated ATA device to the indicate
   * buffer.  Note that this buffer should have had the bytes in its half-words
   * pre-swapped for big<->little endian conversion
   */
  void SetDiskImage(void* buffer, uint32_t sizeBytes);

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

private:
  void SetupInfoBuffer();
  uint32_t GetSector();
  void SetSector(uint32_t sector);
  void ExecuteCommand();

};

#endif
