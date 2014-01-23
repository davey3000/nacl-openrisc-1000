/*
 * Abstract class definition for an emulated device
 */

#ifndef OR_EMULATOR_DEVICE_HPP_
#define OR_EMULATOR_DEVICE_HPP_

#include "ppapi/c/pp_stdint.h"

class Device {

public:
  virtual ~Device() {};

  virtual void Reset() = 0;

  /*
   * Device memory access methods.  The passed address offset is specified in
   * bytes and must be aligned to the access size (8/16/32 bits)
   */
  virtual uint8_t Read8(uint32_t offset) = 0;
  virtual uint16_t Read16(uint32_t offset) = 0;
  virtual uint32_t Read32(uint32_t offset) = 0;

  virtual void Write8(uint32_t offset, uint8_t data) = 0;
  virtual void Write16(uint32_t offset, uint16_t data) = 0;
  virtual void Write32(uint32_t offset, uint32_t data) = 0;

};

#endif
