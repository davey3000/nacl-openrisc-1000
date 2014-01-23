/*
 * Header file for the class implementing an emulated dummy device
 */

#ifndef OR_EMULATOR_DUMMY_DEVICE_HPP_
#define OR_EMULATOR_DUMMY_DEVICE_HPP_

#include "device.hpp"
#include "cpu.hpp"

class DummyDevice : public Device {
private:
  CPU* cpu;

  // Absolute location of the device in the emulator memory (in bytes)
  uint32_t baseAddress;

public:
  DummyDevice(CPU* cpu, uint32_t baseAddress);
  virtual ~DummyDevice();

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
