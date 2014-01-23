/*
 * Header file for the class implementing an emulated keyboard controller
 * device
 */

#ifndef OR_EMULATOR_KEYBOARD_DEVICE_HPP_
#define OR_EMULATOR_KEYBOARD_DEVICE_HPP_

#include "device.hpp"
#include "cpu.hpp"

class KeyboardDevice : public Device {
private:
  // JS keycodes to US keyboard keycodes
  static const uint8_t KEY_CODES[256];

private:
  CPU* cpu;

public:
  KeyboardDevice(CPU* cpu);
  virtual ~KeyboardDevice();

  virtual void Reset();

  /*
   * Called by the main thread to signal key-presses.  Thread-safe
   */
  void OnKeyDown(uint8_t charCode);
  void OnKeyUp(uint8_t charCode);

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
