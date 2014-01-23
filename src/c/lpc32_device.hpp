/*
 * Header file for the class implementing an emulated LPC32xx touchscreen
 * controller device
 */

#ifndef OR_EMULATOR_LPC32_DEVICE_HPP_
#define OR_EMULATOR_LPC32_DEVICE_HPP_

#include "device.hpp"
#include "cpu.hpp"

class LPC32Device : public Device {
private:
  enum DevAddr {
    ADDR_STAT    = 0x00,
    ADDR_SEL     = 0x04,
    ADDR_CON     = 0x08,
    ADDR_FIFO    = 0x0C,
    ADDR_DTR     = 0x10,
    ADDR_RTR     = 0x14,
    ADDR_UTR     = 0x18,
    ADDR_TTR     = 0x1C,
    ADDR_DXP     = 0x20,
    ADDR_MIN_X   = 0x24,
    ADDR_MAX_X   = 0x28,
    ADDR_MIN_Y   = 0x2C,
    ADDR_MAX_Y   = 0x30,
    ADDR_AUX_UTR = 0x34,
    ADDR_AUX_MIN = 0x38,
    ADDR_AUX_MAX = 0x3C
  };

  enum DevStatus {
    ST_ADCCON_AUTO_EN = (1 << 0),  // automatic ts event capture
    ST_STAT_FIFO_EMPTY = (1 << 7), // fifo is empty; 
    ST_FIFO_TS_P_LEVEL = (1 << 31) // touched
  };
  
private:
  CPU* cpu;

  uint32_t control_reg;
  uint32_t status_reg;

public:
  LPC32Device(CPU* cpu);
  virtual ~LPC32Device();

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
