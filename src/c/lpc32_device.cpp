/*
 * Class implementing an emulated LPC32 touchscreen controller device
 */

// REVISIT: doesn't actually generate any touch events yet

#include "lpc32_device.hpp"

LPC32Device::LPC32Device(CPU* cpu) :
  cpu(cpu)
{
  Reset();
}

LPC32Device::~LPC32Device() {
}

void LPC32Device::Reset() {
  control_reg = 0;
  status_reg = ST_STAT_FIFO_EMPTY;
}

uint8_t LPC32Device::Read8(uint32_t offset) {
  cpu->DebugMessage("INFO (FB): 8-bit reads not supported -- 0x%08x", offset);
  return 0;
}

uint16_t LPC32Device::Read16(uint32_t offset) {
  cpu->DebugMessage("INFO (FB): 16-bit reads not supported -- 0x%08x", offset);
  return 0;
}

uint32_t LPC32Device::Read32(uint32_t offset) {
  switch (offset) {
  case ADDR_CON:
    return control_reg;

  case ADDR_STAT:
    cpu->ClearInterrupt(9);
    return status_reg;

  case ADDR_FIFO:
    return 0;
    
  default:
    return 0;
  }
}

void LPC32Device::Write8(uint32_t offset, uint8_t data) {
  cpu->DebugMessage("INFO (FB): 8-bit writes not supported -- 0x%08x, data 0x%02x", offset, data);
}

void LPC32Device::Write16(uint32_t offset, uint16_t data) {
  cpu->DebugMessage("INFO (FB): 16-bit writes not supported -- 0x%08x, data 0x%04x", offset, data);
}

void LPC32Device::Write32(uint32_t offset, uint32_t data) {
  switch (offset) {
  case ADDR_CON:
    control_reg = data;
    break;

  case ADDR_SEL:
  case ADDR_MIN_X:
  case ADDR_MAX_X:
  case ADDR_MIN_Y:
  case ADDR_MAX_Y:
  case ADDR_AUX_UTR:
  case ADDR_AUX_MIN:
  case ADDR_AUX_MAX:
  case ADDR_RTR:
  case ADDR_DTR:
  case ADDR_TTR:
  case ADDR_DXP:
  case ADDR_UTR:
    break;

  default:
    break;
  }
}
