/*
 * Class implementing an emulated dummy device
 */

#include "dummy_device.hpp"

DummyDevice::DummyDevice(CPU* cpu, uint32_t baseAddress) :
  cpu(cpu),
  baseAddress(baseAddress)
{
}

DummyDevice::~DummyDevice() {
}

void DummyDevice::Reset() {
}

uint8_t DummyDevice::Read8(uint32_t offset) {
  cpu->DebugMessage("INFO (dummy device): 8-bit read access at addr 0x%08x", (baseAddress | offset));
  return 0;
}

uint16_t DummyDevice::Read16(uint32_t offset) {
  cpu->DebugMessage("INFO (dummy device): 16-bit read access at addr 0x%08x", (baseAddress | offset));
  return 0;
}

uint32_t DummyDevice::Read32(uint32_t offset) {
  cpu->DebugMessage("INFO (dummy device): 32-bit read access at addr 0x%08x", (baseAddress | offset));
  return 0;
}

void DummyDevice::Write8(uint32_t offset, uint8_t data) {
  cpu->DebugMessage("INFO (dummy device): 8-bit write access at addr 0x%08x", (baseAddress | offset));
}

void DummyDevice::Write16(uint32_t offset, uint16_t data) {
  cpu->DebugMessage("INFO (dummy device): 16-bit write access at addr 0x%08x", (baseAddress | offset));
}

void DummyDevice::Write32(uint32_t offset, uint32_t data) {
  cpu->DebugMessage("INFO (dummy device): 32-bit write access at addr 0x%08x", (baseAddress | offset));
}
