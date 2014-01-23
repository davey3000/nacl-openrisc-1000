/*
 * Class implementing an emulated frame buffer device
 */

#include "ppapi/c/ppb_image_data.h"
#include "ppapi/cpp/image_data.h"

#include "util.hpp"

#include "fb_device.hpp"

FrameBufferDevice::FrameBufferDevice(pp::Instance* ppInstance,
                                     CPU* cpu,
                                     uint32_t* ram) :
  ppInstance(ppInstance),
  cpu(cpu),
  ram(ram),
  enabled(false),
  baseAddr(0),
  callbackFactory(this)
{
}

FrameBufferDevice::~FrameBufferDevice() {
}

void FrameBufferDevice::Reset() {
}

void FrameBufferDevice::SetGraphics2DContext(pp::Graphics2D newGraphics) {
  graphics = newGraphics;

  // If the copy of the context is null then there is no Flush() outstanding
  // which means the main display loop isn't running, so restart the loop
  if (graphicsCopy.is_null()) {
    UpdateDisplayLoop(0);
  }
}

void FrameBufferDevice::OnViewScaleChange(float scale) {
  graphics.SetScale(scale);
}

uint8_t FrameBufferDevice::Read8(uint32_t offset) {
  cpu->DebugMessage("INFO (FB): 8-bit reads not supported -- 0x%08x", offset);
  return 0;
}

uint16_t FrameBufferDevice::Read16(uint32_t offset) {
  cpu->DebugMessage("INFO (FB): 16-bit reads not supported -- 0x%08x", offset);
  return 0;
}

uint32_t FrameBufferDevice::Read32(uint32_t offset) {
  uint32_t data;

  fprintf(stderr, "DEBUG: FB read -- addr=0x%08x\n", offset);

  switch (offset) {
  case 0x00: // Control register
    data = (enabled ? 0x1 : 0x0);

  case 0x14: // Buffer address register
    data = baseAddr;
    break;

  default:
    data = 0;
    break;
  }

  return util::ByteSwap32(data);
}

void FrameBufferDevice::Write8(uint32_t offset, uint8_t data) {
  cpu->DebugMessage("INFO (FB): 8-bit writes not supported -- 0x%08x, data 0x%02x", offset, data);
}

void FrameBufferDevice::Write16(uint32_t offset, uint16_t data) {
  cpu->DebugMessage("INFO (FB): 16-bit writes not supported -- 0x%08x, data 0x%04x", offset, data);
}

void FrameBufferDevice::Write32(uint32_t offset, uint32_t data) {
  data = util::ByteSwap32(data);

  switch (offset) {
  case 0x00: // Control register
    enabled = data & 0x1;
    break;

  case 0x14: // Buffer address register
    baseAddr = data;
    break;

  default:
    fprintf(stderr, "DEBUG: FB unknown write -- addr=0x%08x, data=0x%08x\n", offset, data);
    break;
  }
}

// REVISIT: need to check native image data format and convert if needed
void FrameBufferDevice::UpdateDisplayLoop(int32_t data) {
  if (graphics.is_null()) {
    // Graphics context is null so can't update.  Clear the copy as well so the
    // next DidChangeView() will restart the display loop
    graphicsCopy = graphics;
  } else {
    const pp::Size size = graphics.size();
    const uint32_t* const ramBuf = ram;
    const PP_ImageDataFormat format = pp::ImageData::GetNativeImageDataFormat();
    pp::ImageData imageData(ppInstance,
                            format,
                            size,
                            !enabled);
    uint32_t* targetBuf = static_cast<uint32_t*>(imageData.data());
    
    if (targetBuf != NULL) {
      if (enabled) {
        if (format == PP_IMAGEDATAFORMAT_BGRA_PREMUL) {
          for (uint32_t i = 0; i < (size.width() * size.height()); ++i) {
            targetBuf[i] = ramBuf[i + (baseAddr >> 2)] | 0xff000000;
          }
        } else {
          // REVISIT: conversion untested
          for (uint32_t i = 0; i < (size.width() * size.height()); ++i) {
            const uint32_t val = ramBuf[i + (baseAddr >> 2)];
            targetBuf[i] = ((val | 0xff000000) & 0xff00ff00)
              | ((val & 0xff) << 16)
              | ((val & 0xff0000) >> 16);
          }
        }
      }
      
      graphics.ReplaceContents(&imageData);
    }
    
    graphicsCopy = graphics;
    graphics.Flush(callbackFactory.NewCallback(&FrameBufferDevice::UpdateDisplayLoop));
  }
}
