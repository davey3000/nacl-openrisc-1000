/*
 * Header file for the class implementing an emulated frame buffer device
 */

/*
 * Note that when this class is destroyed, it does _not_ destroy the linked
 * pp::Graphics2D context
 */

#ifndef OR_EMULATOR_FRAMEBUFFER_DEVICE_HPP_
#define OR_EMULATOR_FRAMEBUFFER_DEVICE_HPP_

#include <pthread.h>

#include <atomic>

#include "ppapi/cpp/graphics_2d.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/utility/completion_callback_factory.h"

#include "device.hpp"
#include "cpu.hpp"

class FrameBufferDevice : public Device {
private:
  pp::Instance* ppInstance;
  CPU* cpu;
  uint32_t* ram;
  pp::Graphics2D graphics;
  pp::Graphics2D graphicsCopy;

  std::atomic<bool> enabled;
  std::atomic<uint_least32_t> baseAddr;

  pp::CompletionCallbackFactory<FrameBufferDevice> callbackFactory;

public:
  FrameBufferDevice(pp::Instance* ppInstance, CPU* cpu, uint32_t* ram);
  virtual ~FrameBufferDevice();

  virtual void Reset();

  /*
   * Set the 2D graphics context linked to the frame buffer.  This should only
   * be called once before the frame buffer is used, and is _not_ thread-safe.
   * Calls after the first will have no effect.
   */
  void SetGraphics2DContext(pp::Graphics2D newGraphics);

  /*
   * Update the 2D graphics context due to a view scale change
   */
  void OnViewScaleChange(float scale);

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
  /*
   * Method that updates the graphics context from the emulated RAM and sets a
   * callback to itself to update the context again after the next vsync
   */
  void UpdateDisplayLoop(int32_t data);

};

#endif
