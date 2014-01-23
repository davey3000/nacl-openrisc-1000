/*
 * Header file for the class implementing an emulated UART device
 */

#ifndef OR_EMULATOR_UART_DEVICE_HPP_
#define OR_EMULATOR_UART_DEVICE_HPP_

#include <mutex>
#include <queue>

#include "cpu.hpp"

class UARTDevice : public Device {
private:
  static const uint32_t UART_LSR_DATA_READY = 0x1;
  static const uint32_t UART_LSR_FIFO_EMPTY = 0x20;
  static const uint32_t UART_LSR_TRANSMITTER_EMPTY = 0x40;
  
  static const uint32_t UART_IER_THRI = 0x02; /* Enable Transmitter holding register int. */
  static const uint32_t UART_IER_RDI = 0x01; /* Enable receiver data interrupt */
  
  static const uint32_t UART_IIR_MSI = 0x00; /* Modem status interrupt (Low priority) */
  static const uint32_t UART_IIR_NO_INT = 0x01;
  static const uint32_t UART_IIR_THRI = 0x02; /* Transmitter holding register empty */
  static const uint32_t UART_IIR_RDI = 0x04; /* Receiver data interrupt */
  static const uint32_t UART_IIR_RLSI = 0x06; /* Receiver line status interrupt (High p.) */
  static const uint32_t UART_IIR_CTI = 0x0c; /* Character timeout */
  
  static const uint32_t UART_LCR_DLAB = 0x80; /* Divisor latch access bit */
  
  static const uint32_t UART_DLL = 0; /* R/W: Divisor Latch Low, DLAB=1 */
  static const uint32_t UART_DLH = 1; /* R/W: Divisor Latch High, DLAB=1 */
  
  static const uint32_t UART_IER = 1; /* R/W: Interrupt Enable Register */
  static const uint32_t UART_IIR = 2; /* R: Interrupt ID Register */
  static const uint32_t UART_FCR = 2; /* W: FIFO Control Register */
  static const uint32_t UART_LCR = 3; /* R/W: Line Control Register */
  static const uint32_t UART_MCR = 4; /* W: Modem Control Register */
  static const uint32_t UART_LSR = 5; /* R: Line Status Register */
  static const uint32_t UART_MSR = 6; /* R: Modem Status Register */

private:
  CPU* cpu;

  /* 
   * Device registers
   */
  uint32_t LCR;
  uint32_t LSR;
  uint32_t MSR;
  uint32_t IIR;
  uint32_t ints;
  uint32_t IER;
  uint32_t DLL;
  uint32_t DLH;
  uint32_t FCR;
  uint32_t MCR;
  uint8_t input;

  // Character FIFO (for display on an emulated terminal screen)
  std::queue<uint8_t> fifo;
  std::mutex fifoMutex;

public:
  UARTDevice(CPU* cpu);
  virtual ~UARTDevice();

  virtual void Reset();

  /*
   * Called when a character has been received by the terminal linked to the
   * emulator.  Thread-safe.
   */
  void ReceiveChar(uint8_t x);

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
  void ThrowCTI();
  void ThrowTHRI();
  void NextInterrupt();
  void ClearInterrupt(uint32_t line);

};

#endif
