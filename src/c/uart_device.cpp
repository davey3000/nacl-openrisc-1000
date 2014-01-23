/*
 * Class implementing an emulated UART device
 */

// REVISIT: device registers only accessible via 8-bit accesses.  Is this okay?

#include "uart_device.hpp"

UARTDevice::UARTDevice(CPU* cpu) : cpu(cpu) {
  Reset();
}

UARTDevice::~UARTDevice() {
  Reset();
}

void UARTDevice::Reset() {
  std::lock_guard<std::mutex> lock(fifoMutex);

  LCR = 0x3; // Line Control, reset, character has 8 bits
  LSR = UART_LSR_TRANSMITTER_EMPTY | UART_LSR_FIFO_EMPTY; // Line Status register, Transmitter serial register empty and Transmitter buffer register empty
  MSR = 0; // modem status register
  IIR = UART_IIR_NO_INT; // // Interrupt Identification, no interrupt
  ints = 0x0; // no interrupt pending
  IER = 0x0; //Interrupt Enable
  DLL = 0;
  DLH = 0;
  FCR = 0x0; // FIFO Control;
  MCR = 0x0; // Modem Control
  input = 0;

  while (!fifo.empty()) {
    fifo.pop();
  }
}

void UARTDevice::ReceiveChar(uint8_t x) {
  std::lock_guard<std::mutex> lock(fifoMutex);

  fifo.push(x);
  if (fifo.size() >= 1) {
    input = fifo.front();
    fifo.pop();
    ClearInterrupt(UART_IIR_CTI);
    LSR |= UART_LSR_DATA_READY;
    ThrowCTI();
  }
};

void UARTDevice::ThrowCTI() {
  ints |= (1 << UART_IIR_CTI);
  if (!(IER & UART_IER_RDI)) {
    return;
  }
  if ((IIR != UART_IIR_RLSI) && (IIR != UART_IIR_RDI)) {
    IIR = UART_IIR_CTI;
    cpu->RaiseInterrupt(0x2);
  }
};

void UARTDevice::ThrowTHRI() {
  ints |= (1 << UART_IIR_THRI);
  if (!(IER & UART_IER_THRI)) {
    return;
  }
  if ((IIR & UART_IIR_NO_INT) || (IIR == UART_IIR_MSI) || (IIR == UART_IIR_THRI)) {
    IIR = UART_IIR_THRI;
    cpu->RaiseInterrupt(0x2);
  }
};

void UARTDevice::NextInterrupt() {
  if ((ints & (1 << UART_IIR_CTI)) && (IER & UART_IER_RDI)) {
    ThrowCTI();
  } else if ((ints & (1 << UART_IIR_THRI)) && (IER & UART_IER_THRI)) {
    ThrowTHRI();
  } else {
    IIR = UART_IIR_NO_INT;
    cpu->ClearInterrupt(0x2);
  }
};

void UARTDevice::ClearInterrupt(uint32_t line) {
  ints &= ~(1 << line);
  IIR = UART_IIR_NO_INT;
  if (line != IIR) {
    return;
  }
  NextInterrupt();
};

uint8_t UARTDevice::Read8(uint32_t offset) {
  uint8_t ret;

  if (LCR & UART_LCR_DLAB) {
    switch (offset) {
    case UART_DLL:
      return DLL;
      break;
    case UART_DLH:
      return DLH;
      break;
    }
  }
  switch (offset) {
  case 0:
    ret = input;
    input = 0;
    ClearInterrupt(UART_IIR_RDI);
    ClearInterrupt(UART_IIR_CTI);
    {
      std::lock_guard<std::mutex> lock(fifoMutex);

      if (fifo.size() >= 1) {
        input = fifo.front();
        fifo.pop();
        LSR |= UART_LSR_DATA_READY;
      } else {
        LSR &= ~UART_LSR_DATA_READY;
      }
    }
    return ret;
  case UART_IER:
    return IER & 0x0F;
  case UART_MSR:
    return MSR;
  case UART_IIR:
    {
      ret = (IIR & 0x0f) | 0xC0; // the two top bits are always set
      if (IIR == UART_IIR_THRI) {
        ClearInterrupt(UART_IIR_THRI);
      }
      return ret;
    }
  case UART_LCR:
    return LCR;
  case UART_LSR:
    return LSR;

  default:
    cpu->DebugMessage("INFO (UART): 8-bit read of unsupported offset 0x%08x", offset);
    return 0;
  }
}

uint16_t UARTDevice::Read16(uint32_t offset) {
  cpu->DebugMessage("INFO (UART): 16-bit reads not supported -- 0x%08x", offset);
  return 0;
}

uint32_t UARTDevice::Read32(uint32_t offset) {
  cpu->DebugMessage("INFO (UART): 32-bit reads not supported -- 0x%08x", offset);
  return 0;
}

void UARTDevice::Write8(uint32_t offset, uint8_t data) {
  data &= 0xFF;
  if (LCR & UART_LCR_DLAB) {
    switch (offset) {
    case UART_DLL:
      DLL = data;
      return;
    case UART_DLH:
      DLH = data;
      return;
    }
  }

  switch (offset) {
  case 0:
    LSR &= ~UART_LSR_FIFO_EMPTY;
    cpu->PostMessage((int32_t)data);
    // Data is sent with a latency of zero!
    LSR |= UART_LSR_FIFO_EMPTY; // send buffer is empty					
    ThrowTHRI();
    break;
  case UART_IER:
    // 2 = 10b ,5=101b, 7=111b
    IER = data & 0x0F; // only the first four bits are valid
    // Ok, check immediately if there is an interrupt pending
    NextInterrupt();
    break;
  case UART_FCR:
    FCR = data;
    if (FCR & 2) {
      std::lock_guard<std::mutex> lock(fifoMutex);

      while (!fifo.empty()) {
        fifo.pop();
      }
    }
    break;
  case UART_LCR:
    LCR = data;
    break;
  case UART_MCR:
    MCR = data;
    break;
  default:
    cpu->DebugMessage("INFO (UART): 8-bit write of unsupported offset 0x%08x, data 0x%02x", offset, data);
    break;
  }
}

void UARTDevice::Write16(uint32_t offset, uint16_t data) {
  cpu->DebugMessage("INFO (UART): 16-bit writes not supported -- 0x%08x, data 0x%04x", offset, data);
}

void UARTDevice::Write32(uint32_t offset, uint32_t data) {
  cpu->DebugMessage("INFO (UART): 32-bit writes not supported -- 0x%08x, data 0x%08x", offset, data);
}
