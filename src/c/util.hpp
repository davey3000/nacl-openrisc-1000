/*
 * Header file for the utility namespace of the OpenRISC 1000 emulator
 */

#ifndef OR_EMULATOR_UTIL_HPP_
#define OR_EMULATOR_UTIL_HPP_

#include <stdarg.h>

#include "ppapi/c/pp_stdint.h"

namespace util {

  char* VprintfToNewString(const char* format, va_list args);

  uint16_t ByteSwap16(uint16_t num);
  uint32_t ByteSwap32(uint32_t num);

}

#endif
