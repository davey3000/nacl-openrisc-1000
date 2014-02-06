/*
 * Header file for the utility namespace of the OpenRISC 1000 emulator
 */

#ifndef OR_EMULATOR_UTIL_HPP_
#define OR_EMULATOR_UTIL_HPP_

#include <stdarg.h>

#include "ppapi/c/pp_stdint.h"

namespace util {

  /*
   * Printf to a newly allocated C string.
   * @param[in] format A printf format string.
   * @param[in] args The printf arguments.
   * @return The newly constructed string. Caller takes ownership.
   */
  char* VprintfToNewString(const char* format, va_list args);

  /*
   * Swap bytes in the passed value to switch between big and little endian
   * (or vice versa)
   */
  inline uint16_t ByteSwap16(uint16_t num) {
    return ((num & 0xff) << 8) | ((num >> 8) & 0xff);
  }

  inline uint32_t ByteSwap32(uint32_t num) {
    return (((num >> 24) & 0x000000ff) |
            ((num <<  8) & 0x00ff0000) |
            ((num >>  8) & 0x0000ff00) |
            ((num << 24) & 0xff000000));
  }

}

#endif
