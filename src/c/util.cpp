/*
 * Utility namespace of the OpenRISC 1000 emulator
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "util.hpp"

namespace util {

  /*
   * Printf to a newly allocated C string.
   * @param[in] format A printf format string.
   * @param[in] args The printf arguments.
   * @return The newly constructed string. Caller takes ownership. */
  char* VprintfToNewString(const char* format, va_list args) {
    va_list args_copy;
    int length;
    char* buffer;
    int result;
    
    va_copy(args_copy, args);
    length = vsnprintf(NULL, 0, format, args);
    buffer = (char*)malloc(length + 1); /* +1 for NULL-terminator. */
    result = vsnprintf(&buffer[0], length + 1, format, args_copy);
    if (result != length) {
      assert(0);
      return NULL;
    }
    return buffer;
  }
  
  /*
   * Printf to a new PP_Var.
   * @param[in] format A print format string.
   * @param[in] ... The printf arguments.
   * @return A new PP_Var.
   */
  /*pp::Var PrintfToVar(const char* format, ...) {
     char* string;
     va_list args;
     pp::Var var;
     
     va_start(args, format);
     string = VprintfToNewString(format, args);
     va_end(args);
     
     var = pp::Var(string);
     free(string);
     
     return var;
   }*/

  /*
   * Swap bytes in the passed value to switch between big and little endian
   * (or vice versa)
   */
  uint16_t ByteSwap16(uint16_t num) {
    return ((num & 0xff) << 8) | ((num >> 8) & 0xff);
  }

  uint32_t ByteSwap32(uint32_t num) {
    return (((num >> 24) & 0x000000ff) |
            ((num <<  8) & 0x00ff0000) |
            ((num >>  8) & 0x0000ff00) |
            ((num << 24) & 0xff000000));
  }
}
