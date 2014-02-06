/*
 * Utility namespace of the OpenRISC 1000 emulator
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "util.hpp"

namespace util {

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
}
