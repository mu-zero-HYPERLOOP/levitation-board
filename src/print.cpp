#include <Arduino.h>
#include <stdarg.h>

constexpr bool DISABLE_PRINT = false;

void debugPrintf(const char *format, ...) {
  if constexpr (!DISABLE_PRINT) {
    va_list args;
    va_start(args, format);
    Serial.vprintf(format, args);
    va_end(args);
  }
}

void debugPrintChar(char c) {
  if constexpr (!DISABLE_PRINT) {
    Serial.print(c);
  }
}

void debugPrintFlush() {
  if constexpr (!DISABLE_PRINT) {
    Serial.flush();
  }
}
