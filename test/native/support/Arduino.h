// Minimal Arduino.h for Salacia's native test build.
//
// Base (types, Print/Stream plumbing, pgmspace macros) adapted from
// Triton/Loofdoes Spuitcomputer's test/arduino_stubs -- same underlying
// problem (compile AUnit + Arduino-shaped firmware code with a plain host
// compiler). Extended with what Salacia additionally needs: a real
// millis()/micros() (Salacia's channel watchdogs use actual elapsed time,
// not per-test-file overrides), and EpoxyDuino-compatible digitalReadValue()/
// digitalWriteValue() test-control hooks -- test_autosteer_source.cpp drives
// pin state through these, matching the API AUnit-based Arduino tests
// conventionally use. Like EpoxyDuino's own implementation, the pin mock
// state is a uint32_t bitmask, so only pins < 32 are controllable this way
// (test_autosteer_source.cpp already works around this for kEncoder1BPin=34;
// see its own comments and Salacia/test/README.md's "Known gaps").
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <chrono>
#include "Print.h"
#include "Stream.h"
#include "WString.h"

typedef uint8_t  byte;
typedef unsigned int word;

#define HIGH 1
#define LOW  0
#define INPUT  0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define MSBFIRST 1
#define LSBFIRST 0

#define CHANGE  1
#define FALLING 2
#define RISING  3

// Standard AVR analog pin aliases (identity mapping, matching EpoxyDuino's
// own pins_arduino_avr.h) -- A14-A17 (Teensy-specific, beyond this range) are
// in TeensyCompat.h instead, included at the bottom of this file.
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9

#ifndef FLASH_STRING_HELPER_DEFINED
#define FLASH_STRING_HELPER_DEFINED
class __FlashStringHelper {};
#endif

// PROGMEM/PSTR/F()/pgm_read_*/*_P are also defined by pgmspace.h (which
// AUnit's own Flash.h includes separately when EPOXY_DUINO is set) -- guard
// with #ifndef so both can be included in the same translation unit without
// a harmless-but-noisy redefinition warning.
#ifndef PROGMEM
#  define PROGMEM
#endif
#ifndef PSTR
#  define PSTR(x)         (x)
#endif
#ifndef F
#  define F(x)            ((const __FlashStringHelper*)(x))
#endif
#ifndef pgm_read_byte
#  define pgm_read_byte(p)  (*(const uint8_t*)(p))
#endif
#ifndef pgm_read_word
#  define pgm_read_word(p)  (*(const uint16_t*)(p))
#endif
#ifndef pgm_read_ptr
#  define pgm_read_ptr(p)   (*(const void* const*)(p))
#endif
#ifndef strlen_P
#  define strlen_P          strlen
#  define strcmp_P          strcmp
#  define strncmp_P         strncmp
#  define strcasecmp_P      strcasecmp
#  define memcpy_P          memcpy
#endif

#ifndef SERIAL_PORT_MONITOR
#define SERIAL_PORT_MONITOR Serial
#endif

#ifdef _WIN32
#  include <string.h>
#  ifndef strcasecmp
#    define strcasecmp  _stricmp
#    define strncasecmp _strnicmp
#  endif
#endif

inline void pinMode(uint8_t, uint8_t) {}
inline void analogWrite(uint8_t, int) {}
inline void analogReference(uint8_t) {}

inline int analogRead(uint8_t) { return 0; } // no ADC on host; always 0, matching EpoxyDuino

// ---------------------------------------------------------------------------
// digitalRead()/digitalWrite() with EpoxyDuino-compatible mock-control hooks.
// C++17 inline variables (one definition across every translation unit that
// includes this header) avoid needing a separate Arduino.cpp.
// ---------------------------------------------------------------------------
namespace internal_arduino_stub
{
inline uint32_t g_digitalReadPinValues = 0;
inline uint32_t g_digitalWritePinValues = 0;
}

inline void digitalWrite(uint8_t pin, uint8_t val)
{
    if (pin >= 32) return;
    if (val == 0) internal_arduino_stub::g_digitalWritePinValues &= ~(uint32_t(1) << pin);
    else           internal_arduino_stub::g_digitalWritePinValues |=  (uint32_t(1) << pin);
}

inline uint8_t digitalWriteValue(uint8_t pin)
{
    if (pin >= 32) return 0;
    return (internal_arduino_stub::g_digitalWritePinValues & (uint32_t(1) << pin)) != 0;
}

inline int digitalRead(uint8_t pin)
{
    if (pin >= 32) return 0;
    return (internal_arduino_stub::g_digitalReadPinValues & (uint32_t(1) << pin)) != 0;
}

inline void digitalReadValue(uint8_t pin, uint8_t val)
{
    if (pin >= 32) return;
    if (val == 0) internal_arduino_stub::g_digitalReadPinValues &= ~(uint32_t(1) << pin);
    else           internal_arduino_stub::g_digitalReadPinValues |=  (uint32_t(1) << pin);
}

// Real elapsed time -- Salacia's channel watchdogs (e.g. NeptuneUdpChannel::
// watchdogTick()) compare deltas of actual millis() values, not a
// per-test-file fake.
inline unsigned long millis()
{
    using namespace std::chrono;
    return static_cast<unsigned long>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

inline unsigned long micros()
{
    using namespace std::chrono;
    return static_cast<unsigned long>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}

// Deliberately no min()/max() macros here (unlike Triton's stubs) -- Salacia's
// compiled sources never call bare min()/max(), and #defining them would
// break every std::min/std::max call inside <vector>/<algorithm>, which
// support/HardwareSerial.h and support/NativeEthernetUdp.h both use.

// EpoxyDuino globals -- defined in native_main.cpp, read by AUnit's TestRunner
extern int epoxy_argc;
extern const char* const* epoxy_argv;

// Global debug-output Serial -- writes to real stdout via Print's default
// write(). Named distinctly from support/HardwareSerial.h's `HardwareSerial`
// class (the GPS-port fake, a different role) to avoid a name clash.
class NativeSerial : public Stream
{
public:
    void begin(unsigned long) {}
    void end() {}
    operator bool() { return true; }
};

extern NativeSerial Serial;

// Print::print(String) needs the complete String type from WString.h
#include "PrintImpl.h"

// Real Teensy/Arduino cores make HardwareSerial available transitively
// through <Arduino.h> -- UbxGpsSource.hpp/SbfGpsSource.hpp rely on that same
// transitive availability rather than including "HardwareSerial.h" directly.
#include "HardwareSerial.h"

#include "TeensyCompat.h"
