// Inline implementations of Print::print(String) that need WString.h's
// complete type. Included at the bottom of Arduino.h, after WString.h.
// Adapted from Triton's test/arduino_stubs.
#pragma once
#include "Print.h"
#include "WString.h"

inline size_t Print::print(const String& s)   { return print(s.c_str()); }
inline size_t Print::println(const String& s) { size_t r = print(s); return r + println(); }
