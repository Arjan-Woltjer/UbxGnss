// Teensy/Arduino-core extras used by Salacia that neither real Arduino nor
// EpoxyDuino's generic AVR-flavored core provide, for the native test build.
// Included from support/Arduino.h, after EpoxyDuino's real Arduino.h is
// already visible.
#pragma once

// attachInterrupt()/detachInterrupt() are declared extern "C" by EpoxyDuino's
// Arduino.h but never defined (EpoxyDuino has no interrupt simulation) --
// AutosteerSource::begin() references them, so the native build needs a
// resolvable (no-op) definition. Matching extern "C" + inline lets this
// header be included from every translation unit without a link conflict.
extern "C" {
inline void attachInterrupt(uint8_t /*interruptNum*/, void (* /*userFunc*/)(void), int /*mode*/) {}
inline void detachInterrupt(uint8_t /*interruptNum*/) {}
}

// Teensy-specific fast digital I/O: timing doesn't matter on host, so these
// just forward to the regular (test-controllable) digitalRead()/digitalWrite().
inline int digitalReadFast(uint8_t pin) { return digitalRead(pin); }
inline void digitalWriteFast(uint8_t pin, uint8_t val) { digitalWrite(pin, val); }

// Teensy PWM extension; no PWM hardware to configure on host.
inline void analogWriteFrequency(uint8_t /*pin*/, uint32_t /*frequency*/) {}

// Teensy 4's built-in die-temperature sensor (issue #18's HealthReport). Fixed room-temperature
// value -- nothing in this test suite asserts on an exact reading, just that a HealthReport
// response carries a value through end to end.
inline float tempmonGetTemp() { return 25.0f; }

// Seeeduino/Adafruit SAMD core's combined frequency+duty PWM call. Salacia's own real
// target is always Teensy (TEENSYDUINO is defined there), but SteeringActuator
// (Shared Firmware Libs/, also consumed by NeptuneGPS Triton's SAMD21-based
// Motorcontroller) falls back to this API on any build where TEENSYDUINO isn't defined
// -- including this native test build -- so it needs to resolve here too, matching
// analogWrite()'s own no-op-with-no-readback convention above (no test in this suite
// asserts on the exact PWM duty value).
inline void pwm(uint8_t /*pin*/, uint32_t /*freq*/, uint32_t /*value*/) {}

// Teensy digital pins double as their own interrupt numbers.
inline uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }

// Teensy 4.1 analog pin numbers used by Salacia's AutosteerPins.hpp beyond
// what EpoxyDuino's AVR-style pins_arduino_avr.h defines (A0-A9). These are
// placeholder values for the test double, not real Teensy A14-A17 (38-41) --
// harmless here since analogRead() always returns 0 on EpoxyDuino regardless
// of pin, so these never interact with the digitalRead()/digitalWrite() mock
// state even if a value happened to coincide with a digital pin literal used
// elsewhere, exactly like real Teensy hardware where analog-capable pins are
// also usable as plain digital pins.
#ifndef A14
#define A14 14
#endif
#ifndef A15
#define A15 15
#endif
#ifndef A16
#define A16 16
#endif
#ifndef A17
#define A17 17
#endif
