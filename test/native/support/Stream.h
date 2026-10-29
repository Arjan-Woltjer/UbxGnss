// Minimal Stream base class for Salacia's native test build (adapted from
// Triton's test/arduino_stubs). Only available()/read()/peek() are needed --
// none of Salacia's compiled sources use Stream's higher-level parseInt()/
// find()/readBytes() helpers, so those aren't provided.
#pragma once
#include "Print.h"

class Stream : public Print {
  public:
    virtual ~Stream() {}
    virtual int available() { return 0; }
    virtual int read()      { return -1; }
    virtual int peek()      { return -1; }
};
