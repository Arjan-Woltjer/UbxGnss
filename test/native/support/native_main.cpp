// Host entry point for Salacia's native test build, replacing reliance on
// EpoxyDuino's own cores/epoxy/main.cpp -- that file requires <termios.h>
// (raw terminal mode) to put the console in a specific input mode, which
// doesn't exist on Windows/MinGW and doesn't buy anything for a
// non-interactive AUnit run anyway. Adapted from Triton/Loofdoes
// Spuitcomputer's test/arduino_stubs/native_main.cpp.
#include "Arduino.h"
#include <AUnit.h>

// Global Serial instance (declared extern in Arduino.h)
NativeSerial Serial;

// EpoxyDuino globals read by TestRunner::processCommandLine()
int epoxy_argc = 0;
const char* const* epoxy_argv = nullptr;

// Declared in SalaciaNativeTests.ino
void setup();
void loop();

int main(int argc, char* argv[])
{
    epoxy_argc = argc;
    epoxy_argv = (const char* const*)argv;
    setup();
    // AUnit calls exit() once all tests are resolved (EPOXY_DUINO path)
    while (true)
    {
        loop();
    }
    return 0;
}
