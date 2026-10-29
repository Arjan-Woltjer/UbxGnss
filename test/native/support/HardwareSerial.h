// Fake HardwareSerial for Salacia's native (EpoxyDuino) test build.
//
// EpoxyDuino has no HardwareSerial type (only StdioSerial, which talks to the
// process's actual stdin/stdout). UbxGpsSource/SbfGpsSource take a
// HardwareSerial& constructor parameter, and NtripChannel relays RTCM bytes
// onto a GPS source's Stream* (backed by the same object) -- this fake gives
// tests a byte queue they can push bytes into and a log they can inspect
// what was written.
#pragma once

#include <Stream.h>

#include <cstddef>
#include <vector>

class HardwareSerial : public Stream
{
public:
    HardwareSerial() = default;

    // Print::write(const uint8_t*, size_t) (buffer write, e.g. UbxGpsSource's poll-frame send)
    // would otherwise be hidden by this class's own write(uint8_t) override below -- C++ name
    // lookup stops at the first class in the hierarchy that declares any overload of a given
    // name, it doesn't merge overload sets across inheritance the way real Teensyduino's own
    // HardwareSerial (which declares both directly) doesn't run into this. Real Arduino cores
    // never hit this because they always redeclare both; only this synthetic stub needed the
    // explicit using-declaration to match.
    using Print::write;

    int available() override
    {
        return static_cast<int>(rx_.size() - rxIndex_);
    }

    int read() override
    {
        if (rxIndex_ >= rx_.size())
        {
            return -1;
        }
        return rx_[rxIndex_++];
    }

    int peek() override
    {
        if (rxIndex_ >= rx_.size())
        {
            return -1;
        }
        return rx_[rxIndex_];
    }

    size_t write(uint8_t byte) override
    {
        txLog_.push_back(byte);
        return 1;
    }

    // A generously large, always-available headroom -- this fake has no real ring buffer to run
    // out of, and tests care about what gets written, not throttling the write across ticks.
    int availableForWrite() override
    {
        return 256;
    }

    // Real HardwareSerial::begin() re-inits the peripheral at a new baud without touching
    // already-queued rx_/txLog_ content (Teensy's own begin() only resets ring-buffer head/tail
    // indices, not the registered storage -- see UbxGpsSource::SetUartBaud()'s comment), so this
    // fake matches that: records the requested baud for tests to inspect, nothing else.
    void begin(uint32_t baud)
    {
        mockLastBeginBaud_ = baud;
    }

    // ---- test control ----

    uint32_t mockLastBeginBaud() const
    {
        return mockLastBeginBaud_;
    }

    // Queues bytes to be returned by subsequent available()/read()/peek() calls.
    void mockPushRx(const uint8_t* data, size_t length)
    {
        for (size_t i = 0; i < length; ++i)
        {
            rx_.push_back(data[i]);
        }
    }

    void mockPushRxByte(uint8_t byte)
    {
        rx_.push_back(byte);
    }

    // Every byte passed to write(), in call order (e.g. relayed RTCM bytes).
    const std::vector<uint8_t>& mockTxLog() const
    {
        return txLog_;
    }

    void mockClear()
    {
        rx_.clear();
        rxIndex_ = 0;
        txLog_.clear();
    }

private:
    std::vector<uint8_t> rx_;
    size_t rxIndex_ = 0;
    std::vector<uint8_t> txLog_;
    uint32_t mockLastBeginBaud_ = 0;
};
