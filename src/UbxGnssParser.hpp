/*
  UbxGnssParser - Decodes a u-blox UBX byte stream into the last valid NAV-PVT, NAV-RELPOSNED, ACK and firmware version.
  Copyright (C) 2011-2026 J.A. Woltjer.
  All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <stdint.h>

#include "UbxGnssProtocol.hpp"

namespace ubxgnss
{
// The byte-by-byte decoder that was NeptuneGPS Salacia's UbxGpsSource::decodeByte(),
// moved here unchanged in behaviour so Salacia and Proteus share it. No UART, no Arduino:
// the caller reads its own port and hands every byte to Feed(). That keeps the parser
// testable on the host and leaves each firmware its own port handling (Salacia's config
// mode takes the UART over; Proteus also relays RTCM into it).
//
// Only the four message types below are accepted, each at its exact length (MON-VER within
// its range); any other class/id/length resynchronises at the next preamble. A payload is
// assembled in scratch and copied out only once its checksum validates, so a corrupt frame
// never replaces the last good one.
enum class UbxMessage : uint8_t
{
    None,
    NavPvt,
    NavRelPosNed,
    Ack,      // ACK-ACK or ACK-NAK; see GetAck()
    MonVer,
};

class UbxGnssParser
{
public:
    UbxGnssParser();

    // Feeds one byte. Returns the message this byte completed with a valid checksum, else
    // None.
    UbxMessage Feed(uint8_t byte);

    // Drops a frame in flight (after a baud change, say). The last good messages stay.
    void Reset();

    // The last valid frames, header and checksum included. Zero payload, with the header
    // filled in, until the first one arrives.
    inline const NavPvtFrame& GetNavPvt() const { return latestPvt; }
    inline const NavRelPosNedFrame& GetNavRelPosNed() const { return latestRpn; }

    // The last ACK-ACK/NAK: whether it was an ACK, and the class/id it answers.
    inline bool GetAckIsAck() const { return ackIsAck; }
    inline uint8_t GetAckedClass() const { return ackedClass; }
    inline uint8_t GetAckedId() const { return ackedId; }

    // What followed "FWVER=" in the last MON-VER that had it ("HPG 1.51"); empty until then.
    inline const char* GetFirmwareVersion() const { return firmwareVersion; }

    // Frames whose checksum failed, for diagnostics.
    inline uint32_t GetChecksumErrors() const { return checksumErrors; }

    static constexpr uint16_t kPvtBodyLength = kUbxPvtPayloadLength - kUbxHeaderLength - kUbxChecksumLength;
    static constexpr uint16_t kRpnBodyLength = kUbxRpnPayloadLength - kUbxHeaderLength - kUbxChecksumLength;
    // Sized to Salacia's GpsUnitStatus::firmwareVersion, which copies it byte for byte.
    static constexpr uint8_t kFirmwareVersionSize = 24;

private:
    enum class Stage : uint8_t
    {
        Sync1,
        Sync2,
        Class,
        Id,
        LengthLow,
        LengthHigh,
        Payload,
        ChecksumA,
        ChecksumB
    };

    Stage stage = Stage::Sync1;
    uint8_t msgClass = 0;
    uint8_t msgId = 0;
    uint16_t payloadLength = 0;
    uint16_t payloadIndex = 0;
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    uint8_t expectedChecksumA = 0;

    NavPvtFrame pvtScratch{};
    NavRelPosNedFrame rpnScratch{};
    uint8_t ackScratch[kUbxAckPayloadLength]{};
    uint8_t monVerScratch[kUbxMonVerMaxPayloadLength]{};

    NavPvtFrame latestPvt{};
    NavRelPosNedFrame latestRpn{};
    bool ackIsAck = false;
    uint8_t ackedClass = 0;
    uint8_t ackedId = 0;
    char firmwareVersion[kFirmwareVersionSize]{};
    uint32_t checksumErrors = 0;

    bool acceptsCurrentHeader() const;
    UbxMessage completeFrame();
    void extractFirmwareVersion(const uint8_t* payload, uint16_t length);
};
}
