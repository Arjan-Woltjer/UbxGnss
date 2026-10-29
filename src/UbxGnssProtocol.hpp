/*
  UbxGnssProtocol - Declares the u-blox UBX constants and message layouts both NeptuneGPS firmwares decode.
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

// Moved out of NeptuneGPS Salacia (protocol/NeptuneProtocol.hpp), byte for byte, so Salacia
// and NeptuneGPS Proteus decode the F9P with one parser instead of two (Proteus
// docs/gnss-can-module-plan.md, phase 1). Salacia's host protocol carries NavPvtFrame and
// NavRelPosNedFrame as they are, header and checksum included, which is why these are whole
// UBX frames rather than bare payloads, and why their layout must never change here
// without Salacia's ICD changing with it. The static_asserts at the bottom pin the sizes.
namespace ubxgnss
{
inline constexpr uint8_t kUbxPreamble1 = 0xB5;
inline constexpr uint8_t kUbxPreamble2 = 0x62;
inline constexpr uint16_t kUbxHeaderLength = 6;     // preambles, class, id, 16-bit length
inline constexpr uint16_t kUbxChecksumLength = 2;

inline constexpr uint8_t kUbxNavClass = 0x01;
inline constexpr uint8_t kUbxNavPvtId = 0x07;
inline constexpr uint8_t kUbxNavRpnId = 0x3C;       // NAV-RELPOSNED
inline constexpr uint16_t kUbxPvtPayloadLength = 100;   // whole frame: 6 + 92 + 2
inline constexpr uint16_t kUbxRpnPayloadLength = 72;    // whole frame: 6 + 64 + 2

// UBX-ACK-ACK/NAK: the receiver's answer to an input message such as UBX-CFG-VALSET. A
// fixed 2-byte payload: the class and id of the message it answers.
inline constexpr uint8_t kUbxAckClass = 0x05;
inline constexpr uint8_t kUbxAckAckId = 0x01;
inline constexpr uint8_t kUbxAckNakId = 0x00;
inline constexpr uint16_t kUbxAckPayloadLength = 2;

inline constexpr uint8_t kUbxCfgClass = 0x06;
inline constexpr uint8_t kUbxCfgValSetId = 0x8A;

// UBX-MON-VER: a fixed 40-byte swVersion(30)/hwVersion(10) block, then zero or more 30-byte
// extension strings, one of which is conventionally "FWVER=..." (e.g. "FWVER=HPG 1.51" on
// the bench F9Ps). Variable length, so the parser accepts a range; the cap of 10 extensions
// is generous against the 220 bytes (6 extensions) seen live, not a spec maximum.
inline constexpr uint8_t kUbxMonClass = 0x0A;
inline constexpr uint8_t kUbxMonVerId = 0x04;
inline constexpr uint16_t kUbxMonVerFixedFieldsLength = 40;
inline constexpr uint16_t kUbxMonVerExtensionLength = 30;
inline constexpr uint16_t kUbxMonVerMaxPayloadLength = kUbxMonVerFixedFieldsLength + 10 * kUbxMonVerExtensionLength;

// The MON-VER poll: an empty-payload request for class/id 0x0A/0x04, checksum included.
inline constexpr uint8_t kUbxMonVerPoll[8] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};

// NAV-PVT flag bits the consumers read (u-blox F9 HPG interface description).
inline constexpr uint8_t kPvtValidDate = 0x01;           // validityFlags ("valid")
inline constexpr uint8_t kPvtValidTime = 0x02;
inline constexpr uint8_t kPvtFullyResolved = 0x04;
inline constexpr uint8_t kPvtGnssFixOk = 0x01;           // statusFlags ("flags")
inline constexpr uint8_t kPvtDiffSoln = 0x02;
inline constexpr uint8_t kPvtHeadVehValid = 0x20;
inline constexpr uint8_t kPvtCarrSolnShift = 6;          // statusFlags bits 6-7
inline constexpr uint8_t kPvtCarrSolnMask = 0x03;
inline constexpr uint8_t kPvtCarrSolnFloat = 1;
inline constexpr uint8_t kPvtCarrSolnFixed = 2;

#pragma pack(push, 1)
struct NavPvtFrame
{
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t clas;
    uint8_t id;
    uint16_t length;

    // Named fields and raw[] alias the same bytes (the frame minus header and checksum), so
    // the byte-by-byte parser writes raw[] and everything else reads named fields.
    union
    {
        struct
        {
            uint32_t itow;
            uint16_t year;
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t minute;
            uint8_t second;
            uint8_t validityFlags;
            uint32_t timeAccuracy;
            int32_t nano;
            uint8_t fixType;
            uint8_t statusFlags;
            uint8_t additionalFlags;
            uint8_t numSv;
            int32_t lon;
            int32_t lat;
            int32_t heightEllipsoid;
            int32_t heightMsl;
            uint32_t horizontalAccuracy;
            uint32_t verticalAccuracy;
            int32_t velNorth;
            int32_t velEast;
            int32_t velDown;
            int32_t groundSpeed;
            int32_t headingMotion;
            uint32_t speedAccuracy;
            uint32_t headingAccuracy;
            uint16_t pdop;
            uint16_t flags2;
            uint8_t reserved[4];
            int32_t headingVehicle;
            int16_t magneticDeclination;
            uint16_t magneticAccuracy;
        };
        uint8_t raw[kUbxPvtPayloadLength - kUbxHeaderLength - kUbxChecksumLength];
    };

    uint8_t checksumA;
    uint8_t checksumB;
};

struct NavRelPosNedFrame
{
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t clas;
    uint8_t id;
    uint16_t length;

    union
    {
        struct
        {
            uint8_t version;
            uint8_t reserved0;
            uint16_t refStationId;
            uint32_t itow;
            int32_t relPosNorth;
            int32_t relPosEast;
            int32_t relPosDown;
            int32_t relPosLength;
            int32_t relPosHeading;
            uint8_t reserved1[4];
            int8_t relPosHpNorth;
            int8_t relPosHpEast;
            int8_t relPosHpDown;
            int8_t relPosHpLength;
            uint32_t accNorth;
            uint32_t accEast;
            uint32_t accDown;
            uint32_t accLength;
            uint32_t accHeading;
            uint8_t reserved2[4];
            uint8_t flags[4];
        };
        uint8_t raw[kUbxRpnPayloadLength - kUbxHeaderLength - kUbxChecksumLength];
    };

    uint8_t checksumA;
    uint8_t checksumB;
};
#pragma pack(pop)

static_assert(sizeof(NavPvtFrame) == kUbxPvtPayloadLength, "NAV-PVT frame layout drifted.");
static_assert(sizeof(NavRelPosNedFrame) == kUbxRpnPayloadLength, "NAV-RELPOSNED frame layout drifted.");

// The UBX 8-bit Fletcher checksum, over class, id, length and payload.
inline void GUpdateUbxChecksum(uint8_t* checksumA, uint8_t* checksumB, uint8_t byte)
{
    *checksumA = static_cast<uint8_t>(*checksumA + byte);
    *checksumB = static_cast<uint8_t>(*checksumB + *checksumA);
}
}
