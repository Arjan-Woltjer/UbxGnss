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
#include "UbxGnssParser.hpp"

#include <string.h>

namespace ubxgnss
{
UbxGnssParser::UbxGnssParser()
{
    latestPvt.preamble1 = kUbxPreamble1;
    latestPvt.preamble2 = kUbxPreamble2;
    latestPvt.clas = kUbxNavClass;
    latestPvt.id = kUbxNavPvtId;
    latestPvt.length = kPvtBodyLength;

    latestRpn.preamble1 = kUbxPreamble1;
    latestRpn.preamble2 = kUbxPreamble2;
    latestRpn.clas = kUbxNavClass;
    latestRpn.id = kUbxNavRpnId;
    latestRpn.length = kRpnBodyLength;
}

void UbxGnssParser::Reset()
{
    stage = Stage::Sync1;
    msgClass = 0;
    msgId = 0;
    payloadLength = 0;
    payloadIndex = 0;
    checksumA = 0;
    checksumB = 0;
    expectedChecksumA = 0;
}

bool UbxGnssParser::acceptsCurrentHeader() const
{
    if (msgClass == kUbxNavClass && msgId == kUbxNavPvtId)
    {
        return payloadLength == kPvtBodyLength;
    }
    if (msgClass == kUbxNavClass && msgId == kUbxNavRpnId)
    {
        return payloadLength == kRpnBodyLength;
    }
    if (msgClass == kUbxAckClass && (msgId == kUbxAckAckId || msgId == kUbxAckNakId))
    {
        return payloadLength == kUbxAckPayloadLength;
    }
    if (msgClass == kUbxMonClass && msgId == kUbxMonVerId)
    {
        return payloadLength >= kUbxMonVerFixedFieldsLength && payloadLength <= kUbxMonVerMaxPayloadLength;
    }
    return false;
}

UbxMessage UbxGnssParser::Feed(uint8_t byte)
{
    switch (stage)
    {
    case Stage::Sync1:
        if (byte == kUbxPreamble1)
        {
            stage = Stage::Sync2;
        }
        break;
    case Stage::Sync2:
        if (byte == kUbxPreamble2)
        {
            checksumA = 0;
            checksumB = 0;
            stage = Stage::Class;
        }
        else
        {
            stage = (byte == kUbxPreamble1) ? Stage::Sync2 : Stage::Sync1;
        }
        break;
    case Stage::Class:
        msgClass = byte;
        GUpdateUbxChecksum(&checksumA, &checksumB, byte);
        stage = Stage::Id;
        break;
    case Stage::Id:
        msgId = byte;
        GUpdateUbxChecksum(&checksumA, &checksumB, byte);
        stage = Stage::LengthLow;
        break;
    case Stage::LengthLow:
        payloadLength = byte;
        GUpdateUbxChecksum(&checksumA, &checksumB, byte);
        stage = Stage::LengthHigh;
        break;
    case Stage::LengthHigh:
        payloadLength = static_cast<uint16_t>(payloadLength | (static_cast<uint16_t>(byte) << 8));
        GUpdateUbxChecksum(&checksumA, &checksumB, byte);
        if (acceptsCurrentHeader())
        {
            payloadIndex = 0;
            stage = Stage::Payload;
        }
        else
        {
            Reset();
        }
        break;
    case Stage::Payload:
        if (msgClass == kUbxNavClass && msgId == kUbxNavPvtId)
        {
            pvtScratch.raw[payloadIndex] = byte;
        }
        else if (msgClass == kUbxNavClass && msgId == kUbxNavRpnId)
        {
            rpnScratch.raw[payloadIndex] = byte;
        }
        else if (msgClass == kUbxAckClass)
        {
            ackScratch[payloadIndex] = byte;
        }
        else
        {
            monVerScratch[payloadIndex] = byte;
        }
        GUpdateUbxChecksum(&checksumA, &checksumB, byte);
        ++payloadIndex;
        if (payloadIndex >= payloadLength)
        {
            stage = Stage::ChecksumA;
        }
        break;
    case Stage::ChecksumA:
        expectedChecksumA = byte;
        stage = Stage::ChecksumB;
        break;
    case Stage::ChecksumB:
    {
        UbxMessage completed = UbxMessage::None;
        if (checksumA == expectedChecksumA && checksumB == byte)
        {
            completed = completeFrame();
        }
        else
        {
            ++checksumErrors;
        }
        Reset();
        return completed;
    }
    }
    return UbxMessage::None;
}

UbxMessage UbxGnssParser::completeFrame()
{
    if (msgClass == kUbxNavClass && msgId == kUbxNavPvtId)
    {
        memcpy(latestPvt.raw, pvtScratch.raw, sizeof(latestPvt.raw));
        latestPvt.checksumA = checksumA;
        latestPvt.checksumB = checksumB;
        return UbxMessage::NavPvt;
    }
    if (msgClass == kUbxNavClass && msgId == kUbxNavRpnId)
    {
        memcpy(latestRpn.raw, rpnScratch.raw, sizeof(latestRpn.raw));
        latestRpn.checksumA = checksumA;
        latestRpn.checksumB = checksumB;
        return UbxMessage::NavRelPosNed;
    }
    if (msgClass == kUbxAckClass)
    {
        ackIsAck = (msgId == kUbxAckAckId);
        ackedClass = ackScratch[0];
        ackedId = ackScratch[1];
        return UbxMessage::Ack;
    }
    extractFirmwareVersion(monVerScratch, payloadLength);
    return UbxMessage::MonVer;
}

void UbxGnssParser::extractFirmwareVersion(const uint8_t* payload, uint16_t length)
{
    static const char kPrefix[] = "FWVER=";
    constexpr uint16_t kPrefixLength = sizeof(kPrefix) - 1;

    for (uint16_t offset = kUbxMonVerFixedFieldsLength; offset + kUbxMonVerExtensionLength <= length;
         offset += kUbxMonVerExtensionLength)
    {
        const char* extension = reinterpret_cast<const char*>(payload + offset);
        if (memcmp(extension, kPrefix, kPrefixLength) != 0)
        {
            continue;
        }

        // Extensions are null-padded but not null-terminated when the value fills the field,
        // so the scan is bounded by the field, never by a terminator.
        const uint16_t valueMaxLength = kUbxMonVerExtensionLength - kPrefixLength;
        uint16_t valueLength = 0;
        while (valueLength < valueMaxLength && extension[kPrefixLength + valueLength] != '\0')
        {
            ++valueLength;
        }

        const uint16_t copyLength =
            valueLength < kFirmwareVersionSize - 1 ? valueLength : static_cast<uint16_t>(kFirmwareVersionSize - 1);
        memcpy(firmwareVersion, extension + kPrefixLength, copyLength);
        firmwareVersion[copyLength] = '\0';
        return;
    }
    // No FWVER= extension in this reply: the version stays whatever it was.
}
}
