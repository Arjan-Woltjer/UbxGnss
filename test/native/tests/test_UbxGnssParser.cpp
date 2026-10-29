/*
  test_UbxGnssParser - Pins the UBX decoder both NeptuneGPS firmwares share.
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
// Ported from NeptuneGPS Salacia's test_UbxGpsSource.cpp along with the parser itself. The
// UART-facing cases (MON-VER polling, external owner, raw tap, baud change) stay in Salacia;
// these are the ones about what the byte stream decodes to.
#include <AUnit.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "UbxGnssParser.hpp"

using namespace ubxgnss;

namespace
{
constexpr uint16_t kPvtBodyLength = UbxGnssParser::kPvtBodyLength;
constexpr uint16_t kRpnBodyLength = UbxGnssParser::kRpnBodyLength;

std::vector<uint8_t> GBuildFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* body, uint16_t bodyLength)
{
    std::vector<uint8_t> frame = {kUbxPreamble1, kUbxPreamble2, msgClass, msgId,
                                  static_cast<uint8_t>(bodyLength & 0xFF), static_cast<uint8_t>(bodyLength >> 8)};
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    for (size_t i = 2; i < frame.size(); ++i)
    {
        GUpdateUbxChecksum(&checksumA, &checksumB, frame[i]);
    }
    for (uint16_t i = 0; i < bodyLength; ++i)
    {
        frame.push_back(body[i]);
        GUpdateUbxChecksum(&checksumA, &checksumB, body[i]);
    }
    frame.push_back(checksumA);
    frame.push_back(checksumB);
    return frame;
}

// Every message the frame completed, in order.
std::vector<UbxMessage> GFeed(UbxGnssParser& parser, const std::vector<uint8_t>& bytes)
{
    std::vector<UbxMessage> completed;
    for (const uint8_t byte : bytes)
    {
        const UbxMessage message = parser.Feed(byte);
        if (message != UbxMessage::None)
        {
            completed.push_back(message);
        }
    }
    return completed;
}

NavPvtFrame GSamplePvt()
{
    NavPvtFrame pvt{};
    pvt.itow = 123456789u;
    pvt.year = 2026;
    pvt.month = 7;
    pvt.day = 26;
    pvt.fixType = 3;
    pvt.numSv = 14;
    pvt.lon = -73123456;
    pvt.lat = 401234567;
    pvt.headingVehicle = 987654;
    return pvt;
}

NavRelPosNedFrame GSampleRpn()
{
    NavRelPosNedFrame rpn{};
    rpn.itow = 987654321u;
    rpn.refStationId = 42;
    rpn.relPosNorth = 1000;
    rpn.relPosEast = -2000;
    rpn.relPosLength = 3000;
    return rpn;
}

// A MON-VER payload the shape of a bench F9P's reply (HPG 1.51, ZED-F9P): the 40-byte fixed
// block, then one null-padded 30-byte block per extension.
std::vector<uint8_t> GMonVerPayload(const std::vector<std::string>& extensions)
{
    std::vector<uint8_t> payload(kUbxMonVerFixedFieldsLength, 0);
    const std::string sw = "EXT CORE 1.00";
    const std::string hw = "00190000";
    std::memcpy(payload.data(), sw.data(), sw.size());
    std::memcpy(payload.data() + 30, hw.data(), hw.size());
    for (const std::string& extension : extensions)
    {
        std::vector<uint8_t> block(kUbxMonVerExtensionLength, 0);
        std::memcpy(block.data(), extension.data(), std::min(extension.size(), size_t(kUbxMonVerExtensionLength)));
        payload.insert(payload.end(), block.begin(), block.end());
    }
    return payload;
}
}

test(UbxGnssParser, beforeAnyFrame_headersAreFilledAndPayloadIsZero)
{
    const UbxGnssParser parser;
    assertEqual(parser.GetNavPvt().preamble1, kUbxPreamble1);
    assertEqual(parser.GetNavPvt().id, kUbxNavPvtId);
    assertEqual(parser.GetNavPvt().length, kPvtBodyLength);
    assertEqual(parser.GetNavPvt().fixType, static_cast<uint8_t>(0));
    assertEqual(parser.GetNavRelPosNed().id, kUbxNavRpnId);
    assertEqual(parser.GetNavRelPosNed().length, kRpnBodyLength);
    assertEqual(parser.GetFirmwareVersion()[0], '\0');
}

test(UbxGnssParser, validPvtFrame_decodesWithItsChecksum)
{
    UbxGnssParser parser;
    const NavPvtFrame sample = GSamplePvt();
    const std::vector<uint8_t> frame = GBuildFrame(kUbxNavClass, kUbxNavPvtId, sample.raw, kPvtBodyLength);
    const std::vector<UbxMessage> completed = GFeed(parser, frame);

    assertEqual(completed.size(), size_t(1));
    assertTrue(completed[0] == UbxMessage::NavPvt);
    assertEqual(parser.GetNavPvt().lat, sample.lat);
    assertEqual(parser.GetNavPvt().lon, sample.lon);
    assertEqual(parser.GetNavPvt().numSv, sample.numSv);
    assertEqual(parser.GetNavPvt().checksumA, frame[frame.size() - 2]);
    assertEqual(parser.GetNavPvt().checksumB, frame[frame.size() - 1]);
}

test(UbxGnssParser, validRelPosNedFrame_decodes)
{
    UbxGnssParser parser;
    const NavRelPosNedFrame sample = GSampleRpn();
    const std::vector<UbxMessage> completed =
        GFeed(parser, GBuildFrame(kUbxNavClass, kUbxNavRpnId, sample.raw, kRpnBodyLength));
    assertEqual(completed.size(), size_t(1));
    assertTrue(completed[0] == UbxMessage::NavRelPosNed);
    assertEqual(parser.GetNavRelPosNed().relPosEast, sample.relPosEast);
    assertEqual(parser.GetNavRelPosNed().refStationId, sample.refStationId);
}

test(UbxGnssParser, concatenatedPvtAndRpn_bothDecodeInOrder)
{
    UbxGnssParser parser;
    const NavPvtFrame pvt = GSamplePvt();
    const NavRelPosNedFrame rpn = GSampleRpn();
    std::vector<uint8_t> bytes = GBuildFrame(kUbxNavClass, kUbxNavPvtId, pvt.raw, kPvtBodyLength);
    const std::vector<uint8_t> second = GBuildFrame(kUbxNavClass, kUbxNavRpnId, rpn.raw, kRpnBodyLength);
    bytes.insert(bytes.end(), second.begin(), second.end());

    const std::vector<UbxMessage> completed = GFeed(parser, bytes);
    assertEqual(completed.size(), size_t(2));
    assertTrue(completed[0] == UbxMessage::NavPvt);
    assertTrue(completed[1] == UbxMessage::NavRelPosNed);
}

// Garbage, including a lone 0xB5 and a 0xB5 0xB5 pair, before a real frame.
test(UbxGnssParser, garbageBeforeFrame_resyncs)
{
    UbxGnssParser parser;
    const NavPvtFrame sample = GSamplePvt();
    std::vector<uint8_t> bytes = {0x00, 0xB5, 0x13, 0xFF, 0xB5, 0xB5};
    const std::vector<uint8_t> frame = GBuildFrame(kUbxNavClass, kUbxNavPvtId, sample.raw, kPvtBodyLength);
    bytes.insert(bytes.end(), frame.begin() + 1, frame.end());   // the frame's own 0xB5 is the last garbage byte

    const std::vector<UbxMessage> completed = GFeed(parser, bytes);
    assertEqual(completed.size(), size_t(1));
    assertEqual(parser.GetNavPvt().lat, sample.lat);
}

test(UbxGnssParser, corruptedChecksum_rejectedAndLastGoodKept)
{
    UbxGnssParser parser;
    const NavPvtFrame good = GSamplePvt();
    GFeed(parser, GBuildFrame(kUbxNavClass, kUbxNavPvtId, good.raw, kPvtBodyLength));

    NavPvtFrame other = GSamplePvt();
    other.lat = 1;
    std::vector<uint8_t> bad = GBuildFrame(kUbxNavClass, kUbxNavPvtId, other.raw, kPvtBodyLength);
    bad[bad.size() - 1] ^= 0xFF;

    assertEqual(GFeed(parser, bad).size(), size_t(0));
    assertEqual(parser.GetNavPvt().lat, good.lat);
    assertEqual(parser.GetChecksumErrors(), static_cast<uint32_t>(1));
}

test(UbxGnssParser, truncatedFrame_doesNotDecode)
{
    UbxGnssParser parser;
    const NavPvtFrame sample = GSamplePvt();
    std::vector<uint8_t> frame = GBuildFrame(kUbxNavClass, kUbxNavPvtId, sample.raw, kPvtBodyLength);
    frame.resize(frame.size() - 10);
    assertEqual(GFeed(parser, frame).size(), size_t(0));
    assertEqual(parser.GetNavPvt().lat, 0);
}

// An unknown class/id, or a known one at the wrong length, resyncs at the next preamble
// instead of reading a payload into a buffer it does not fit.
test(UbxGnssParser, unrecognizedClassIdOrLength_resyncsWithoutCorruptingState)
{
    UbxGnssParser parser;
    const uint8_t junk[4] = {1, 2, 3, 4};
    std::vector<uint8_t> bytes = GBuildFrame(0x02, 0x15, junk, sizeof(junk));            // RXM-RAWX
    const std::vector<uint8_t> wrongLength = GBuildFrame(kUbxNavClass, kUbxNavPvtId, junk, sizeof(junk));
    bytes.insert(bytes.end(), wrongLength.begin(), wrongLength.end());
    const NavPvtFrame sample = GSamplePvt();
    const std::vector<uint8_t> good = GBuildFrame(kUbxNavClass, kUbxNavPvtId, sample.raw, kPvtBodyLength);
    bytes.insert(bytes.end(), good.begin(), good.end());

    const std::vector<UbxMessage> completed = GFeed(parser, bytes);
    assertEqual(completed.size(), size_t(1));
    assertTrue(completed[0] == UbxMessage::NavPvt);
    assertEqual(parser.GetNavPvt().lat, sample.lat);
}

test(UbxGnssParser, ackAndNak_decodeWithTheAnsweredClassAndId)
{
    UbxGnssParser parser;
    const uint8_t answered[2] = {kUbxCfgClass, kUbxCfgValSetId};

    assertTrue(GFeed(parser, GBuildFrame(kUbxAckClass, kUbxAckAckId, answered, 2))[0] == UbxMessage::Ack);
    assertTrue(parser.GetAckIsAck());
    assertEqual(parser.GetAckedClass(), kUbxCfgClass);
    assertEqual(parser.GetAckedId(), kUbxCfgValSetId);

    assertTrue(GFeed(parser, GBuildFrame(kUbxAckClass, kUbxAckNakId, answered, 2))[0] == UbxMessage::Ack);
    assertFalse(parser.GetAckIsAck());
}

// FWVER= is the second extension on a real F9P, so the parser must scan, not take the first.
test(UbxGnssParser, monVer_extractsTheFwverExtension)
{
    UbxGnssParser parser;
    const std::vector<uint8_t> payload = GMonVerPayload(
        {"ROM BASE 0x118B2060", "FWVER=HPG 1.51", "PROTVER=27.50", "MOD=ZED-F9P", "GPS;GLO;GAL;BDS", "SBAS;QZSS"});
    const std::vector<UbxMessage> completed =
        GFeed(parser, GBuildFrame(kUbxMonClass, kUbxMonVerId, payload.data(), static_cast<uint16_t>(payload.size())));
    assertEqual(completed.size(), size_t(1));
    assertTrue(completed[0] == UbxMessage::MonVer);
    assertEqual(parser.GetFirmwareVersion(), "HPG 1.51");
}

test(UbxGnssParser, monVerWithoutFwver_leavesTheVersionEmpty)
{
    UbxGnssParser parser;
    const std::vector<uint8_t> payload = GMonVerPayload({"MOD=ZED-F9P"});
    GFeed(parser, GBuildFrame(kUbxMonClass, kUbxMonVerId, payload.data(), static_cast<uint16_t>(payload.size())));
    assertEqual(parser.GetFirmwareVersion(), "");
}

// "FWVER=" plus 24 characters fills the 30-byte field with no terminator; the copy is
// bounded by the field and truncated to the 23 characters the buffer holds.
test(UbxGnssParser, fwverFillingItsField_isBoundedAndTruncated)
{
    UbxGnssParser parser;
    const std::vector<uint8_t> payload = GMonVerPayload({"FWVER=" + std::string(24, 'X')});
    GFeed(parser, GBuildFrame(kUbxMonClass, kUbxMonVerId, payload.data(), static_cast<uint16_t>(payload.size())));
    assertEqual(parser.GetFirmwareVersion(), std::string(23, 'X').c_str());
}

test(UbxGnssParser, oversizedMonVer_isRejectedAndTheParserResyncs)
{
    UbxGnssParser parser;
    const std::vector<uint8_t> oversized(kUbxMonVerMaxPayloadLength + 1, 0);
    std::vector<uint8_t> bytes =
        GBuildFrame(kUbxMonClass, kUbxMonVerId, oversized.data(), static_cast<uint16_t>(oversized.size()));
    const NavPvtFrame sample = GSamplePvt();
    const std::vector<uint8_t> good = GBuildFrame(kUbxNavClass, kUbxNavPvtId, sample.raw, kPvtBodyLength);
    bytes.insert(bytes.end(), good.begin(), good.end());

    const std::vector<UbxMessage> completed = GFeed(parser, bytes);
    assertEqual(completed.size(), size_t(1));
    assertTrue(completed[0] == UbxMessage::NavPvt);
}

test(UbxGnssParser, resetMidFrame_dropsItAndKeepsTheLastGood)
{
    UbxGnssParser parser;
    const NavPvtFrame first = GSamplePvt();
    GFeed(parser, GBuildFrame(kUbxNavClass, kUbxNavPvtId, first.raw, kPvtBodyLength));

    NavPvtFrame second = GSamplePvt();
    second.lat = 7;
    const std::vector<uint8_t> frame = GBuildFrame(kUbxNavClass, kUbxNavPvtId, second.raw, kPvtBodyLength);
    GFeed(parser, std::vector<uint8_t>(frame.begin(), frame.begin() + 40));
    parser.Reset();
    GFeed(parser, std::vector<uint8_t>(frame.begin() + 40, frame.end()));
    assertEqual(parser.GetNavPvt().lat, first.lat);

    GFeed(parser, frame);
    assertEqual(parser.GetNavPvt().lat, second.lat);
}

test(UbxGnssParser, monVerPoll_isTheEmptyRequestWithItsChecksum)
{
    const std::vector<uint8_t> expected = GBuildFrame(kUbxMonClass, kUbxMonVerId, nullptr, 0);
    assertEqual(expected.size(), sizeof(kUbxMonVerPoll));
    for (size_t i = 0; i < expected.size(); ++i)
    {
        assertEqual(kUbxMonVerPoll[i], expected[i]);
    }
}
