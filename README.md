# UbxGnss

The u-blox UBX decoding that both NeptuneGPS firmwares use to read a ZED-F9P:

- [NeptuneGPS Salacia](https://github.com/Arjan-Woltjer/NeptuneGPS_Salacia), the GNSS
  module that talks to the tablet over Ethernet;
- [NeptuneGPS Proteus](https://github.com/Arjan-Woltjer/NeptuneGPS_Proteus), the GNSS
  module that broadcasts on the ISOBUS like a John Deere StarFire.

It was Salacia's own decoder. It moved here unchanged in behaviour, so the two firmwares
share one parser instead of keeping two copies in step (Proteus
`docs/gnss-can-module-plan.md`, phase 1). Same pattern as SteeringActuator, which Salacia
and Triton's Motorcontroller share.

## What is in it

- `src/UbxGnssProtocol.hpp` — the UBX constants and the `NavPvtFrame` /
  `NavRelPosNedFrame` layouts, plus the NAV-PVT flag bits and the UBX checksum.
  - These are whole frames, header and checksum included, because Salacia's host protocol
    carries them byte for byte.
  - Their layout is pinned by `static_assert`s and must not change without Salacia's ICD
    changing with it.
- `src/UbxGnssParser` — the byte-by-byte decoder for NAV-PVT, NAV-RELPOSNED, ACK-ACK/NAK
  and MON-VER (the "FWVER=" firmware string).
  - It has no UART and no Arduino dependency: the caller reads its own port and passes
    each byte to `Feed()`.
  - Only those four messages are accepted, each at its exact length. Anything else
    resynchronises at the next preamble.
  - A frame replaces the last good one only once its checksum validates.

The port handling stays in each firmware. Salacia's config mode takes the UART over, and
Proteus also relays RTCM corrections into it.

## Using it

Add it as a git submodule at `lib/UbxGnss`. PlatformIO's library finder picks up `src/`
for board builds. A native env names it explicitly:
`-I lib/UbxGnss/src` and `+<../lib/UbxGnss/src/UbxGnssParser.cpp>`.

```cpp
#include "UbxGnssParser.hpp"

ubxgnss::UbxGnssParser parser;
while (port.available() > 0)
{
    if (parser.Feed(static_cast<uint8_t>(port.read())) == ubxgnss::UbxMessage::NavPvt)
    {
        const ubxgnss::NavPvtFrame& pvt = parser.GetNavPvt();
        // ...
    }
}
```

The repository is public so that consumer CI can fetch the submodule without a deploy
key.

## Tests

```
pio run -e native && .pio/build/native/program
```

The suite was ported from Salacia's `test_UbxGpsSource.cpp` together with the parser. The
UART-facing cases (MON-VER polling, raw tap, external owner, baud change) stay in Salacia.

## License

LGPL-3.0-or-later, see `COPYING` and `COPYING.LESSER`.
