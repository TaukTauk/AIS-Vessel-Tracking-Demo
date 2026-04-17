# CLAUDE.md — AIS Vessel Tracking Demo

This file is the single entry point for understanding this project.
Read it before touching any code or document.

---

## What This Project Is

A self-contained C program that demonstrates the full AIS (Automatic
Identification System) stack — encoding, decoding, vessel state
tracking, reporting rate compliance, and visual output. It simulates
realistic vessel traffic in the Gulf of Thailand and replays it as a
showcase of how AIS works at the protocol level.

**Not** a real-time tracker. **Not** a production system.
A teaching and showcase tool that compiles and runs with `make run`.

---

## Quick Start

```bash
cd ais_demo
make          # build
./ais_demo    # run — generates sample.nmea and map.html
open map.html # view vessel chart in browser
make test     # run all ~90 unit tests
make clean    # remove generated files
```

**Requirements:** GCC (C11), libm. No external libraries.

---

## Project Status

| Phase | Description | Status |
|---|---|---|
| Phase 1 | Encode/decode Type 1, vessel tracking, terminal + HTML output | ✅ Complete |
| Phase 1b | Clean implementation — error handling, validity flags, circular history | ✅ Complete (155 tests pass) |
| Phase 2 | Message Type 5 — ship name, IMO number, destination | ✅ Complete (two-fragment decode, destination + ETA in map popup) |
| Phase 3 | Live input — stdin / RTL-SDR feed | ⬜ Planned |
| Phase 4 | Animated HTML map with playhead | ⬜ Planned |

---

## Repository Layout

```
ais_demo/
├── CLAUDE.md              ← you are here
│
├── src/                   ← source code
│   ├── error.h            ← AisStatus enum (incl. AIS_FRAG_PENDING) + AisErrorCtx
│   ├── ais.h / ais.c      ← NMEA parsing, AIS encode/decode (Type 1 + Type 5)
│   ├── vessel.h / vessel.c ← vessel state table, EMA, circular history
│   ├── output.h / output.c ← terminal table + Leaflet HTML map
│   └── main.c             ← sample generation + replay
│
├── objs/                  ← compiled object files (generated, gitignored)
│
├── tests/                 ← unit tests
│   ├── testlib.h          ← minimal test macro framework
│   └── test_all.c         ← 155 tests across 14 suites (A–N)
│
├── docs/                  ← project documentation
│   ├── PLANNING.md        ← architecture, scope, sample data design
│   ├── DATA_STRUCTURES.md ← every struct, every field, and why
│   ├── ALGORITHMS.md      ← every calculation with derivations
│   ├── ERROR_HANDLING.md  ← error strategy, AisStatus enum, decision table
│   ├── TEST_PLAN.md       ← 90-test plan, one suite per algorithm
│   └── REFERENCE.md      ← all standards and resources with descriptions
│
├── Makefile
├── sample.nmea            ← generated at runtime (gitignored)
└── map.html               ← generated at runtime (gitignored)
```

---

## Standards Compliance

This project is built against three IMO documents and one ITU standard:

| Document | What it governs |
|---|---|
| IMO Resolution MSC.74(69) Annex 3 | AIS performance requirements, original reporting rate Table 1 |
| IMO Resolution A.1106(29) | Operational guidelines, updated reporting rate Table 2 |
| IMO SN/Circ.227 | Installation guidelines, sensor interfaces, ship type codes |
| ITU-R M.1371-6 | Complete protocol spec — all bit layouts, 6-bit codec, TDMA |

The AIVDM/AIVDO decoding guide by Eric S. Raymond
(https://gpsd.gitlab.io/gpsd/AIVDM.html) is the practical
developer reference used alongside the official standards.

---

## Architecture in One Page

```
ENCODE PATH (simulation → file)
  SampleVessel data
    → IMO reporting rate lookup  [ais.c: ais_reporting_interval()]
    → Next-TX scheduler          [main.c: next_tx[] array]
    → Rhumb line propagation     [main.c: propagate()]
    → Unit conversion + rounding [ais.c: × 600000, × 10 + 0.5]
    → Bit array packing          [ais.c: set_uint / set_int]
    → 6-bit ASCII encode         [ais.c: bits_to_payload()]
    → NMEA checksum + format     [ais.c: nmea_cs()]
    → sample.nmea

DECODE PATH — Type 1/2/3 (single fragment)
  sample.nmea
    → NMEA checksum validate     [ais.c: nmea_valid()]
    → Field splitter             [ais.c: split_fields()]  ← never strtok
    → 6-bit ASCII decode         [ais.c: payload_to_bits()]
    → Bit array extraction       [ais.c: get_uint / get_int]
    → Sign extension             [ais.c: get_int()]
    → Unit conversion            [ais.c: / 600000, / 10]
    → Sentinel detection         [ais.c: pos_valid, sog_valid, ...]
    → AISMsg1 struct
    → Vessel lookup/create       [vessel.c: vessel_find_or_create()]
    → EMA interval update        [vessel.c: α=0.2 moving average]
    → Circular history append    [vessel.c: history[] circular buffer]
    → Compliance classify        [vessel.c: ratio vs expected interval]
    → Vessel struct

DECODE PATH — Type 5 (two fragments)
  sample.nmea (fragment 1)
    → NMEA checksum validate     [ais.c: nmea_valid()]
    → Fragment buffer store      [ais.c: s_frags[], keyed on seq_id]
    → returns AIS_FRAG_PENDING

  sample.nmea (fragment 2)
    → NMEA checksum validate     [ais.c: nmea_valid()]
    → Fragment buffer lookup     [ais.c: s_frags[] by seq_id]
    → Concatenate payloads       [ais.c: combined[]]
    → 6-bit text decode          [ais.c: decode_text() for name/callsign/dest]
    → Bit field extraction       [ais.c: get_uint() for IMO/dims/ETA/draught]
    → AISMsg5 struct
    → Vessel static update       [vessel.c: vessel_update_static()]
    → Vessel struct (name, IMO, destination, ETA, dimensions)

  Vessel table
    → Terminal ANSI table        [output.c: output_terminal()]
    → Leaflet HTML map           [output.c: output_html()]
```

---

## Key Design Decisions

**1. One `uint8_t` per bit in the bit array** — not packed bytes.
Eliminates all cross-byte boundary arithmetic. Each `bits[61]` maps
directly to bit 61 in the ITU-R M.1371 table. Costs 8× memory but
the payload is only 168 bytes; the clarity benefit is overwhelming.
See ALGORITHMS.md §3.

**2. `AisStatus` enum return codes everywhere.**
No function returns a raw `int` with ad-hoc 0/1 meaning. No
`exit()` inside library functions (`ais.c`, `vessel.c`). Only
`main.c` terminates the program. See ERROR_HANDLING.md §3.

**3. Sentinel values are information, not errors.**
Latitude 91°, heading 511, SOG 1023 — these are valid AIS protocol
states. The decoder sets validity flags (`pos_valid`, `sog_valid`,
`heading_valid`) rather than returning an error. Output functions
check flags before rendering. See ERROR_HANDLING.md §8.

**4. `strtok` is banned.**
The NMEA sequential message ID field (field [3]) is legitimately
empty for single-fragment messages, producing two consecutive commas
`,,`. `strtok` silently skips empty fields, causing field [4] to be
read as field [3], payload as channel, etc. Use `split_fields()`
everywhere. See ERROR_HANDLING.md §6.

**5. Circular history buffer.**
The prototype stopped recording when the history array was full.
The clean implementation wraps around, overwriting the oldest fix.
Two indices: `history_len` (total fixes, capped at `MAX_HISTORY`)
and `history_head` (oldest fix index when full).
See ERROR_HANDLING.md §9b.

**7. Type 5 two-fragment reassembly with a static slot buffer.**
Type 5 messages are 424 bits — too long for one NMEA sentence. They arrive
as two `!AIVDM,2,1,<seq_id>,...` / `!AIVDM,2,2,<seq_id>,...` pairs.
A fixed 8-slot `FragSlot` buffer in `ais.c` stores fragment 1 keyed on
`seq_id` and returns `AIS_FRAG_PENDING`. When fragment 2 arrives, the slot
is retrieved, payloads concatenated, and the 424-bit message decoded.
See `ais_parse_nmea5()`.

**6. EMA with α=0.2 for interval tracking.**
Adapts when a vessel changes speed. O(1) memory. "Effective memory"
of ~5 samples. Seeded with the first gap directly.
See ALGORITHMS.md §9.

---

## Document Reading Order

For a new developer or for returning context:

1. **CLAUDE.md** (this file) — project overview in one page
2. **docs/PLANNING.md** — architecture, sample data, standards context
3. **docs/DATA_STRUCTURES.md** — every struct and why each field exists
4. **docs/ALGORITHMS.md** — every calculation with derivations
5. **docs/ERROR_HANDLING.md** — error strategy before writing any code
6. **docs/TEST_PLAN.md** — 90 tests, one per algorithm
7. **docs/REFERENCE.md** — standards and resources

---

## Glossary

| Term | Meaning |
|---|---|
| AIS | Automatic Identification System — mandatory vessel tracking system |
| MMSI | Maritime Mobile Service Identity — 9-digit vessel ID |
| MID | Maritime Identification Digits — first 3 digits of MMSI, country code |
| SOG | Speed Over Ground — actual speed over seabed (knots) |
| COG | Course Over Ground — actual direction of movement (degrees true) |
| ROT | Rate of Turn — degrees per minute, signed (+ = right, − = left) |
| NMEA | National Marine Electronics Association — defines sentence format |
| VHF | Very High Frequency — AIS transmits on 161.975 MHz and 162.025 MHz |
| TDMA | Time Division Multiple Access — how vessels share the channel |
| SOLAS | Safety of Life at Sea — IMO convention mandating AIS carriage |
| VTS | Vessel Traffic Service — shore-based equivalent of air traffic control |
| Class A | Full SOLAS-compliant AIS (cargo > 300 GT, all passenger ships) |
| Class B | Voluntary AIS for smaller vessels (lower reporting rate) |
| RAIM | Receiver Autonomous Integrity Monitoring — GPS self-check flag |
| WGS-84 | World Geodetic System 1984 — coordinate datum used by AIS |
| Rhumb line | Path of constant bearing — how autopilots navigate |
| EMA | Exponential Moving Average — rolling interval statistic |
