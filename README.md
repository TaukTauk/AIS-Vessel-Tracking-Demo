# AIS Vessel Tracking Demo

A self-contained C program that demonstrates the full AIS (Automatic
Identification System) stack — encoding, decoding, vessel state tracking,
reporting rate compliance, and interactive visual output.

It simulates realistic vessel traffic in the Gulf of Thailand and replays it
as a showcase of how AIS works at the protocol level.

**Not** a real-time tracker. **Not** a production system.  
A teaching and showcase tool that compiles and runs with two commands.

---

## Quick Start

```bash
make          # build (requires GCC with C11 and libm — no other dependencies)
make run      # generate sample.nmea, replay it, print terminal table, write map.html
open map.html # view the interactive vessel chart in a browser
make test     # run the full test suite (~185 tests)
make clean    # remove build artefacts and generated files
```

---

## What It Demonstrates

| Concept | Where |
|---|---|
| AIS 6-bit payload encoding | `src/ais.c` — `encode_6bit`, `bits_to_payload` |
| Type 1 position report encode/decode | `src/ais.c` — `ais_encode_msg1`, `ais_parse_nmea` |
| Type 5 static data encode/decode | `src/ais.c` — `ais_encode_msg5`, `ais_parse_nmea5` |
| Two-fragment NMEA reassembly | `src/ais.c` — static `FragSlot` buffer |
| IMO reporting rate compliance | `src/ais.c` — `ais_reporting_interval` |
| Rhumb-line position propagation | `src/ais.c` — `propagate` |
| Vessel state table + EMA intervals | `src/vessel.c` |
| Circular position history | `src/vessel.c` — `history[]`, `history_head` |
| ANSI terminal table | `src/output.c` — `output_terminal` |
| Leaflet.js interactive map | `src/output.c` — `output_html` |

---

## What the Map Shows

- Six simulated vessels in the Gulf of Thailand
- Each vessel rendered as a **ship silhouette** (pointed bow, notched stern) rotated to its course
- Colour-coded by ship type: cargo (orange), passenger (blue), tanker (red), other (grey)
- Dashed trail lines showing position history
- Click any vessel for a popup: MMSI, SOG, COG, heading, nav status, destination, ETA, and reporting rate compliance

---

## Standards

| Document | What it governs |
|---|---|
| ITU-R M.1371-6 | Complete protocol spec — all bit layouts, 6-bit codec |
| IMO Resolution MSC.74(69) Annex 3 | Reporting rate Table 1 |
| IMO Resolution A.1106(29) | Updated reporting rate Table 2 |
| IMO SN/Circ.227 | Ship type codes, sensor interfaces |

Developer reference: [AIVDM/AIVDO Protocol Decoding](https://gpsd.gitlab.io/gpsd/AIVDM.html) by Eric S. Raymond.

---

## Project Layout

```
ais_demo/
├── src/
│   ├── error.h / ais.h / ais.c      ← codec, encode/decode, error handling
│   ├── vessel.h / vessel.c           ← state table, EMA, circular history
│   ├── output.h / output.c           ← terminal + HTML map
│   └── main.c                        ← simulation + replay orchestration
├── tests/
│   ├── testlib.h                     ← minimal assert macros
│   └── test_all.c                    ← 185 tests across 16 suites (A–P)
├── docs/                             ← architecture and algorithm documents
├── objs/                             ← object files (generated)
├── Makefile
├── CLAUDE.md                         ← detailed developer guide
└── README.md                         ← this file
```

---

## Phase Roadmap

| Phase | Description | Status |
|---|---|---|
| 1 | Encode/decode Type 1, vessel tracking, terminal + HTML output | ✅ Complete |
| 1b | Clean implementation — error handling, validity flags, circular history | ✅ Complete |
| 2 | Message Type 5 — ship name, IMO number, destination, two-fragment reassembly | ✅ Complete |
| 3 | Live input — stdin / RTL-SDR feed | ⬜ Planned |
| 4 | Animated HTML map with playhead | ⬜ Planned |
