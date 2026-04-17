# AIS Vessel Tracking Demo — Planning Document

**Project:** `ais_demo`  
**Language:** C (C11, GCC)  
**Scope:** Showcase demo — Gulf of Thailand simulation  
**Standards:** IMO MSC.74(69), SN/Circ.227, ITU-R M.1371  

---

## 1. Purpose

This project is a self-contained demonstration of how the **Automatic Identification System (AIS)** works at the protocol level. It is not a real-time tracker — it simulates realistic vessel traffic and replays it to showcase every layer of the AIS stack: encoding, decoding, vessel state management, reporting rate compliance, and visual output.

The goal is to serve as a **teaching and showcase tool** that a developer or maritime engineer can run with a single `make run` command, and immediately understand how AIS data flows from a ship's transceiver to a display system.

---

## 2. What AIS Is (and Why It Matters)

AIS is the mandatory vessel identification system for ships over 300 GT on international voyages (SOLAS V/19). Every compliant vessel broadcasts:

- **Who it is** — MMSI, name, IMO number, call sign
- **Where it is** — GPS lat/lon in WGS-84 datum
- **How it's moving** — speed over ground, course over ground, heading, rate of turn
- **What it's doing** — navigational status (underway, anchored, fishing, etc.)

These broadcasts happen over VHF radio (161.975 MHz and 162.025 MHz) using a self-organized time-division multiple access scheme called **(S)TDMA**, allowing thousands of vessels to share two channels without central coordination.

---

## 3. Core Concepts Demonstrated

### 3.1 NMEA 0183 Sentence Structure

Every AIS message arrives wrapped in an NMEA 0183 sentence. The format is:

```
!AIVDM,<total>,<num>,<seq_id>,<channel>,<payload>,<fill_bits>*<checksum>
```

| Field | Example | Meaning |
|---|---|---|
| Sentence type | `!AIVDM` | AIS message received (VDM = VHF Data Message) |
| Total fragments | `1` | How many sentences this message is split into |
| Fragment number | `1` | Which fragment this is |
| Sequential ID | *(empty)* | ID for multi-fragment reassembly (can be blank) |
| Channel | `A` | VHF channel A (161.975 MHz) or B (162.025 MHz) |
| Payload | `15M67N0000...` | AIS data encoded as 6-bit ASCII |
| Fill bits | `0` | Padding bits at the end of the last fragment |
| Checksum | `*73` | XOR of all bytes between `!` and `*` |

**Problem this demo solves:** The empty sequential ID field (two consecutive commas `,,`) breaks `strtok`-based parsers that silently skip empty tokens. The demo uses a manual field splitter that preserves empty fields.

### 3.2 AIS 6-bit ASCII Payload Encoding

The AIS payload is not plain text. Each character encodes 6 bits:

```
decoded_value = ASCII_value - 48
if decoded_value >= 40: decoded_value -= 8
```

This gives a value 0–63 per character. A 28-character payload carries 168 bits.

**How we solve it:** The encoder and decoder work on a raw `uint8_t bits[168]` array. Helper functions `get_uint()`, `get_int()`, `set_uint()`, `set_int()` extract and insert fields at arbitrary bit positions and widths. The 6-bit codec converts between this bit array and the payload string.

### 3.3 Message Type 1 — Position Report Class A

The most common AIS message. Bit layout (168 bits / 28 chars):

| Field | Bits | Type | Unit | Notes |
|---|---|---|---|---|
| Message type | 0–5 | uint | — | Must be 1, 2, or 3 |
| MMSI | 8–37 | uint | — | 9-digit vessel identifier |
| Nav status | 38–41 | uint | — | 0=underway, 1=anchor, 7=fishing... |
| Rate of turn | 42–49 | int | °/min | –128 = not available |
| Speed over ground | 50–59 | uint | 1/10 knot | 1023 = not available |
| Longitude | 61–88 | int signed | 1/10000 min | 181° = not available |
| Latitude | 89–115 | int signed | 1/10000 min | 91° = not available |
| Course over ground | 116–127 | uint | 1/10 degree | 3600 = not available |
| True heading | 128–136 | uint | degrees | 511 = not available |
| UTC timestamp | 137–142 | uint | seconds | Second within the minute |

**Coordinate encoding:** Latitude and longitude are stored as integer units of 1/10000 of a minute of arc. To convert:

```
lat_raw = (int32_t)(latitude_degrees × 600000)
lon_raw = (int32_t)(longitude_degrees × 600000)
```

A 27-bit signed integer covers ±90° (max value ≈ 54,000,000). A 28-bit signed covers ±180°.

### 3.4 Reporting Rate (IMO MSC.74(69) Annex 3, Table 1)

AIS Class A equipment must transmit at intervals determined by the vessel's speed and manoeuvring state:

| Condition | Interval |
|---|---|
| Ship at anchor or moored | 3 minutes |
| 0–14 knots | 12 seconds |
| 0–14 knots, changing course | 4 seconds |
| 14–23 knots | 6 seconds |
| 14–23 knots, changing course | 2 seconds |
| > 23 knots | 3 seconds |
| > 23 knots, changing course | 2 seconds |

**Why this matters:** A coast guard VTS station receiving AIS must know how stale any given position fix is. A fast cargo ship at 24 knots changes position by ~12 metres every second — a 3-second interval keeps the picture current. A vessel at anchor barely moves; 3-minute intervals are sufficient.

**How the demo uses this:** Six vessels are configured at different speeds. The simulation generates exactly the right number of NMEA sentences per vessel over 10 minutes. The replay then measures each vessel's observed average interval and compares it to the expected value, reporting compliance.

### 3.5 Vessel State Tracking

A vessel tracker is fundamentally a **key-value store keyed by MMSI** where each record holds the latest known state plus metadata about when and how often it was updated.

**Data held per vessel:**

- Latest position (lat, lon), SOG, COG, heading, nav status
- First seen / last seen timestamps
- Total update count
- Rolling average interval (exponential moving average: `avg = avg × 0.8 + gap × 0.2`)
- Position history trail (up to 200 fixes) for map rendering
- Expected interval (from IMO table) for compliance checking

### 3.6 Position Propagation (Simulation Only)

To generate realistic movement, each vessel's position is advanced between updates using the rhumb line approximation:

```
distance_nm = sog_knots × (interval_seconds / 3600)
Δlat = distance_nm × cos(COG) / 60          [degrees]
Δlon = distance_nm × sin(COG) / (60 × cos(lat))  [degrees]
```

This is a flat-Earth approximation valid for short distances (< 100 nm). For the Gulf of Thailand scenario (vessels moving at most ~4 nm per update cycle), it is accurate to within a few metres.

---

## 4. Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        main.c                           │
│                                                         │
│  [1] generate_sample_log()  → sample.nmea               │
│      6 vessels × correct IMO intervals × 10 minutes     │
│                                                         │
│  [2] replay_log()           → VesselTable               │
│      Parse T= timestamp prefix, decode NMEA, update     │
│                                                         │
│  [3] output_terminal()      → stdout (ANSI table)        │
│  [4] output_html()          → map.html (Leaflet map)     │
└────────┬──────────────┬──────────────┬───────────────────┘
         │              │              │
    ┌────▼────┐   ┌──────▼─────┐  ┌───▼────────┐
    │  ais.c  │   │  vessel.c  │  │  output.c  │
    │         │   │            │  │            │
    │ encode  │   │ find/create│  │ ANSI table │
    │ decode  │   │ update     │  │ HTML/Leaflet│
    │ cs check│   │ history    │  │            │
    │ interval│   │ compliance │  │            │
    └─────────┘   └────────────┘  └────────────┘
```

### File Responsibilities

| File | Responsibility |
|---|---|
| `ais.h / ais.c` | NMEA checksum, 6-bit codec, bit array helpers, Type 1 encoder/decoder, IMO interval table |
| `vessel.h / vessel.c` | Vessel state struct, MMSI-keyed lookup, state update, rolling interval calculation, position history |
| `output.h / output.c` | ANSI terminal table with compliance colour coding; Leaflet.js HTML map with vessel markers, trails, popup data |
| `main.c` | Sample data generation (with T= timestamps), log replay, orchestration |

---

## 5. Sample Data Design

Six vessels are chosen to represent distinct cases in the IMO reporting rate table:

| Vessel | MMSI | Type | SOG | Expected Interval | Role in demo |
|---|---|---|---|---|---|
| THAI STAR 1 | 567001001 | Cargo (71) | 8.5 kts | 12 s | Mid-speed, heading NE |
| PATTAYA EXPRESS | 567001002 | Passenger (61) | 18.0 kts | 6 s | Fast ferry, heading S |
| GULF PIONEER | 567001003 | Tanker (81) | 5.5 kts | 12 s | Slow tanker, heading W |
| OCEAN HARVEST | 567001004 | Fishing (30) | 3.2 kts | 12 s | Nav status = fishing |
| JAKARTA MARU | 525001001 | Cargo (71) | 24.5 kts | 3 s | Fastest — most messages |
| SEA ANCHOR | 566002001 | Tanker (81) | 0.0 kts | 180 s | Nav status = at anchor |

MMSI prefixes follow real-world MID (Maritime Identification Digits) allocations:
- `567` — Thailand
- `525` — Indonesia
- `566` — Singapore

Positions are within the Gulf of Thailand (10–13°N, 100–102°E), a realistic maritime operational area.

The simulation runs for **600 seconds (10 minutes)**. Total messages generated: ~459. This is enough to produce meaningful interval statistics for every vessel while keeping the sample file small.

---

## 6. Output Design

### 6.1 Terminal Table

Two-section ANSI table rendered with UTF-8 box-drawing characters:

**Section 1 — Current Vessel State**
- MMSI, vessel name, last known lat/lon, SOG (colour-coded by speed band), COG, navigational status

**Section 2 — Reporting Rate Compliance**
- MMSI, expected interval (from IMO table), observed average interval, message count
- Colour: GREEN = compliant, YELLOW = minor deviation (±20%), RED = non-compliant, DIM = insufficient data

### 6.2 HTML Map (Leaflet.js)

- Dark-themed layout with OpenStreetMap tile layer
- Each vessel rendered as a **rotated arrow (▲)** pointing along its COG, colour-coded by ship type
- **Dashed polyline trail** showing position history
- **Popup** on click: vessel name, MMSI, type, position, SOG/COG, nav status, update count, expected interval
- **Reporting rate panel** below the map (full table, same compliance logic as terminal)

---

## 7. Standards Compliance

| Requirement | Source | How Addressed |
|---|---|---|
| AIS payload bit layout | ITU-R M.1371 | Exact bit offsets in encoder/decoder |
| 6-bit ASCII codec | ITU-R M.1371 | `encode_6bit()` / `decode_6bit()` |
| WGS-84 coordinate datum | MSC.74(69) §3.1 | `lat × 600000`, `lon × 600000` as signed integers |
| Reporting rate table | MSC.74(69) Annex 3, Table 1 | `ais_reporting_interval()` in ais.c |
| Nav status values 0–15 | ITU-R M.1371 Table 18 | `ais_nav_status_str[]` array |
| Ship type coding | ITU-R M.1371 Table 18 | `ship_type_name()` maps first digit |
| NMEA sentence checksum | NMEA 0183 §6.3 | XOR of bytes between `!` and `*` |
| MMSI MID allocation | ITU Radio Regulations | Thai (567), Indonesian (525), Singapore (566) MMSIs used |

---

## 8. Known Limitations of This Demo

| Limitation | Reason | Fix for Production |
|---|---|---|
| Message Types 1/2/3 only | Sufficient for position tracking | Add Type 5 (static/voyage data: ship name, IMO number, destination) |
| Single-fragment messages only | Simplifies demo | Add fragment reassembly buffer keyed on (MMSI, seq_id) |
| No real RF input | Demo is file-based | Pipe from `rtl_ais` or a TCP NMEA feed via stdin |
| No course-change detection | Interval shown is non-maneuvering | Track heading delta over time; halve interval when > 5°/30s |
| Flat-Earth position propagation | Acceptable for < 100 nm | Use haversine or Vincenty for long-range accuracy |
| Fixed vessel list | Hardcoded in main.c | Load vessel catalogue from CSV or database |

---

## 9. Planned Extensions

### Phase 2 — Message Type 5 (Static & Voyage Data)

Type 5 carries the ship name, IMO number, call sign, ship type, dimensions, destination, and ETA. It is 426 bits across **two** NMEA fragments — this makes it the natural first target for multi-fragment reassembly.

What changes:
- `ais.h`: Add `AISMsg5` struct
- `ais.c`: Add Type 5 decoder + fragment reassembly buffer
- `vessel.c`: Populate `name`, `imo_number`, `call_sign`, `destination`, `eta` fields
- `output.c`: Show destination and ETA in HTML popup

### Phase 3 — Live Input Mode

Replace the sample log generator with a stdin reader. The program already processes one NMEA line at a time, so the change is minimal:

```c
// Replace generate_sample_log() + fopen() with:
FILE *f = stdin;   // or fopen("/dev/ttyUSB0", "r") for serial
```

For RTL-SDR: pipe `rtl_ais` output directly:
```bash
rtl_ais -n | ./ais_demo
```

For a network AIS feed (e.g., AISHub, aisstream.io WebSocket): use a small adapter that writes NMEA lines to stdout.

### Phase 4 — Animated Map

Replace the static HTML snapshot with a time-series animation:
- Embed the full position history as a JavaScript timeline
- Use `setInterval` to advance a playhead and update marker positions
- Add play/pause/speed controls
- Show a "vessel trail" that fades as it ages

---

## 10. Build & Run

```bash
# Clone / enter project
cd ais_demo

# Build (requires gcc, libm)
make

# Run demo
./ais_demo

# Open map in browser
open map.html          # macOS
xdg-open map.html      # Linux
start map.html         # Windows

# Clean all generated files
make clean
```

**Dependencies:** GCC (or any C11 compiler), `libm` (standard on all platforms). No external libraries required. The HTML map loads Leaflet.js from a CDN at runtime.

---

## 11. References

| Document | Title |
|---|---|
| IMO Resolution MSC.74(69) Annex 3 | Recommendation on Performance Standards for Universal Shipborne AIS |
| IMO SN/Circ.227 | Guidelines for the Installation of a Shipborne AIS |
| IMO Resolution A.1106(29) | Revised Guidelines for the Onboard Operational Use of Shipborne AIS |
| ITU-R M.1371-5 | Technical Characteristics for an AIS Using TDMA in the VHF Maritime Mobile Band |
| NMEA 0183 Standard | Interface Standard for Marine Electronic Devices |
| IEC 61993-2 | AIS Class A — Operational and Performance Requirements |
