# AIS Demo — Data Structures Reference

**Project:** `ais_demo`  
**Files covered:** `ais.h`, `vessel.h`, `main.c`

This document describes every data structure in the project — what each field stores, what type it is and why, and how the design decision connects to the AIS protocol or practical tracking requirements.

---

## Table of Contents

1. [Primitive Foundations](#1-primitive-foundations)
2. [AISMsg1 — Decoded Position Report](#2-aismsg1--decoded-position-report)
3. [VesselFix — Single Historical Position](#3-vesselfix--single-historical-position)
4. [Vessel — Full Vessel Record](#4-vessel--full-vessel-record)
5. [VesselTable — Live Vessel Register](#5-vesseltable--live-vessel-register)
6. [SampleVessel — Simulation Seed](#6-samplevessel--simulation-seed)
7. [Internal: Bit Array](#7-internal-bit-array)
8. [Design Trade-offs Summary](#8-design-trade-offs-summary)

---

## 1. Primitive Foundations

Before looking at structs, it is worth noting that this project uses fixed-width integer types (`stdint.h`) throughout rather than `int` or `long`. This matters for AIS because the protocol is defined in exact bit widths, and `int` is architecture-dependent.

| Type | Width | Why used |
|---|---|---|
| `uint8_t` | 8 bits | Flags, indices, small counters, single bytes |
| `uint16_t` | 16 bits | Heading (9-bit field; `uint8_t` would overflow) |
| `uint32_t` | 32 bits | MMSI (30-bit field), raw bit extraction |
| `int8_t` | 8 bits | Rate of turn (signed, –128 to +127) |
| `int32_t` | 32 bits | Signed lat/lon before division |
| `float` | 32 bits | Lat, lon, SOG, COG after unit conversion |
| `time_t` | platform | Wall-clock timestamps for interval calculation |

---

## 2. `AISMsg1` — Decoded Position Report

**File:** `ais.h`  
**Represents:** One fully decoded AIS Message Type 1, 2, or 3 (Position Report Class A)  
**Source:** A single `!AIVDM` NMEA sentence from the log

```c
typedef struct {
    uint8_t  msg_type;
    uint8_t  repeat;
    uint32_t mmsi;
    uint8_t  nav_status;
    int8_t   rot;
    float    sog;
    uint8_t  pos_accuracy;
    float    longitude;
    float    latitude;
    float    cog;
    uint16_t heading;
    uint8_t  timestamp;
    uint8_t  raim;
    time_t   received_at;
} AISMsg1;
```

This struct is the **output of the decoder** and the **input to the vessel tracker**. It is ephemeral — created on the stack per message, read once, then discarded. Its design mirrors the AIS bit layout exactly so that a developer can cross-reference field by field against ITU-R M.1371.

### Field-by-field breakdown

---

#### `uint8_t msg_type`

**AIS bits:** 0–5 (6 bits)  
**Valid values:** 1, 2, or 3

Types 1, 2, and 3 are all Class A position reports with identical field layouts. The distinction is in transmission mode:
- Type 1 — autonomous (scheduled)
- Type 2 — assigned (shore station commanded)
- Type 3 — response to interrogation (polling)

`uint8_t` is used because the field is 6 bits wide and the value never exceeds 63. The decoder checks this field first and rejects any other message type, making it a guard for the rest of the decode.

---

#### `uint8_t repeat`

**AIS bits:** 6–7 (2 bits)  
**Valid values:** 0–3

Indicates how many times this message has been re-broadcast by a repeater station. When it reaches 3, repeaters must not re-transmit it further, preventing infinite relay loops. In tracking software this field is mostly informational, but it can flag data quality: a message relayed three hops is older and potentially less accurate than a direct reception.

`uint8_t` — maximum value is 3, fits in 2 bits. Any unsigned byte type would work; `uint8_t` signals intent.

---

#### `uint32_t mmsi`

**AIS bits:** 8–37 (30 bits)  
**Valid values:** 000000000–999999999

The Maritime Mobile Service Identity is the vessel's unique radio identifier. Nine digits. The first three digits are the MID (Maritime Identification Digit), which encodes the country of registration — for example 567 = Thailand, 525 = Indonesia.

`uint32_t` is the correct choice: a 9-digit decimal number fits in 30 bits (max 2^30 = ~1.07 billion, well above 999,999,999). `int` would also work on 32-bit and 64-bit systems, but using the explicit-width type documents that this comes from a 30-bit protocol field.

This is the **primary key** of the vessel table. Every lookup, insert, and update is keyed on MMSI.

---

#### `uint8_t nav_status`

**AIS bits:** 38–41 (4 bits)  
**Valid values:** 0–15

Encodes what the vessel is doing. Directly affects required reporting interval (vessels at anchor report every 3 minutes; underway vessels every 2–12 seconds). The 16 possible values are:

| Value | Meaning |
|---|---|
| 0 | Under way using engine |
| 1 | At anchor |
| 2 | Not under command |
| 3 | Restricted manoeuvrability |
| 5 | Moored |
| 7 | Engaged in fishing |
| 8 | Under way sailing |
| 15 | Not defined / default |

`uint8_t` — field is 4 bits, values 0–15 fit comfortably. The `& 0xF` mask in output code is defensive in case of malformed data.

---

#### `int8_t rot`

**AIS bits:** 42–49 (8 bits, signed)  
**Valid values:** –128 to +126; –128 = not available

Rate of Turn in a non-linear encoding. The actual ROT in degrees per minute is:

```
ROT_sensor = (rot_ais / 4.733)²  × sign(rot_ais)
```

Special values:
- `+127` — turning right at > 5°/30 s (no ROT indicator fitted)
- `-127` — turning left at > 5°/30 s (no ROT indicator fitted)
- `-128` — not available (default when no ROT sensor)

`int8_t` is the natural signed 8-bit type. The encoding is signed because left turns are negative, right turns positive. In this demo ROT is stored but not acted upon (–128 is always encoded), which is correct — a demo without a simulated ROT sensor should not fabricate this value.

---

#### `float sog`

**AIS bits:** 50–59 (10 bits)  
**Protocol unit:** 1/10 knot  
**Stored as:** knots (already divided by 10)

Speed Over Ground. The raw field holds `speed × 10`, so 8.5 knots is stored as integer 85. The decoder divides by 10.0 immediately, and the rest of the system works in knots. Special value: 1023 (102.3) = not available.

`float` is chosen over `double` because:
1. AIS resolution is 0.1 knot — far less precision than even `float` provides
2. SOG is used in position propagation arithmetic where `sinf`/`cosf` already return `float`
3. Memory alignment with other `float` fields in the struct

---

#### `uint8_t pos_accuracy`

**AIS bits:** 60 (1 bit)  
**Values:** 0 = low (> 10 m GNSS), 1 = high (< 10 m, differential GNSS)

A single bit indicating whether the position was fixed with standard GPS accuracy or with a DGNSS correction applied. In a production tracker this would be displayed as a confidence indicator on the map. In this demo it is stored but not rendered, keeping the struct protocol-complete.

`uint8_t` for a 1-bit field is the smallest practical type in C. A `bool` (`_Bool`) would also work but is less idiomatic in low-level protocol code.

---

#### `float longitude` / `float latitude`

**AIS bits:** Longitude 61–88 (28 bits signed), Latitude 89–115 (27 bits signed)  
**Protocol unit:** 1/10000 minute of arc  
**Stored as:** decimal degrees

The raw integer from the bit field is divided by 600,000 to get decimal degrees:

```
latitude_deg  = raw_lat  / 600000.0
longitude_deg = raw_lon  / 600000.0
```

The constant 600,000 comes from: 10,000 (units per minute) × 60 (minutes per degree).

**Why `float` and not `double`?** AIS coordinate resolution is 1/10000 minute ≈ 0.185 metres. A `float` has ~7 significant decimal digits. At latitude 12° N, the decimal degree representation is `12.XXXXXX`. Seven digits gives resolution to ~0.0001° ≈ 11 metres — slightly coarser than the protocol, but adequate for display and position propagation over short distances. `double` would be more accurate but is unnecessary for a demo and would misalign with the other `float` fields.

Special not-available values: longitude 181°, latitude 91°. These are checked in a production decoder but not in this demo.

---

#### `float cog`

**AIS bits:** 116–127 (12 bits)  
**Protocol unit:** 1/10 degree  
**Stored as:** degrees (0.0 – 359.9)

Course Over Ground — the actual direction of movement over the seabed, which can differ from heading in the presence of current or wind. Raw value is divided by 10.0. Special value: 3600 = not available.

`float` — same rationale as SOG. Resolution is 0.1°, well within float precision. COG is used directly as the rotation angle for the map arrow marker.

---

#### `uint16_t heading`

**AIS bits:** 128–136 (9 bits)  
**Valid values:** 0–359; 511 = not available

True heading — the direction the bow is pointing, from a gyrocompass. Differs from COG when there is leeway or current. 9 bits can hold 0–511. `uint8_t` would overflow (max 255), so `uint16_t` is required. Not divided — it is already in whole degrees.

---

#### `uint8_t timestamp`

**AIS bits:** 137–142 (6 bits)  
**Values:** 0–59 = UTC second; 60 = no fix; 61 = manual; 62 = EPFS estimated; 63 = inoperative

The second within the current UTC minute at which the position was generated. This is not a full timestamp — it only records seconds. In a real deployment this is used to assess position staleness when combined with the receiver's UTC clock. In this demo it is populated with `t % 60` from the simulation clock and can drive the "seconds ago" display in a future extension.

`uint8_t` — 6-bit field, values 0–63.

---

#### `uint8_t raim`

**AIS bits:** 148 (1 bit)  
**Values:** 0 = not in use, 1 = RAIM in use

Receiver Autonomous Integrity Monitoring flag. RAIM is a self-check that the GNSS receiver performs to detect satellite signal errors. When set, the position has been verified to meet accuracy standards. Stored but not used in this demo.

---

#### `time_t received_at`

**Not from AIS protocol** — added by the decoder  
**Purpose:** Wall-clock or simulation-clock time at which this sentence was processed

This field does not exist in the AIS bit stream. It is injected by the replay engine from the `T=` prefix in the sample log. It is the foundation of the interval calculation:

```
gap = current_msg.received_at - vessel->last_seen
avg_interval = avg_interval × 0.8 + gap × 0.2
```

`time_t` is the C standard type for calendar time (seconds since epoch). Using `time_t` instead of `int` makes the intent unambiguous and keeps the code portable across platforms where `time_t` may be 32 or 64 bits.

---

## 3. `VesselFix` — Single Historical Position

**File:** `vessel.h`  
**Represents:** One snapshot of a vessel's position and motion at a specific moment

```c
typedef struct {
    float  lat, lon;
    float  sog;
    float  cog;
    time_t t;
} VesselFix;
```

`VesselFix` is deliberately minimal. It exists only to power two things:

1. **The map trail** — the dashed polyline showing where the vessel has been
2. **Future animation** — a playhead scrubbing through time needs position + motion at each step

### Why not just store lat/lon?

SOG and COG are included so that an animation engine can interpolate between two fixes. Given `fix[i]` and `fix[i+1]`, you can estimate the position at any intermediate time using the rhumb line formula with `fix[i].sog` and `fix[i].cog`. Without them, interpolation would require computing speed from the position delta, which is noisier.

### Why `float` for lat/lon here but not `double`?

The trail is for visual rendering — the HTML map canvas is typically 1000–2000 pixels wide covering 5–10 degrees of longitude. One pixel ≈ 0.003°. `float` precision of ~0.0001° is 30× finer than pixel resolution, more than adequate.

### `time_t t`

The absolute time of this fix. Required to:
- Label trail points on the animated map
- Calculate speed between fixes as a sanity check
- Determine how old the trail is (fading older fixes)

---

## 4. `Vessel` — Full Vessel Record

**File:** `vessel.h`  
**Represents:** Everything known about one vessel over the course of the tracking session

```c
typedef struct {
    uint32_t mmsi;
    char     name[21];
    uint8_t  ship_type;

    float    lat, lon;
    float    sog, cog;
    uint16_t heading;
    uint8_t  nav_status;

    time_t   first_seen;
    time_t   last_seen;
    int      update_count;

    int      expected_interval_s;
    double   avg_interval_s;

    VesselFix history[MAX_HISTORY];
    int       history_len;
} Vessel;
```

This is the central record. It is updated in place every time a new `AISMsg1` arrives for that MMSI. All fields are either the latest known state or accumulated statistics.

### Identity fields

#### `uint32_t mmsi`

Same type and value as in `AISMsg1`. The primary key of the vessel table. Never changes during a session — if a vessel's MMSI changes, it would appear as a new vessel.

#### `char name[21]`

AIS Type 5 messages carry the vessel name as 20 characters of 6-bit ASCII. The field is 20 chars + 1 for the null terminator = 21. In this demo the name is injected by `vessel_set_static()` from the hardcoded `SampleVessel` table because Type 5 is not yet decoded. The fixed-length array avoids dynamic allocation; vessels are short-lived in memory.

#### `uint8_t ship_type`

Two-digit ITU code (70 = cargo, 80 = tanker, 60 = passenger, 30 = fishing). The first digit categorises the vessel class; the second indicates cargo hazard category. Used in the HTML map to colour-code markers.

### Latest dynamic state

#### `float lat, lon` / `float sog, cog` / `uint16_t heading` / `uint8_t nav_status`

Mirrors the relevant fields from `AISMsg1`. These are **overwritten** on each update — they always reflect the most recent received position. The history array (below) is what preserves the trail.

Types are identical to `AISMsg1` for the same reasons. The only redundancy is intentional: `Vessel` owns its latest state independently from the transient `AISMsg1`.

### Tracking metadata

#### `time_t first_seen`

Set once, on the first message for this MMSI. Used to calculate total tracking duration: `last_seen - first_seen`. Useful for the "contact established" display in a full tracker.

#### `time_t last_seen`

Updated on every message. The interval calculation is:

```c
gap = msg.received_at - vessel->last_seen
```

#### `int update_count`

Total number of Type 1/2/3 messages received for this vessel. Together with the tracking duration, this gives an independent verification of the reporting rate:

```
observed_interval ≈ (last_seen - first_seen) / (update_count - 1)
```

The `update_count - 1` denominator gives the number of *gaps* between updates. `int` is appropriate — in a 10-minute demo the max is ~201 for the fastest vessel (24.5 kts at 3s intervals).

### Compliance fields

#### `int expected_interval_s`

The IMO-mandated interval in seconds for this vessel's current speed and status, recomputed on every update by calling `ais_reporting_interval(sog, nav_status)`. Stored (rather than recomputed at output time) so that any intermediate change in SOG is reflected in the stored value.

`int` rather than `uint8_t` because the largest value is 180 (seconds), which fits in `uint8_t`, but `int` is cleaner for arithmetic with `double avg_interval_s`.

#### `double avg_interval_s`

The rolling observed average interval, updated with an **exponential moving average**:

```c
avg = avg × 0.8 + gap × 0.2
```

The weight 0.2 on the new sample means recent intervals have more influence than old ones, which is appropriate for a system where vessels change speed (and thus reporting rate) during a session.

`double` is used here — unlike positions where `float` precision is sufficient, the interval calculation involves subtracting two large `time_t` values. Using `float` could introduce rounding error when the absolute timestamps are large. `double` gives 15 significant digits, well above any risk of precision loss.

**Why EMA rather than a simple mean?** A simple mean (`sum / count`) would require storing the sum, and would weight a vessel's first interval the same as its most recent one. The EMA is a single scalar, uses O(1) space and update time, and adapts smoothly to speed changes.

### Position history

#### `VesselFix history[MAX_HISTORY]`

A fixed-size ring of historical positions. `MAX_HISTORY = 200` is enough to hold the full 10-minute trail for the fastest vessel (201 messages at 3s intervals). For slower vessels, the array is only partially filled.

**Why a static array, not a dynamic list?**
- No `malloc`/`free` needed — simpler code, no memory leaks
- All `Vessel` records are the same size — the entire `VesselTable` is one contiguous allocation
- In an embedded or real-time environment, dynamic allocation is often prohibited
- 200 × 20 bytes = 4,000 bytes per vessel × 64 vessels = 256 KB total — well within any reasonable memory budget

**Why not a circular buffer?**
In this demo history is append-only (once the array fills, new fixes are silently dropped). A circular buffer would allow indefinite tracking by overwriting the oldest fix, at the cost of more complex index arithmetic. This is the right next step for production code but unnecessary for a 10-minute demo.

#### `int history_len`

Current number of valid entries in `history[]`. Avoids scanning the array for the last non-zero entry.

---

## 5. `VesselTable` — Live Vessel Register

**File:** `vessel.h`  
**Represents:** The complete set of all tracked vessels

```c
typedef struct {
    Vessel vessels[MAX_VESSELS];
    int    count;
} VesselTable;
```

The simplest possible vessel store: a flat array with a count.

### `Vessel vessels[MAX_VESSELS]`

`MAX_VESSELS = 64`. The entire table lives on the stack (or as a local in `main`). No dynamic allocation, no hash table, no linked list.

**Why a flat array?**

For a demo with 6 vessels, linear search (`O(n)`) is indistinguishable from `O(1)`. Even at the maximum of 64 vessels, the search loop touches at most 64 × 4 bytes (MMSI comparison) — one cache line. The simplicity benefit of a flat array far outweighs the theoretical performance cost at this scale.

**Why 64?**
A real VTS station tracks hundreds of vessels simultaneously. For this demo 64 is more than sufficient and keeps the struct under 1 MB. The constant is defined in the header so it can be raised without changing any other code.

### `int count`

The number of populated entries in `vessels[]`. New vessels are appended at `vessels[count++]`. Deletion is not implemented — in a demo, vessels never leave the picture.

---

## 6. `SampleVessel` — Simulation Seed

**File:** `main.c`  
**Represents:** The initial configuration for one simulated vessel

```c
typedef struct {
    uint32_t mmsi;
    const char *name;
    uint8_t  ship_type;
    uint8_t  nav_status;
    float    lat, lon;
    float    sog;
    float    cog;
    uint16_t heading;
} SampleVessel;
```

This struct exists only in the simulation layer — it has no equivalent in the AIS protocol. Its job is to seed the generator with the initial state of each vessel. Once the NMEA log is written, `SampleVessel` is no longer referenced.

### Key differences from `Vessel`

| `SampleVessel` | `Vessel` |
|---|---|
| `const char *name` (pointer to literal) | `char name[21]` (owned buffer) |
| No history, no timestamps | Full tracking metadata |
| Read-only, static | Mutated on every message |
| Used only during generation | Used during replay and output |

The `const char *name` pointer is safe here because `SampleVessel` entries are `static const` and the pointed-to string literals have static storage duration. This would be unsafe if `SampleVessel` were used beyond program startup.

### Why `float` for lat/lon here?

Starting positions are given to 4 decimal places (~11 m precision). `float` is fully adequate and matches the type used everywhere else for geographic coordinates.

---

## 7. Internal: Bit Array

**Not a named struct — implicit `uint8_t bits[168]`**  
**Used in:** `ais.c` encoder and decoder

The AIS payload is 168 bits. The internal representation during encode/decode is:

```c
uint8_t bits[168];   /* one bit per byte — wasteful but simple */
```

Each element holds exactly one bit (0 or 1). This is memory-inefficient (168 bytes to store 168 bits; a packed representation would use 21 bytes), but it has two important advantages:

1. **Indexing is trivial** — `bits[61]` is the first bit of the longitude field. No shifting or masking needed in the field-extraction code.
2. **Debugging is easy** — you can print `bits[0..167]` as a string of `'0'` and `'1'` and read it off against the ITU-R M.1371 table.

The helper functions `set_uint`, `set_int`, `get_uint`, `get_int` all operate on this array using a start index and a length:

```c
uint32_t mmsi = get_uint(bits, 8, 30);   // bits 8–37 inclusive
```

This maps directly to the protocol specification language ("bits 8–37: MMSI, 30 bits unsigned"), making the code self-documenting.

**Why not a packed `uint8_t[21]` with bitwise operations?**

The packed approach would be faster and more memory-efficient, but the bit-shift arithmetic becomes error-prone, especially for fields that straddle byte boundaries (e.g., COG spans bits 116–127, crossing byte boundaries at 120 and 127). The one-bit-per-byte approach eliminates all of that complexity in a component that runs once per message.

---

## 8. Design Trade-offs Summary

| Decision | Choice Made | Alternative | Why this choice |
|---|---|---|---|
| Vessel lookup | Linear scan (`O(n)`) | Hash map by MMSI | Sufficient for ≤ 64 vessels; no extra code |
| Vessel storage | Static array | `malloc`/linked list | No fragmentation, simpler lifecycle |
| Position history | Fixed array, append-only | Circular buffer | Adequate for 10-min demo; circular is the next step |
| Coordinates | `float` | `double` | AIS resolution (0.185 m) < float precision |
| Interval stat | Exponential MA | Simple mean | Adapts to speed changes; O(1) space |
| Bit array | 1 byte per bit | Packed bits | Self-documenting; trivial field indexing |
| Timestamps | `time_t` | `uint32_t` | Standard type; no precision risk with subtraction |
| String storage | `char name[21]` | `char *` + malloc | Fixed size; no memory management needed |
