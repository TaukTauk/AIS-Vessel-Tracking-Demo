# AIS Demo — Calculations & Algorithms Reference

**Project:** `ais_demo`  
**Files covered:** `ais.c`, `vessel.c`, `main.c`

This document describes every calculation and algorithm in the project — what problem it solves, the mathematics behind it, how it is implemented in C, and why it was chosen over alternatives.

---

## Table of Contents

1. [NMEA Checksum — XOR Parity](#1-nmea-checksum--xor-parity)
2. [AIS 6-bit ASCII Codec](#2-ais-6-bit-ascii-codec)
3. [Bit Array Serialisation — set/get helpers](#3-bit-array-serialisation--setget-helpers)
4. [Two's Complement Sign Extension](#4-twos-complement-sign-extension)
5. [Coordinate Unit Conversion](#5-coordinate-unit-conversion)
6. [SOG and COG Unit Conversion](#6-sog-and-cog-unit-conversion)
7. [Rounding on Encode](#7-rounding-on-encode)
8. [IMO Reporting Rate Decision](#8-imo-reporting-rate-decision)
9. [Exponential Moving Average — Interval Tracking](#9-exponential-moving-average--interval-tracking)
10. [Rhumb Line Position Propagation](#10-rhumb-line-position-propagation)
11. [Simulation Clock — Next-TX Scheduler](#11-simulation-clock--next-tx-scheduler)
12. [MMSI Default Name Generation](#12-mmsi-default-name-generation)
13. [Compliance Classification](#13-compliance-classification)
14. [Algorithm Interaction Map](#14-algorithm-interaction-map)

---

## 1. NMEA Checksum — XOR Parity

### Problem

Every NMEA 0183 sentence must carry a checksum so the receiver can detect transmission errors (bit flips, truncated lines, partial reads). A sentence without a valid checksum should be rejected before any parsing.

### Algorithm

The NMEA checksum is the **bitwise XOR of all bytes** between (but not including) the leading `!` or `$` and the trailing `*`.

```
checksum = byte[1] XOR byte[2] XOR ... XOR byte[n]
```

The result is encoded as two uppercase hexadecimal digits appended after `*`.

**Example:**

```
!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*73
         ↑                                 ↑
       start                              end (exclusive)
```

XOR every byte from `A` through `0` (the character just before `*`). The result must equal `0x73`.

**Why XOR?**

- O(n) with a single pass, no multiplication or division
- Detects any single-bit error in the protected range
- Detects any odd number of flipped bits
- Result is always 8 bits (one `uint8_t`), so the hex encoding is always exactly two characters
- Widely used in serial protocol checksums because of its simplicity in both hardware and software

**Weakness:** XOR does not detect transpositions (swapping two identical bytes cancels out). For maritime safety data this is acceptable because the AIS protocol adds its own forward error correction at the radio layer.

### Implementation

```c
static uint8_t nmea_cs(const char *s) {
    uint8_t cs = 0;
    const char *p = s + 1;          /* skip leading '!' */
    while (*p && *p != '*')
        cs ^= (uint8_t)*p++;
    return cs;
}

static int nmea_valid(const char *s) {
    const char *star = strchr(s, '*');
    if (!star || strlen(star) < 3) return 0;
    return nmea_cs(s) == (uint8_t)strtol(star + 1, NULL, 16);
}
```

`strtol(star + 1, NULL, 16)` reads the two hex digits as a base-16 integer. The cast to `uint8_t` truncates to one byte before comparison, handling any leading zeros or whitespace that `strtol` might include.

### In the encoder

After building the NMEA sentence without the checksum, `nmea_cs()` is called on the complete core string and the result is formatted into the output:

```c
snprintf(out, out_size, "%s*%02X\n", core, nmea_cs(core));
```

`%02X` formats as uppercase hex, zero-padded to two characters. This guarantees the checksum is always two hex digits, matching the NMEA standard exactly.

---

## 2. AIS 6-bit ASCII Codec

### Problem

AIS payloads are binary data (a stream of bits from the ship's transponder). They must be transmitted as printable ASCII characters inside the NMEA sentence. A direct base64 encoding would change the payload length unpredictably. The AIS standard defines its own fixed 6-bit-per-character encoding.

### Algorithm

Each 6-bit group maps to one printable ASCII character via a two-step formula.

**Encoding (6-bit value → ASCII character):**

```
step 1:  ascii = value + 48
step 2:  if ascii > 87:  ascii += 8
```

**Decoding (ASCII character → 6-bit value):**

```
step 1:  value = ascii - 48
step 2:  if value >= 40:  value -= 8
```

### Why the gap at +8?

The ASCII table has a non-printable region between decimal 88 (`X`) and 96 (backtick `` ` ``). Values 40–47 after subtracting 48 would land in ASCII range 88–95 — that block contains characters `X`, `Y`, `Z`, `[`, `\`, `]`, `^`, `_` which are technically printable but were historically avoided in marine terminals. The `+8` shift jumps over ASCII 88–95, landing in the range 96–103 (`` ` ``, `a`, `b`, `c`, `d`, `e`, `f`, `g`).

**Full mapping table (selected values):**

| 6-bit value | Step 1 (+48) | Step 2 (+8 if >87) | Final ASCII |
|---|---|---|---|
| 0 | 48 | — | `0` |
| 9 | 57 | — | `9` |
| 39 | 87 | — | `W` |
| 40 | 88 | 96 | `` ` `` |
| 47 | 95 | 103 | `g` |
| 63 | 111 | — | `o` |

Values 0–39 map to ASCII 48–87 (`0`–`W`).  
Values 40–63 map to ASCII 96–119 (`` ` ``–`w`).

The range 88–95 (`X`–`_`) is never produced by valid AIS encoding.

### Implementation

```c
static char encode_6bit(uint8_t v) {
    v &= 0x3F;      /* mask to 6 bits — defensive against overflow */
    v += 48;
    if (v > 87) v += 8;
    return (char)v;
}

static uint8_t decode_6bit(char c) {
    uint8_t v = (uint8_t)c - 48u;
    if (v >= 40) v -= 8;
    return v & 0x3Fu;    /* mask ensures 6-bit output even for bad input */
}
```

The `& 0x3F` masks (`0x3F` = `0b00111111` = 63 decimal) serve as defensive bounds. In the encoder they prevent a stray value > 63 from corrupting the mapping. In the decoder they ensure the result fits in 6 bits even if the input character is outside the valid AIS range.

### Bit ordering

When packing 6-bit values into the bit array:

```c
/* payload_to_bits: each payload char → 6 bits, MSB first */
for (int b = 5; b >= 0; b--)
    bits[i * 6 + (5 - b)] = (v >> b) & 1u;
```

The most significant bit of the 6-bit value is placed first (at the lower bit index). This matches the AIS standard's big-endian bit numbering where bit 0 of the payload is the MSB of the first character.

---

## 3. Bit Array Serialisation — set/get helpers

### Problem

AIS message fields are packed at arbitrary bit offsets and arbitrary widths. For example, longitude starts at bit 61 and is 28 bits wide. No C type maps cleanly to that. The helpers provide a uniform API: "write/read N bits starting at position P."

### `get_uint` — read unsigned field

```c
static uint32_t get_uint(const uint8_t *bits, int start, int len) {
    uint32_t result = 0;
    for (int i = 0; i < len; i++)
        result = (result << 1) | bits[start + i];
    return result;
}
```

**Algorithm:** Shift-and-OR accumulator, MSB first.

Starting with `result = 0`, for each bit position from `start` to `start + len - 1`:
1. Shift `result` left by 1 (making room for the next bit)
2. OR in the current bit from `bits[start + i]`

After `len` iterations, `result` holds the numeric value of the bit field.

**Example** — reading 4-bit nav_status at bit 38:

```
bits[38..41] = [0, 0, 0, 0]   → result = 0   (underway)
bits[38..41] = [0, 0, 0, 1]   → result = 1   (at anchor)
bits[38..41] = [0, 1, 1, 1]   → result = 7   (fishing)
```

**Why iterate rather than bitmask?**

Bitmask arithmetic on a packed byte array requires careful handling of cross-byte fields. The one-bit-per-byte array eliminates this — indexing `bits[61]` through `bits[88]` for a 28-bit field is trivially correct. The loop is O(n) in field width, but field widths are at most 30 bits (MMSI), so the maximum iteration count is small and constant.

### `set_uint` — write unsigned field

```c
static void set_uint(uint8_t *bits, int start, int len, uint32_t val) {
    for (int i = len - 1; i >= 0; i--) {
        bits[start + i] = val & 1u;   /* write LSB of val to last position */
        val >>= 1;                     /* shift val right for next bit */
    }
}
```

**Algorithm:** Reverse iteration, LSB-first extraction.

The loop fills from `start + len - 1` (the LSB position) down to `start` (the MSB position), extracting one bit at a time from the value's LSB and shifting right.

**Example** — writing MMSI 567001001 (30 bits) at bit 8:

```
567001001 in binary (30 bits):
  100001110001010010001101101001

bits[8]  = 1  (MSB)
bits[9]  = 0
...
bits[37] = 1  (LSB)
```

### `get_int` / `set_int` — signed fields

Latitude and longitude are **signed** fields (negative for South/West). The protocol uses two's complement representation.

```c
static int32_t get_int(const uint8_t *bits, int start, int len) {
    uint32_t raw = get_uint(bits, start, len);
    /* Sign-extend if MSB set */
    if (len < 32 && (raw & (1u << (len - 1))))
        raw |= ~((1u << len) - 1u);
    return (int32_t)raw;
}
```

`get_uint` extracts the bits as an unsigned value. Then: if the MSB of the field is 1 (meaning the number is negative in two's complement), **sign-extend** by filling all upper bits with 1s.

**How sign extension works:**

For a 27-bit signed field with value `0b111...1` (all ones = −1 in two's complement):

1. `raw = get_uint(...)` → `0x07FFFFFF` (27 bits of 1s)
2. `(1u << 26)` = the MSB mask for a 27-bit field
3. `raw & (1u << 26)` → non-zero → field is negative
4. `~((1u << 27) - 1u)` → `0xF8000000` (upper 5 bits are 1)
5. `raw |= 0xF8000000` → `0xFFFFFFFF` → cast to `int32_t` → −1

For `set_int`, the value is simply masked to the field width:

```c
static void set_int(uint8_t *bits, int start, int len, int32_t val) {
    uint32_t mask = (len < 32) ? ((1u << len) - 1u) : 0xFFFFFFFFu;
    set_uint(bits, start, len, (uint32_t)val & mask);
}
```

Casting `int32_t` to `uint32_t` reinterprets the two's complement bit pattern without changing any bits. The mask then discards all bits above position `len − 1`, leaving exactly the correct two's complement representation in the field width required.

---

## 4. Two's Complement Sign Extension

This algorithm is important enough to explain in isolation because it appears in several places implicitly.

### The problem

A C `int32_t` is 32 bits. An AIS latitude field is 27 bits. When you extract 27 bits and place them in a 32-bit integer, the upper 5 bits are zero. For positive values, that is correct. For negative values (e.g., a latitude of −10°), the two's complement representation in 27 bits has bit 26 (the MSB) set to 1, but bits 27–31 in the 32-bit integer are still 0. The number reads as a large positive value instead of the intended negative.

### The fix — OR with a sign mask

Given raw field value `v` of width `n` bits:

```
if bit (n-1) of v is 1:
    v = v | (~0 << n)       /* fill upper bits with 1s */
```

In C: `raw |= ~((1u << len) - 1u)`

**Derivation:**
- `(1u << len)` — the value just above the field range
- `(1u << len) - 1u` — a mask of exactly `len` ones: `0x07FFFFFF` for len=27
- `~(...)` — bitwise NOT: all bits above position `len-1` become 1: `0xF8000000`
- `raw |= ...` — sets all upper bits to 1, converting to correct 32-bit negative

**Concrete example — longitude of −99.5°:**

```
raw degrees = -99.5
raw protocol value = -99.5 × 600000 = -59700000

In 28-bit two's complement:  0xFC84F5E0
  binary: 1111 1100 1000 0100 1111 0101 1110 0000
  MSB (bit 27) = 1 → negative

After get_uint: 0x0C84F5E0  (upper 4 bits zeroed out — WRONG as int32)
After sign ext: 0xFC84F5E0  (upper 4 bits filled with 1 — CORRECT as int32)
As int32_t:     -59700000 ✓
Divide by 600000: -99.5° ✓
```

---

## 5. Coordinate Unit Conversion

### Problem

AIS transmits latitude and longitude as integers (no floating point on radio), yet the tracking system needs decimal degrees for display and computation.

### Protocol unit

The AIS standard (ITU-R M.1371) defines coordinates in units of **1/10000 of a minute of arc**.

```
resolution = 1/10000 minute = 0.0001 minute = 0.0001/60 degree ≈ 0.00000167°
```

At the equator, 1° latitude ≈ 111,111 m, so:

```
position_resolution = 0.00000167° × 111,111 m/° ≈ 0.185 m
```

AIS coordinates are thus accurate to approximately 18 cm — more than sufficient for any maritime navigation purpose.

### Conversion constant

To convert from decimal degrees to the integer protocol unit:

```
raw = degrees × 10000 × 60 = degrees × 600000
```

The factor 60 converts degrees to minutes; the factor 10000 scales minutes to the protocol unit.

```c
set_int(bits, 61, 28, (int32_t)(lon * 600000.0f));   /* encoder */
msg->longitude = get_int(bits, 61, 28) / 600000.0f;  /* decoder */
```

**Why multiply first, then cast to int32_t?**

The product `lon × 600000.0f` is computed in floating point before truncation. This avoids integer overflow — the maximum longitude value in protocol units is 180 × 600000 = 108,000,000, which fits in int32_t (max ~2.1 billion) but would overflow int16_t (max 32,767).

**Latitude bit width:**

Latitude range is ±90°. Max protocol value: 90 × 600000 = 54,000,000.  
Required bits: ⌈log₂(54,000,000 × 2 + 1)⌉ = ⌈log₂(108,000,001)⌉ = 27 bits ✓

**Longitude bit width:**

Longitude range is ±180°. Max protocol value: 180 × 600000 = 108,000,000.  
Required bits: ⌈log₂(108,000,000 × 2 + 1)⌉ = ⌈log₂(216,000,001)⌉ = 28 bits ✓

This is why the AIS standard allocates 27 bits for latitude and 28 bits for longitude — the minimum needed to represent the full range.

---

## 6. SOG and COG Unit Conversion

### Speed Over Ground (SOG)

**Protocol unit:** 1/10 knot  
**Range:** 0–1022 (= 0–102.2 knots); 1023 = not available

```c
/* Encoder */
set_uint(bits, 50, 10, (uint32_t)(sog * 10.0f + 0.5f));

/* Decoder */
msg->sog = get_uint(bits, 50, 10) / 10.0f;
```

10 bits can represent 0–1023. The protocol dedicates 1022 to the max speed and 1023 as the "not available" sentinel.

### Course Over Ground (COG)

**Protocol unit:** 1/10 degree  
**Range:** 0–3599 (= 0.0°–359.9°); 3600 = not available

```c
/* Encoder */
set_uint(bits, 116, 12, (uint32_t)(cog * 10.0f + 0.5f));

/* Decoder */
msg->cog = get_uint(bits, 116, 12) / 10.0f;
```

12 bits can represent 0–4095, so 3600 as a "not available" sentinel leaves plenty of headroom.

Both encode with `× 10`, decode with `/ 10`. The pattern is consistent: **multiply to encode, divide to decode**.

---

## 7. Rounding on Encode

### Problem

Converting a `float` to an integer always truncates toward zero. For encoding, truncation introduces a systematic negative bias.

**Example — SOG 8.5 knots:**

```
8.5 × 10.0f = 85.0f → (uint32_t)85 ✓   (exact in this case)

But: 8.56 × 10.0f = 85.6f → (uint32_t)85  (truncated, loses 0.6)
     8.56 × 10.0f + 0.5f = 86.1f → (uint32_t)86  (rounded correctly)
```

### Fix — add 0.5 before truncation

```c
(uint32_t)(sog * 10.0f + 0.5f)
(uint32_t)(cog * 10.0f + 0.5f)
```

Adding 0.5 before truncation is equivalent to round-half-up rounding. For a value `x`:

```
floor(x + 0.5) = round(x)    for all x ≥ 0
```

This is a standard technique for integer rounding in C (which lacks a `round()` function for integers, only for float/double). Note: `lroundf()` from `<math.h>` would also work but adds a function call overhead for a trivial case.

**Why only on encode?**

The decoder divides the integer by 10.0f. Integer division is exact — there is no fractional part to round.

---

## 8. IMO Reporting Rate Decision

### Problem

A vessel tracker needs to know how frequently each vessel should be transmitting. This is used to detect non-compliant vessels (transmitting too rarely or too often) and to judge how stale a position fix is.

### Algorithm

A simple speed-band lookup with priority override for navigational status:

```c
int ais_reporting_interval(float sog, uint8_t nav_status) {
    if (nav_status == 1 || nav_status == 5) return 180;   /* anchored/moored */
    if (sog < 0.1f) return 180;                           /* effectively stopped */
    if (sog <= 14.0f) return 12;
    if (sog <= 23.0f) return 6;
    return 3;
}
```

**Source:** IMO Resolution MSC.74(69), Annex 3, Table 1 (reproduced exactly)

| Condition | Interval | Reason |
|---|---|---|
| At anchor / moored | 180 s | Vessel not moving, position changes are irrelevant |
| 0–14 knots | 12 s | Slow vessel, position changes slowly |
| 14–23 knots | 6 s | Medium speed — update rate doubles |
| > 23 knots | 3 s | Fast vessel — position changes ~12 m/s, must update frequently |

**Why nav_status is checked first:**

A vessel can be at anchor with sog > 0 (current or wind causing slight motion). The navigational status declared by the officer is authoritative — a vessel at anchor should use the 180-second rate regardless of the GPS-reported speed. The `sog < 0.1f` fallback catches undeclared-but-stationary vessels.

**The maneuvering halving rule:**

The IMO table also specifies halved intervals when changing course. This is not implemented in the demo because course change detection requires comparing two consecutive headings, which would require storing the previous heading in the vessel record. This is a noted limitation, documented in PLANNING.md.

---

## 9. Exponential Moving Average — Interval Tracking

### Problem

Given a stream of AIS messages arriving at irregular but approximately periodic intervals, compute a representative "average interval" that:
- Adapts when the vessel changes speed (and thus changes reporting rate)
- Does not require storing all past intervals (O(1) memory)
- Is not overly sensitive to individual anomalies

### Algorithm — Exponential Moving Average (EMA)

```c
double gap = (double)(now - v->last_seen);
if (gap > 0 && gap < 600) {
    if (v->avg_interval_s < 0.001)
        v->avg_interval_s = gap;          /* first gap — seed directly */
    else
        v->avg_interval_s = v->avg_interval_s * 0.8 + gap * 0.2;
}
```

The EMA recurrence relation:

```
EMA_new = α × new_sample + (1 − α) × EMA_old
```

With α = 0.2:

```
EMA_new = 0.2 × gap + 0.8 × EMA_old
```

### What α = 0.2 means

α controls how quickly the average responds to new data.

- **α = 1.0** — no smoothing. EMA equals the most recent gap. Maximum sensitivity, maximum noise.
- **α = 0.0** — no updating. EMA never changes from its initial value.
- **α = 0.2** — 20% weight on the newest sample. The "effective memory" (number of samples that meaningfully contribute) is approximately `1/α = 5` samples.

After a vessel changes speed, the EMA converges to the new interval in roughly `1/α = 5` messages:

```
Old rate: 12 s interval  → EMA = 12.0
After change to 6 s:
  msg 1: 0.2×6  + 0.8×12.0  = 10.8
  msg 2: 0.2×6  + 0.8×10.8  = 9.84
  msg 3: 0.2×6  + 0.8×9.84  = 9.07
  msg 4: 0.2×6  + 0.8×9.07  = 8.46
  msg 5: 0.2×6  + 0.8×8.46  = 7.97
  ...converges toward 6.0
```

### The gap guard: `gap < 600`

Gaps larger than 600 seconds (10 minutes) are ignored. This handles:
- The first message after a cold start (gap from epoch 0 is enormous)
- Log restarts or file seeks mid-replay
- A vessel going silent and reappearing after an extended period

Without this guard, a single large gap would completely dominate the EMA and make the interval look far longer than the actual transmission rate.

### Seeding the first value

When `avg_interval_s < 0.001` (effectively zero — never been set), the first observed gap is used directly rather than computing `0.2 × gap + 0.8 × 0`, which would permanently underweight the initial sample.

---

## 10. Rhumb Line Position Propagation

### Problem

The simulation must advance each vessel's geographic position forward in time at its given speed and course. Given a starting position (lat₀, lon₀), a speed (knots), a course (degrees true), and an elapsed time (seconds), compute the new (lat₁, lon₁).

### Rhumb Line vs Great Circle

There are two common approximations:

| Method | Accuracy | Complexity |
|---|---|---|
| Flat Earth (Cartesian) | Good near equator, degrades > 100 nm | O(1), no trig |
| **Rhumb Line (used here)** | Good for any heading up to ~100 nm | O(1), sin/cos |
| Great Circle | Exact for any distance | O(1), more trig |

A **rhumb line** (or loxodrome) is a path that maintains a constant bearing. Ships on autopilot follow rhumb lines. The key geometric property: as you move north or south, lines of longitude converge, so the same angular change in longitude represents a shorter east-west distance at higher latitudes.

### Algorithm

```c
static void propagate(float *lat, float *lon, float sog, float cog, int dt_sec) {
    if (sog < 0.01f) return;

    float cog_rad  = cog * (float)M_PI / 180.0f;
    float lat_rad  = *lat * (float)M_PI / 180.0f;
    float dt_hr    = dt_sec / 3600.0f;
    float dist_nm  = sog * dt_hr;

    *lat += dist_nm * cosf(cog_rad) / 60.0f;
    *lon += dist_nm * sinf(cog_rad) / (60.0f * cosf(lat_rad));
}
```

**Step 1 — Distance travelled:**

```
dist_nm = sog [knots] × dt_hr [hours]
```

One knot = one nautical mile per hour. Multiplying by elapsed time in hours gives distance in nautical miles.

**Step 2 — North-south displacement:**

```
Δlat = dist_nm × cos(COG) / 60.0
```

- `cos(COG)` extracts the northward component of the course vector
  - COG = 0° (due North):   cos(0)   = 1.0   → full northward motion
  - COG = 90° (due East):   cos(90°) = 0.0   → no north-south displacement
  - COG = 180° (due South): cos(180°) = −1.0 → southward motion
- Division by 60 converts nautical miles to degrees (1° latitude = 60 nm exactly)

**Step 3 — East-west displacement:**

```
Δlon = dist_nm × sin(COG) / (60.0 × cos(lat))
```

- `sin(COG)` extracts the eastward component
- The denominator `60 × cos(lat)` corrects for longitude convergence:
  - At the equator (lat = 0°): `cos(0) = 1.0` → no correction needed
  - At 60°N: `cos(60°) = 0.5` → the same angular longitude change represents half the east-west distance
  - At 90°N (pole): `cos(90°) = 0` → division by zero (lines of longitude converge to a point)

The correction factor `cos(lat)` is the exact rhumb line correction for small distances. For vessels in the Gulf of Thailand (10–13°N), `cos(12°) ≈ 0.978`, so the correction is less than 3% — small but meaningful for position accuracy.

### Error analysis

For the demo, the fastest vessel (24.5 kts, 3-second interval) travels:

```
dist = 24.5 kts × (3/3600) hrs = 0.0204 nm ≈ 37.8 m per update
```

The rhumb line approximation error is proportional to `dist²`, which at this scale is negligible (< 1 cm). The flat-earth error would be comparable at these distances. Both approximations are valid for a 10-minute simulation over a ~50 nm area.

### Why not `haversine`?

The haversine formula calculates the great-circle distance between two points and is appropriate for route planning over hundreds of miles. For short-interval dead reckoning (< 1 nm per step), the rhumb line approximation is simpler to implement, numerically stable, and matches how vessel autopilots actually work.

---

## 11. Simulation Clock — Next-TX Scheduler

### Problem

The simulation must generate AIS messages for six vessels, each with a different transmission interval, interleaved chronologically over 600 seconds. This is essentially a **discrete-event simulation** where events are transmissions.

### Algorithm

```c
int next_tx[N_VESSELS];

for (int i = 0; i < N_VESSELS; i++)
    next_tx[i] = 0;    /* all vessels transmit at t=0 */

for (int t = 0; t <= sim_duration; t++) {
    for (int i = 0; i < N_VESSELS; i++) {
        if (t < next_tx[i]) continue;

        int interval = ais_reporting_interval(sog[i], nav_status[i]);

        propagate(&lat[i], &lon[i], sog[i], cog[i], interval);

        /* emit NMEA sentence */

        next_tx[i] = t + interval;
    }
}
```

**Data structure:** `next_tx[i]` holds the next scheduled transmission time for vessel `i`.

**Outer loop:** The simulation clock `t` advances by 1 second per tick. This is the finest resolution needed — the shortest interval is 3 seconds, so ticking 1 second at a time never misses a scheduled transmission.

**Inner loop:** For each vessel, check if the current time has reached its scheduled transmission. If so, emit a message and schedule the next one at `t + interval`.

**Why not a priority queue?**

For 6 vessels, iterating all 6 on every tick is `O(6 × 600) = 3,600` operations. A min-heap priority queue would reduce this to `O(n log n)` event scheduling but would cost significantly more code for negligible gain at this scale. For a simulation with thousands of vessels, a priority queue would be the right choice.

**Chronological ordering:**

Because all vessels are checked at the same `t`, messages from different vessels that happen to share a transmission tick are emitted in vessel-index order. In a real AIS system, (S)TDMA ensures they occupy different time slots within the same second. For the demo, the ordering within a tick does not affect correctness.

---

## 12. MMSI Default Name Generation

### Problem

When a new MMSI is first seen in the log, no name is known yet (static data arrives separately in Type 5 messages, not yet decoded). The vessel needs a unique, human-readable placeholder.

### Algorithm

```c
snprintf(v->name, sizeof(v->name), "VESSEL-%u", mmsi % 100000);
```

`mmsi % 100000` takes the last 5 digits of the MMSI. Since MMSIs are 9 digits and the MID (country prefix) occupies the first 3, the last 6 digits are the vessel-unique portion. The last 5 of those are enough to produce a human-distinguishable label.

**Example:**

| MMSI | `% 100000` | Default name |
|---|---|---|
| 567001001 | 1001 | `VESSEL-1001` |
| 567001002 | 1002 | `VESSEL-1002` |
| 525001001 | 1001 | `VESSEL-1001` ← collision! |

**Collision:** Two vessels from different countries can have the same last 5 digits. In this demo `vessel_set_static()` always overwrites the default before the output stage runs, so collisions are never visible. In a production system you would use the full MMSI as the label, or look up the name from a vessel database.

---

## 13. Compliance Classification

### Problem

Given an observed average interval and an expected interval from the IMO table, classify the vessel as compliant, slightly off, or non-compliant.

### Algorithm

```c
double ratio = actual / expected;

if (ratio < 0.8 || ratio > 1.5)   → "NON-COMPLIANT"
if (ratio < 0.95 || ratio > 1.2)  → "MINOR DEVIATION"
else                               → "OK"
```

The ratio `actual / expected` normalises the comparison so it works identically for a 3-second interval and a 180-second interval.

**Threshold rationale:**

| Ratio | Meaning | Example (expected=12s) |
|---|---|---|
| < 0.8 | Over-reporting by > 25% | Observed < 9.6 s |
| 0.8 – 0.95 | Slightly over-reporting | 9.6 s – 11.4 s |
| 0.95 – 1.2 | Compliant band | 11.4 s – 14.4 s |
| 1.2 – 1.5 | Slightly under-reporting | 14.4 s – 18 s |
| > 1.5 | Under-reporting by > 50% | Observed > 18 s |

The compliant band is asymmetric (±5% low, +20% high). This reflects real-world practice: the IMO table specifies maximum intervals, not exact ones. Transmitting slightly faster than required is harmless; transmitting slower is a safety concern.

**Why ratio and not absolute difference?**

The absolute difference between 3 seconds and 4 seconds (1 second) feels similar to the difference between 180 seconds and 181 seconds — but they represent very different compliance levels (33% vs 0.6% over-interval). The ratio normalises this correctly.

---

## 14. Algorithm Interaction Map

The algorithms do not operate in isolation. This diagram shows the data flow and which calculations feed which:

```
    SAMPLE data
         │
         │  (sog, cog, lat, lon, interval)
         ▼
  ┌──────────────────┐
  │  Reporting Rate  │  ← IMO Table lookup (§8)
  │  Decision        │
  └────────┬─────────┘
           │  interval_s
           ▼
  ┌──────────────────┐
  │  Next-TX         │  ← Discrete event scheduler (§11)
  │  Scheduler       │
  └────────┬─────────┘
           │  trigger at t
           ▼
  ┌──────────────────┐
  │  Rhumb Line      │  ← Propagation (§10)
  │  Propagation     │     dist = sog × dt_hr
  └────────┬─────────┘     Δlat = dist × cos(COG) / 60
           │  new lat, lon  Δlon = dist × sin(COG) / (60 × cos(lat))
           ▼
  ┌──────────────────┐
  │  Rounding        │  ← × 10 + 0.5 (§7)
  │  + Unit Convert  │  ← × 600000 (§5, §6)
  └────────┬─────────┘
           │  integer protocol values
           ▼
  ┌──────────────────┐
  │  set_uint/int    │  ← Bit array serialisation (§3, §4)
  │  bit packing     │
  └────────┬─────────┘
           │  uint8_t bits[168]
           ▼
  ┌──────────────────┐
  │  6-bit ASCII     │  ← Codec (§2)
  │  encode          │
  └────────┬─────────┘
           │  payload string
           ▼
  ┌──────────────────┐
  │  NMEA Checksum   │  ← XOR parity (§1)
  │  + format        │
  └────────┬─────────┘
           │  "T=NNNN !AIVDM,...*XX\n"
           ▼
       sample.nmea
           │
           │  (replay reads line by line)
           ▼
  ┌──────────────────┐     ┌──────────────────┐
  │  NMEA Checksum   │────▶│  6-bit ASCII     │  ← Codec decode (§2)
  │  validate        │     │  decode          │
  └──────────────────┘     └────────┬─────────┘
        (§1)                        │  uint8_t bits[]
                                    ▼
                           ┌──────────────────┐
                           │  get_uint/int    │  ← Bit extraction (§3)
                           │  sign extension  │  ← Sign extension (§4)
                           └────────┬─────────┘
                                    │  raw integer fields
                                    ▼
                           ┌──────────────────┐
                           │  Unit conversion │  ← ÷ 600000, ÷ 10 (§5, §6)
                           └────────┬─────────┘
                                    │  AISMsg1 struct
                                    ▼
                           ┌──────────────────┐
                           │  Vessel Update   │
                           │  EMA interval    │  ← EMA (§9)
                           │  + history trail │
                           └────────┬─────────┘
                                    │  Vessel struct
                                    ▼
                           ┌──────────────────┐
                           │  Compliance      │  ← Ratio classification (§13)
                           │  Classification  │
                           └──────────────────┘
                                    │
                              Terminal + HTML
```
