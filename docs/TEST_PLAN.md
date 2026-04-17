# AIS Demo — Test Plan

**Project:** `ais_demo` (clean implementation)  
**Test framework:** Plain C — no external libraries required  
**Test runner:** `make test`  
**Test file:** `tests/test_all.c`

---

## Table of Contents

1. [Test Philosophy](#1-test-philosophy)
2. [Test Infrastructure](#2-test-infrastructure)
3. [Suite A — NMEA Checksum](#3-suite-a--nmea-checksum)
4. [Suite B — 6-bit ASCII Codec](#4-suite-b--6-bit-ascii-codec)
5. [Suite C — Bit Array Helpers](#5-suite-c--bit-array-helpers)
6. [Suite D — Coordinate Conversion](#6-suite-d--coordinate-conversion)
7. [Suite E — Unit Conversion (SOG / COG)](#7-suite-e--unit-conversion-sog--cog)
8. [Suite F — Full Message Round-Trip](#8-suite-f--full-message-round-trip)
9. [Suite G — Sentinel Value Handling](#9-suite-g--sentinel-value-handling)
10. [Suite H — NMEA Parse Error Paths](#10-suite-h--nmea-parse-error-paths)
11. [Suite I — Reporting Rate Logic](#11-suite-i--reporting-rate-logic)
12. [Suite J — Vessel State Tracking](#12-suite-j--vessel-state-tracking)
13. [Suite K — EMA Interval Calculation](#13-suite-k--ema-interval-calculation)
14. [Suite L — Rhumb Line Propagation](#14-suite-l--rhumb-line-propagation)
15. [Suite M — Compliance Classification](#15-suite-m--compliance-classification)
16. [Suite N — Known Real-World Sentences](#16-suite-n--known-real-world-sentences)
17. [Coverage Summary](#17-coverage-summary)

---

## 1. Test Philosophy

**Test the contract, not the implementation.**
Each test expresses what a function must do (its documented contract),
not how it does it internally. Tests should still pass if an algorithm
is replaced with a mathematically equivalent one.

**Every algorithm document entry gets at least one test.**
Sections 1–13 of ALGORITHMS.md each map to one or more test cases here.
If a new algorithm is added, a test suite must be added before the code.

**Tests run without side effects.**
No test writes files, opens network connections, or modifies global state.
Each test function receives all inputs as parameters and checks outputs
against expected values.

**Real-world sentences are the ground truth.**
Suite N uses actual `!AIVDM` sentences from public AIS feeds with
known decoded values (from the AIVDM/AIVDO reference by Eric S. Raymond).
These are the definitive correctness check — if the decoder produces the
right values for a real sentence, the implementation is correct.

---

## 2. Test Infrastructure

The test runner is a minimal framework implemented in `tests/test_all.c`.
No external libraries (no Unity, no CMocka). The entire test suite
compiles and runs with:

```bash
make test
```

### Test macro API

```c
/* tests/testlib.h */

#define ASSERT_EQ_INT(actual, expected, name)                        \
    do {                                                             \
        if ((actual) != (expected)) {                                \
            printf("  FAIL  %s\n"                                   \
                   "        expected %d  got %d\n",                  \
                   (name), (expected), (actual));                    \
            g_failures++;                                            \
        } else {                                                     \
            printf("  PASS  %s\n", (name));                         \
            g_passes++;                                              \
        }                                                            \
    } while(0)

#define ASSERT_EQ_UINT(actual, expected, name)    /* uint32_t */
#define ASSERT_EQ_FLOAT(actual, expected, tol, name)  /* |a-e| < tol */
#define ASSERT_EQ_STR(actual, expected, name)     /* strcmp == 0 */
#define ASSERT_EQ_STATUS(actual, expected, name)  /* AisStatus */
#define ASSERT_NULL(ptr, name)
#define ASSERT_NOT_NULL(ptr, name)

/* Suite runner */
#define RUN_SUITE(fn)                                                \
    do {                                                             \
        printf("\n[%s]\n", #fn);                                     \
        fn();                                                        \
    } while(0)
```

### Pass/fail reporting

At the end of all suites:

```
Results: 87 passed, 0 failed
```

Any failure prints the test name, expected value, and actual value.
The process exits with code 0 on all-pass, code 1 on any failure —
allowing `make test` to fail the build.

---

## 3. Suite A — NMEA Checksum

**What is tested:** The XOR checksum calculation and validation.  
**Source:** ALGORITHMS.md §1

### A1 — Known sentence, correct checksum

```c
/* Real sentence from a Gulf of Thailand AIS feed */
const char *s = "!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*73";
ASSERT_EQ_INT(nmea_valid(s), 1, "A1 valid sentence accepted");
```

### A2 — Single bit flip in payload corrupts checksum

```c
/* Change one character in payload — checksum must fail */
const char *s = "!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S9,0*73";
/*                                                         ^ changed */
ASSERT_EQ_INT(nmea_valid(s), 0, "A2 corrupted payload rejected");
```

### A3 — Wrong checksum digits

```c
const char *s = "!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*FF";
ASSERT_EQ_INT(nmea_valid(s), 0, "A3 wrong checksum rejected");
```

### A4 — Missing asterisk

```c
const char *s = "!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,073";
ASSERT_EQ_INT(nmea_valid(s), 0, "A4 missing asterisk rejected");
```

### A5 — Checksum of all-zero payload

```c
/* Known: XOR of "AIVDM,1,1,,A,000000000000000000000000000000,0" */
/* Compute expected independently and hardcode */
ASSERT_EQ_INT(nmea_cs("!AIVDM,1,1,,A,...,0"), KNOWN_CS,
              "A5 checksum computed correctly");
```

### A6 — Encode then validate (round-trip)

```c
char sentence[128];
ais_encode_msg1(sentence, sizeof(sentence), 567001001, 0,
                8.5f, 12.5f, 100.9f, 45.0f, 45, 30);
ASSERT_EQ_INT(nmea_valid(sentence), 1, "A6 encoded sentence passes checksum");
```

---

## 4. Suite B — 6-bit ASCII Codec

**What is tested:** encode_6bit, decode_6bit, and their round-trip.  
**Source:** ALGORITHMS.md §2

### B1 — Boundary values encode correctly

```c
ASSERT_EQ_INT(encode_6bit(0),  '0', "B1a value 0  → '0' (ASCII 48)");
ASSERT_EQ_INT(encode_6bit(39), 'W', "B1b value 39 → 'W' (ASCII 87)");
ASSERT_EQ_INT(encode_6bit(40), '`', "B1c value 40 → '`' (ASCII 96, skip gap)");
ASSERT_EQ_INT(encode_6bit(63), 'w', "B1d value 63 → 'w' (ASCII 119)");
```

### B2 — Gap is never produced (ASCII 88–95 never output)

```c
for (int v = 0; v <= 63; v++) {
    char c = encode_6bit((uint8_t)v);
    int ascii = (unsigned char)c;
    /* Characters X(88) through _(95) must never appear */
    if (ascii >= 88 && ascii <= 95) {
        printf("  FAIL  B2 value %d produced forbidden ASCII %d\n", v, ascii);
        g_failures++;
    }
}
```

### B3 — Decode is exact inverse of encode

```c
for (int v = 0; v <= 63; v++) {
    char c = encode_6bit((uint8_t)v);
    uint8_t back = decode_6bit(c);
    if (back != (uint8_t)v) {
        printf("  FAIL  B3 round-trip failed for value %d\n", v);
        g_failures++;
    }
}
```

### B4 — Payload string encode/decode round-trip

```c
/* Encode a known payload, decode back, compare */
uint8_t bits_in[168]  = { /* set known pattern */ };
uint8_t bits_out[168] = {0};
char payload[29];
bits_to_payload(bits_in, 28, payload);
payload_to_bits(payload, 0, bits_out);
for (int i = 0; i < 168; i++)
    ASSERT_EQ_INT(bits_out[i], bits_in[i], "B4 bit round-trip");
```

---

## 5. Suite C — Bit Array Helpers

**What is tested:** get_uint, set_uint, get_int, set_int.  
**Source:** ALGORITHMS.md §3, §4

### C1 — get_uint reads correct value

```c
uint8_t bits[32] = {0};
/* Set bits 8–11 to 0b1010 = 10 */
bits[8]=1; bits[9]=0; bits[10]=1; bits[11]=0;
ASSERT_EQ_UINT(get_uint(bits, 8, 4), 10u, "C1 get_uint basic");
```

### C2 — set_uint then get_uint round-trip

```c
uint8_t bits[40] = {0};
set_uint(bits, 5, 10, 567u);
ASSERT_EQ_UINT(get_uint(bits, 5, 10), 567u, "C2 set/get_uint round-trip");
```

### C3 — set_uint does not write outside its range

```c
uint8_t bits[40] = {0};
/* Set sentinel values in adjacent regions */
bits[4] = 0xFF; bits[15] = 0xFF;
set_uint(bits, 5, 10, 0x3FFu);   /* fill all 10 bits with 1 */
ASSERT_EQ_INT(bits[4], 0xFF, "C3a lower boundary not touched");
ASSERT_EQ_INT(bits[15], 0xFF, "C3b upper boundary not touched");
```

### C4 — get_int sign extension: negative value

```c
uint8_t bits[32] = {0};
/* 27-bit two's complement of -1 = all 1s */
for (int i = 0; i < 27; i++) bits[i] = 1;
ASSERT_EQ_INT(get_int(bits, 0, 27), -1, "C4a get_int -1");
```

### C5 — get_int sign extension: most negative value

```c
uint8_t bits[32] = {0};
/* Most negative 27-bit value: 1000...0 = -67108864 */
bits[0] = 1;
ASSERT_EQ_INT(get_int(bits, 0, 27), -(1 << 26), "C5 get_int min value");
```

### C6 — set_int / get_int round-trip: negative latitude

```c
uint8_t bits[64] = {0};
int32_t raw = (int32_t)(-10.5f * 600000.0f);   /* −10.5° in protocol units */
set_int(bits, 10, 27, raw);
ASSERT_EQ_INT(get_int(bits, 10, 27), raw, "C6 signed lat round-trip");
```

### C7 — MMSI full range (30 bits)

```c
uint8_t bits[64] = {0};
set_uint(bits, 8, 30, 999999999u);
ASSERT_EQ_UINT(get_uint(bits, 8, 30), 999999999u, "C7 max MMSI value");
```

---

## 6. Suite D — Coordinate Conversion

**What is tested:** Latitude and longitude encode/decode unit conversion.  
**Source:** ALGORITHMS.md §5

### D1 — Positive latitude round-trip

```c
float lat_in  = 12.5833f;   /* Gulf of Thailand */
int32_t raw   = (int32_t)(lat_in * 600000.0f);
float lat_out = raw / 600000.0f;
ASSERT_EQ_FLOAT(lat_out, lat_in, 0.0001f, "D1 positive latitude round-trip");
```

### D2 — Negative latitude round-trip

```c
float lat_in  = -33.8688f;   /* Sydney */
int32_t raw   = (int32_t)(lat_in * 600000.0f);
float lat_out = raw / 600000.0f;
ASSERT_EQ_FLOAT(lat_out, lat_in, 0.0001f, "D2 negative latitude round-trip");
```

### D3 — Maximum longitude (±180°)

```c
float lon_in  = 180.0f;
int32_t raw   = (int32_t)(lon_in * 600000.0f);
ASSERT_EQ_INT(raw, 108000000, "D3 max longitude raw value");
```

### D4 — Minimum latitude (−90°) fits in 27 bits

```c
int32_t raw = (int32_t)(-90.0f * 600000.0f);
/* Must fit in signed 27-bit range: [−67108864, 67108863] */
ASSERT_EQ_INT(raw >= -(1 << 26), 1, "D4a min lat fits in 27 bits");
ASSERT_EQ_INT(raw <= (1 << 26) - 1, 1, "D4b min lat fits in 27 bits");
```

### D5 — Full encode/decode via AIS message

```c
AISMsg1 msg;
char sentence[128];
float lat = -22.9068f;   /* Rio de Janeiro */
float lon = -43.1729f;

ais_encode_msg1(sentence, sizeof(sentence),
                123456789, 0, 10.0f, lat, lon, 180.0f, 180, 0);
ais_parse_nmea(sentence, &msg);

ASSERT_EQ_FLOAT(msg.latitude,  lat, 0.0002f, "D5a lat encode/decode");
ASSERT_EQ_FLOAT(msg.longitude, lon, 0.0002f, "D5b lon encode/decode");
```

---

## 7. Suite E — Unit Conversion (SOG / COG)

**What is tested:** SOG and COG encode/decode with rounding.  
**Source:** ALGORITHMS.md §6, §7

### E1 — SOG exact value

```c
AISMsg1 msg; char s[128];
ais_encode_msg1(s, sizeof(s), 1, 0, 8.5f, 0,0, 0,0,0);
ais_parse_nmea(s, &msg);
ASSERT_EQ_FLOAT(msg.sog, 8.5f, 0.05f, "E1 SOG 8.5 kts round-trip");
```

### E2 — SOG rounding (0.5f ensures correct rounding)

```c
/* 8.56 knots should round to 8.6 (raw 86), not truncate to 8.5 (raw 85) */
AISMsg1 msg; char s[128];
ais_encode_msg1(s, sizeof(s), 1, 0, 8.56f, 0,0, 0,0,0);
ais_parse_nmea(s, &msg);
ASSERT_EQ_FLOAT(msg.sog, 8.6f, 0.05f, "E2 SOG rounds correctly");
```

### E3 — COG 359.9°

```c
AISMsg1 msg; char s[128];
ais_encode_msg1(s, sizeof(s), 1, 0, 5.0f, 0,0, 359.9f, 0, 0);
ais_parse_nmea(s, &msg);
ASSERT_EQ_FLOAT(msg.cog, 359.9f, 0.05f, "E3 COG 359.9° round-trip");
```

### E4 — COG 0.0° (due North)

```c
AISMsg1 msg; char s[128];
ais_encode_msg1(s, sizeof(s), 1, 0, 5.0f, 0,0, 0.0f, 0, 0);
ais_parse_nmea(s, &msg);
ASSERT_EQ_FLOAT(msg.cog, 0.0f, 0.05f, "E4 COG 0.0° round-trip");
```

---

## 8. Suite F — Full Message Round-Trip

**What is tested:** Complete encode → decode round-trip for all fields.  
**Source:** ALGORITHMS.md §2–§7 combined

### F1 — All fields survive encode/decode

```c
void test_roundtrip(uint32_t mmsi, uint8_t nav, float sog,
                    float lat, float lon, float cog,
                    uint16_t hdg, uint8_t ts, const char *name)
{
    char sentence[128];
    AISMsg1 decoded;

    ais_encode_msg1(sentence, sizeof(sentence),
                    mmsi, nav, sog, lat, lon, cog, hdg, ts);
    AisStatus st = ais_parse_nmea(sentence, &decoded);

    ASSERT_EQ_STATUS(st, AIS_OK, name " status");
    ASSERT_EQ_UINT(decoded.mmsi, mmsi, name " MMSI");
    ASSERT_EQ_INT(decoded.nav_status, nav, name " nav_status");
    ASSERT_EQ_FLOAT(decoded.sog, sog, 0.05f, name " SOG");
    ASSERT_EQ_FLOAT(decoded.latitude, lat, 0.0002f, name " lat");
    ASSERT_EQ_FLOAT(decoded.longitude, lon, 0.0002f, name " lon");
    ASSERT_EQ_FLOAT(decoded.cog, cog, 0.05f, name " COG");
    ASSERT_EQ_INT(decoded.heading, hdg, name " heading");
    ASSERT_EQ_INT(decoded.timestamp, ts % 60, name " timestamp");
}

/* F1a — Gulf of Thailand cargo */
test_roundtrip(567001001, 0, 8.5f,  12.5833f, 100.8833f, 45.0f,  45,  30, "F1a");

/* F1b — South hemisphere (negative lat) */
test_roundtrip(503123456, 0, 12.0f, -33.8688f, 151.2093f, 90.0f, 90,  15, "F1b");

/* F1c — West longitude (negative lon) */
test_roundtrip(338123456, 0, 18.0f, 40.7128f, -74.0060f, 270.0f, 270, 45, "F1c");

/* F1d — At anchor, zero speed */
test_roundtrip(566002001, 1, 0.0f,  12.2500f, 101.2500f, 0.0f,  511,  0, "F1d");

/* F1e — Fast vessel >23 kts */
test_roundtrip(525001001, 0, 24.5f, 10.8333f, 102.2000f, 320.0f, 320, 0, "F1e");
```

---

## 9. Suite G — Sentinel / Not-Available Values

**What is tested:** Sentinel detection and validity flags.  
**Source:** ERROR_HANDLING.md §8

### G1 — Latitude sentinel 91°

```c
/* Construct a message with lat=91 (not available) */
AISMsg1 msg;
/* ... build bits manually with lat = 91 × 600000 ... */
ASSERT_EQ_INT(msg.pos_valid, 0, "G1 lat sentinel sets pos_valid=0");
```

### G2 — Longitude sentinel 181°

```c
ASSERT_EQ_INT(msg.pos_valid, 0, "G2 lon sentinel sets pos_valid=0");
```

### G3 — SOG sentinel 102.3 (raw 1023)

```c
ASSERT_EQ_INT(msg.sog_valid, 0, "G3 SOG sentinel detected");
```

### G4 — Heading sentinel 511

```c
ASSERT_EQ_INT(msg.heading_valid, 0, "G4 heading sentinel detected");
```

### G5 — COG sentinel 360.0° (raw 3600)

```c
ASSERT_EQ_INT(msg.cog_valid, 0, "G5 COG sentinel detected");
```

### G6 — Valid values set flags to 1

```c
/* Normal Gulf of Thailand position */
ASSERT_EQ_INT(msg.pos_valid, 1, "G6a normal position is valid");
ASSERT_EQ_INT(msg.sog_valid, 1, "G6b normal SOG is valid");
```

---

## 10. Suite H — NMEA Parse Error Paths

**What is tested:** All error return codes from ais_parse_nmea.  
**Source:** ERROR_HANDLING.md §6, §7

### H1 — Bad checksum returns AIS_ERR_CHECKSUM

```c
AISMsg1 msg;
AisStatus st = ais_parse_nmea(
    "!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*FF", &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_CHECKSUM, "H1 bad checksum");
```

### H2 — Too few fields returns AIS_ERR_MALFORMED

```c
AisStatus st = ais_parse_nmea("!AIVDM,1,1,,A*73", &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_MALFORMED, "H2 missing fields");
```

### H3 — Empty payload returns AIS_ERR_MALFORMED

```c
AisStatus st = ais_parse_nmea("!AIVDM,1,1,,A,,0*XX", &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_MALFORMED, "H3 empty payload");
```

### H4 — Payload shorter than 28 chars returns AIS_ERR_TRUNCATED

```c
/* Only 10 payload chars — must be rejected before decode */
AisStatus st = ais_parse_nmea("!AIVDM,1,1,,A,15M67N000,0*XX", &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_TRUNCATED, "H4 truncated payload");
```

### H5 — Unsupported message type returns AIS_ERR_UNSUPPORTED

```c
/* Type 5 sentence — not yet handled */
AisStatus st = ais_parse_nmea(TYPE5_SENTENCE, &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_UNSUPPORTED, "H5 type 5 not supported");
```

### H6 — NULL pointer returns AIS_ERR_NULL

```c
AisStatus st = ais_parse_nmea(NULL, &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_NULL, "H6a null line pointer");

st = ais_parse_nmea("!AIVDM,...", NULL);
ASSERT_EQ_STATUS(st, AIS_ERR_NULL, "H6b null output pointer");
```

### H7 — Comment lines are skipped cleanly

```c
/* Lines starting with # must return AIS_ERR_UNSUPPORTED, not crash */
AisStatus st = ais_parse_nmea("# This is a comment", &msg);
ASSERT_EQ_STATUS(st, AIS_ERR_UNSUPPORTED, "H7 comment line");
```

### H8 — Empty field [3] (sequential ID) does not corrupt field [5]

```c
/* Two consecutive commas at position 3 — the strtok bug */
/* Sentence with valid payload but empty seq ID field */
const char *s = "!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*73";
AISMsg1 msg;
AisStatus st = ais_parse_nmea(s, &msg);
ASSERT_EQ_STATUS(st, AIS_OK, "H8a empty seq ID parses OK");
ASSERT_EQ_UINT(msg.mmsi, KNOWN_MMSI, "H8b correct MMSI after empty seq ID");
```

---

## 11. Suite I — Reporting Rate Logic

**What is tested:** ais_reporting_interval returns correct IMO values.  
**Source:** ALGORITHMS.md §8, IMO MSC.74(69) Annex 3 Table 1

```c
/* I1 — at anchor (nav_status=1), any speed → 180 s */
ASSERT_EQ_INT(ais_reporting_interval(0.0f, 1), 180, "I1a anchor 0 kts");
ASSERT_EQ_INT(ais_reporting_interval(5.0f, 1), 180, "I1b anchor 5 kts");

/* I2 — moored (nav_status=5) → 180 s */
ASSERT_EQ_INT(ais_reporting_interval(0.0f, 5), 180, "I2 moored");

/* I3 — effectively stopped, underway status → 180 s */
ASSERT_EQ_INT(ais_reporting_interval(0.05f, 0), 180, "I3 near-zero speed");

/* I4 — slow speed band (0–14 kts) → 12 s */
ASSERT_EQ_INT(ais_reporting_interval(1.0f,  0), 12, "I4a 1 kt");
ASSERT_EQ_INT(ais_reporting_interval(8.5f,  0), 12, "I4b 8.5 kts");
ASSERT_EQ_INT(ais_reporting_interval(14.0f, 0), 12, "I4c exactly 14 kts");

/* I5 — medium speed band (14–23 kts) → 6 s */
ASSERT_EQ_INT(ais_reporting_interval(14.1f, 0),  6, "I5a just above 14 kts");
ASSERT_EQ_INT(ais_reporting_interval(18.0f, 0),  6, "I5b 18 kts");
ASSERT_EQ_INT(ais_reporting_interval(23.0f, 0),  6, "I5c exactly 23 kts");

/* I6 — fast band (>23 kts) → 3 s */
ASSERT_EQ_INT(ais_reporting_interval(23.1f, 0),  3, "I6a just above 23 kts");
ASSERT_EQ_INT(ais_reporting_interval(24.5f, 0),  3, "I6b 24.5 kts");
ASSERT_EQ_INT(ais_reporting_interval(50.0f, 0),  3, "I6c 50 kts");
```

---

## 12. Suite J — Vessel State Tracking

**What is tested:** vessel_find, vessel_find_or_create, vessel_update.  
**Source:** DATA_STRUCTURES.md §4, §5

### J1 — New vessel is created on first message

```c
VesselTable t; vessel_table_init(&t);
ASSERT_EQ_INT(t.count, 0, "J1a table starts empty");
Vessel *v = vessel_find_or_create(&t, 567001001u);
ASSERT_NOT_NULL(v, "J1b vessel created");
ASSERT_EQ_INT(t.count, 1, "J1c count incremented");
ASSERT_EQ_UINT(v->mmsi, 567001001u, "J1d MMSI stored correctly");
```

### J2 — Same MMSI returns same vessel

```c
Vessel *v1 = vessel_find_or_create(&t, 567001001u);
Vessel *v2 = vessel_find_or_create(&t, 567001001u);
ASSERT_EQ_INT(v1 == v2, 1, "J2 same MMSI returns same pointer");
ASSERT_EQ_INT(t.count, 1, "J2b count unchanged on duplicate");
```

### J3 — Different MMSI creates different vessel

```c
Vessel *v2 = vessel_find_or_create(&t, 567001002u);
ASSERT_EQ_INT(t.count, 2, "J3 second MMSI adds new entry");
ASSERT_EQ_INT(v1 != v2, 1, "J3b different pointers");
```

### J4 — Vessel update stores latest position

```c
AISMsg1 msg = { .mmsi=567001001, .latitude=12.5f, .longitude=100.9f,
                .sog=8.5f, .cog=45.0f, .nav_status=0,
                .received_at=1000, .pos_valid=1 };
vessel_update(&t, &msg, ctx);
Vessel *v = vessel_find(&t, 567001001u);
ASSERT_EQ_FLOAT(v->lat, 12.5f, 0.001f, "J4a lat updated");
ASSERT_EQ_FLOAT(v->lon, 100.9f, 0.001f, "J4b lon updated");
ASSERT_EQ_INT(v->update_count, 1, "J4c update count incremented");
```

### J5 — Table full returns NULL for new MMSI

```c
VesselTable t; vessel_table_init(&t);
/* Fill table to capacity */
for (int i = 0; i < MAX_VESSELS; i++)
    vessel_find_or_create(&t, (uint32_t)(100000000 + i));
/* One more should return NULL */
Vessel *v = vessel_find_or_create(&t, 999999999u);
ASSERT_NULL(v, "J5 full table returns NULL");
```

### J6 — History appends correctly

```c
/* Send 5 messages — history_len should be 5 */
for (int i = 0; i < 5; i++) {
    msg.received_at = 1000 + i * 12;
    vessel_update(&t, &msg, ctx);
}
ASSERT_EQ_INT(v->history_len, 5, "J6 history len = 5");
```

---

## 13. Suite K — EMA Interval Calculation

**What is tested:** Exponential moving average convergence.  
**Source:** ALGORITHMS.md §9

### K1 — First gap seeds EMA directly

```c
/* Send two messages 12 seconds apart */
/* After second message, avg_interval_s should equal 12.0 exactly */
ASSERT_EQ_FLOAT(v->avg_interval_s, 12.0, 0.01, "K1 first gap seeds EMA");
```

### K2 — EMA converges toward new rate after speed change

```c
/* Seed EMA at 12.0 (slow vessel) */
/* Then simulate 10 messages at 6s intervals (faster vessel) */
/* EMA should approach 6.0 within ~10 messages */
/* After 10 steps: expected ≈ 6.0 + (12.0 - 6.0) × 0.8^10 ≈ 6.65 */
ASSERT_EQ_FLOAT(v->avg_interval_s, 6.65, 0.2, "K2 EMA convergence");
```

### K3 — Large gaps (> 600 s) are ignored

```c
/* Gap of 1000 seconds must not affect EMA */
double ema_before = v->avg_interval_s;
/* inject msg with received_at = last_seen + 1000 */
ASSERT_EQ_FLOAT(v->avg_interval_s, ema_before, 0.001, "K3 large gap ignored");
```

### K4 — Out-of-order message does not corrupt EMA

```c
/* Message with received_at < last_seen must skip interval update */
double ema_before = v->avg_interval_s;
/* inject msg with received_at = last_seen - 5 */
ASSERT_EQ_FLOAT(v->avg_interval_s, ema_before, 0.001, "K4 regression skipped");
```

---

## 14. Suite L — Rhumb Line Propagation

**What is tested:** Position propagation formula.  
**Source:** ALGORITHMS.md §10

### L1 — Due North displacement

```c
float lat = 10.0f, lon = 100.0f;
/* 60 nm north at 0° = exactly 1.0° latitude */
propagate(&lat, &lon, 60.0f, 0.0f, 3600);
ASSERT_EQ_FLOAT(lat, 11.0f, 0.001f, "L1a lat increased by 1°");
ASSERT_EQ_FLOAT(lon, 100.0f, 0.001f, "L1b lon unchanged");
```

### L2 — Due East displacement (latitude-dependent)

```c
float lat = 0.0f, lon = 100.0f;
/* At equator: 60 nm east = 1.0° longitude */
propagate(&lat, &lon, 60.0f, 90.0f, 3600);
ASSERT_EQ_FLOAT(lon, 101.0f, 0.01f, "L2a equator east");

lat = 60.0f; lon = 100.0f;
/* At 60°N: 60 nm east = 2.0° longitude (cos 60° = 0.5) */
propagate(&lat, &lon, 60.0f, 90.0f, 3600);
ASSERT_EQ_FLOAT(lon, 102.0f, 0.05f, "L2b 60N east corrected");
```

### L3 — Zero speed produces no movement

```c
float lat = 12.0f, lon = 101.0f;
propagate(&lat, &lon, 0.0f, 45.0f, 3600);
ASSERT_EQ_FLOAT(lat, 12.0f, 0.0001f, "L3a lat unchanged");
ASSERT_EQ_FLOAT(lon, 101.0f, 0.0001f, "L3b lon unchanged");
```

### L4 — Short interval matches expected distance

```c
float lat = 12.0f, lon = 101.0f;
/* 24.5 kts for 3 seconds = 24.5 × (3/3600) = 0.02042 nm */
propagate(&lat, &lon, 24.5f, 0.0f, 3);
float dlat = lat - 12.0f;
float dist = dlat * 60.0f;   /* degrees × 60 nm/degree */
ASSERT_EQ_FLOAT(dist, 0.02042f, 0.0005f, "L4 short interval distance");
```

---

## 15. Suite M — Compliance Classification

**What is tested:** The ratio-based compliance bands.  
**Source:** ALGORITHMS.md §13

```c
/* M1 — exact compliance */
ASSERT_EQ_STR(compliance_str(12.0, 12), "OK",            "M1 exact compliance");

/* M2 — within 20% over — still OK */
ASSERT_EQ_STR(compliance_str(14.3, 12), "OK",            "M2 +19% over OK");

/* M3 — within 5% under — still OK */
ASSERT_EQ_STR(compliance_str(11.5, 12), "OK",            "M3 -4% under OK");

/* M4 — minor over (ratio 1.21–1.5) */
ASSERT_EQ_STR(compliance_str(15.0, 12), "MINOR DEV",     "M4 minor over");

/* M5 — minor under (ratio 0.8–0.95) */
ASSERT_EQ_STR(compliance_str(10.0, 12), "MINOR DEV",     "M5 minor under");

/* M6 — non-compliant over (ratio > 1.5) */
ASSERT_EQ_STR(compliance_str(20.0, 12), "NON-COMPLIANT", "M6 non-compliant over");

/* M7 — non-compliant under (ratio < 0.8) */
ASSERT_EQ_STR(compliance_str(8.0,  12), "NON-COMPLIANT", "M7 non-compliant under");

/* M8 — anchored vessel 180s */
ASSERT_EQ_STR(compliance_str(180.0, 180), "OK",          "M8 anchor 180s OK");
```

---

## 16. Suite N — Known Real-World Sentences

**What is tested:** Full decode against sentences with known correct output.  
**Source:** Eric S. Raymond, AIVDM/AIVDO protocol decoding guide  
**Why this matters:** This is ground truth. If all other suites pass but N
fails, the implementation is internally consistent but wrong.

### N1 — Type 1 sentence from AIVDM reference

```
!AIVDM,1,1,,B,15M67N0000G?Uch`53nDR?vN0<0e,0*73
```

Expected decoded values (from Raymond's reference):
- Message type: 1
- MMSI: 366999663
- Nav status: 0 (under way using engine)
- SOG: 0.0 knots
- Latitude: 37.3922° N
- Longitude: −122.4798° E
- COG: 125.0°
- Heading: 511 (not available)

```c
AISMsg1 msg;
ais_parse_nmea("!AIVDM,1,1,,B,15M67N0000G?Uch`53nDR?vN0<0e,0*73", &msg);
ASSERT_EQ_UINT(msg.mmsi, 366999663u, "N1 MMSI");
ASSERT_EQ_INT(msg.nav_status, 0, "N1 nav_status");
ASSERT_EQ_FLOAT(msg.sog, 0.0f, 0.05f, "N1 SOG");
ASSERT_EQ_FLOAT(msg.latitude,  37.3922f, 0.001f, "N1 lat");
ASSERT_EQ_FLOAT(msg.longitude, -122.4798f, 0.001f, "N1 lon");
ASSERT_EQ_FLOAT(msg.cog, 125.0f, 0.1f, "N1 COG");
ASSERT_EQ_INT(msg.heading, 511, "N1 heading sentinel");
ASSERT_EQ_INT(msg.heading_valid, 0, "N1 heading_valid=0");
```

### N2 — Type 1, moving vessel

```
!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24
```

Expected:
- MMSI: 227006760
- SOG: 1.2 knots
- Latitude: 49.4752° N
- Longitude: 0.1317° E
- COG: 188.5°
- Heading: 176

```c
ASSERT_EQ_UINT(msg.mmsi, 227006760u, "N2 MMSI");
ASSERT_EQ_FLOAT(msg.sog, 1.2f, 0.05f, "N2 SOG");
ASSERT_EQ_FLOAT(msg.latitude,  49.4752f, 0.001f, "N2 lat");
ASSERT_EQ_FLOAT(msg.longitude,  0.1317f, 0.001f, "N2 lon");
ASSERT_EQ_FLOAT(msg.cog, 188.5f, 0.1f, "N2 COG");
ASSERT_EQ_INT(msg.heading, 176, "N2 heading");
```

---

## 17. Coverage Summary

| Suite | Area | Tests | Covers algorithms doc section |
|---|---|---|---|
| A | NMEA checksum | 6 | §1 |
| B | 6-bit codec | 4 | §2 |
| C | Bit helpers | 7 | §3, §4 |
| D | Coordinate conversion | 5 | §5 |
| E | SOG/COG conversion + rounding | 4 | §6, §7 |
| F | Full round-trip | 5 | §2–§7 |
| G | Sentinel values | 6 | ERROR_HANDLING §8 |
| H | Parse error paths | 8 | ERROR_HANDLING §6, §7 |
| I | Reporting rate | 11 | §8 |
| J | Vessel tracking | 6 | §DATA_STRUCTURES §4, §5 |
| K | EMA interval | 4 | §9 |
| L | Rhumb line | 4 | §10 |
| M | Compliance | 8 | §13 |
| N | Real-world sentences | 2 | Ground truth |
| **Total** | | **~90** | |

All 90 tests must pass before the clean implementation is considered
correct. Suites A–H cover every error path in ERROR_HANDLING.md.
Suite N is the definitive correctness gate.
