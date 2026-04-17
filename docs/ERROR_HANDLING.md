# AIS Demo — Error Handling Strategy

**Project:** `ais_demo` (clean implementation)  
**Applies to:** All modules — `ais.c`, `vessel.c`, `output.c`, `main.c`

This document defines how every category of error is detected, reported, and
recovered from in the new implementation. It must be read before writing any
code. All design decisions here take precedence over the prototype.

---

## Table of Contents

1. [Error Philosophy](#1-error-philosophy)
2. [Error Categories](#2-error-categories)
3. [Return Code Convention](#3-return-code-convention)
4. [Error Reporting Mechanism](#4-error-reporting-mechanism)
5. [Category 1 — I/O Errors](#5-category-1--io-errors)
6. [Category 2 — NMEA Parse Errors](#6-category-2--nmea-parse-errors)
7. [Category 3 — AIS Decode Errors](#7-category-3--ais-decode-errors)
8. [Category 4 — Sentinel / Not-Available Values](#8-category-4--sentinel--not-available-values)
9. [Category 5 — Resource Exhaustion](#9-category-5--resource-exhaustion)
10. [Category 6 — Data Quality Warnings](#10-category-6--data-quality-warnings)
11. [What the Prototype Did Wrong](#11-what-the-prototype-did-wrong)
12. [Error Handling Decision Table](#12-error-handling-decision-table)

---

## 1. Error Philosophy

Three principles govern every error handling decision in this project:

**1. Never silently discard data.**
The prototype returned `0` on a bad checksum and moved on without logging
anything. In a real AIS feed, bad sentences are common — noise, partial reads,
collisions. Silent discard makes it impossible to diagnose whether a vessel
is missing because it left the area or because its messages are being dropped
by a parser bug. Every rejected sentence must be counted and categorised.

**2. Distinguish fatal from recoverable.**
A missing input file is fatal — there is nothing to process. A single bad NMEA
sentence is recoverable — skip it and continue. A full vessel table is
recoverable for new vessels — log a warning and continue tracking existing
ones. The code must never `exit()` from inside a library function
(`ais.c`, `vessel.c`). Only `main.c` may terminate the program.

**3. Sentinel values are not errors.**
The AIS protocol defines explicit "not available" values for every field
(latitude 91°, longitude 181°, SOG 102.3, heading 511, etc.). These are valid
protocol states, not decode failures. A vessel reporting heading 511 has no
heading sensor — that is information, not corruption. The new implementation
must store and propagate sentinel values rather than treating them as errors.

---

## 2. Error Categories

All errors in this project fall into six categories:

| # | Category | Examples | Fatal? |
|---|---|---|---|
| 1 | I/O | File not found, read failure, write failure | Depends |
| 2 | NMEA parse | Bad checksum, wrong sentence type, missing fields | No |
| 3 | AIS decode | Unsupported message type, truncated payload | No |
| 4 | Sentinel values | lat=91°, heading=511, SOG=102.3 | Never |
| 5 | Resource exhaustion | Vessel table full, history buffer full | No |
| 6 | Data quality | Implausible speed, future timestamp, duplicate MMSI | No |

---

## 3. Return Code Convention

Every function that can fail must return a status code. The project uses a
single `AisStatus` enum defined in a new `error.h` header, imported by all
modules.

```c
/* error.h */
typedef enum {
    AIS_OK               = 0,   /* success                                */
    AIS_ERR_IO           = 1,   /* file open / read / write failed        */
    AIS_ERR_CHECKSUM     = 2,   /* NMEA XOR checksum mismatch             */
    AIS_ERR_MALFORMED    = 3,   /* sentence missing required fields       */
    AIS_ERR_UNSUPPORTED  = 4,   /* message type not handled               */
    AIS_ERR_TRUNCATED    = 5,   /* payload too short for message type     */
    AIS_ERR_TABLE_FULL   = 6,   /* vessel table has no free slots         */
    AIS_ERR_NULL         = 7,   /* NULL pointer passed to function        */
} AisStatus;

/* Human-readable string for logging */
const char *ais_status_str(AisStatus s);
```

### Rules for return codes

- Functions that fill a struct return `AisStatus` and write to an
  out-parameter: `AisStatus ais_parse_nmea(const char *line, AISMsg1 *out)`
- Functions that return a pointer return `NULL` on failure. The caller checks
  for `NULL` before use.
- `void` functions are only permitted when failure is impossible
  (e.g. `vessel_table_init`).
- No function returns a raw `int` with ad-hoc 0/1 meanings. Every integer
  return is either `AisStatus` or a count of items processed.

### Why an enum over plain int?

A function returning `int` where `0 = success, -1 = error` gives the caller
no information about *what* went wrong. An `AisStatus` enum makes every failure
mode explicit, enables exhaustive `switch` handling, and makes log messages
self-documenting.

---

## 4. Error Reporting Mechanism

### The error context

A new `AisErrorCtx` struct accumulates statistics across the lifetime of a
processing session. It is allocated in `main.c` and passed into functions that
may produce errors.

```c
/* error.h */
typedef struct {
    /* Counters per error category */
    int sentences_read;
    int sentences_ok;
    int err_checksum;
    int err_malformed;
    int err_unsupported;
    int err_truncated;
    int err_table_full;
    int warn_sentinel;
    int warn_quality;

    /* Last error detail (overwritten on each new error) */
    AisStatus   last_status;
    char        last_msg[128];
    int         last_line;
} AisErrorCtx;

void ais_error_init(AisErrorCtx *ctx);
void ais_error_log(AisErrorCtx *ctx, AisStatus s,
                   int line_num, const char *fmt, ...);
void ais_error_summary(const AisErrorCtx *ctx);
```

### Logging behaviour

`ais_error_log()` does two things:

1. Increments the relevant counter in `ctx`
2. If `AIS_VERBOSE` is defined at compile time, prints the error to `stderr`
   with line number and category

In normal (non-verbose) mode, individual errors are silent. Only the summary
is printed at the end of the run. This keeps the terminal output clean during
a normal demo, while allowing `make verbose` to expose every dropped sentence
for debugging.

```bash
make                    # silent error accumulation
make CFLAGS=-DAIS_VERBOSE   # print every bad sentence to stderr
```

### Why not `fprintf(stderr, ...)` directly in each function?

Scattering `fprintf` calls through library functions has two problems:

1. The functions become impure — they have side effects (I/O) even when
   the caller doesn't want them (e.g. in a unit test).
2. There is no way to aggregate statistics without the caller counting
   return codes manually.

The `AisErrorCtx` separates the concerns: library functions report *what*
happened via `AisStatus`; the context decides *how* to report it.

---

## 5. Category 1 — I/O Errors

### Input file

```c
AisStatus ais_open_log(const char *path, FILE **out);
```

**Failure:** `fopen()` returns NULL.  
**Action:** Return `AIS_ERR_IO`. Log `strerror(errno)` via error context.  
**Recovery:** Fatal in the main program — no file means nothing to process.
`main.c` prints a usage message and exits with code 1.

```c
/* main.c pattern */
FILE *f;
if (ais_open_log(path, &f) != AIS_OK) {
    fprintf(stderr, "Cannot open '%s': %s\n", path, strerror(errno));
    return 1;
}
```

### Mid-stream read failure

`fgets()` returning NULL mid-loop can mean EOF (normal) or a read error.
Distinguish them:

```c
while (fgets(line, sizeof(line), f)) {
    /* process line */
}
if (ferror(f)) {
    ais_error_log(ctx, AIS_ERR_IO, line_num, "Read error: %s", strerror(errno));
}
/* feof(f) is normal — no error */
```

### Output file (HTML map)

**Failure:** `fopen()` for write returns NULL.  
**Action:** Log warning, continue. The terminal output is still valid.
The map is a "best effort" output — its failure should not abort the session.

---

## 6. Category 2 — NMEA Parse Errors

These are the most common errors in real-world AIS feeds.

### 2a — Bad checksum

```
!AIVDM,1,1,,A,15M67N...,0*FF   ← computed CS is 0x73, not 0xFF
```

**Detection:** `nmea_cs(s) != parsed_hex`  
**Action:** Return `AIS_ERR_CHECKSUM`. Increment `ctx->err_checksum`.  
**Recovery:** Skip this sentence entirely. Do not attempt to decode a
sentence with a bad checksum — the payload bits may be corrupted anywhere.

**Important:** A checksum failure on a line that does not begin with `!AIVDM`
or `!AIVDO` is expected (comment lines, Type 5 sentences not yet supported).
The parser must check the sentence type *before* the checksum to avoid
logging spurious checksum errors for intentionally skipped sentences.

### 2b — Missing or insufficient fields

```
!AIVDM,1,1,,A*73    ← payload field missing entirely
!AIVDM,1,1*73       ← truncated mid-sentence
```

**Detection:** Field count after splitting < 7, or `fields[5]` is empty.  
**Action:** Return `AIS_ERR_MALFORMED`. Increment `ctx->err_malformed`.  
**Recovery:** Skip.

### 2c — Empty payload

```
!AIVDM,1,1,,A,,0*XX   ← payload is empty string
```

**Detection:** `strlen(fields[5]) == 0`  
**Action:** Return `AIS_ERR_MALFORMED`.  
**Recovery:** Skip.

### 2d — The strtok empty-field bug (fixed in clean version)

The prototype used `strtok()` which silently skips consecutive delimiters.
The NMEA sequential message ID field (field [3]) is legitimately empty for
single-fragment messages, producing `,,` in the sentence. The clean version
uses a manual field splitter throughout:

```c
/* Correct approach — always */
static int split_fields(char *buf, char **fields, int max_fields) {
    int n = 0;
    char *p = buf;
    while (n < max_fields) {
        fields[n++] = p;
        char *q = p;
        while (*q && *q != ',' && *q != '*') q++;
        if (*q == '\0') break;
        *q = '\0';
        p = q + 1;
    }
    return n;
}
```

This is not optional — `strtok` must not appear anywhere in the clean
implementation.

---

## 7. Category 3 — AIS Decode Errors

### 3a — Unsupported message type

The clean version handles Types 1, 2, 3 (position reports). Type 5
(static/voyage data) will be added in a later phase.

**Detection:** `get_uint(bits, 0, 6)` returns a value outside {1, 2, 3}.  
**Action:** Return `AIS_ERR_UNSUPPORTED`. Increment `ctx->err_unsupported`.  
**Note:** This is not a data error — Type 5 and other types are valid AIS.
The error reflects a limitation of the current decoder, not a problem with
the input.

### 3b — Truncated payload

A Type 1/2/3 message requires exactly 28 payload characters (168 bits).
A shorter payload means the sentence was truncated in transmission.

**Detection:** `strlen(payload) < 28`  
**Action:** Return `AIS_ERR_TRUNCATED`. Increment `ctx->err_truncated`.  
**Recovery:** Skip. Do not attempt to decode a partial message — fields
at the end of the bit array would read as zero, producing plausible but
wrong values (e.g. lat = 0°, lon = 0° — the Gulf of Guinea, off the coast
of West Africa).

**Why "plausible but wrong" is dangerous:**

The prototype decoded whatever was in the bit array without checking payload
length. A truncated payload that happens to pass the checksum (possible if
only the fill bits were corrupted) would silently produce a vessel at 0°N,
0°E. In a display system, this looks like a real vessel in the Atlantic.
Length validation prevents this.

---

## 8. Category 4 — Sentinel / Not-Available Values

These are **not errors**. They must be stored in the decoded struct and
propagated to the output layer, which decides how to display them.

| Field | Sentinel value | Meaning |
|---|---|---|
| Latitude | 91.0° | No position fix available |
| Longitude | 181.0° | No position fix available |
| SOG | 102.3 knots (raw 1023) | SOG not available |
| COG | 360.0° (raw 3600) | COG not available |
| Heading | 511 | No heading sensor |
| ROT | −128 | No ROT sensor / not available |
| Timestamp | 60 | No electronic fix |
| Timestamp | 61 | Manual position entry |
| Timestamp | 62 | Dead reckoning estimate |
| Timestamp | 63 | Positioning system inoperative |

### Storage rule

`AISMsg1` stores the decoded float values as-is, including sentinels.
The `AisMsg1` struct gains boolean validity flags:

```c
typedef struct {
    /* ... existing fields ... */

    /* Validity flags — 1 = field contains real data, 0 = sentinel */
    uint8_t  pos_valid;      /* lat and lon are real */
    uint8_t  sog_valid;
    uint8_t  cog_valid;
    uint8_t  heading_valid;
    uint8_t  rot_valid;
} AISMsg1;
```

These flags are set during decode:

```c
msg->latitude   = get_int(bits, 89, 27) / 600000.0f;
msg->pos_valid  = (msg->latitude  < 91.0f && msg->longitude < 181.0f) ? 1 : 0;
msg->sog        = get_uint(bits, 50, 10) / 10.0f;
msg->sog_valid  = (msg->sog < 102.3f) ? 1 : 0;
```

### Display rule

Output functions check validity flags before rendering:

```c
/* Terminal output */
if (v->pos_valid)
    printf("%.4f%c", fabsf(v->lat), v->lat >= 0 ? 'N' : 'S');
else
    printf("N/A       ");

/* HTML map — do not emit marker if position unknown */
if (!v->pos_valid) continue;
```

### Why not just check for sentinel values at the display layer?

Checking `lat == 91.0f` everywhere in the output code is error-prone —
float comparison is unreliable, and the check has to be duplicated in every
output path. Centralising the validity check in the decoder and propagating
the flag is cleaner and safer.

---

## 9. Category 5 — Resource Exhaustion

### 9a — Vessel table full

**Detection:** `t->count >= MAX_VESSELS` in `vessel_find_or_create()`  
**Action:** Return `NULL`. Log `AIS_ERR_TABLE_FULL` via error context.  
**Recovery:** Continue processing. Messages for known vessels are still
updated. Only new vessels that would require a new slot are dropped.

The caller (`vessel_update`) must handle the NULL return:

```c
Vessel *v = vessel_find_or_create(t, msg->mmsi);
if (!v) {
    ais_error_log(ctx, AIS_ERR_TABLE_FULL, 0,
                  "Table full — dropped MMSI %u", msg->mmsi);
    return AIS_ERR_TABLE_FULL;
}
```

**Why not realloc?**

The clean implementation keeps the static array for the same reasons as the
prototype — no fragmentation, no lifecycle management, predictable memory.
For a showcase demo with a known vessel count, `MAX_VESSELS = 256` is
a reasonable ceiling. Raise it if needed — no other code changes required.

### 9b — History buffer full

**Detection:** `v->history_len >= MAX_HISTORY`  
**Action:** Switch to circular buffer mode — overwrite the oldest fix.
This is the upgrade from the prototype which simply stopped recording.

```c
/* Clean implementation — circular history */
int slot = v->history_len % MAX_HISTORY;
v->history[slot] = fix;
if (v->history_len < MAX_HISTORY)
    v->history_len++;
else
    v->history_head = (v->history_head + 1) % MAX_HISTORY;
```

This requires two index fields: `history_len` (total fixes received, capped
at MAX_HISTORY) and `history_head` (index of the oldest fix when the buffer
is full). The output layer iterates from `history_head` to produce the trail
in chronological order.

**No error is logged** — history overflow is an expected operational state
for long-running sessions, not a problem.

---

## 10. Category 6 — Data Quality Warnings

These are conditions where the data is technically valid (checksum passes,
fields decode) but the values are implausible. They indicate sensor problems
on the transmitting vessel. They are logged as warnings, not errors — the
data is stored and displayed, but flagged.

### 6a — Implausible speed

SOG > 50 knots for a non-HSC vessel type. Most cargo ships and tankers
cannot exceed 25 knots.

```c
if (msg->sog_valid && msg->sog > 50.0f && v->ship_type / 10 != 4)
    ais_error_log(ctx, AIS_OK, 0,
                  "WARN implausible SOG %.1f kts for MMSI %u",
                  msg->sog, msg->mmsi);
```

Note `AIS_OK` is passed — this increments `warn_quality` without counting
as a decode failure.

### 6b — Position jump

If a vessel's new position is more than `SOG × elapsed_time × 3` nautical
miles from the last known position, the fix is likely from a different vessel
or a GPS glitch.

```c
float expected_max_dist = v->sog * (gap_s / 3600.0f) * 3.0f;
float actual_dist = haversine(v->lat, v->lon, msg->latitude, msg->longitude);
if (actual_dist > expected_max_dist && expected_max_dist > 0.1f)
    ais_error_log(ctx, AIS_OK, 0,
                  "WARN position jump %.1f nm for MMSI %u", actual_dist, msg->mmsi);
```

The factor of 3 gives generous tolerance for slow vessels and vessels that
just appeared in the tracking area.

### 6c — Timestamp regression

If a message's `received_at` is earlier than `v->last_seen`, the log is
out of order. The data is still stored, but the interval calculation is
skipped for this message to avoid a negative gap corrupting the EMA.

```c
if (msg->received_at < v->last_seen) {
    ais_error_log(ctx, AIS_OK, 0,
                  "WARN out-of-order message for MMSI %u", msg->mmsi);
    /* skip interval update, still update position */
}
```

---

## 11. What the Prototype Did Wrong

A direct list of the specific error handling gaps in the prototype that the
clean implementation must fix:

| Issue | Prototype behaviour | Clean behaviour |
|---|---|---|
| Bad checksum | Return 0, no log | Return `AIS_ERR_CHECKSUM`, increment counter |
| Unsupported type | Return 0, no log | Return `AIS_ERR_UNSUPPORTED`, increment counter |
| Truncated payload | Decode garbage, no error | Check length, return `AIS_ERR_TRUNCATED` |
| Sentinel lat/lon | Stored and displayed as `0.0000N 0.0000E` | Detected, flagged, displayed as `N/A` |
| Sentinel heading 511 | Displayed as `511` | Displayed as `N/A` |
| History full | Silently stopped recording | Circular buffer — oldest fix overwritten |
| Vessel table full | Silently returned NULL | Logged, caller informed |
| `strtok` empty field bug | Fixed in prototype via manual splitter | Never use `strtok` — enforced |
| No error summary | Silent | `ais_error_summary()` always printed at end |
| `exit()` in library code | Not present but not prohibited | Explicitly prohibited |

---

## 12. Error Handling Decision Table

Quick reference for every error condition, usable during implementation:

| Condition | Function | Return | Log | Continue? |
|---|---|---|---|---|
| File not found | `ais_open_log` | `AIS_ERR_IO` | Yes | No — fatal |
| Mid-stream read error | `replay_log` loop | `AIS_ERR_IO` | Yes | No — fatal |
| HTML write failed | `output_html` | `AIS_ERR_IO` | Yes | Yes |
| Checksum mismatch | `ais_parse_nmea` | `AIS_ERR_CHECKSUM` | Counted | Yes |
| Wrong sentence type | `ais_parse_nmea` | `AIS_ERR_UNSUPPORTED` | Counted | Yes |
| Field count < 7 | `ais_parse_nmea` | `AIS_ERR_MALFORMED` | Counted | Yes |
| Empty payload | `ais_parse_nmea` | `AIS_ERR_MALFORMED` | Counted | Yes |
| Payload length < 28 | `ais_decode_msg1` | `AIS_ERR_TRUNCATED` | Counted | Yes |
| Unsupported msg type | `ais_decode_msg1` | `AIS_ERR_UNSUPPORTED` | Counted | Yes |
| Sentinel lat/lon | decode | `AIS_OK` (flag set) | No | Yes |
| Sentinel SOG/COG/HDG | decode | `AIS_OK` (flag set) | No | Yes |
| Vessel table full | `vessel_find_or_create` | `NULL` | Yes | Yes |
| History buffer full | `vessel_update` | — | No | Yes (circular) |
| Implausible SOG | `vessel_update` | `AIS_OK` | Warn | Yes |
| Position jump | `vessel_update` | `AIS_OK` | Warn | Yes |
| Out-of-order message | `vessel_update` | `AIS_OK` | Warn | Yes (no EMA) |
| NULL pointer argument | any function | `AIS_ERR_NULL` | Yes | Depends |
