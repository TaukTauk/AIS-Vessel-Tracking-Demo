#ifndef AIS_H
#define AIS_H

#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include "error.h"

/* ── Decoded Type 1/2/3 position report ─────────────────────────────────── */

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

    /* validity flags — 1 = real data, 0 = sentinel */
    uint8_t  pos_valid;
    uint8_t  sog_valid;
    uint8_t  cog_valid;
    uint8_t  heading_valid;
    uint8_t  rot_valid;
} AISMsg1;

/* ── Decoded Type 5 static / voyage data ────────────────────────────────── */

typedef struct {
    uint8_t  msg_type;       /* 5 */
    uint8_t  repeat;
    uint32_t mmsi;
    uint8_t  ais_version;
    uint32_t imo_number;
    char     call_sign[8];   /* 7 chars + null */
    char     name[21];       /* 20 chars + null */
    uint8_t  ship_type;
    uint16_t dim_bow;        /* metres forward of GPS */
    uint16_t dim_stern;      /* metres aft of GPS */
    uint8_t  dim_port;       /* metres port of GPS */
    uint8_t  dim_stbd;       /* metres starboard of GPS */
    uint8_t  epfd;           /* position fix type */
    uint8_t  eta_month;      /* 1–12; 0 = N/A */
    uint8_t  eta_day;        /* 1–31; 0 = N/A */
    uint8_t  eta_hour;       /* 0–23; 24 = N/A */
    uint8_t  eta_min;        /* 0–59; 60 = N/A */
    uint8_t  draught_raw;    /* 1/10 metres; 0 = N/A */
    char     destination[21]; /* 20 chars + null */
    uint8_t  dte;
    time_t   received_at;
} AISMsg5;

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Parse one !AIVDM line (no T= prefix) into *out. */
AisStatus ais_parse_nmea(const char *line, AISMsg1 *out);

/* Encode a Type 1 message into an NMEA sentence (no T= prefix, with \n). */
AisStatus ais_encode_msg1(char *out, size_t out_size,
                          uint32_t mmsi, uint8_t nav_status,
                          float sog, float lat, float lon,
                          float cog, uint16_t heading, uint8_t ts);

/* Encode Type 5 into two NMEA sentences (seq_id 1–9, with \n). */
AisStatus ais_encode_msg5(char *frag1, size_t f1_size,
                           char *frag2, size_t f2_size,
                           uint8_t seq_id, const AISMsg5 *msg);

/* Parse Type 5 fragments.  Returns AIS_FRAG_PENDING after the first.
   Returns AIS_OK (filling *out) when the second fragment arrives. */
AisStatus ais_parse_nmea5(const char *line, AISMsg5 *out);

/* IMO MSC.74(69) Table 1 — reporting interval in seconds. */
int ais_reporting_interval(float sog, uint8_t nav_status);

/* Human-readable nav status string (index 0–15). */
const char *ais_nav_status_str(uint8_t nav_status);

/* ── Internal helpers exposed for testing ───────────────────────────────── */

uint8_t  nmea_cs(const char *s);
int      nmea_valid(const char *s);

char     encode_6bit(uint8_t v);
uint8_t  decode_6bit(char c);

void     bits_to_payload(const uint8_t *bits, int n_chars, char *out);
void     payload_to_bits(const char *payload, int fill_bits, uint8_t *bits);

uint32_t get_uint(const uint8_t *bits, int start, int len);
void     set_uint(uint8_t *bits, int start, int len, uint32_t val);
int32_t  get_int(const uint8_t *bits, int start, int len);
void     set_int(uint8_t *bits, int start, int len, int32_t val);

/* Rhumb-line position propagation (used by main.c and tests). */
void     propagate(float *lat, float *lon, float sog, float cog, int dt_sec);

#endif /* AIS_H */
