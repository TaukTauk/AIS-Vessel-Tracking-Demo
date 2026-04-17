#ifndef VESSEL_H
#define VESSEL_H

#include <stdint.h>
#include <time.h>
#include "error.h"
#include "ais.h"

#define MAX_HISTORY  200
#define MAX_VESSELS   64

/* ── Single historical position snapshot ────────────────────────────────── */

typedef struct {
    float  lat, lon;
    float  sog, cog;
    time_t t;
} VesselFix;

/* ── Full vessel record ──────────────────────────────────────────────────── */

typedef struct {
    uint32_t mmsi;
    char     name[21];
    uint8_t  ship_type;

    float    lat, lon;
    float    sog, cog;
    uint16_t heading;
    uint8_t  nav_status;

    /* validity flags mirrored from the latest AISMsg1 */
    uint8_t  pos_valid;
    uint8_t  sog_valid;
    uint8_t  cog_valid;
    uint8_t  heading_valid;

    /* Type 5 static / voyage data (populated when a Type 5 message is decoded) */
    uint32_t imo_number;
    char     call_sign[8];
    char     destination[21];
    uint8_t  eta_month, eta_day, eta_hour, eta_min;
    uint8_t  draught_raw;
    uint16_t dim_bow, dim_stern;
    uint8_t  dim_port, dim_stbd;
    uint8_t  static_valid;   /* 1 once a Type 5 message has been received */

    time_t   first_seen;
    time_t   last_seen;
    int      update_count;

    int      expected_interval_s;
    double   avg_interval_s;

    VesselFix history[MAX_HISTORY];
    int       history_len;   /* count of valid entries, capped at MAX_HISTORY */
    int       history_head;  /* index of oldest fix when buffer is full */
} Vessel;

/* ── Vessel table ────────────────────────────────────────────────────────── */

typedef struct {
    Vessel vessels[MAX_VESSELS];
    int    count;
} VesselTable;

/* ── API ─────────────────────────────────────────────────────────────────── */

void     vessel_table_init(VesselTable *t);
Vessel  *vessel_find(VesselTable *t, uint32_t mmsi);
Vessel  *vessel_find_or_create(VesselTable *t, uint32_t mmsi);
AisStatus vessel_update(VesselTable *t, const AISMsg1 *msg, AisErrorCtx *ctx);

/* Set vessel name and ship_type from static data (legacy helper). */
void      vessel_set_static(VesselTable *t, uint32_t mmsi,
                             const char *name, uint8_t ship_type);

/* Update vessel with decoded Type 5 static/voyage data. */
AisStatus vessel_update_static(VesselTable *t, const AISMsg5 *msg,
                                AisErrorCtx *ctx);

/* Compliance classification — returns a static string. */
const char *compliance_str(double actual_s, int expected_s);

#endif /* VESSEL_H */
