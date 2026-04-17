#include "ais.h"
#include "vessel.h"
#include "output.h"
#include "error.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define SAMPLE_LOG   "sample.nmea"
#define MAP_HTML     "map.html"
#define SIM_DURATION 600   /* seconds */

/* ── Simulation seed data ────────────────────────────────────────────────── */

typedef struct {
    uint32_t    mmsi;
    const char *name;
    uint8_t     ship_type;
    uint8_t     nav_status;
    float       lat, lon;
    float       sog, cog;
    uint16_t    heading;
    /* Type 5 static / voyage data */
    uint32_t    imo_number;
    const char *call_sign;
    const char *destination;
    uint8_t     draught_raw;   /* 1/10 metres */
    uint16_t    dim_bow, dim_stern;
    uint8_t     dim_port, dim_stbd;
} SampleVessel;

#define N_VESSELS 6

static const SampleVessel VESSELS[N_VESSELS] = {
    /*  MMSI       Name              type nav  lat       lon       sog   cog    hdg
        IMO       callsign   destination      draught bow stern port stbd */
    { 567001001, "THAI STAR 1",     71, 0, 12.5833f, 100.8833f,  8.5f,  45.0f,  45,
      9164263, "HSTH1",  "LAEM CHABANG", 55, 120, 30, 15, 10 },
    { 567001002, "PATTAYA EXPRESS", 61, 0, 12.7500f, 100.9500f, 18.0f, 180.0f, 180,
      9238147, "HSPX2",  "PATTAYA",      30,  60, 20, 10,  8 },
    { 567001003, "GULF PIONEER",    81, 0, 12.3000f, 101.2000f,  5.5f, 270.0f, 270,
      9312504, "HSGP3",  "RAYONG",       40,  80, 25, 12,  8 },
    { 567001004, "OCEAN HARVEST",   30, 7, 11.8333f, 101.4500f,  3.2f,  90.0f,  90,
             0, "HSOH4",  "SATTAHIP",    20,  30, 10,  5,  4 },
    { 525001001, "JAKARTA MARU",    71, 0, 10.8333f, 102.2000f, 24.5f, 320.0f, 320,
      9451823, "YBMR1",  "JAKARTA",      72, 160, 45, 20, 15 },
    { 566002001, "SEA ANCHOR",      81, 1, 12.2500f, 101.2500f,  0.0f,   0.0f, 511,
             0, "9VSA2",  "",            15,  45, 12,  8,  6 },
};

/* ── Sample log generator ────────────────────────────────────────────────── */

static int generate_sample_log(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "Cannot write '%s': %s\n", path, strerror(errno));
        return 1;
    }

    /* Mutable state for each vessel */
    float lat[N_VESSELS], lon[N_VESSELS], sog[N_VESSELS], cog[N_VESSELS];
    uint8_t nav[N_VESSELS];
    uint16_t hdg[N_VESSELS];
    int next_tx[N_VESSELS], next_tx5[N_VESSELS];

    for (int i = 0; i < N_VESSELS; i++) {
        lat[i]      = VESSELS[i].lat;
        lon[i]      = VESSELS[i].lon;
        sog[i]      = VESSELS[i].sog;
        cog[i]      = VESSELS[i].cog;
        nav[i]      = VESSELS[i].nav_status;
        hdg[i]      = VESSELS[i].heading;
        next_tx[i]  = 0;
        next_tx5[i] = 0;  /* Type 5 first at t=0 */
    }

    char sentence[256];
    int total = 0;

    for (int t = 0; t <= SIM_DURATION; t++) {
        for (int i = 0; i < N_VESSELS; i++) {

            /* ── Type 5 (static/voyage) — every 6 minutes ── */
            if (t == next_tx5[i]) {
                AISMsg5 m5;
                memset(&m5, 0, sizeof(m5));
                m5.mmsi        = VESSELS[i].mmsi;
                m5.imo_number  = VESSELS[i].imo_number;
                strncpy(m5.call_sign,   VESSELS[i].call_sign,   sizeof(m5.call_sign)   - 1);
                strncpy(m5.name,        VESSELS[i].name,        sizeof(m5.name)        - 1);
                strncpy(m5.destination, VESSELS[i].destination, sizeof(m5.destination) - 1);
                m5.ship_type   = VESSELS[i].ship_type;
                m5.dim_bow     = VESSELS[i].dim_bow;
                m5.dim_stern   = VESSELS[i].dim_stern;
                m5.dim_port    = VESSELS[i].dim_port;
                m5.dim_stbd    = VESSELS[i].dim_stbd;
                m5.draught_raw = VESSELS[i].draught_raw;
                m5.eta_month   = 0;   /* N/A */
                m5.eta_day     = 0;
                m5.eta_hour    = 24;
                m5.eta_min     = 60;

                char f1[256], f2[256];
                ais_encode_msg5(f1, sizeof(f1), f2, sizeof(f2),
                                (uint8_t)(1 + i % 9), &m5);

                for (int fi = 0; fi < 2; fi++) {
                    char *fr = (fi == 0) ? f1 : f2;
                    int flen = (int)strlen(fr);
                    if (flen > 0 && fr[flen - 1] == '\n') fr[flen - 1] = '\0';
                    fprintf(f, "T=%d %s\n", t, fr);
                    total++;
                }
                next_tx5[i] = t + 360;
            }

            /* ── Type 1 (position) ── */
            if (t < next_tx[i]) continue;

            int interval = ais_reporting_interval(sog[i], nav[i]);
            propagate(&lat[i], &lon[i], sog[i], cog[i], interval);

            ais_encode_msg1(sentence, sizeof(sentence),
                            VESSELS[i].mmsi, nav[i], sog[i],
                            lat[i], lon[i], cog[i], hdg[i],
                            (uint8_t)(t % 60));

            int slen = (int)strlen(sentence);
            if (slen > 0 && sentence[slen - 1] == '\n')
                sentence[slen - 1] = '\0';

            fprintf(f, "T=%d %s\n", t, sentence);
            total++;
            next_tx[i] = t + interval;
        }
    }

    fclose(f);
    printf("Generated %s — %d sentences\n", path, total);
    return 0;
}

/* ── Log replay ──────────────────────────────────────────────────────────── */

static int replay_log(const char *path, VesselTable *table, AisErrorCtx *ctx)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "Cannot open '%s': %s\n", path, strerror(errno));
        return 1;
    }

    char line[512];
    int line_num = 0;

    while (fgets(line, sizeof(line), f)) {
        line_num++;

        /* Strip newline */
        int len = (int)strlen(line);
        if (len > 0 && line[len - 1] == '\n') line[len - 1] = '\0';

        ctx->sentences_read++;

        /* Parse T=NNN prefix */
        time_t ts = 0;
        const char *nmea = line;

        if (strncmp(line, "T=", 2) == 0) {
            char *space = strchr(line, ' ');
            if (space) {
                *space = '\0';
                ts = (time_t)atoi(line + 2);
                *space = ' ';
                nmea = space + 1;
            }
        }

        /* Peek at field[1] (total fragments) to route to the right parser */
        const char *comma = strchr(nmea, ',');
        int is_multi = (comma && atoi(comma + 1) == 2);

        if (is_multi) {
            AISMsg5 msg5;
            AisStatus st5 = ais_parse_nmea5(nmea, &msg5);
            if (st5 == AIS_OK) {
                msg5.received_at = ts;
                ctx->sentences_ok++;
                vessel_update_static(table, &msg5, ctx);
            } else if (st5 == AIS_FRAG_PENDING) {
                ctx->sentences_ok++;   /* valid fragment, not yet a full message */
            } else {
                ais_error_log(ctx, st5, line_num, "%s", nmea);
            }
        } else {
            AISMsg1 msg;
            AisStatus st = ais_parse_nmea(nmea, &msg);
            if (st != AIS_OK) {
                ais_error_log(ctx, st, line_num, "%s", nmea);
                continue;
            }
            msg.received_at = ts;
            ctx->sentences_ok++;
            vessel_update(table, &msg, ctx);
        }
    }

    if (ferror(f)) {
        ais_error_log(ctx, AIS_ERR_IO, line_num, "Read error: %s",
                      strerror(errno));
        fclose(f);
        return 1;
    }

    fclose(f);
    return 0;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* 1. Generate sample NMEA log */
    if (generate_sample_log(SAMPLE_LOG) != 0)
        return 1;

    /* 2. Replay and build vessel table */
    VesselTable table;
    vessel_table_init(&table);

    AisErrorCtx ctx;
    ais_error_init(&ctx);

    if (replay_log(SAMPLE_LOG, &table, &ctx) != 0)
        return 1;

    /* 3. Output */
    output_terminal(&table);

    AisStatus st = output_html(&table, MAP_HTML);
    if (st == AIS_OK)
        printf("Map written to %s\n", MAP_HTML);
    else
        fprintf(stderr, "Warning: could not write %s\n", MAP_HTML);

    ais_error_summary(&ctx);
    return 0;
}
