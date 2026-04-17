#define _POSIX_C_SOURCE 199309L   /* nanosleep, signal */

#include "ais.h"
#include "vessel.h"
#include "output.h"
#include "error.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#define SAMPLE_LOG   "sample.nmea"
#define MAP_HTML     "map.html"
#define SIM_DURATION 600   /* seconds */

/* ── Signal handler ──────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_interrupted = 0;
static void handle_sigint(int sig) { (void)sig; g_interrupted = 1; }

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

#define N_VESSELS 30

static const SampleVessel VESSELS[N_VESSELS] = {
    /*  MMSI       Name                   type nav  lat        lon        sog    cog    hdg
        IMO       callsign   destination    draught bow stern port stbd */

    /* ── Cargo ──────────────────────────────────────────────────────────────── */
    { 567001001, "THAI STAR 1",          71, 0, 13.0000f, 100.8500f, 12.0f, 185.0f, 185,
      9164263, "HSTH1",  "LAEM CHABANG",  72, 140, 35, 18, 12 },
    { 525001001, "JAKARTA MARU",         71, 0, 10.8333f, 102.2000f, 24.5f, 320.0f, 320,
      9451823, "YBMR1",  "JAKARTA",       72, 160, 45, 20, 15 },
    { 566001001, "SINGAPORE GLORY",      71, 0, 11.5000f, 101.5000f, 20.0f,  15.0f,  15,
      9382910, "9VSG1",  "BANGKOK",       68, 150, 40, 18, 14 },
    { 477201001, "HARBOUR STAR",         72, 0, 12.2000f, 100.6000f, 12.0f, 355.0f, 355,
      9275148, "VRHS1",  "SI RACHA",      55, 120, 30, 15, 10 },
    { 431501001, "YOKOHAMA TRADER",      79, 0, 12.8000f, 101.1000f, 15.0f, 210.0f, 210,
      9520374, "JPTY1",  "LAEM CHABANG",  62, 170, 42, 20, 16 },
    { 533001001, "KUALA MAJU",           70, 0, 11.0000f, 101.8000f, 10.0f,  45.0f,  45,
      9387621, "9MKM1",  "PENANG",        48, 100, 28, 12,  9 },
    { 440001001, "BUSAN STAR",           79, 0, 11.0000f, 100.5000f, 18.0f,  35.0f,  35,
      9482015, "DTBS1",  "BUSAN",         60, 155, 40, 18, 14 },
    { 413001001, "XIAMEN PACIFIC",       71, 0, 10.5000f, 101.0000f, 16.0f,  20.0f,  22,
      9503718, "BXPC1",  "XIAMEN",        65, 165, 42, 20, 16 },

    /* ── Tankers ─────────────────────────────────────────────────────────────── */
    { 567001003, "GULF PIONEER",         81, 0, 12.3000f, 101.2000f,  5.5f, 270.0f, 270,
      9312504, "HSGP3",  "RAYONG",        40,  80, 25, 12,  8 },
    { 566002001, "SEA ANCHOR",           81, 1, 12.2500f, 101.2500f,  0.0f,   0.0f, 511,
               0, "9VSA2",  "",           15,  45, 12,  8,  6 },
    { 567002001, "SIAM SPIRIT",          83, 4, 11.5000f, 100.7500f,  4.5f, 168.0f, 170,
      9298145, "HSSS1",  "SRIRACHA",     112, 180, 50, 22, 18 },
    { 533002001, "PETRO MALAY",          80, 0, 10.5000f, 103.1000f, 11.0f, 290.0f, 292,
      9401237, "9MPM1",  "SINGAPORE",     85, 190, 55, 24, 20 },
    { 574001001, "VIET PETROL",          84, 0, 10.2000f, 104.0000f,  9.0f, 245.0f, 248,
      9356892, "XVVP1",  "VUNG TAU",      90, 200, 55, 26, 22 },

    /* ── Passenger ───────────────────────────────────────────────────────────── */
    { 567001002, "PATTAYA EXPRESS",      61, 0, 12.7500f, 100.9500f, 18.0f, 180.0f, 180,
      9238147, "HSPX2",  "PATTAYA",       30,  60, 20, 10,  8 },
    { 567003001, "KOH CHANG FERRY",      61, 0, 11.9500f, 102.1500f, 22.0f,  90.0f,  90,
      9445182, "HSKF1",  "KOH CHANG",     28,  55, 18,  9,  7 },
    { 567004001, "THAI RIVIERA",         69, 0, 12.5000f, 100.8000f, 14.0f,  22.0f,  22,
      9502341, "HSTR1",  "PATTAYA",       45,  90, 25, 14, 10 },

    /* ── Fishing ─────────────────────────────────────────────────────────────── */
    { 567001004, "OCEAN HARVEST",        30, 7, 11.8333f, 101.4500f,  3.2f,  90.0f,  90,
               0, "HSOH4",  "SATTAHIP",  20,  30, 10,  5,  4 },
    { 567005001, "CHON BURI FISH 1",     30, 7, 11.6000f, 101.6000f,  2.5f, 135.0f, 130,
               0, "HSCB5",  "SATTAHIP",  18,  25,  8,  4,  3 },
    { 567005002, "RAYONG TRAWLER",       30, 5, 12.6500f, 101.5000f,  0.0f,   0.0f, 511,
               0, "HSRT5",  "RAYONG",    15,  20,  6,  3,  3 },

    /* ── Special Vessels ─────────────────────────────────────────────────────── */
    { 567006001, "PETCH TUG 1",          52, 3, 13.0500f, 100.9000f,  4.0f, 175.0f, 178,
               0, "HSPT6",  "LAEM CHABANG", 25, 15, 20,  6,  5 },
    { 567006002, "PILOT BOAT 3",         50, 0, 13.0800f, 100.9200f, 12.0f, 190.0f, 188,
               0, "HSCP6",  "LAEM CHABANG",  8, 12, 12,  4,  4 },
    { 567006003, "THAI SAR 1",           51, 0, 12.0000f, 100.6000f, 20.0f,  60.0f,  62,
               0, "HSTS6",  "",          10,  20, 15,  6,  5 },
    { 567007001, "SRIRACHA DREDGE",      33, 3, 13.1000f, 100.8000f,  1.5f,  90.0f,  95,
               0, "HSSD7",  "SI RACHA",  30,  50, 40, 12, 10 },

    /* ── High Speed Craft ────────────────────────────────────────────────────── */
    { 567008001, "SPEEDCAT 1",           40, 0, 12.9500f, 100.8800f, 32.0f, 165.0f, 165,
      9561023, "HSSC8",  "PATTAYA",      15,  40, 12,  6,  5 },

    /* ── Research / Survey ───────────────────────────────────────────────────── */
    { 566003001, "PACIFIC SURVEYOR",     90, 3, 11.2000f, 102.5000f,  3.0f,  45.0f, 511,
      9312890, "9VPS9",  "",             18,  35, 15,  8,  7 },

    /* ── Sailing ─────────────────────────────────────────────────────────────── */
    { 567009001, "PHUKET SAIL",          36, 8, 12.1000f, 101.0000f,  5.0f, 230.0f, 235,
               0, "HSPS9",  "KOH SAMUI",  5,   8,  5,  3,  3 },

    /* ── Offshore / Supply ───────────────────────────────────────────────────── */
    { 525002001, "NATUNA SUPPLY",        99, 0, 10.0000f, 104.0000f, 12.0f, 310.0f, 312,
               0, "YBNS9",  "NATUNA",    35,  50, 20,  8,  8 },
    { 567010001, "GULF ANCHOR TUG",      31, 0,  9.5000f, 101.5000f,  7.0f, 180.0f, 182,
               0, "HSAH9",  "SATTAHIP",  28,  35, 25,  8,  7 },

    /* ── Incident / Irregular ────────────────────────────────────────────────── */
    { 567011001, "NAM WAN",              70, 2, 11.5000f, 102.8000f,  0.5f,  90.0f, 511,
               0, "HSNW1",  "",          35,  55, 18, 10,  8 },   /* not under command */
    { 567012001, "CHAO PHRAYA 3",        70, 6, 12.4000f, 101.8000f,  0.0f,   0.0f, 145,
               0, "HSCP3",  "",          30,  40, 15,  8,  6 },   /* aground */
};

/* ── Sample log generator ────────────────────────────────────────────────── */

static int generate_sample_log(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "Cannot write '%s': %s\n", path, strerror(errno));
        return 1;
    }

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
        next_tx5[i] = 0;
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
                m5.eta_month   = 0;
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

/* speed_mult = 0  → instant replay (no sleep)
   speed_mult > 0  → real-time playback at that multiplier (uses T= timestamps)
   live_clock      → use time(NULL) for received_at when no T= present (stdin) */

#define DRAW_INTERVAL_S 2

static int replay_log(FILE *f, VesselTable *table, AisErrorCtx *ctx,
                      double speed_mult, int live_clock)
{
    char line[512];
    int  line_num   = 0;
    long last_sim_t = 0;

    time_t wall_start = time(NULL);
    double sim_start  = -1.0;
    time_t last_draw  = 0;

    while (fgets(line, sizeof(line), f) && !g_interrupted) {
        line_num++;

        int len = (int)strlen(line);
        if (len > 0 && line[len - 1] == '\n') line[len - 1] = '\0';

        ctx->sentences_read++;

        /* Parse optional T=NNN prefix */
        int     ts_present = 0;
        time_t  ts         = live_clock ? time(NULL) : 0;
        const char *nmea   = line;

        if (strncmp(line, "T=", 2) == 0) {
            char *space = strchr(line, ' ');
            if (space) {
                *space     = '\0';
                long sim_t = atol(line + 2);
                ts         = (time_t)sim_t;
                *space     = ' ';
                nmea       = space + 1;
                ts_present = 1;
                last_sim_t = sim_t;

                /* Real-time pacing: sleep until this sentence's scheduled time */
                if (speed_mult > 0.0) {
                    if (sim_start < 0.0) sim_start = (double)sim_t;
                    double target  = (sim_t - sim_start) / speed_mult;
                    double elapsed = difftime(time(NULL), wall_start);
                    double delay   = target - elapsed;
                    if (delay > 0.0) {
                        struct timespec req;
                        req.tv_sec  = (time_t)delay;
                        req.tv_nsec = (long)((delay - (double)req.tv_sec) * 1e9);
                        nanosleep(&req, NULL);
                    }
                }
            }
        }

        /* Route to the right parser */
        const char *comma   = strchr(nmea, ',');
        int         is_multi = (comma && atoi(comma + 1) == 2);

        if (is_multi) {
            AISMsg5 msg5;
            AisStatus st5 = ais_parse_nmea5(nmea, &msg5);
            if (st5 == AIS_OK) {
                msg5.received_at = ts;
                ctx->sentences_ok++;
                vessel_update_static(table, &msg5, ctx);
            } else if (st5 == AIS_FRAG_PENDING) {
                ctx->sentences_ok++;
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

        /* Periodic display refresh during live / real-time playback */
        if (speed_mult > 0.0 || live_clock) {
            time_t now = time(NULL);
            if (now - last_draw >= DRAW_INTERVAL_S) {
                printf("\033[2J\033[H");
                if (ts_present)
                    printf("[ sim t=%lds  |  real %.0fs elapsed ]\n",
                           last_sim_t, difftime(now, wall_start));
                else
                    printf("[ real %.0fs elapsed ]\n",
                           difftime(now, wall_start));
                output_terminal(table);
                last_draw = now;
            }
        }
    }

    if (!g_interrupted && ferror(f)) {
        ais_error_log(ctx, AIS_ERR_IO, line_num, "Read error: %s",
                      strerror(errno));
        return 1;
    }

    return 0;
}

/* ── Usage ───────────────────────────────────────────────────────────────── */

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s                    Simulate and render (default)\n"
        "  %s --live             Real-time playback at 10x speed\n"
        "  %s --live --speed N   Real-time playback at Nx speed\n"
        "  %s --stdin            Read raw NMEA from stdin\n"
        "  %s --file <path>      Replay a saved NMEA log\n",
        prog, prog, prog, prog, prog);
}

/* ── Main ────────────────────────────────────────────────────────────────── */

typedef enum { MODE_SIMULATE, MODE_LIVE, MODE_STDIN, MODE_FILE } InputMode;

int main(int argc, char **argv)
{
    InputMode   mode       = MODE_SIMULATE;
    double      speed_mult = 10.0;
    const char *file_path  = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--live") == 0) {
            mode = MODE_LIVE;
        } else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
            speed_mult = atof(argv[++i]);
            if (speed_mult <= 0.0) {
                fprintf(stderr, "Error: --speed must be > 0\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--stdin") == 0) {
            mode = MODE_STDIN;
        } else if (strcmp(argv[i], "--file") == 0 && i + 1 < argc) {
            mode      = MODE_FILE;
            file_path = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    VesselTable table;
    vessel_table_init(&table);
    AisErrorCtx ctx;
    ais_error_init(&ctx);

    int rc = 0;

    if (mode == MODE_SIMULATE) {
        if (generate_sample_log(SAMPLE_LOG) != 0) return 1;
        FILE *f = fopen(SAMPLE_LOG, "r");
        if (!f) { fprintf(stderr, "Cannot open '%s': %s\n",
                          SAMPLE_LOG, strerror(errno)); return 1; }
        rc = replay_log(f, &table, &ctx, 0.0, 0);
        fclose(f);

    } else if (mode == MODE_LIVE) {
        if (generate_sample_log(SAMPLE_LOG) != 0) return 1;
        printf("Live playback at %.0fx speed — Ctrl+C to stop and render\n\n",
               speed_mult);
        signal(SIGINT, handle_sigint);
        FILE *f = fopen(SAMPLE_LOG, "r");
        if (!f) { fprintf(stderr, "Cannot open '%s': %s\n",
                          SAMPLE_LOG, strerror(errno)); return 1; }
        rc = replay_log(f, &table, &ctx, speed_mult, 0);
        fclose(f);

    } else if (mode == MODE_STDIN) {
        printf("Reading from stdin — Ctrl+C to stop and render\n\n");
        signal(SIGINT, handle_sigint);
        rc = replay_log(stdin, &table, &ctx, 0.0, 1);

    } else {   /* MODE_FILE */
        FILE *f = fopen(file_path, "r");
        if (!f) { fprintf(stderr, "Cannot open '%s': %s\n",
                          file_path, strerror(errno)); return 1; }
        rc = replay_log(f, &table, &ctx, 0.0, 0);
        fclose(f);
    }

    if (rc != 0) return rc;

    /* Final render — always runs, even after Ctrl+C */
    printf("\033[2J\033[H");
    output_terminal(&table);

    AisStatus st = output_html(&table, MAP_HTML);
    if (st == AIS_OK)
        printf("Map written to %s\n", MAP_HTML);
    else
        fprintf(stderr, "Warning: could not write %s\n", MAP_HTML);

    ais_error_summary(&ctx);
    return 0;
}
