#include "ais.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── NMEA checksum ───────────────────────────────────────────────────────── */

uint8_t nmea_cs(const char *s)
{
    uint8_t cs = 0;
    const char *p = s + 1;      /* skip leading '!' */
    while (*p && *p != '*')
        cs ^= (uint8_t)*p++;
    return cs;
}

int nmea_valid(const char *s)
{
    const char *star = strchr(s, '*');
    if (!star || strlen(star) < 3) return 0;
    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    return nmea_cs(s) == expected;
}

/* ── 6-bit ASCII codec ───────────────────────────────────────────────────── */

char encode_6bit(uint8_t v)
{
    v &= 0x3Fu;
    v += 48u;
    if (v > 87u) v += 8u;
    return (char)v;
}

uint8_t decode_6bit(char c)
{
    uint8_t v = (uint8_t)c - 48u;
    if (v >= 40u) v -= 8u;
    return v & 0x3Fu;
}

void bits_to_payload(const uint8_t *bits, int n_chars, char *out)
{
    for (int i = 0; i < n_chars; i++) {
        uint8_t v = 0;
        for (int b = 0; b < 6; b++)
            v = (uint8_t)((v << 1) | bits[i * 6 + b]);
        out[i] = encode_6bit(v);
    }
    out[n_chars] = '\0';
}

void payload_to_bits(const char *payload, int fill_bits, uint8_t *bits)
{
    int n = (int)strlen(payload);
    for (int i = 0; i < n; i++) {
        uint8_t v = decode_6bit(payload[i]);
        for (int b = 5; b >= 0; b--)
            bits[i * 6 + (5 - b)] = (v >> b) & 1u;
    }
    (void)fill_bits;
}

/* ── Bit array helpers ───────────────────────────────────────────────────── */

uint32_t get_uint(const uint8_t *bits, int start, int len)
{
    uint32_t result = 0;
    for (int i = 0; i < len; i++)
        result = (result << 1) | bits[start + i];
    return result;
}

void set_uint(uint8_t *bits, int start, int len, uint32_t val)
{
    for (int i = len - 1; i >= 0; i--) {
        bits[start + i] = val & 1u;
        val >>= 1;
    }
}

int32_t get_int(const uint8_t *bits, int start, int len)
{
    uint32_t raw = get_uint(bits, start, len);
    if (len < 32 && (raw & (1u << (len - 1))))
        raw |= ~((1u << len) - 1u);
    return (int32_t)raw;
}

void set_int(uint8_t *bits, int start, int len, int32_t val)
{
    uint32_t mask = (len < 32) ? ((1u << len) - 1u) : 0xFFFFFFFFu;
    set_uint(bits, start, len, (uint32_t)val & mask);
}

/* ── NMEA field splitter (never strtok) ──────────────────────────────────── */

static int split_fields(char *buf, char **fields, int max_fields)
{
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

/* ── 6-bit text codec (ship name / call sign / destination) ─────────────── */

static void encode_text(uint8_t *bits, int start, int n_chars, const char *text)
{
    int slen = (int)strlen(text);
    for (int i = 0; i < n_chars; i++) {
        char c = (i < slen) ? text[i] : '@';
        if (c >= 'a' && c <= 'z') c = (char)(c - 32);  /* upper-case */
        uint8_t v;
        if      (c >= '@' && c <= '_') v = (uint8_t)(c - 64); /* 0–31 */
        else if (c >= ' ' && c <= '?') v = (uint8_t)c;        /* 32–63 */
        else                            v = 0;
        set_uint(bits, start + i * 6, 6, v);
    }
}

static void decode_text(const uint8_t *bits, int start, int n_chars, char *out)
{
    for (int i = 0; i < n_chars; i++) {
        uint8_t v = (uint8_t)get_uint(bits, start + i * 6, 6);
        out[i] = (v < 32) ? (char)(v + 64) : (char)v;
    }
    int len = n_chars;
    while (len > 0 && (out[len-1] == '@' || out[len-1] == ' ')) len--;
    out[len] = '\0';
}

/* ── Type 5 fragment reassembly buffer ──────────────────────────────────── */

typedef struct { int active; char seq_id; char payload[128]; } FragSlot;
#define FRAG_BUF_SIZE 8
static FragSlot s_frags[FRAG_BUF_SIZE];

/* ── Encoder ─────────────────────────────────────────────────────────────── */

AisStatus ais_encode_msg1(char *out, size_t out_size,
                          uint32_t mmsi, uint8_t nav_status,
                          float sog, float lat, float lon,
                          float cog, uint16_t heading, uint8_t ts)
{
    if (!out) return AIS_ERR_NULL;

    uint8_t bits[168] = {0};

    set_uint(bits,   0,  6, 1u);                              /* msg type */
    set_uint(bits,   6,  2, 0u);                              /* repeat */
    set_uint(bits,   8, 30, mmsi);
    set_uint(bits,  38,  4, nav_status & 0xFu);
    set_int (bits,  42,  8, -128);                            /* ROT N/A */
    set_uint(bits,  50, 10, (uint32_t)(sog * 10.0f + 0.5f));
    set_uint(bits,  60,  1, 0u);                              /* pos_accuracy */
    set_int (bits,  61, 28, (int32_t)(lon * 600000.0f));
    set_int (bits,  89, 27, (int32_t)(lat * 600000.0f));
    set_uint(bits, 116, 12, (uint32_t)(cog * 10.0f + 0.5f));
    set_uint(bits, 128,  9, heading);
    set_uint(bits, 137,  6, (uint32_t)(ts % 60u));
    /* bits 143–167: maneuver, spare, RAIM, radio — all 0 */

    char payload[29];
    bits_to_payload(bits, 28, payload);

    /* build core string, then append checksum */
    char core[128];
    snprintf(core, sizeof(core), "!AIVDM,1,1,,A,%s,0", payload);
    snprintf(out, out_size, "%s*%02X\n", core, nmea_cs(core));

    return AIS_OK;
}

/* ── Decoder ─────────────────────────────────────────────────────────────── */

AisStatus ais_parse_nmea(const char *line, AISMsg1 *out)
{
    if (!line) return AIS_ERR_NULL;
    if (!out)  return AIS_ERR_NULL;

    /* Only handle !AIVDM / !AIVDO sentences */
    if (strncmp(line, "!AIVDM", 6) != 0 && strncmp(line, "!AIVDO", 6) != 0)
        return AIS_ERR_UNSUPPORTED;

    /* Work on a mutable copy for field splitting */
    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[10];
    int nf = split_fields(buf, fields, 10);

    if (nf < 7)                    return AIS_ERR_MALFORMED;
    if (strlen(fields[5]) == 0)    return AIS_ERR_MALFORMED;
    if (strlen(fields[5]) < 28)    return AIS_ERR_TRUNCATED;
    if (!nmea_valid(line))         return AIS_ERR_CHECKSUM;

    /* Decode payload → bit array */
    int fill = atoi(fields[6]);
    uint8_t bits[168] = {0};
    payload_to_bits(fields[5], fill, bits);

    /* Check message type */
    uint8_t msg_type = (uint8_t)get_uint(bits, 0, 6);
    if (msg_type < 1 || msg_type > 3) return AIS_ERR_UNSUPPORTED;

    /* Extract fields */
    out->msg_type    = msg_type;
    out->repeat      = (uint8_t)get_uint(bits,  6,  2);
    out->mmsi        = get_uint(bits,  8, 30);
    out->nav_status  = (uint8_t)get_uint(bits, 38,  4);
    out->rot         = (int8_t) get_int (bits, 42,  8);
    out->sog         = get_uint(bits, 50, 10) / 10.0f;
    out->pos_accuracy= (uint8_t)get_uint(bits, 60,  1);
    out->longitude   = get_int (bits, 61, 28) / 600000.0f;
    out->latitude    = get_int (bits, 89, 27) / 600000.0f;
    out->cog         = get_uint(bits,116, 12) / 10.0f;
    out->heading     = (uint16_t)get_uint(bits,128,  9);
    out->timestamp   = (uint8_t) get_uint(bits,137,  6);
    out->raim        = (uint8_t) get_uint(bits,148,  1);
    out->received_at = 0;

    /* Validity flags */
    out->pos_valid     = (out->latitude  < 91.0f  && out->longitude < 181.0f) ? 1 : 0;
    out->sog_valid     = (out->sog       < 102.3f) ? 1 : 0;
    out->cog_valid     = (out->cog       < 360.0f) ? 1 : 0;
    out->heading_valid = (out->heading   < 511)    ? 1 : 0;
    out->rot_valid     = (out->rot       != -128)  ? 1 : 0;

    return AIS_OK;
}

/* ── Type 5 encoder ──────────────────────────────────────────────────────── */

AisStatus ais_encode_msg5(char *frag1, size_t f1_size,
                           char *frag2, size_t f2_size,
                           uint8_t seq_id, const AISMsg5 *msg)
{
    if (!frag1 || !frag2 || !msg) return AIS_ERR_NULL;

    /* 424 data bits + 2 fill = 426 bits; use 432-element array (zero-padded) */
    uint8_t bits[432] = {0};

    set_uint(bits,   0,  6, 5u);                     /* msg type */
    set_uint(bits,   6,  2, msg->repeat & 0x3u);
    set_uint(bits,   8, 30, msg->mmsi);
    set_uint(bits,  38,  2, 0u);                     /* AIS version */
    set_uint(bits,  40, 30, msg->imo_number);
    encode_text(bits,  70, 7, msg->call_sign);
    encode_text(bits, 112, 20, msg->name);
    set_uint(bits, 232,  8, msg->ship_type);
    set_uint(bits, 240,  9, msg->dim_bow);
    set_uint(bits, 249,  9, msg->dim_stern);
    set_uint(bits, 258,  6, msg->dim_port);
    set_uint(bits, 264,  6, msg->dim_stbd);
    set_uint(bits, 270,  4, msg->epfd);
    set_uint(bits, 274,  4, msg->eta_month);
    set_uint(bits, 278,  5, msg->eta_day);
    set_uint(bits, 283,  5, msg->eta_hour);
    set_uint(bits, 288,  6, msg->eta_min);
    set_uint(bits, 294,  8, msg->draught_raw);
    encode_text(bits, 302, 20, msg->destination);
    set_uint(bits, 422,  1, msg->dte & 0x1u);
    /* bit 423: spare = 0 */
    /* bits 424–425: fill = 0 */

    char p1[64], p2[16];
    bits_to_payload(bits,       60, p1);   /* fragment 1: 60 chars, 360 bits */
    bits_to_payload(bits + 360, 11, p2);   /* fragment 2: 11 chars, 66 bits (2 fill) */

    char core1[128], core2[128];
    snprintf(core1, sizeof(core1), "!AIVDM,2,1,%u,A,%s,0", (unsigned)seq_id, p1);
    snprintf(core2, sizeof(core2), "!AIVDM,2,2,%u,A,%s,2", (unsigned)seq_id, p2);
    snprintf(frag1, f1_size, "%s*%02X\n", core1, nmea_cs(core1));
    snprintf(frag2, f2_size, "%s*%02X\n", core2, nmea_cs(core2));

    return AIS_OK;
}

/* ── Type 5 decoder (fragment reassembly) ────────────────────────────────── */

AisStatus ais_parse_nmea5(const char *line, AISMsg5 *out)
{
    if (!line) return AIS_ERR_NULL;
    if (!out)  return AIS_ERR_NULL;

    if (strncmp(line, "!AIVDM", 6) != 0 && strncmp(line, "!AIVDO", 6) != 0)
        return AIS_ERR_UNSUPPORTED;

    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *fields[10];
    int nf = split_fields(buf, fields, 10);

    if (nf < 7)                 return AIS_ERR_MALFORMED;
    if (strlen(fields[5]) == 0) return AIS_ERR_MALFORMED;
    if (!nmea_valid(line))      return AIS_ERR_CHECKSUM;

    int total   = atoi(fields[1]);
    int frag_no = atoi(fields[2]);
    char seq_id = fields[3][0];          /* '1'–'9' */
    int fill    = atoi(fields[6]);

    if (total != 2) return AIS_ERR_UNSUPPORTED;

    if (frag_no == 1) {
        /* Find a free or matching slot */
        int slot = 0;
        for (int i = 0; i < FRAG_BUF_SIZE; i++) {
            if (!s_frags[i].active)          { slot = i; break; }
            if (s_frags[i].seq_id == seq_id) { slot = i; break; }
        }
        s_frags[slot].active = 1;
        s_frags[slot].seq_id = seq_id;
        strncpy(s_frags[slot].payload, fields[5],
                sizeof(s_frags[slot].payload) - 1);
        s_frags[slot].payload[sizeof(s_frags[slot].payload) - 1] = '\0';
        return AIS_FRAG_PENDING;
    }

    if (frag_no == 2) {
        int slot = -1;
        for (int i = 0; i < FRAG_BUF_SIZE; i++) {
            if (s_frags[i].active && s_frags[i].seq_id == seq_id) {
                slot = i; break;
            }
        }
        if (slot < 0) return AIS_ERR_MALFORMED;   /* no matching frag 1 */

        char combined[256];
        snprintf(combined, sizeof(combined), "%s%s",
                 s_frags[slot].payload, fields[5]);
        s_frags[slot].active = 0;

        if ((int)strlen(combined) < 71) return AIS_ERR_TRUNCATED;

        uint8_t bits[432] = {0};
        payload_to_bits(combined, fill, bits);

        uint8_t msg_type = (uint8_t)get_uint(bits, 0, 6);
        if (msg_type != 5) return AIS_ERR_UNSUPPORTED;

        out->msg_type    = msg_type;
        out->repeat      = (uint8_t)get_uint(bits,  6,  2);
        out->mmsi        = get_uint(bits,  8, 30);
        out->ais_version = (uint8_t)get_uint(bits, 38,  2);
        out->imo_number  = get_uint(bits, 40, 30);
        decode_text(bits,  70,  7, out->call_sign);
        decode_text(bits, 112, 20, out->name);
        out->ship_type   = (uint8_t)get_uint(bits, 232,  8);
        out->dim_bow     = (uint16_t)get_uint(bits, 240, 9);
        out->dim_stern   = (uint16_t)get_uint(bits, 249, 9);
        out->dim_port    = (uint8_t)get_uint(bits, 258,  6);
        out->dim_stbd    = (uint8_t)get_uint(bits, 264,  6);
        out->epfd        = (uint8_t)get_uint(bits, 270,  4);
        out->eta_month   = (uint8_t)get_uint(bits, 274,  4);
        out->eta_day     = (uint8_t)get_uint(bits, 278,  5);
        out->eta_hour    = (uint8_t)get_uint(bits, 283,  5);
        out->eta_min     = (uint8_t)get_uint(bits, 288,  6);
        out->draught_raw = (uint8_t)get_uint(bits, 294,  8);
        decode_text(bits, 302, 20, out->destination);
        out->dte         = (uint8_t)get_uint(bits, 422,  1);
        out->received_at = 0;

        return AIS_OK;
    }

    return AIS_ERR_MALFORMED;
}

/* ── Reporting rate (IMO MSC.74(69) Table 1) ─────────────────────────────── */

int ais_reporting_interval(float sog, uint8_t nav_status)
{
    if (nav_status == 1 || nav_status == 5) return 180;
    if (sog < 0.1f)   return 180;
    if (sog <= 14.0f) return 12;
    if (sog <= 23.0f) return 6;
    return 3;
}

/* ── Nav status strings ──────────────────────────────────────────────────── */

const char *ais_nav_status_str(uint8_t nav_status)
{
    static const char *names[] = {
        "Underway (engine)", "At anchor", "Not under command",
        "Restricted manoeuvrability", "Constrained by draught",
        "Moored", "Aground", "Engaged in fishing",
        "Under way sailing", "Reserved (9)", "Reserved (10)",
        "Reserved (11)", "Reserved (12)", "Reserved (13)",
        "AIS-SART", "Not defined"
    };
    if (nav_status > 15) return "Unknown";
    return names[nav_status];
}

/* ── Rhumb line propagation ──────────────────────────────────────────────── */

void propagate(float *lat, float *lon, float sog, float cog, int dt_sec)
{
    if (sog < 0.01f) return;

    float cog_rad = cog * (float)M_PI / 180.0f;
    float lat_rad = *lat * (float)M_PI / 180.0f;
    float dt_hr   = dt_sec / 3600.0f;
    float dist_nm = sog * dt_hr;

    *lat += dist_nm * cosf(cog_rad) / 60.0f;
    *lon += dist_nm * sinf(cog_rad) / (60.0f * cosf(lat_rad));
}

/* ── Error context helpers ───────────────────────────────────────────────── */

const char *ais_status_str(AisStatus s)
{
    switch (s) {
        case AIS_OK:              return "OK";
        case AIS_ERR_IO:          return "I/O error";
        case AIS_ERR_CHECKSUM:    return "bad checksum";
        case AIS_ERR_MALFORMED:   return "malformed sentence";
        case AIS_ERR_UNSUPPORTED: return "unsupported message type";
        case AIS_ERR_TRUNCATED:   return "truncated payload";
        case AIS_ERR_TABLE_FULL:  return "vessel table full";
        case AIS_ERR_NULL:        return "null pointer";
        case AIS_FRAG_PENDING:    return "fragment pending";
        default:                  return "unknown";
    }
}

void ais_error_init(AisErrorCtx *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
}

void ais_error_log(AisErrorCtx *ctx, AisStatus s,
                   int line_num, const char *fmt, ...)
{
    if (!ctx) return;

    ctx->last_status = s;
    ctx->last_line   = line_num;

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ctx->last_msg, sizeof(ctx->last_msg), fmt, ap);
    va_end(ap);

    switch (s) {
        case AIS_ERR_CHECKSUM:    ctx->err_checksum++;    break;
        case AIS_ERR_MALFORMED:   ctx->err_malformed++;   break;
        case AIS_ERR_UNSUPPORTED: ctx->err_unsupported++; break;
        case AIS_ERR_TRUNCATED:   ctx->err_truncated++;   break;
        case AIS_ERR_TABLE_FULL:  ctx->err_table_full++;  break;
        case AIS_OK:              ctx->warn_quality++;    break;
        default:                  break;
    }

#ifdef AIS_VERBOSE
    fprintf(stderr, "[line %d] %s: %s\n",
            line_num, ais_status_str(s), ctx->last_msg);
#endif
}

void ais_error_summary(const AisErrorCtx *ctx)
{
    printf("\n--- Parse summary ---\n");
    printf("  Sentences read : %d\n", ctx->sentences_read);
    printf("  OK             : %d\n", ctx->sentences_ok);
    printf("  Bad checksum   : %d\n", ctx->err_checksum);
    printf("  Malformed      : %d\n", ctx->err_malformed);
    printf("  Unsupported    : %d\n", ctx->err_unsupported);
    printf("  Truncated      : %d\n", ctx->err_truncated);
    printf("  Table full     : %d\n", ctx->err_table_full);
    printf("  Quality warns  : %d\n", ctx->warn_quality);
}
