/* tests/test_all.c — AIS demo test suite (~90 tests across 14 suites) */

#include "../src/error.h"
#include "../src/ais.h"
#include "../src/vessel.h"
#include "testlib.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ─────────────────────────────────────────────────────────────────────────
   Suite A — NMEA checksum
   ───────────────────────────────────────────────────────────────────────── */

static void suite_A(void)
{
    /* A1 — valid sentence accepted */
    ASSERT_EQ_INT(
        nmea_valid("!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*74"),
        1, "A1 valid sentence accepted");

    /* A2 — one character changed in payload */
    ASSERT_EQ_INT(
        nmea_valid("!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S9,0*73"),
        0, "A2 corrupted payload rejected");

    /* A3 — wrong checksum digits */
    ASSERT_EQ_INT(
        nmea_valid("!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*FF"),
        0, "A3 wrong checksum rejected");

    /* A4 — missing asterisk */
    ASSERT_EQ_INT(
        nmea_valid("!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,073"),
        0, "A4 missing asterisk rejected");

    /* A5 — all-zero payload checksum computed correctly
       XOR of "AIVDM,1,1,,A,000000000000000000000000000000,0" = 0x26 */
    ASSERT_EQ_INT(
        nmea_cs("!AIVDM,1,1,,A,000000000000000000000000000000,0"),
        0x26, "A5 all-zero payload checksum");

    /* A6 — encode then validate round-trip */
    {
        char sentence[128];
        ais_encode_msg1(sentence, sizeof(sentence),
                        567001001u, 0, 8.5f, 12.5f, 100.9f, 45.0f, 45, 30);
        /* strip trailing newline for nmea_valid */
        int n = (int)strlen(sentence);
        if (n > 0 && sentence[n-1] == '\n') sentence[n-1] = '\0';
        ASSERT_EQ_INT(nmea_valid(sentence), 1, "A6 encoded sentence passes checksum");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite B — 6-bit ASCII codec
   ───────────────────────────────────────────────────────────────────────── */

static void suite_B(void)
{
    /* B1 — boundary values encode correctly */
    ASSERT_EQ_INT(encode_6bit(0),  '0', "B1a value 0  → '0'");
    ASSERT_EQ_INT(encode_6bit(39), 'W', "B1b value 39 → 'W'");
    ASSERT_EQ_INT(encode_6bit(40), '`', "B1c value 40 → '`'");
    ASSERT_EQ_INT(encode_6bit(63), 'w', "B1d value 63 → 'w'");

    /* B2 — ASCII 88–95 never produced */
    int b2_ok = 1;
    for (int v = 0; v <= 63; v++) {
        int ascii = (unsigned char)encode_6bit((uint8_t)v);
        if (ascii >= 88 && ascii <= 95) {
            printf("  FAIL  B2 value %d produced forbidden ASCII %d\n", v, ascii);
            g_failures++;
            b2_ok = 0;
        }
    }
    if (b2_ok) { printf("  PASS  B2 gap never produced\n"); g_passes++; }

    /* B3 — decode is exact inverse of encode */
    int b3_ok = 1;
    for (int v = 0; v <= 63; v++) {
        char c    = encode_6bit((uint8_t)v);
        uint8_t b = decode_6bit(c);
        if (b != (uint8_t)v) {
            printf("  FAIL  B3 round-trip failed for value %d\n", v);
            g_failures++;
            b3_ok = 0;
        }
    }
    if (b3_ok) { printf("  PASS  B3 decode is inverse of encode\n"); g_passes++; }

    /* B4 — payload bit round-trip */
    {
        uint8_t bits_in[168], bits_out[168];
        /* set alternating pattern */
        for (int i = 0; i < 168; i++) bits_in[i] = (uint8_t)(i % 2);
        /* round-trip */
        char payload[29];
        bits_to_payload(bits_in, 28, payload);
        memset(bits_out, 0, sizeof(bits_out));
        payload_to_bits(payload, 0, bits_out);
        int b4_ok = 1;
        for (int i = 0; i < 168; i++) {
            if (bits_out[i] != bits_in[i]) {
                printf("  FAIL  B4 bit %d mismatch (in=%d out=%d)\n",
                       i, bits_in[i], bits_out[i]);
                g_failures++;
                b4_ok = 0;
                break;
            }
        }
        if (b4_ok) { printf("  PASS  B4 payload bit round-trip\n"); g_passes++; }
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite C — Bit array helpers
   ───────────────────────────────────────────────────────────────────────── */

static void suite_C(void)
{
    /* C1 — get_uint reads correct value */
    {
        uint8_t bits[32] = {0};
        bits[8]=1; bits[9]=0; bits[10]=1; bits[11]=0;
        ASSERT_EQ_UINT(get_uint(bits, 8, 4), 10u, "C1 get_uint basic");
    }

    /* C2 — set_uint / get_uint round-trip */
    {
        uint8_t bits[40] = {0};
        set_uint(bits, 5, 10, 567u);
        ASSERT_EQ_UINT(get_uint(bits, 5, 10), 567u, "C2 set/get_uint round-trip");
    }

    /* C3 — set_uint does not write outside its range */
    {
        uint8_t bits[40] = {0};
        bits[4] = 0xFF; bits[15] = 0xFF;
        set_uint(bits, 5, 10, 0x3FFu);
        ASSERT_EQ_INT(bits[4],  0xFF, "C3a lower boundary not touched");
        ASSERT_EQ_INT(bits[15], 0xFF, "C3b upper boundary not touched");
    }

    /* C4 — get_int sign extension: negative one */
    {
        uint8_t bits[32] = {0};
        for (int i = 0; i < 27; i++) bits[i] = 1;
        ASSERT_EQ_INT(get_int(bits, 0, 27), -1, "C4a get_int -1 (27 bits)");
    }

    /* C5 — get_int most negative 27-bit value */
    {
        uint8_t bits[32] = {0};
        bits[0] = 1;   /* 1000...0 = most negative */
        ASSERT_EQ_INT(get_int(bits, 0, 27), -(1 << 26), "C5 get_int min value");
    }

    /* C6 — set_int / get_int round-trip with negative latitude */
    {
        uint8_t bits[64] = {0};
        int32_t raw = (int32_t)(-10.5f * 600000.0f);
        set_int(bits, 10, 27, raw);
        ASSERT_EQ_INT(get_int(bits, 10, 27), raw, "C6 signed lat round-trip");
    }

    /* C7 — full MMSI range (30 bits, max 9 digits) */
    {
        uint8_t bits[64] = {0};
        set_uint(bits, 8, 30, 999999999u);
        ASSERT_EQ_UINT(get_uint(bits, 8, 30), 999999999u, "C7 max MMSI value");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite D — Coordinate conversion
   ───────────────────────────────────────────────────────────────────────── */

static void suite_D(void)
{
    /* D1 — positive latitude round-trip */
    {
        float lat_in  = 12.5833f;
        int32_t raw   = (int32_t)(lat_in * 600000.0f);
        float lat_out = raw / 600000.0f;
        ASSERT_EQ_FLOAT(lat_out, lat_in, 0.0001f, "D1 positive latitude round-trip");
    }

    /* D2 — negative latitude round-trip */
    {
        float lat_in  = -33.8688f;
        int32_t raw   = (int32_t)(lat_in * 600000.0f);
        float lat_out = raw / 600000.0f;
        ASSERT_EQ_FLOAT(lat_out, lat_in, 0.0001f, "D2 negative latitude round-trip");
    }

    /* D3 — maximum longitude raw value */
    {
        int32_t raw = (int32_t)(180.0f * 600000.0f);
        ASSERT_EQ_INT(raw, 108000000, "D3 max longitude raw value");
    }

    /* D4 — minimum latitude fits in 27-bit signed */
    {
        int32_t raw = (int32_t)(-90.0f * 600000.0f);
        ASSERT_EQ_INT(raw >= -(1 << 26), 1, "D4a min lat fits in 27 bits (lower)");
        ASSERT_EQ_INT(raw <= (1 << 26) - 1, 1, "D4b min lat fits in 27 bits (upper)");
    }

    /* D5 — full encode/decode via AIS message */
    {
        AISMsg1 msg;
        char sentence[128];
        float lat = -22.9068f, lon = -43.1729f;
        ais_encode_msg1(sentence, sizeof(sentence),
                        123456789u, 0, 10.0f, lat, lon, 180.0f, 180, 0);
        ais_parse_nmea(sentence, &msg);
        ASSERT_EQ_FLOAT(msg.latitude,  lat, 0.0002f, "D5a lat encode/decode");
        ASSERT_EQ_FLOAT(msg.longitude, lon, 0.0002f, "D5b lon encode/decode");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite E — SOG / COG conversion with rounding
   ───────────────────────────────────────────────────────────────────────── */

static void suite_E(void)
{
    /* E1 — SOG exact value */
    {
        AISMsg1 msg; char s[128];
        ais_encode_msg1(s, sizeof(s), 1u, 0, 8.5f, 0.0f, 0.0f, 0.0f, 0, 0);
        ais_parse_nmea(s, &msg);
        ASSERT_EQ_FLOAT(msg.sog, 8.5f, 0.05f, "E1 SOG 8.5 kts round-trip");
    }

    /* E2 — SOG rounds correctly (8.56 → 8.6, not 8.5) */
    {
        AISMsg1 msg; char s[128];
        ais_encode_msg1(s, sizeof(s), 1u, 0, 8.56f, 0.0f, 0.0f, 0.0f, 0, 0);
        ais_parse_nmea(s, &msg);
        ASSERT_EQ_FLOAT(msg.sog, 8.6f, 0.05f, "E2 SOG rounds correctly");
    }

    /* E3 — COG 359.9° */
    {
        AISMsg1 msg; char s[128];
        ais_encode_msg1(s, sizeof(s), 1u, 0, 5.0f, 0.0f, 0.0f, 359.9f, 0, 0);
        ais_parse_nmea(s, &msg);
        ASSERT_EQ_FLOAT(msg.cog, 359.9f, 0.05f, "E3 COG 359.9° round-trip");
    }

    /* E4 — COG 0.0° */
    {
        AISMsg1 msg; char s[128];
        ais_encode_msg1(s, sizeof(s), 1u, 0, 5.0f, 0.0f, 0.0f, 0.0f, 0, 0);
        ais_parse_nmea(s, &msg);
        ASSERT_EQ_FLOAT(msg.cog, 0.0f, 0.05f, "E4 COG 0.0° round-trip");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite F — Full message round-trip
   ───────────────────────────────────────────────────────────────────────── */

static void test_roundtrip(uint32_t mmsi, uint8_t nav, float sog,
                            float lat, float lon, float cog,
                            uint16_t hdg, uint8_t ts, const char *name)
{
    char sentence[128];
    AISMsg1 decoded;

    AisStatus enc = ais_encode_msg1(sentence, sizeof(sentence),
                                    mmsi, nav, sog, lat, lon, cog, hdg, ts);
    (void)enc;
    AisStatus st = ais_parse_nmea(sentence, &decoded);

    char tag[64];
    snprintf(tag, sizeof(tag), "%s status", name);
    ASSERT_EQ_STATUS(st, AIS_OK, tag);
    snprintf(tag, sizeof(tag), "%s MMSI", name);
    ASSERT_EQ_UINT(decoded.mmsi, mmsi, tag);
    snprintf(tag, sizeof(tag), "%s nav_status", name);
    ASSERT_EQ_INT(decoded.nav_status, nav, tag);
    snprintf(tag, sizeof(tag), "%s SOG", name);
    ASSERT_EQ_FLOAT(decoded.sog, sog, 0.05f, tag);
    snprintf(tag, sizeof(tag), "%s lat", name);
    ASSERT_EQ_FLOAT(decoded.latitude, lat, 0.0002f, tag);
    snprintf(tag, sizeof(tag), "%s lon", name);
    ASSERT_EQ_FLOAT(decoded.longitude, lon, 0.0002f, tag);
    snprintf(tag, sizeof(tag), "%s COG", name);
    ASSERT_EQ_FLOAT(decoded.cog, cog, 0.05f, tag);
    snprintf(tag, sizeof(tag), "%s heading", name);
    ASSERT_EQ_INT(decoded.heading, hdg, tag);
    snprintf(tag, sizeof(tag), "%s timestamp", name);
    ASSERT_EQ_INT(decoded.timestamp, ts % 60, tag);
}

static void suite_F(void)
{
    test_roundtrip(567001001u, 0,  8.5f,  12.5833f,  100.8833f,  45.0f,  45, 30, "F1a");
    test_roundtrip(503123456u, 0, 12.0f, -33.8688f,  151.2093f,  90.0f,  90, 15, "F1b");
    test_roundtrip(338123456u, 0, 18.0f,  40.7128f,  -74.0060f, 270.0f, 270, 45, "F1c");
    test_roundtrip(566002001u, 1,  0.0f,  12.2500f,  101.2500f,   0.0f, 511,  0, "F1d");
    test_roundtrip(525001001u, 0, 24.5f,  10.8333f,  102.2000f, 320.0f, 320,  0, "F1e");
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite G — Sentinel value handling
   ───────────────────────────────────────────────────────────────────────── */

/* Build a Type 1 message from raw bit values, bypassing the float encoder. */
static void decode_raw_bits(uint8_t *bits, AISMsg1 *msg)
{
    /* Wrap in a valid NMEA sentence */
    char payload[29], sentence[256];
    bits_to_payload(bits, 28, payload);
    char core[128];
    snprintf(core, sizeof(core), "!AIVDM,1,1,,A,%s,0", payload);
    snprintf(sentence, sizeof(sentence), "%s*%02X\n", core, nmea_cs(core));
    ais_parse_nmea(sentence, msg);
}

static void suite_G(void)
{
    uint8_t bits[168];
    AISMsg1 msg;

    /* Helper: start from a known-good set of bits */
    #define MAKE_VALID_BITS(b)                                   \
        memset((b), 0, 168);                                     \
        set_uint((b), 0, 6, 1u);           /* msg type 1 */     \
        set_uint((b), 8, 30, 566001001u);  /* MMSI */           \
        set_int ((b), 61, 28, (int32_t)(101.0f * 600000.0f));   \
        set_int ((b), 89, 27, (int32_t)(12.0f  * 600000.0f));   \
        set_uint((b), 50, 10, 80u);   /* SOG 8.0 */             \
        set_uint((b),(116),12, 900u); /* COG 90.0 */            \
        set_uint((b),(128), 9, 90u)   /* heading 90 */

    /* G1 — lat sentinel 91° → pos_valid=0 */
    MAKE_VALID_BITS(bits);
    set_int(bits, 89, 27, (int32_t)(91.0f * 600000.0f));
    decode_raw_bits(bits, &msg);
    ASSERT_EQ_INT(msg.pos_valid, 0, "G1 lat=91 sets pos_valid=0");

    /* G2 — lon sentinel 181° → pos_valid=0 */
    MAKE_VALID_BITS(bits);
    set_int(bits, 61, 28, (int32_t)(181.0f * 600000.0f));
    decode_raw_bits(bits, &msg);
    ASSERT_EQ_INT(msg.pos_valid, 0, "G2 lon=181 sets pos_valid=0");

    /* G3 — SOG sentinel 1023 → sog_valid=0 */
    MAKE_VALID_BITS(bits);
    set_uint(bits, 50, 10, 1023u);
    decode_raw_bits(bits, &msg);
    ASSERT_EQ_INT(msg.sog_valid, 0, "G3 SOG sentinel detected");

    /* G4 — heading sentinel 511 → heading_valid=0 */
    MAKE_VALID_BITS(bits);
    set_uint(bits, 128, 9, 511u);
    decode_raw_bits(bits, &msg);
    ASSERT_EQ_INT(msg.heading_valid, 0, "G4 heading=511 sentinel detected");

    /* G5 — COG sentinel 3600 → cog_valid=0 */
    MAKE_VALID_BITS(bits);
    set_uint(bits, 116, 12, 3600u);
    decode_raw_bits(bits, &msg);
    ASSERT_EQ_INT(msg.cog_valid, 0, "G5 COG=3600 sentinel detected");

    /* G6 — valid values set flags to 1 */
    MAKE_VALID_BITS(bits);
    decode_raw_bits(bits, &msg);
    ASSERT_EQ_INT(msg.pos_valid, 1, "G6a normal position is valid");
    ASSERT_EQ_INT(msg.sog_valid, 1, "G6b normal SOG is valid");

    #undef MAKE_VALID_BITS
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite H — NMEA parse error paths
   ───────────────────────────────────────────────────────────────────────── */

static void suite_H(void)
{
    AISMsg1 msg;

    /* H1 — bad checksum */
    ASSERT_EQ_STATUS(
        ais_parse_nmea("!AIVDM,1,1,,A,15M67N0000G?Uch`53nDR?vN00S8,0*FF", &msg),
        AIS_ERR_CHECKSUM, "H1 bad checksum");

    /* H2 — too few fields */
    ASSERT_EQ_STATUS(
        ais_parse_nmea("!AIVDM,1,1,,A*73", &msg),
        AIS_ERR_MALFORMED, "H2 missing fields");

    /* H3 — empty payload */
    ASSERT_EQ_STATUS(
        ais_parse_nmea("!AIVDM,1,1,,A,,0*26", &msg),
        AIS_ERR_MALFORMED, "H3 empty payload");

    /* H4 — payload shorter than 28 chars (bad checksum too; TRUNCATED wins) */
    ASSERT_EQ_STATUS(
        ais_parse_nmea("!AIVDM,1,1,,A,15M67N000,0*XX", &msg),
        AIS_ERR_TRUNCATED, "H4 truncated payload");

    /* H5 — unsupported message type (Type 5):
       Built by changing first payload char of A1 sentence from '1' to '5';
       checksum 0x74 XOR ('1' XOR '5') = 0x74 XOR 4 = 0x70 */
    ASSERT_EQ_STATUS(
        ais_parse_nmea("!AIVDM,1,1,,A,55M67N0000G?Uch`53nDR?vN00S8,0*70", &msg),
        AIS_ERR_UNSUPPORTED, "H5 type 5 not supported");

    /* H6 — NULL pointers */
    ASSERT_EQ_STATUS(ais_parse_nmea(NULL,  &msg), AIS_ERR_NULL, "H6a null line");
    ASSERT_EQ_STATUS(ais_parse_nmea("!AIVDM,1,1,,A,...", NULL), AIS_ERR_NULL,
                     "H6b null output");

    /* H7 — comment line treated as unsupported */
    ASSERT_EQ_STATUS(
        ais_parse_nmea("# This is a comment", &msg),
        AIS_ERR_UNSUPPORTED, "H7 comment line");

    /* H8 — empty seq ID field (strtok bug prevention)
       Using N1 sentence: MMSI should be 366053240 */
    {
        AisStatus st = ais_parse_nmea(
            "!AIVDM,1,1,,B,15M67N0000G?Uch`53nDR?vN0<0e,0*45", &msg);
        ASSERT_EQ_STATUS(st, AIS_OK, "H8a empty seq ID parses OK");
        ASSERT_EQ_UINT(msg.mmsi, 366053240u, "H8b correct MMSI after empty seq ID");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite I — IMO reporting rate
   ───────────────────────────────────────────────────────────────────────── */

static void suite_I(void)
{
    ASSERT_EQ_INT(ais_reporting_interval(0.0f,  1), 180, "I1a anchor 0 kts");
    ASSERT_EQ_INT(ais_reporting_interval(5.0f,  1), 180, "I1b anchor 5 kts");
    ASSERT_EQ_INT(ais_reporting_interval(0.0f,  5), 180, "I2 moored");
    ASSERT_EQ_INT(ais_reporting_interval(0.05f, 0), 180, "I3 near-zero speed");

    ASSERT_EQ_INT(ais_reporting_interval(1.0f,  0),  12, "I4a 1 kt");
    ASSERT_EQ_INT(ais_reporting_interval(8.5f,  0),  12, "I4b 8.5 kts");
    ASSERT_EQ_INT(ais_reporting_interval(14.0f, 0),  12, "I4c exactly 14 kts");

    ASSERT_EQ_INT(ais_reporting_interval(14.1f, 0),   6, "I5a just above 14 kts");
    ASSERT_EQ_INT(ais_reporting_interval(18.0f, 0),   6, "I5b 18 kts");
    ASSERT_EQ_INT(ais_reporting_interval(23.0f, 0),   6, "I5c exactly 23 kts");

    ASSERT_EQ_INT(ais_reporting_interval(23.1f, 0),   3, "I6a just above 23 kts");
    ASSERT_EQ_INT(ais_reporting_interval(24.5f, 0),   3, "I6b 24.5 kts");
    ASSERT_EQ_INT(ais_reporting_interval(50.0f, 0),   3, "I6c 50 kts");
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite J — Vessel state tracking
   ───────────────────────────────────────────────────────────────────────── */

static void suite_J(void)
{
    VesselTable t;
    AisErrorCtx ctx;
    vessel_table_init(&t);
    ais_error_init(&ctx);

    /* J1 — new vessel created on first message */
    ASSERT_EQ_INT(t.count, 0, "J1a table starts empty");
    Vessel *v = vessel_find_or_create(&t, 567001001u);
    ASSERT_NOT_NULL(v, "J1b vessel created");
    ASSERT_EQ_INT(t.count, 1, "J1c count incremented");
    ASSERT_EQ_UINT(v->mmsi, 567001001u, "J1d MMSI stored");

    /* J2 — same MMSI returns same vessel */
    {
        Vessel *v1 = vessel_find_or_create(&t, 567001001u);
        Vessel *v2 = vessel_find_or_create(&t, 567001001u);
        ASSERT_EQ_INT(v1 == v2, 1, "J2a same pointer");
        ASSERT_EQ_INT(t.count, 1, "J2b count unchanged");
    }

    /* J3 — different MMSI creates new vessel */
    {
        Vessel *v2 = vessel_find_or_create(&t, 567001002u);
        ASSERT_EQ_INT(t.count, 2, "J3a count incremented");
        ASSERT_EQ_INT(vessel_find(&t, 567001001u) != v2, 1, "J3b different pointers");
    }

    /* J4 — vessel_update stores latest position */
    {
        AISMsg1 msg;
        memset(&msg, 0, sizeof(msg));
        msg.mmsi        = 567001001u;
        msg.latitude    = 12.5f;
        msg.longitude   = 100.9f;
        msg.sog         = 8.5f;
        msg.cog         = 45.0f;
        msg.nav_status  = 0;
        msg.received_at = 1000;
        msg.pos_valid   = 1;
        msg.sog_valid   = 1;
        msg.cog_valid   = 1;
        vessel_update(&t, &msg, &ctx);
        Vessel *vv = vessel_find(&t, 567001001u);
        ASSERT_EQ_FLOAT(vv->lat, 12.5f, 0.001f, "J4a lat updated");
        ASSERT_EQ_FLOAT(vv->lon, 100.9f, 0.001f, "J4b lon updated");
        ASSERT_EQ_INT(vv->update_count, 1, "J4c update count");
    }

    /* J5 — table full returns NULL */
    {
        VesselTable t2;
        vessel_table_init(&t2);
        for (int i = 0; i < MAX_VESSELS; i++)
            vessel_find_or_create(&t2, (uint32_t)(100000000 + i));
        Vessel *vv = vessel_find_or_create(&t2, 999999999u);
        ASSERT_NULL(vv, "J5 full table returns NULL");
    }

    /* J6 — history appends correctly */
    {
        VesselTable t2;
        AisErrorCtx ctx2;
        vessel_table_init(&t2);
        ais_error_init(&ctx2);
        AISMsg1 msg;
        memset(&msg, 0, sizeof(msg));
        msg.mmsi      = 111111111u;
        msg.latitude  = 12.0f;
        msg.longitude = 101.0f;
        msg.sog       = 5.0f;
        msg.pos_valid = 1;
        msg.sog_valid = 1;
        for (int i = 0; i < 5; i++) {
            msg.received_at = 1000 + i * 12;
            vessel_update(&t2, &msg, &ctx2);
        }
        Vessel *vv = vessel_find(&t2, 111111111u);
        ASSERT_EQ_INT(vv->history_len, 5, "J6 history len = 5");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite K — EMA interval calculation
   ───────────────────────────────────────────────────────────────────────── */

static void suite_K(void)
{
    VesselTable t;
    AisErrorCtx ctx;
    vessel_table_init(&t);
    ais_error_init(&ctx);

    AISMsg1 msg;
    memset(&msg, 0, sizeof(msg));
    msg.mmsi      = 222222222u;
    msg.latitude  = 12.0f;
    msg.longitude = 101.0f;
    msg.sog       = 8.5f;
    msg.pos_valid = 1;
    msg.sog_valid = 1;
    msg.nav_status = 0;

    /* K1 — first gap seeds EMA directly (two messages 12s apart) */
    msg.received_at = 1000;
    vessel_update(&t, &msg, &ctx);
    msg.received_at = 1012;
    vessel_update(&t, &msg, &ctx);
    Vessel *v = vessel_find(&t, 222222222u);
    ASSERT_EQ_FLOAT(v->avg_interval_s, 12.0, 0.01, "K1 first gap seeds EMA");

    /* K2 — EMA converges toward new rate after speed change
       Seed at 12s, then 10 messages at 6s. After 10 steps EMA ≈ 6.65 */
    v->avg_interval_s = 12.0;
    time_t base = 2000;
    for (int i = 0; i < 10; i++) {
        msg.received_at = base + i * 6;
        vessel_update(&t, &msg, &ctx);
    }
    ASSERT_EQ_FLOAT(v->avg_interval_s, 6.65, 0.3, "K2 EMA convergence");

    /* K3 — large gaps (> 600 s) ignored */
    {
        double ema_before = v->avg_interval_s;
        msg.received_at = v->last_seen + 1000;
        vessel_update(&t, &msg, &ctx);
        ASSERT_EQ_FLOAT(v->avg_interval_s, ema_before, 0.001, "K3 large gap ignored");
    }

    /* K4 — out-of-order message does not corrupt EMA */
    {
        double ema_before = v->avg_interval_s;
        msg.received_at = v->last_seen - 5;   /* regression */
        vessel_update(&t, &msg, &ctx);
        ASSERT_EQ_FLOAT(v->avg_interval_s, ema_before, 0.001, "K4 regression skipped");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite L — Rhumb line propagation
   ───────────────────────────────────────────────────────────────────────── */

static void suite_L(void)
{
    /* L1 — due north: 60 nm in 1 hour = exactly 1.0° latitude */
    {
        float lat = 10.0f, lon = 100.0f;
        propagate(&lat, &lon, 60.0f, 0.0f, 3600);
        ASSERT_EQ_FLOAT(lat, 11.0f, 0.001f, "L1a lat increased 1°");
        ASSERT_EQ_FLOAT(lon, 100.0f, 0.001f, "L1b lon unchanged");
    }

    /* L2a — due east at equator: 60 nm = 1.0° longitude */
    {
        float lat = 0.0f, lon = 100.0f;
        propagate(&lat, &lon, 60.0f, 90.0f, 3600);
        ASSERT_EQ_FLOAT(lon, 101.0f, 0.01f, "L2a equator east");
    }

    /* L2b — due east at 60°N: 60 nm = 2.0° longitude (cos 60° = 0.5) */
    {
        float lat = 60.0f, lon = 100.0f;
        propagate(&lat, &lon, 60.0f, 90.0f, 3600);
        ASSERT_EQ_FLOAT(lon, 102.0f, 0.05f, "L2b 60°N east corrected");
    }

    /* L3 — zero speed produces no movement */
    {
        float lat = 12.0f, lon = 101.0f;
        propagate(&lat, &lon, 0.0f, 45.0f, 3600);
        ASSERT_EQ_FLOAT(lat, 12.0f, 0.0001f, "L3a lat unchanged");
        ASSERT_EQ_FLOAT(lon, 101.0f, 0.0001f, "L3b lon unchanged");
    }

    /* L4 — 24.5 kts for 3 s = 0.02042 nm northward */
    {
        float lat = 12.0f, lon = 101.0f;
        propagate(&lat, &lon, 24.5f, 0.0f, 3);
        float dist = (lat - 12.0f) * 60.0f;
        ASSERT_EQ_FLOAT(dist, 0.02042f, 0.0005f, "L4 short interval distance");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite M — Compliance classification
   ───────────────────────────────────────────────────────────────────────── */

static void suite_M(void)
{
    ASSERT_EQ_STR(compliance_str(12.0,  12), "OK",            "M1 exact compliance");
    ASSERT_EQ_STR(compliance_str(14.3,  12), "OK",            "M2 +19% over OK");
    ASSERT_EQ_STR(compliance_str(11.5,  12), "OK",            "M3 -4% under OK");
    ASSERT_EQ_STR(compliance_str(15.0,  12), "MINOR DEV",     "M4 minor over");
    ASSERT_EQ_STR(compliance_str(10.0,  12), "MINOR DEV",     "M5 minor under");
    ASSERT_EQ_STR(compliance_str(20.0,  12), "NON-COMPLIANT", "M6 non-compliant over");
    ASSERT_EQ_STR(compliance_str(8.0,   12), "NON-COMPLIANT", "M7 non-compliant under");
    ASSERT_EQ_STR(compliance_str(180.0, 180),"OK",            "M8 anchor 180s OK");
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite N — Known real-world sentences
   ───────────────────────────────────────────────────────────────────────── */

static void suite_N(void)
{
    AISMsg1 msg;

    /* N1 — real-world AIS sentence (corrected checksum *45)
       Decoded: MMSI=366053240, SOG=0.0, lat=70.0432N, lon=-122.4237W,
       COG=116.0, heading=511 (N/A) */
    {
        AisStatus st = ais_parse_nmea(
            "!AIVDM,1,1,,B,15M67N0000G?Uch`53nDR?vN0<0e,0*45", &msg);
        ASSERT_EQ_STATUS(st, AIS_OK,      "N1 parse OK");
        ASSERT_EQ_UINT(msg.mmsi, 366053240u, "N1 MMSI");
        ASSERT_EQ_INT(msg.nav_status, 0,  "N1 nav_status");
        ASSERT_EQ_FLOAT(msg.sog,   0.0f,     0.05f,  "N1 SOG");
        ASSERT_EQ_FLOAT(msg.latitude,   70.0432f,  0.001f, "N1 lat");
        ASSERT_EQ_FLOAT(msg.longitude, -122.4237f, 0.001f, "N1 lon");
        ASSERT_EQ_FLOAT(msg.cog,  116.0f,    0.1f,   "N1 COG");
        ASSERT_EQ_INT(msg.heading, 511,      "N1 heading sentinel");
        ASSERT_EQ_INT(msg.heading_valid, 0,  "N1 heading_valid=0");
    }

    /* N2 — real-world AIS sentence (checksum *24 verified correct)
       Decoded: MMSI=265547250, SOG=13.9, lat=57.6604N, lon=11.8330E,
       COG=40.4, heading=41 */
    {
        AisStatus st = ais_parse_nmea(
            "!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24", &msg);
        ASSERT_EQ_STATUS(st, AIS_OK,      "N2 parse OK");
        ASSERT_EQ_UINT(msg.mmsi, 265547250u, "N2 MMSI");
        ASSERT_EQ_FLOAT(msg.sog,  13.9f,  0.05f,  "N2 SOG");
        ASSERT_EQ_FLOAT(msg.latitude,  57.6604f, 0.001f, "N2 lat");
        ASSERT_EQ_FLOAT(msg.longitude, 11.8330f, 0.001f, "N2 lon");
        ASSERT_EQ_FLOAT(msg.cog,  40.4f,  0.1f,   "N2 COG");
        ASSERT_EQ_INT(msg.heading, 41,    "N2 heading");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite O — Type 5 encode / decode round-trip
   ───────────────────────────────────────────────────────────────────────── */

static AisStatus roundtrip5(const AISMsg5 *in, AISMsg5 *out)
{
    char f1[256], f2[256];
    AisStatus s = ais_encode_msg5(f1, sizeof(f1), f2, sizeof(f2), 1, in);
    if (s != AIS_OK) return s;
    s = ais_parse_nmea5(f1, out);
    if (s != AIS_FRAG_PENDING) return (s == AIS_OK) ? AIS_ERR_MALFORMED : s;
    return ais_parse_nmea5(f2, out);
}

static void suite_O(void)
{
    AISMsg5 in, out;
    memset(&in, 0, sizeof(in));
    in.mmsi        = 567001001u;
    in.imo_number  = 9164263u;
    in.ship_type   = 71;
    in.dim_bow     = 120;
    in.dim_stern   = 30;
    in.dim_port    = 15;
    in.dim_stbd    = 10;
    in.draught_raw = 55;
    in.eta_month   = 4;
    in.eta_day     = 20;
    in.eta_hour    = 8;
    in.eta_min     = 30;
    strncpy(in.call_sign,   "HSTH1",        sizeof(in.call_sign)   - 1);
    strncpy(in.name,        "THAI STAR 1",  sizeof(in.name)        - 1);
    strncpy(in.destination, "LAEM CHABANG", sizeof(in.destination) - 1);

    AisStatus st = roundtrip5(&in, &out);
    ASSERT_EQ_STATUS(st, AIS_OK, "O1 round-trip status");

    /* O2 — core fields */
    ASSERT_EQ_UINT(out.mmsi,       567001001u, "O2a MMSI");
    ASSERT_EQ_UINT(out.imo_number, 9164263u,   "O2b IMO number");
    ASSERT_EQ_INT (out.ship_type,  71,         "O2c ship type");
    ASSERT_EQ_INT (out.msg_type,    5,         "O2d msg_type = 5");

    /* O3 — text fields */
    ASSERT_EQ_STR(out.call_sign,   "HSTH1",        "O3a call sign");
    ASSERT_EQ_STR(out.name,        "THAI STAR 1",  "O3b vessel name");
    ASSERT_EQ_STR(out.destination, "LAEM CHABANG", "O3c destination");

    /* O4 — dimensions */
    ASSERT_EQ_INT(out.dim_bow,   120, "O4a dim_bow");
    ASSERT_EQ_INT(out.dim_stern,  30, "O4b dim_stern");
    ASSERT_EQ_INT(out.dim_port,   15, "O4c dim_port");
    ASSERT_EQ_INT(out.dim_stbd,   10, "O4d dim_stbd");

    /* O5 — ETA */
    ASSERT_EQ_INT(out.eta_month,  4,  "O5a eta_month");
    ASSERT_EQ_INT(out.eta_day,   20,  "O5b eta_day");
    ASSERT_EQ_INT(out.eta_hour,   8,  "O5c eta_hour");
    ASSERT_EQ_INT(out.eta_min,   30,  "O5d eta_min");

    /* O6 — draught */
    ASSERT_EQ_INT(out.draught_raw, 55, "O6 draught_raw");

    /* O7 — ETA N/A sentinels preserved */
    {
        AISMsg5 in2, out2;
        memset(&in2, 0, sizeof(in2));
        in2.mmsi      = 1u;
        in2.eta_month = 0;
        in2.eta_day   = 0;
        in2.eta_hour  = 24;
        in2.eta_min   = 60;
        strncpy(in2.name, "TEST", sizeof(in2.name) - 1);
        AisStatus st2 = roundtrip5(&in2, &out2);
        ASSERT_EQ_STATUS(st2, AIS_OK,    "O7 NA ETA round-trip");
        ASSERT_EQ_INT(out2.eta_hour, 24, "O7a eta_hour=24 sentinel");
        ASSERT_EQ_INT(out2.eta_min,  60, "O7b eta_min=60 sentinel");
    }

    /* O8 — 20-char name exactly fills the field */
    {
        AISMsg5 in3, out3;
        memset(&in3, 0, sizeof(in3));
        in3.mmsi = 2u;
        snprintf(in3.name, sizeof(in3.name), "%s", "ABCDEFGHIJKLMNOPQRST");
        roundtrip5(&in3, &out3);
        ASSERT_EQ_STR(out3.name, "ABCDEFGHIJKLMNOPQRST", "O8 20-char name");
    }

    /* O9 — lower-case input is upper-cased on encode */
    {
        AISMsg5 in4, out4;
        memset(&in4, 0, sizeof(in4));
        in4.mmsi = 3u;
        strncpy(in4.destination, "jakarta", sizeof(in4.destination) - 1);
        roundtrip5(&in4, &out4);
        ASSERT_EQ_STR(out4.destination, "JAKARTA", "O9 lower-case up-cased");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Suite P — Fragment reassembly
   ───────────────────────────────────────────────────────────────────────── */

static void suite_P(void)
{
    AISMsg5 in, out;
    memset(&in, 0, sizeof(in));
    in.mmsi = 567001002u;
    strncpy(in.name, "PATTAYA EXPRESS", sizeof(in.name) - 1);
    in.ship_type = 61;

    char f1[256], f2[256];
    ais_encode_msg5(f1, sizeof(f1), f2, sizeof(f2), 2, &in);

    /* P1 — first fragment returns AIS_FRAG_PENDING */
    ASSERT_EQ_STATUS(ais_parse_nmea5(f1, &out), AIS_FRAG_PENDING,
                     "P1 first fragment is PENDING");

    /* P2 — second fragment completes the message */
    AisStatus st = ais_parse_nmea5(f2, &out);
    ASSERT_EQ_STATUS(st, AIS_OK, "P2 second fragment is OK");
    ASSERT_EQ_UINT(out.mmsi, 567001002u, "P2b MMSI correct");
    ASSERT_EQ_STR(out.name, "PATTAYA EXPRESS", "P2c name correct");

    /* P3 — frag 2 with no matching frag 1 returns AIS_ERR_MALFORMED */
    {
        AISMsg5 dummy_in, dummy_out;
        memset(&dummy_in, 0, sizeof(dummy_in));
        dummy_in.mmsi = 4u;
        char df1[256], df2[256];
        ais_encode_msg5(df1, sizeof(df1), df2, sizeof(df2), 7, &dummy_in);
        /* send frag 2 without frag 1 */
        ASSERT_EQ_STATUS(ais_parse_nmea5(df2, &dummy_out), AIS_ERR_MALFORMED,
                         "P3 orphan frag 2 rejected");
        /* clean up: send frag 1 to clear any residue (no assert needed) */
        ais_parse_nmea5(df1, &dummy_out);
    }

    /* P4 — NULL inputs */
    ASSERT_EQ_STATUS(ais_parse_nmea5(NULL, &out), AIS_ERR_NULL,
                     "P4a null line");
    ASSERT_EQ_STATUS(ais_parse_nmea5(f1, NULL), AIS_ERR_NULL,
                     "P4b null output");

    /* P5 — bad checksum on frag 1 */
    {
        char bad_f1[256];
        strncpy(bad_f1, f1, sizeof(bad_f1) - 1);
        bad_f1[sizeof(bad_f1) - 1] = '\0';
        /* Corrupt the last two hex digits of the checksum */
        char *star = strrchr(bad_f1, '*');
        if (star) { star[1] = 'F'; star[2] = 'F'; }
        AISMsg5 tmp;
        ASSERT_EQ_STATUS(ais_parse_nmea5(bad_f1, &tmp), AIS_ERR_CHECKSUM,
                         "P5 bad checksum on frag 1");
    }
}

/* ─────────────────────────────────────────────────────────────────────────
   Main test runner
   ───────────────────────────────────────────────────────────────────────── */

int main(void)
{
    RUN_SUITE(suite_A);
    RUN_SUITE(suite_B);
    RUN_SUITE(suite_C);
    RUN_SUITE(suite_D);
    RUN_SUITE(suite_E);
    RUN_SUITE(suite_F);
    RUN_SUITE(suite_G);
    RUN_SUITE(suite_H);
    RUN_SUITE(suite_I);
    RUN_SUITE(suite_J);
    RUN_SUITE(suite_K);
    RUN_SUITE(suite_L);
    RUN_SUITE(suite_M);
    RUN_SUITE(suite_N);
    RUN_SUITE(suite_O);
    RUN_SUITE(suite_P);

    printf("\nResults: %d passed, %d failed\n", g_passes, g_failures);
    return (g_failures > 0) ? 1 : 0;
}
