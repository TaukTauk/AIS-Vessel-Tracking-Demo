#include "output.h"
#include "ais.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ── ANSI colour codes ───────────────────────────────────────────────────── */
#define ANSI_RESET  "\033[0m"
#define ANSI_BOLD   "\033[1m"
#define ANSI_DIM    "\033[2m"
#define ANSI_GREEN  "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_RED    "\033[31m"
#define ANSI_CYAN   "\033[36m"

/* ── Ship type name (first digit of two-digit ITU code) ──────────────────── */
static const char *ship_type_name(uint8_t ship_type)
{
    switch (ship_type / 10) {
        case 2: return "WIG";
        case 3: return "Fishing";
        case 4: return "HSC";
        case 5: return "Special";
        case 6: return "Passenger";
        case 7: return "Cargo";
        case 8: return "Tanker";
        case 9: return "Other";
        default: return "Unknown";
    }
}

/* ── Terminal output ─────────────────────────────────────────────────────── */

AisStatus output_terminal(const VesselTable *t)
{
    if (!t) return AIS_ERR_NULL;

    /* Section 1: Vessel state */
    printf(ANSI_BOLD
           "\n╔══════════════════════════════════════════════════════════════════╗\n"
           "║              AIS VESSEL TRACKING DEMO — VESSEL STATE            ║\n"
           "╚══════════════════════════════════════════════════════════════════╝\n"
           ANSI_RESET);

    printf(ANSI_BOLD
           "%-12s %-18s %-14s %-14s %-7s %-7s %-22s\n"
           ANSI_RESET,
           "MMSI", "Name", "Latitude", "Longitude",
           "SOG", "COG", "Nav Status");
    printf("%-12s %-18s %-14s %-14s %-7s %-7s %-22s\n",
           "────────────", "──────────────────", "──────────────",
           "──────────────", "───────", "───────", "──────────────────────");

    for (int i = 0; i < t->count; i++) {
        const Vessel *v = &t->vessels[i];

        char lat_str[16], lon_str[16], sog_str[10], cog_str[10];

        if (v->pos_valid)
            snprintf(lat_str, sizeof(lat_str), "%.4f%c",
                     fabsf(v->lat), v->lat >= 0 ? 'N' : 'S');
        else
            snprintf(lat_str, sizeof(lat_str), "N/A");

        if (v->pos_valid)
            snprintf(lon_str, sizeof(lon_str), "%.4f%c",
                     fabsf(v->lon), v->lon >= 0 ? 'E' : 'W');
        else
            snprintf(lon_str, sizeof(lon_str), "N/A");

        if (v->sog_valid)
            snprintf(sog_str, sizeof(sog_str), "%.1fkt", v->sog);
        else
            snprintf(sog_str, sizeof(sog_str), "N/A");

        if (v->cog_valid)
            snprintf(cog_str, sizeof(cog_str), "%.1f°", v->cog);
        else
            snprintf(cog_str, sizeof(cog_str), "N/A");

        /* SOG colour */
        const char *sog_col = ANSI_RESET;
        if (v->sog_valid) {
            if (v->sog > 23.0f)      sog_col = ANSI_RED;
            else if (v->sog > 14.0f) sog_col = ANSI_YELLOW;
            else if (v->sog > 0.1f)  sog_col = ANSI_GREEN;
        }

        printf("%-12u %-18s %-14s %-14s %s%-7s%s %-7s %-22s\n",
               v->mmsi, v->name, lat_str, lon_str,
               sog_col, sog_str, ANSI_RESET,
               cog_str, ais_nav_status_str(v->nav_status));
    }

    /* Section 2: Reporting rate compliance */
    printf(ANSI_BOLD
           "\n╔══════════════════════════════════════════════════════════════════╗\n"
           "║            REPORTING RATE COMPLIANCE (IMO MSC.74(69))           ║\n"
           "╚══════════════════════════════════════════════════════════════════╝\n"
           ANSI_RESET);

    printf(ANSI_BOLD
           "%-12s %-18s %-9s %-12s %-8s %-14s\n"
           ANSI_RESET,
           "MMSI", "Name", "Expected", "Observed", "Count", "Compliance");
    printf("%-12s %-18s %-9s %-12s %-8s %-14s\n",
           "────────────", "──────────────────", "─────────",
           "────────────", "────────", "──────────────");

    for (int i = 0; i < t->count; i++) {
        const Vessel *v = &t->vessels[i];
        const char *cs  = (v->update_count < 3)
                          ? "NO DATA"
                          : compliance_str(v->avg_interval_s,
                                           v->expected_interval_s);

        const char *col = ANSI_DIM;
        if (v->update_count >= 3) {
            if (strcmp(cs, "OK") == 0)                col = ANSI_GREEN;
            else if (strcmp(cs, "MINOR DEV") == 0)    col = ANSI_YELLOW;
            else if (strcmp(cs, "NON-COMPLIANT") == 0) col = ANSI_RED;
        }

        char obs_str[16];
        if (v->avg_interval_s > 0.001)
            snprintf(obs_str, sizeof(obs_str), "%.1fs", v->avg_interval_s);
        else
            snprintf(obs_str, sizeof(obs_str), "—");

        printf("%-12u %-18s %3ds       %-12s %-8d %s%-14s%s\n",
               v->mmsi, v->name,
               v->expected_interval_s, obs_str,
               v->update_count,
               col, cs, ANSI_RESET);
    }
    printf("\n");
    return AIS_OK;
}

/* ── HTML / Leaflet map output ───────────────────────────────────────────── */

static const char *marker_colour(uint8_t ship_type)
{
    switch (ship_type / 10) {
        case 3: return "#2ecc71";   /* fishing  — green */
        case 6: return "#3498db";   /* passenger — blue */
        case 7: return "#e67e22";   /* cargo    — orange */
        case 8: return "#e74c3c";   /* tanker   — red */
        default: return "#95a5a6";  /* other    — grey */
    }
}

AisStatus output_html(const VesselTable *t, const char *path)
{
    if (!t || !path) return AIS_ERR_NULL;

    FILE *f = fopen(path, "w");
    if (!f) return AIS_ERR_IO;

    /* Centre map on Gulf of Thailand / South China Sea */
    float centre_lat = 11.3f, centre_lon = 102.0f;

    fprintf(f,
        "<!DOCTYPE html>\n<html>\n<head>\n"
        "<meta charset='utf-8'/>\n"
        "<title>AIS Vessel Tracking Demo</title>\n"
        "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'/>\n"
        "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>\n"
        "<style>\n"
        "  body { margin:0; background:#1a1a2e; color:#eee; font-family:monospace; }\n"
        "  #map { height:600px; }\n"
        "  #panel { padding:16px; }\n"
        "  table { border-collapse:collapse; width:100%%; }\n"
        "  th,td { border:1px solid #444; padding:6px 10px; text-align:left; }\n"
        "  th { background:#16213e; }\n"
        "  .ok { color:#2ecc71; } .minor { color:#f39c12; } .bad { color:#e74c3c; }\n"
        ""
        "</style>\n</head>\n<body>\n"
        "<div id='map'></div>\n"
        "<div id='panel'>\n"
        "  <h2>Reporting Rate Compliance — IMO MSC.74(69) Table 1</h2>\n"
        "  <table>\n"
        "    <tr><th>MMSI</th><th>Name</th><th>Type</th>"
        "<th>Expected</th><th>Observed</th><th>Count</th><th>Status</th></tr>\n"
    );

    for (int i = 0; i < t->count; i++) {
        const Vessel *v = &t->vessels[i];
        const char *cs = (v->update_count < 3)
                         ? "NO DATA"
                         : compliance_str(v->avg_interval_s,
                                          v->expected_interval_s);
        const char *css_cls = "ok";
        if (strcmp(cs, "MINOR DEV") == 0)    css_cls = "minor";
        else if (strcmp(cs, "NON-COMPLIANT") == 0) css_cls = "bad";
        else if (strcmp(cs, "NO DATA") == 0)  css_cls = "";

        char obs_str[16];
        if (v->avg_interval_s > 0.001)
            snprintf(obs_str, sizeof(obs_str), "%.1fs", v->avg_interval_s);
        else
            snprintf(obs_str, sizeof(obs_str), "—");

        fprintf(f,
            "    <tr><td>%u</td><td>%s</td><td>%s</td>"
            "<td>%ds</td><td>%s</td><td>%d</td>"
            "<td class='%s'>%s</td></tr>\n",
            v->mmsi, v->name, ship_type_name(v->ship_type),
            v->expected_interval_s, obs_str,
            v->update_count, css_cls, cs);
    }

    fprintf(f, "  </table>\n</div>\n<script>\n");
    fprintf(f,
        "var map = L.map('map').setView([%.4f, %.4f], 8);\n"
        "L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',"
        "{attribution:'&copy; OpenStreetMap &copy; CARTO'}).addTo(map);\n",
        centre_lat, centre_lon);

    for (int i = 0; i < t->count; i++) {
        const Vessel *v = &t->vessels[i];
        if (!v->pos_valid) continue;

        const char *col = marker_colour(v->ship_type);

        /* Trail (dashed polyline) — iterate in chronological order */
        if (v->history_len > 1) {
            fprintf(f, "L.polyline([");
            for (int j = 0; j < v->history_len; j++) {
                int idx = (v->history_head + j) % MAX_HISTORY;
                fprintf(f, "[%.5f,%.5f]%s",
                        v->history[idx].lat, v->history[idx].lon,
                        (j < v->history_len - 1) ? "," : "");
            }
            fprintf(f,
                "],{color:'%s',weight:1,dashArray:'4 4',opacity:0.5}).addTo(map);\n",
                col);
        }

        /* Heading for arrow rotation */
        float dir = v->cog_valid ? v->cog : 0.0f;

        /* Popup content */
        char sog_s[16], cog_s[16], hdg_s[16];
        if (v->sog_valid) snprintf(sog_s, sizeof(sog_s), "%.1f kt", v->sog);
        else              snprintf(sog_s, sizeof(sog_s), "N/A");
        if (v->cog_valid) snprintf(cog_s, sizeof(cog_s), "%.1f°", v->cog);
        else              snprintf(cog_s, sizeof(cog_s), "N/A");
        if (v->heading_valid) snprintf(hdg_s, sizeof(hdg_s), "%u°", v->heading);
        else                  snprintf(hdg_s, sizeof(hdg_s), "N/A");

        const char *cs = (v->update_count < 3)
                         ? "NO DATA"
                         : compliance_str(v->avg_interval_s,
                                          v->expected_interval_s);

        char dest_s[32], eta_s[32];
        if (v->static_valid && v->destination[0])
            snprintf(dest_s, sizeof(dest_s), "%s", v->destination);
        else
            snprintf(dest_s, sizeof(dest_s), "N/A");

        if (v->static_valid && v->eta_hour != 24)
            snprintf(eta_s, sizeof(eta_s), "%02u/%02u %02u:%02u UTC",
                     v->eta_month, v->eta_day, v->eta_hour, v->eta_min);
        else
            snprintf(eta_s, sizeof(eta_s), "N/A");

        /* Ship-silhouette icon: bow at top, stern has notch — rotate by COG */
        fprintf(f,
            "L.marker([%.5f,%.5f],{icon:L.divIcon({"
            "html:'<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"16\" height=\"24\""
            " viewBox=\"0 0 16 24\""
            " style=\"transform:rotate(%.0fdeg);transform-origin:8px 12px;display:block;\">"
            "<polygon points=\"8,1 16,12 13,22 8,18 3,22 0,12\""
            " fill=\"%s\" stroke=\"rgba(255,255,255,0.75)\" stroke-width=\"1.5\""
            " stroke-linejoin=\"round\"/></svg>',"
            "iconSize:[16,24],iconAnchor:[8,12],className:''"
            "})}).bindPopup("
            "'<b>%s</b><br>MMSI: %u<br>Type: %s<br>"
            "SOG: %s &nbsp; COG: %s<br>HDG: %s<br>"
            "Nav: %s<br>Dest: %s<br>ETA: %s<br>"
            "Updates: %d<br>Expected: %ds<br>Compliance: %s'"
            ").addTo(map);\n",
            v->lat, v->lon, dir, col,
            v->name, v->mmsi, ship_type_name(v->ship_type),
            sog_s, cog_s, hdg_s,
            ais_nav_status_str(v->nav_status), dest_s, eta_s,
            v->update_count, v->expected_interval_s, cs);
    }

    fprintf(f, "</script>\n</body>\n</html>\n");
    fclose(f);
    return AIS_OK;
}
