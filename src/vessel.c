#include "vessel.h"
#include "ais.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

void vessel_table_init(VesselTable *t)
{
    memset(t, 0, sizeof(*t));
}

Vessel *vessel_find(VesselTable *t, uint32_t mmsi)
{
    for (int i = 0; i < t->count; i++)
        if (t->vessels[i].mmsi == mmsi)
            return &t->vessels[i];
    return NULL;
}

Vessel *vessel_find_or_create(VesselTable *t, uint32_t mmsi)
{
    Vessel *v = vessel_find(t, mmsi);
    if (v) return v;

    if (t->count >= MAX_VESSELS) return NULL;

    v = &t->vessels[t->count++];
    memset(v, 0, sizeof(*v));
    v->mmsi = mmsi;
    snprintf(v->name, sizeof(v->name), "VESSEL-%u", mmsi % 100000u);
    return v;
}

AisStatus vessel_update(VesselTable *t, const AISMsg1 *msg, AisErrorCtx *ctx)
{
    if (!t || !msg) return AIS_ERR_NULL;

    Vessel *v = vessel_find_or_create(t, msg->mmsi);
    if (!v) {
        if (ctx)
            ais_error_log(ctx, AIS_ERR_TABLE_FULL, 0,
                          "Table full — dropped MMSI %u", msg->mmsi);
        return AIS_ERR_TABLE_FULL;
    }

    /* Update latest dynamic state */
    v->lat         = msg->latitude;
    v->lon         = msg->longitude;
    v->sog         = msg->sog;
    v->cog         = msg->cog;
    v->heading     = msg->heading;
    v->nav_status  = msg->nav_status;
    v->pos_valid   = msg->pos_valid;
    v->sog_valid   = msg->sog_valid;
    v->cog_valid   = msg->cog_valid;
    v->heading_valid = msg->heading_valid;

    /* Timestamps and counters */
    if (v->update_count == 0)
        v->first_seen = msg->received_at;

    /* EMA interval update */
    if (v->update_count > 0 && msg->received_at > v->last_seen) {
        double gap = (double)(msg->received_at - v->last_seen);
        if (gap > 0 && gap < 600) {
            if (v->avg_interval_s < 0.001)
                v->avg_interval_s = gap;
            else
                v->avg_interval_s = v->avg_interval_s * 0.8 + gap * 0.2;
        }
    }

    v->last_seen = msg->received_at;
    v->update_count++;

    /* Expected interval from IMO table */
    v->expected_interval_s = ais_reporting_interval(msg->sog, msg->nav_status);

    /* Append to history (circular buffer) */
    if (msg->pos_valid) {
        VesselFix fix = {
            .lat = msg->latitude,
            .lon = msg->longitude,
            .sog = msg->sog,
            .cog = msg->cog,
            .t   = msg->received_at
        };

        if (v->history_len < MAX_HISTORY) {
            v->history[v->history_len] = fix;
            v->history_len++;
        } else {
            /* Buffer full — overwrite oldest, advance head */
            v->history[v->history_head] = fix;
            v->history_head = (v->history_head + 1) % MAX_HISTORY;
        }
    }

    /* Data quality warnings */
    if (ctx && msg->sog_valid && msg->sog > 50.0f && (v->ship_type / 10) != 4)
        ais_error_log(ctx, AIS_OK, 0,
                      "WARN implausible SOG %.1f kts for MMSI %u",
                      msg->sog, msg->mmsi);

    return AIS_OK;
}

void vessel_set_static(VesselTable *t, uint32_t mmsi,
                       const char *name, uint8_t ship_type)
{
    Vessel *v = vessel_find(t, mmsi);
    if (!v) return;
    strncpy(v->name, name, sizeof(v->name) - 1);
    v->name[sizeof(v->name) - 1] = '\0';
    v->ship_type = ship_type;
}

AisStatus vessel_update_static(VesselTable *t, const AISMsg5 *msg,
                                AisErrorCtx *ctx)
{
    if (!t || !msg) return AIS_ERR_NULL;

    Vessel *v = vessel_find_or_create(t, msg->mmsi);
    if (!v) {
        if (ctx)
            ais_error_log(ctx, AIS_ERR_TABLE_FULL, 0,
                          "Table full — dropped MMSI %u (Type 5)", msg->mmsi);
        return AIS_ERR_TABLE_FULL;
    }

    snprintf(v->name,        sizeof(v->name),        "%s", msg->name);
    snprintf(v->call_sign,   sizeof(v->call_sign),   "%s", msg->call_sign);
    snprintf(v->destination, sizeof(v->destination), "%s", msg->destination);

    v->ship_type   = msg->ship_type;
    v->imo_number  = msg->imo_number;
    v->eta_month   = msg->eta_month;
    v->eta_day     = msg->eta_day;
    v->eta_hour    = msg->eta_hour;
    v->eta_min     = msg->eta_min;
    v->draught_raw = msg->draught_raw;
    v->dim_bow     = msg->dim_bow;
    v->dim_stern   = msg->dim_stern;
    v->dim_port    = msg->dim_port;
    v->dim_stbd    = msg->dim_stbd;
    v->static_valid = 1;

    return AIS_OK;
}

const char *compliance_str(double actual_s, int expected_s)
{
    if (actual_s < 0.001 || expected_s <= 0) return "NO DATA";
    double ratio = actual_s / (double)expected_s;
    if (ratio < 0.8 || ratio > 1.5)  return "NON-COMPLIANT";
    if (ratio < 0.95 || ratio > 1.2) return "MINOR DEV";
    return "OK";
}
