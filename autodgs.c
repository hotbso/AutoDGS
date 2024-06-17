/*
 * AutoDGS
 *
 * (c) Jonathan Harris 2006-2013
 * (c) Holger Teutsch 2023
 *
 * Licensed under GNU LGPL v2.1.
 */

#ifdef _MSC_VER
#  define _USE_MATH_DEFINES
#  define _CRT_SECURE_NO_DEPRECATE
#endif

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <assert.h>

#ifdef _MSC_VER
#  define PATH_MAX MAX_PATH
#  define strcasecmp(s1, s2) _stricmp(s1, s2)
#endif

#include "autodgs.h"

/* Constants */
static const float D2R=M_PI/180.0;
static const float F2M=0.3048;	/* 1 ft [m] */

/* DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand) */
static const float CAP_A = 15;  /* Capture */
static const float CAP_Z = 100;	/* (50-80 in Safedock2 flier) */

static const float AZI_A = 15;	    /* provide azimuth guidance */
static const float AZI_DISP_A = 10; /* max value for display */
static const float AZI_Z = 90;

static const float GOOD_Z= 0.5;     /* stop position for nw */
static const float GOOD_X = 2.0;    /* for mw */

static const float REM_Z = 12;	/* Distance remaining from here on*/

/* place DGS at this dist from stop position, exported as dataref */
static float dgs_ramp_dist_default = 25.0;
static float dgs_ramp_dist;
static int dgs_ramp_dist_override;  // through API
static int dgs_ramp_dist_set;

/* types */
typedef enum
{
    DISABLED=0, INACTIVE, ACTIVE, ENGAGED, TRACK, GOOD, BAD, PARKED, DONE
} state_t;

const char * const state_str[] = {
    "DISABLED", "INACTIVE", "ACTIVE", "ENGAGED",
    "TRACK", "GOOD", "BAD", "PARKED", "DONE" };

enum {
    API_OPERATION_MODE,
    API_STATE,
    API_ON_GROUND,
    API_DGS_RAMP_DIST_DEFAULT,

    API_STATE_STR,  // convenience drefs
    API_OPERATION_MODE_STR,
    API_RAMP
};

typedef enum
{
    MODE_AUTO,
    MODE_MANUAL
} opmode_t;

const char * const opmode_str[] = { "Automatic", "Manual" };

/* Globals */
static const char pluginName[]="AutoDGS";
static const char pluginSig[] ="hotbso.AutoDGS";
static const char pluginDesc[]="Automatically provides DGS for gateway airports";

static opmode_t operation_mode = MODE_AUTO;

static state_t state = DISABLED;
static float timestamp;

XPLMCommandRef cycle_dgs_cmdr;
static XPLMCommandRef move_dgs_closer_cmdr, activate_cmdr, toggle_ui_cmdr, toggle_jetway_cmdr;

/* Datarefs */
static XPLMDataRef plane_x_dr, plane_y_dr, plane_z_dr, is_helicopter_dr, y_agl_dr;
static XPLMDataRef plane_lat_dr, plane_lon_dr, plane_elevation_dr, plane_true_psi_dr;
static XPLMDataRef gear_fnrml_dr, acf_cg_y_dr, acf_cg_z_dr, gear_z_dr;
static XPLMDataRef beacon_dr, parkbrake_dr, acf_icao_dr, total_running_time_sec_dr;
static XPLMDataRef percent_lights_dr, xp_version_dr, eng_running_dr, sin_wave_dr;
XPLMDataRef vr_enabled_dr;
static XPLMProbeRef probe_ref;

/* Published DataRef values */
static int status, track, lr;
static int icao[4];
static float azimuth, distance;

/* Internal state */
static float now;           /* current timestamp */
static int beacon_state, beacon_last_pos;   /* beacon state, last switch_pos, ts of last switch actions */
static float beacon_off_ts, beacon_on_ts;
static int use_engine_running;              /* instead of beacon, e.g. MD11 */
static int dont_connect_jetway;             /* e.g. for ZIBO with own ground service */

static airportdb_t airportdb;
const airport_t *arpt;
static int on_ground = 1;
static float on_ground_ts;
static float stand_x, stand_y, stand_z, stand_sin_hgt, stand_cos_hgt, stand_hdg;
static float dgs_pos_x, dgs_pos_y, dgs_pos_z;

static float plane_nw_z, plane_mw_z, plane_cg_z;   // z value of plane's 0 to fw, mw and cg
static float pe_y_plane_0;        // pilot eye y to plane's 0 point
static int pe_y_plane_0_valid;
static int is_helicopter;

static char selected_ramp[RAMP_NAME_LEN];
static const ramp_start_t *nearest_ramp;
float nearest_ramp_ts; // timestamp of last find_nearest_ramp()

static int update_dgs_log_ts;   // throttling of logging
int dgs_type = 0;
static float sin_wave_prev;
static XPLMObjectRef dgs_obj[2];

enum _DGS_DREF {
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_AZIMUTH,
    DGS_DR_DISTANCE,
    DGS_DR_ICAO_0,
    DGS_DR_ICAO_1,
    DGS_DR_ICAO_2,
    DGS_DR_ICAO_3,
    DGS_DR_BRIGHTNESS,
    DGS_DR_NUM             // # of drefs
};

// keep exactly the same order as list above
static const char *dgs_dlist_dr[] = {
    "hotbso/dgs/status",
    "hotbso/dgs/lr",
    "hotbso/dgs/track",
    "hotbso/dgs/azimuth",
    "hotbso/dgs/distance",
    "hotbso/dgs/icao_0",
    "hotbso/dgs/icao_1",
    "hotbso/dgs/icao_2",
    "hotbso/dgs/icao_3",
    "hotbso/dgs/vdgs_brightness",
    NULL
};

static XPLMInstanceRef dgs_inst_ref;

#define SQR(x) ((x) * (x))

static inline float
clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static void
reset_state(state_t new_state)
{
    if (state != new_state)
        logMsg("setting state to %s", state_str[new_state]);

    state = new_state;
    nearest_ramp = NULL;
    dgs_ramp_dist = dgs_ramp_dist_default;
    if (state == INACTIVE) {
        selected_ramp[0] = '\0';
        arpt = NULL;
        update_ui(1);
    }

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }
}

/* set mode to arrival */
static void
set_active(void)
{
    if (! on_ground) {
        logMsg("can't set active when not on ground");
        return;
    }

    if (state > INACTIVE)
        return;

    beacon_state = beacon_last_pos = XPLMGetDatai(beacon_dr);
    beacon_on_ts = beacon_off_ts = -10.0;

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);
    char airport_id[50];

    /* can be a teleportation so play it safe */
    arpt = NULL;
    reset_state(INACTIVE);
    unload_distant_airport_tiles(&airportdb, GEO_POS2(lat, lon));

    /* find and load airport I'm on now */
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, airport_id,
                NULL, NULL);
        logMsg("now on airport: %s", airport_id);

        arpt = adb_airport_lookup_by_ident(&airportdb, airport_id);
    }

    if (NULL == arpt) {
        logMsg("not a global + IFR airport, sorry no DGS");
        return;
    }

    logMsg("found in DGS cache: %s, new state: ACTIVE", arpt->icao);
    state = ACTIVE;
    dgs_ramp_dist_set = 0;
    update_ui(1);
}

static int
check_beacon(void)
{
    if (use_engine_running) {
        int er[8];
        int n = XPLMGetDatavi(eng_running_dr, er, 0, 8);
        for (int i = 0; i < n; i++)
            if (er[i])
                return 1;

        return 0;
    }

    /* when checking the beacon guard against power transitions when switching
       to the APU generator (e.g. for the ToLiss fleet).
       Report only state transitions when the new state persisted for 3 seconds */

    int beacon = XPLMGetDatai(beacon_dr);
    if (beacon) {
        if (! beacon_last_pos) {
            beacon_on_ts = now;
            beacon_last_pos = 1;
        } else if (now > beacon_on_ts + 3.0)
            beacon_state = 1;
    } else {
        if (beacon_last_pos) {
            beacon_off_ts = now;
            beacon_last_pos = 0;
        } else if (now > beacon_off_ts + 3.0)
            beacon_state = 0;
   }

   return beacon_state;
}

// dummy accessor routine
static float
getdgsfloat(XPLMDataRef inRefcon)
{
    return -1.0;
}

// API accessor routines
static int
api_getint(XPLMDataRef ref)
{
    switch ((long long)ref) {
        case API_STATE:
            return state;
        case API_OPERATION_MODE:
            return operation_mode;
        case API_ON_GROUND:
            return on_ground;
    }

    return 0;
}

static float
api_getfloat(XPLMDataRef ref)
{
    switch ((long long)ref) {
        case API_DGS_RAMP_DIST_DEFAULT:
            return dgs_ramp_dist_default;
    }

    return 0;
}

static void
api_setint(XPLMDataRef ref, int val)
{
    switch ((long long)ref) {
        case API_OPERATION_MODE:
            ; // required by some gcc versions
            opmode_t mode = (opmode_t)val;
            if (mode != MODE_AUTO && mode != MODE_MANUAL) {
                logMsg("API: trying to set invalid operation_mode %d, ignored", val);
                return;
            }

            if (mode == operation_mode) // Lua hammers writeable drefs in a frame loop
                return;

            logMsg("API: operation_mode set to %s", opmode_str[mode]);
            operation_mode = mode;
            break;
    }
}

static void
api_setfloat(XPLMDataRef ref, float val)
{
    switch ((long long)ref) {
        case API_DGS_RAMP_DIST_DEFAULT:
            if (val == dgs_ramp_dist_default) // Lua hammers writeable drefs in a frame loop
                return;

            dgs_ramp_dist_default = val;
            dgs_ramp_dist_override = 1;
            logMsg("API: dgs_ramp_dist_default set to %0.1f", dgs_ramp_dist_default);
            break;
    }
}

static int
api_getbytes(XPLMDataRef ref, void *out, int ofs, int n)
{
    static const int buflen = 34;
    char buf[buflen];

    if (out == NULL)
        return buflen;

    if (n <= 0 || ofs < 0 || ofs >= buflen)
        return 0;

    if (n + ofs > buflen)
        n = buflen - ofs;

    memset(buf, 0, buflen);

    switch ((long long)ref) {
        case API_STATE_STR:
            strncpy(buf, state_str[state], buflen - 1);
            break;

        case API_OPERATION_MODE_STR:
            strncpy(buf, opmode_str[operation_mode], buflen - 1);
            break;

        case API_RAMP:
            if (state >= ENGAGED)
                strncpy(buf, nearest_ramp->name, buflen - 1);
            break;
    }

    memcpy(out, buf, n);
    return n;
}

// move dgs some distance away
static void
set_dgs_pos(void)
{
    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (!dgs_ramp_dist_set) {
        /* determine dgs_ramp_dist_default depending on pilot eye height agl */
        if (! dgs_ramp_dist_override && pe_y_plane_0_valid) {
            float plane_x = XPLMGetDataf(plane_x_dr);
            float plane_y = XPLMGetDataf(plane_y_dr);
            float plane_z = XPLMGetDataf(plane_z_dr);

            /* get terrain y below plane y */

            if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, plane_x, plane_y, plane_z, &probeinfo)) {
                logMsg("XPLMProbeTerrainXYZ failed");
                reset_state(INACTIVE);
                return;
            }

            /* pilot eye above agl */
            float pe_agl = plane_y - probeinfo.locationY + pe_y_plane_0;

            // 4.3 ~ 1 / tan(13°) -> 13° down look
            dgs_ramp_dist_default = MAX(8.0, MIN(4.3 * pe_agl, 30.0));
            logMsg("setting DGS default distance, pe_agl: %0.2f, dist: %0.1f", pe_agl, dgs_ramp_dist_default);
        }

        dgs_ramp_dist = dgs_ramp_dist_default;
        dgs_ramp_dist_set = 1;
    }

    // xform (0, -dgs_ramp_dist) into global frame
    dgs_pos_x = stand_x + -stand_sin_hgt * (-dgs_ramp_dist);
    dgs_pos_z = stand_z +  stand_cos_hgt * (-dgs_ramp_dist);

    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, dgs_pos_x, stand_y, dgs_pos_z, &probeinfo)) {
        logMsg("XPLMProbeTerrainXYZ failed");
        reset_state(ACTIVE);
        return;
    }

    dgs_pos_y = probeinfo.locationY;
}

/* hooks for the ui */
void
set_selected_ramp(const char *ui_selected_ramp)
{
    logMsg("set_selected_ramp to '%s'", ui_selected_ramp);
    if (0 == strcmp(ui_selected_ramp, "Automatic")) {
        selected_ramp[0] = '\0';
        if (state > ACTIVE)
            reset_state(ACTIVE);
    } else {
        strcpy(selected_ramp, ui_selected_ramp);
        if (state > ACTIVE)
            reset_state(ACTIVE);
    }
}

void
set_dgs_type(int new_dgs_type)
{
    if (new_dgs_type == dgs_type)
        return;

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }

    dgs_type = new_dgs_type;
}

static void
find_nearest_ramp()
{
    assert(arpt != NULL);

    // check whether we already have a selected ramp
    if (nearest_ramp && selected_ramp[0])
        return;

    double dist = 1.0E10;
    const ramp_start_t *min_ramp = NULL;

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);

    float plane_elevation = XPLMGetDataf(plane_elevation_dr);
    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (probe_ref == NULL)
        probe_ref = XPLMCreateProbe(xplm_ProbeY);

    for (const ramp_start_t *ramp = avl_first(&arpt->ramp_starts); ramp != NULL;
        ramp = AVL_NEXT(&arpt->ramp_starts, ramp)) {

        // heading in local system
        float local_hdgt = rel_angle(ramp->hdgt, plane_hdgt);

        if (fabs(local_hdgt) > 90.0)
            continue;   // not looking to ramp

        double s_x, s_y, s_z;
        XPLMWorldToLocal(ramp->pos.lat, ramp->pos.lon, plane_elevation, &s_x, &s_y, &s_z);

        /* transform into gate local coordinate system */
        float s_sin_hgt = sinf(D2R * ramp->hdgt);
        float s_cos_hgt = cosf(D2R * ramp->hdgt);

        if (selected_ramp[0] == '\0') {
            // xlate + rotate into stand frame
            float dx = plane_x - s_x;
            float dz = plane_z - s_z;

            float local_x =  s_cos_hgt * dx + s_sin_hgt * dz;
            float local_z = -s_sin_hgt * dx + s_cos_hgt * dz;

            // nose wheel
            float nw_z = local_z - plane_nw_z;
            float nw_x = local_x + plane_nw_z * sinf(D2R * local_hdgt);

            float d = sqrt(SQR(nw_x) + SQR(nw_z));
            if (d > CAP_Z + 50) // fast exit
                continue;

            //logMsg("stand: %s, z: %2.1f, x: %2.1f", ramp->name, nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                //logMsg("behind: %s", ramp->name);
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / D2R;
                //logMsg("angle to plane: %s, %3.1f", ramp->name, angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = rel_angle(local_hdgt, -angle);
                //logMsg("rel_to_stand: %s, nw_x: %0.1f, local_hdgt %0.1f, rel_to_stand: %0.1f",
                //       ramp->name, nw_x, local_hdgt, rel_to_stand);
                if ((nw_x > 10.0 && rel_to_stand < -60.0)
                    || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    //logMsg("drive by %s", ramp->name);
                    continue;
                }
            }

            // for the final comparison give azimuth a higher weight
            static const float azi_weight = 4.0;
            d = sqrt(SQR(azi_weight * nw_x)+ SQR(nw_z));

            if (d < dist) {
                //logMsg("new min: %s, z: %2.1f, x: %2.1f", ramp->name, nw_z, nw_x);
                dist = d;
                min_ramp = ramp;
                stand_x = s_x;
                stand_y = s_y;
                stand_z = s_z;
                stand_sin_hgt = s_sin_hgt;
                stand_cos_hgt = s_cos_hgt;
            }

        } else if (0 == strcmp(ramp->name, selected_ramp)) {
            dist = 0.0;
            min_ramp = ramp;
            stand_x = s_x;
            stand_y = s_y;
            stand_z = s_z;
            stand_sin_hgt = s_sin_hgt;
            stand_cos_hgt = s_cos_hgt;
            break;
        }
    }

    if (min_ramp != NULL && min_ramp != nearest_ramp) {
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, stand_x, stand_y, stand_z, &probeinfo)) {
            logMsg("XPLMProbeTerrainXYZ failed");
            reset_state(ACTIVE);
            return;
        }

        logMsg("ramp: %s, %f, %f, %f, dist: %f, is_wet: %d", min_ramp->name, min_ramp->pos.lat, min_ramp->pos.lon,
               min_ramp->hdgt, dist, probeinfo.is_wet);

        if (probeinfo.is_wet) {
            reset_state(ACTIVE);
            return;
        }

        stand_y = probeinfo.locationY;
        stand_hdg = min_ramp->hdgt;
        nearest_ramp = min_ramp;
        set_dgs_pos();
        state = ENGAGED;
    }
}

static float
run_state_machine()
{
    if (state <= INACTIVE)
        return 2.0;

    // throttle costly search
    if (now > nearest_ramp_ts + 2.0) {
        find_nearest_ramp();
        nearest_ramp_ts = now;
    }

    if (nearest_ramp == NULL) {
        state = ACTIVE;
        return 2.0;
    }


    int lr_prev = lr;
    int track_prev = track;
    float distance_prev = distance;

    float loop_delay = 0.2;
    state_t new_state = state;

    // xform plane pos into stand local coordinate system
    float dx = XPLMGetDataf(plane_x_dr) - stand_x;
    float dz = XPLMGetDataf(plane_z_dr) - stand_z;
    float local_x =  stand_cos_hgt * dx + stand_sin_hgt * dz;
    float local_z = -stand_sin_hgt * dx + stand_cos_hgt * dz;

    // relative reading to stand +/- 180
    float local_hdgt = rel_angle(stand_hdg, XPLMGetDataf(plane_true_psi_dr));

    // nose wheel
    float nw_z = local_z - plane_nw_z;
    float nw_x = local_x + plane_nw_z * sinf(D2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - plane_mw_z;
    float mw_x = local_x + plane_mw_z * sinf(D2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = clampf((nw_z - 6.0) / 20.0, 0.0, 1.0);
    float plane_z_dr = (1.0 - a) * plane_nw_z + a * plane_mw_z;
    float z_dr = local_z - plane_z_dr;
    float x_dr = local_x + plane_z_dr * sinf(D2R * local_hdgt);

    if (fabs(x_dr) > 0.5 && z_dr > 0)
        azimuth = atanf(x_dr / (z_dr + 0.5 * dgs_ramp_dist)) / D2R;
    else
        azimuth = 0.0;

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 0.5 * dgs_ramp_dist)) / D2R;
    else
        azimuth_nw = 0.0;

    int locgood = (fabsf(mw_x) <= GOOD_X && fabsf(nw_z) <= GOOD_Z);
    int beacon_on = check_beacon();

    status = lr = track = 0;
    distance = nw_z - GOOD_Z;

    // catch the phase ~180° point -> the Marshaller's arm is straight
    float sin_wave = XPLMGetDataf(sin_wave_dr);
    int phase180 = (sin_wave_prev > 0.0) && (sin_wave <= 0.0);
    sin_wave_prev = sin_wave;

    /* set drefs according to *current* state */
    switch (state) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance <= CAP_Z) && (fabsf(azimuth_nw) <= CAP_A))
                    new_state = TRACK;
            } else { // not beacon_on
                new_state = DONE;
            }
            break;

        case TRACK:
            if (!beacon_on) {       // don't get stuck in TRACK
                new_state = DONE;
                break;
            }

            if (locgood) {
                new_state = GOOD;
                break;
            }

            if (nw_z < -GOOD_Z) {
                new_state = BAD;
                break;
            }

            if ((distance > CAP_Z) || (fabsf(azimuth_nw) > CAP_A)) {
                new_state = ENGAGED;    // moving away from current gate
                break;
            }

            status = 1;	/* plane id */
            if (distance > AZI_Z || fabsf(azimuth_nw) > AZI_A) {
                track=1;	/* lead-in only */
                break;
            }

            /* compute distance and guidance commands */
            azimuth = clampf(azimuth, -AZI_A, AZI_A);
            float req_hdgt = -3.5 * azimuth;        // to track back to centerline
            float d_hdgt = req_hdgt - local_hdgt;   // degrees to turn

            if (now > update_dgs_log_ts + 2.0)
                logMsg("azimuth: %0.1f, mw: (%0.1f, %0.1f), nw: (%0.1f, %0.1f), ref: (%0.1f, %0.1f), "
                       "x: %0.1f, local_hdgt: %0.1f, d_hdgt: %0.1f",
                       azimuth, mw_x, mw_z, nw_x, nw_z,
                       x_dr, z_dr,
                       local_x, local_hdgt, d_hdgt);

            if (d_hdgt < -1.5)
                lr = 2;
            else if (d_hdgt > 1.5)
                lr = 1;

            /* xform azimuth to values required ob OBJ */
            azimuth = clamp(azimuth, -AZI_DISP_A, AZI_DISP_A) * 4.0 / AZI_DISP_A;
            azimuth=((float)((int)(azimuth * 2))) / 2;  // round to 0.5 increments

            if (distance <= REM_Z/2) {
                track = 3;
                loop_delay = 0.03;
            } else /* azimuth only */
                track = 2;

            if (! phase180) { // no wild oscillation
                lr = lr_prev;

                // sync transition with Marshaller's arm movement
                if (dgs_type == 0 && track == 3 && track_prev == 2) {
                    track = track_prev;
                    distance = distance_prev;
                }
            }
            break;

        case GOOD:
            /* @stop position*/
            status = 2; lr = 3;

            int parkbrake_set = (XPLMGetDataf(parkbrake_dr) > 0.5);
            if (!locgood)
                new_state = TRACK;
            else if (parkbrake_set || !beacon_on)
                new_state = PARKED;
            break;

        case BAD:
            if (!beacon_on
                && (now > timestamp + 5.0)) {
                reset_state(INACTIVE);
                return loop_delay;
            }

            if (nw_z >= -GOOD_Z)
                new_state = TRACK;
            else {
                /* Too far */
                status = 4;
                lr = 3;
            }
            break;

        case PARKED:
            status = 3;
            lr = 0;
            /* wait for beacon off */
            if (! beacon_on) {
                new_state = DONE;
                if (operation_mode == MODE_AUTO && ! dont_connect_jetway)
                    XPLMCommandOnce(toggle_jetway_cmdr);
            }
            break;

        case DONE:
            if (now > timestamp + 5.0) {
                reset_state(INACTIVE);
                return loop_delay;
            }
            break;

        default:
            break;
    }

    if (new_state != state) {
        logMsg("state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;
        timestamp = now;
        return -1;  // see you on next frame
    }

    if (state > ACTIVE) {
        /* xform drefs into required constraints for the OBJs */
        if (track == 0 || track == 1) {
            distance = 0;
            azimuth = 0.0;
        }

        distance = clampf(distance, -GOOD_Z, REM_Z);

        // is not necessary for Marshaller + SafedockT2
        // distance=((float)((int)((distance)*2))) / 2;    // multiple of 0.5m

        // don't flood the log
        if (now > update_dgs_log_ts + 2.0) {
            update_dgs_log_ts = now;
            logMsg("ramp: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, azimuth: %0.1f",
                   nearest_ramp->name, state_str[state], status, track, lr, distance, azimuth);
        }

        XPLMDrawInfo_t drawinfo;
        float drefs[DGS_DR_NUM];
        memset(drefs, 0, sizeof(drefs));

        drawinfo.structSize = sizeof(drawinfo);
        drawinfo.x = dgs_pos_x;
        drawinfo.y = dgs_pos_y;
        drawinfo.z = dgs_pos_z;
        drawinfo.heading = stand_hdg;
        drawinfo.pitch = drawinfo.roll = 0.0;

        if (dgs_inst_ref == NULL) {
            dgs_inst_ref = XPLMCreateInstance(dgs_obj[dgs_type], dgs_dlist_dr);
            if (dgs_inst_ref == NULL) {
                logMsg("error creating instance");
                state = DISABLED;
                return 0.0;
            }
        }

        drefs[DGS_DR_STATUS] = status;
        drefs[DGS_DR_TRACK] = track;
        drefs[DGS_DR_DISTANCE] = distance;
        drefs[DGS_DR_AZIMUTH] = azimuth;
        drefs[DGS_DR_LR] = lr;

        if (state == TRACK) {
            for (int i = 0; i < 4; i++)
                drefs[DGS_DR_ICAO_0 + i] = icao[i];

            if (isalpha(icao[3]))
                drefs[DGS_DR_ICAO_3] += 0.98;    // bug in VDGS
        }

        static const float min_brightness = 0.025;   // relativ to 1
        float brightness = min_brightness + (1 - min_brightness) * powf(1 - XPLMGetDataf(percent_lights_dr), 1.5);
        drefs[DGS_DR_BRIGHTNESS] = brightness;
        XPLMInstanceSetPosition(dgs_inst_ref, &drawinfo, drefs);
    }

    return loop_delay;
}

static float
flight_loop_cb(float inElapsedSinceLastCall,
               float inElapsedTimeSinceLastFlightLoop, int inCounter,
               void *inRefcon)
{
    float loop_delay = 2.0;

    now = XPLMGetDataf(total_running_time_sec_dr);
    int og;
    if (is_helicopter)
        og = (XPLMGetDataf(y_agl_dr) < 10.0);
    else
        og = (XPLMGetDataf(gear_fnrml_dr) != 0.0);

    if (og != on_ground && now > on_ground_ts + 10.0) {
        on_ground = og;
        on_ground_ts = now;
        logMsg("transition to on_ground: %d", on_ground);

        if (on_ground) {
            if (operation_mode == MODE_AUTO)
                set_active();
        } else {
            // transition to airborne
            reset_state(INACTIVE);
            if (probe_ref) {
                XPLMDestroyProbe(probe_ref);
                probe_ref = NULL;
            }
        }
    }

    if (state >= ACTIVE)
        loop_delay = run_state_machine();

    return loop_delay;
}

/* call backs for commands */
static int
cmd_cycle_dgs_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    set_dgs_type(!dgs_type);
    update_ui(1);
    return 0;
}

static int
cmd_activate_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    logMsg("cmd manually_activate");
    set_active();
    return 0;
}

static int
cmd_move_dgs_closer(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase || state < ENGAGED)
        return 0;

    if (dgs_ramp_dist > 12.0) {
        dgs_ramp_dist -= 2.0;
        logMsg("dgs_ramp_dist reduced to %0.1f", dgs_ramp_dist);
        set_dgs_pos();
    }

    return 0;
}

static int
cmd_toggle_ui_cb(XPLMCommandRef cmdr, XPLMCommandPhase phase, void *ref)
{
    UNUSED(ref);
    if (xplm_CommandBegin != phase)
        return 0;

    logMsg("cmd toggle_ui");
    toggle_ui();
    return 0;
}

/* call back for menu */
static void
menu_cb(void *menu_ref, void *item_ref)
{
    XPLMCommandOnce(*(XPLMCommandRef *)item_ref);
}

static int
find_icao_in_file(const char *acf_icao, const char *dir, const char *fn)
{
    char fn_full[512];
    snprintf(fn_full, sizeof(fn_full) - 1, "%s%s", dir, fn);

    int res = 0;
    FILE *f = fopen(fn_full, "r");
    if (f) {
        logMsg("check whether acf '%s' is in exception file %s", acf_icao, fn_full);
        char line[100];
        while (fgets(line, sizeof(line), f)) {
            char *cptr = strchr(line, '\r');
            if (cptr)
                *cptr = '\0';
            cptr = strchr(line, '\n');
            if (cptr)
                *cptr = '\0';

            if (0 == strcmp(line, acf_icao)) {
                logMsg("found acf %s in %s", acf_icao, fn);
                res = 1;
                break;
            }
        }

        fclose(f);
    }

    return res;
}

/* =========================== plugin entry points ===============================================*/
PLUGIN_API int
XPluginStart(char *outName, char *outSig, char *outDesc)
{
    sprintf(outName, "%s v%s", pluginName, VERSION);
    strcpy(outSig,  pluginSig);
    strcpy(outDesc, pluginDesc);

	log_init(XPLMDebugString, "AutoDGS");

    logMsg("startup " VERSION);

    /* Refuse to initialise if Fat plugin has been moved out of its folder */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);			/* Get paths in posix format under X-Plane 10+ */
    XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

    char xpdir[512];
	XPLMGetSystemPath(xpdir);

    char cache_path[512];
    snprintf(cache_path, sizeof(cache_path), "%sOutput/caches/AutoDGS.cache", xpdir);
    fix_pathsep(cache_path);                        /* libacfutils requires a canonical path sep */
    airportdb_create(&airportdb, xpdir, cache_path);
    airportdb.ifr_only = B_TRUE;
    airportdb.autodgs_airports_only = B_TRUE;
    if (!recreate_cache(&airportdb)) {
        logMsg("init failure: recreate_cache failed");
        return 0;
    }

    /* Datarefs */
    xp_version_dr     = XPLMFindDataRef("sim/version/xplane_internal_version");
    plane_x_dr        = XPLMFindDataRef("sim/flightmodel/position/local_x");
    plane_y_dr        = XPLMFindDataRef("sim/flightmodel/position/local_y");
    plane_z_dr        = XPLMFindDataRef("sim/flightmodel/position/local_z");
    gear_fnrml_dr     = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    plane_lat_dr      = XPLMFindDataRef("sim/flightmodel/position/latitude");
    plane_lon_dr      = XPLMFindDataRef("sim/flightmodel/position/longitude");
    plane_elevation_dr= XPLMFindDataRef("sim/flightmodel/position/elevation");
    plane_true_psi_dr = XPLMFindDataRef("sim/flightmodel2/position/true_psi");
    parkbrake_dr      = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");
    beacon_dr         = XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    eng_running_dr    = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
    acf_icao_dr       = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    acf_cg_y_dr       = XPLMFindDataRef("sim/aircraft/weight/acf_cgY_original");
    acf_cg_z_dr       = XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    gear_z_dr         = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
    is_helicopter_dr  = XPLMFindDataRef("sim/aircraft2/metadata/is_helicopter");
    y_agl_dr          = XPLMFindDataRef("sim/flightmodel2/position/y_agl");
    total_running_time_sec_dr = XPLMFindDataRef("sim/time/total_running_time_sec");
    percent_lights_dr = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
    sin_wave_dr       = XPLMFindDataRef("sim/graphics/animation/sin_wave_2");
    vr_enabled_dr     = XPLMFindDataRef("sim/graphics/VR/enabled");

    /* Published scalar datarefs, as we draw with the instancing API the accessors will never be called */
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL, NULL, getdgsfloat,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);

    /* API datarefs */
    XPLMRegisterDataAccessor("AutoDGS/operation_mode", xplmType_Int, 1, api_getint, api_setint, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_OPERATION_MODE, (void *)API_OPERATION_MODE);

    XPLMRegisterDataAccessor("AutoDGS/operation_mode_str", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_OPERATION_MODE_STR, NULL);

    XPLMRegisterDataAccessor("AutoDGS/state", xplmType_Int, 0, api_getint, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_STATE, NULL);

    XPLMRegisterDataAccessor("AutoDGS/on_ground", xplmType_Int, 0, api_getint, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_ON_GROUND, NULL);

    XPLMRegisterDataAccessor("AutoDGS/dgs_ramp_dist_default", xplmType_Float, 1, NULL, NULL,
                             api_getfloat, api_setfloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                             (void *)API_DGS_RAMP_DIST_DEFAULT, (void *)API_DGS_RAMP_DIST_DEFAULT);

    XPLMRegisterDataAccessor("AutoDGS/state_str", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_STATE_STR, NULL);

    XPLMRegisterDataAccessor("AutoDGS/ramp", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_RAMP, NULL);

    int is_XP11 = (XPLMGetDatai(xp_version_dr) < 120000);
    const char *obj_name[2];

    if (is_XP11) {
        logMsg("XP11 detected");
        obj_name[0] = "Resources/plugins/AutoDGS/resources/Marshaller_XP11.obj";
        obj_name[1] = "Resources/plugins/AutoDGS/resources/SafedockT2-6m-pole_XP11.obj";
    } else {
        obj_name[0] = "Resources/plugins/AutoDGS/resources/Marshaller.obj";
        obj_name[1] = "Resources/plugins/AutoDGS/resources/SafedockT2-6m-pole.obj";
    }

    for (int i = 0; i < 2; i++) {
        dgs_obj[i] = XPLMLoadObject(obj_name[i]);

        if (dgs_obj[i] == NULL) {
            logMsg("error loading obj: %s", obj_name[i]);
            return 0;
        }
    }

    /* own commands */
    cycle_dgs_cmdr = XPLMCreateCommand("AutoDGS/cycle_dgs", "Cycle DGS between Marshaller, VDGS");
    XPLMRegisterCommandHandler(cycle_dgs_cmdr, cmd_cycle_dgs_cb, 0, NULL);

    move_dgs_closer_cmdr = XPLMCreateCommand("AutoDGS/move_dgs_closer", "Move DGS closer by 2m");
    XPLMRegisterCommandHandler(move_dgs_closer_cmdr, cmd_move_dgs_closer, 0, NULL);

    activate_cmdr = XPLMCreateCommand("AutoDGS/activate", "Manually activate searching for stands");
    XPLMRegisterCommandHandler(activate_cmdr, cmd_activate_cb, 0, NULL);

    toggle_ui_cmdr = XPLMCreateCommand("AutoDGS/toggle_ui", "Open UI");
    XPLMRegisterCommandHandler(toggle_ui_cmdr, cmd_toggle_ui_cb, 0, NULL);

    /* menu */
    XPLMMenuID menu = XPLMFindPluginsMenu();
    int sub_menu = XPLMAppendMenuItem(menu, "AutoDGS", NULL, 1);
    XPLMMenuID adgs_menu = XPLMCreateMenu("AutoDGS", menu, sub_menu, menu_cb, NULL);

    XPLMAppendMenuItem(adgs_menu, "Manually activate", &activate_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Cycle DGS", &cycle_dgs_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Move DGS closer by 2m", &move_dgs_closer_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Toggle UI", &toggle_ui_cmdr, 0);

    /* foreign commands */
    toggle_jetway_cmdr = XPLMFindCommand("sim/ground_ops/jetway");

    XPLMRegisterFlightLoopCallback(flight_loop_cb, 2.0, NULL);
    return 1;
}

PLUGIN_API void
XPluginStop(void)
{
    XPLMUnregisterFlightLoopCallback(flight_loop_cb, NULL);
    if (probe_ref)
        XPLMDestroyProbe(probe_ref);

    for (int i = 0; i < 2; i++)
        if (dgs_obj[i])
            XPLMUnloadObject(dgs_obj[i]);
}

PLUGIN_API int
XPluginEnable(void)
{
    state = INACTIVE;
    return 1;
}

PLUGIN_API void
XPluginDisable(void)
{
    reset_state(DISABLED);
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID in_from, long in_msg, void *in_param)
{
    UNUSED(in_from);

    /* my plane loaded */
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        char acf_icao[41];

        reset_state(INACTIVE);
        memset(acf_icao, 0, sizeof(acf_icao));
        XPLMGetDatab(acf_icao_dr, acf_icao, 0, 40);

        for (int i=0; i<4; i++)
            icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';

        acf_icao[4] = '\0';

        plane_cg_z = F2M * XPLMGetDataf(acf_cg_z_dr);

        float gear_z[2];
        if (2 == XPLMGetDatavf(gear_z_dr, gear_z, 0, 2)) {      // nose + main wheel
            plane_nw_z = -gear_z[0];
            plane_mw_z = -gear_z[1];
        } else
            plane_nw_z = plane_mw_z = plane_cg_z;         // fall back to CG

        is_helicopter = XPLMGetDatai(is_helicopter_dr);

        pe_y_plane_0_valid = 0;
        pe_y_plane_0 = 0.0;

        if (! is_helicopter) {
            /* unfortunately the *default* pilot eye y coordinate is not published in
               a dataref, only the dynamic values.
               Therefore we pull it from the acf file. */

            char acf_path[512];
            char acf_file[256];

            XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
            logMsg(acf_path);


            FILE *acf = fopen(acf_path, "r");
            if (acf) {
                char line[200];
                while (fgets(line, sizeof(line), acf)) {
                    if (line == strstr(line, "P acf/_pe_xyz/1 ")) {
                        if (1 == sscanf(line + 16, "%f", &pe_y_plane_0)) {
                            pe_y_plane_0 -= XPLMGetDataf(acf_cg_y_dr);
                            pe_y_plane_0 *= F2M;
                            pe_y_plane_0_valid = 1;
                        }
                        break;
                    }
                }

                fclose(acf);
            }
        }

        /* check whether acf is listed in exception files */
        use_engine_running = 0;
        dont_connect_jetway = 0;

        char dir[512];
        dir[0] = '\0';
        XPLMGetPluginInfo(XPLMGetMyID(), NULL, dir, NULL, NULL);
        char *cptr = strrchr(dir, '/');    /* basename */
        if (cptr) {
            *cptr = '\0';
            cptr = strrchr(dir, '/');       /* one level up */
        }

        if (cptr)
            *(cptr + 1) = '\0';             /* keep the / */

        if (find_icao_in_file(acf_icao, dir, "acf_use_engine_running.txt"))
            use_engine_running = 1;

        if (find_icao_in_file(acf_icao, dir, "acf_dont_connect_jetway.txt"))
            dont_connect_jetway = 1;


        logMsg("plane loaded: %c%c%c%c, plane_cg_z: %1.2f, plane_nw_z: %1.2f, plane_mw_z: %1.2f, "
               "pe_y_plane_0_valid: %d, pe_y_plane_0: %0.2f, is_helicopter: %d",
               icao[0], icao[1], icao[2], icao[3], plane_cg_z, plane_nw_z, plane_mw_z,
               pe_y_plane_0_valid, pe_y_plane_0, is_helicopter);
    }
}
