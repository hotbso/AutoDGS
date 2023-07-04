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

#define XPLM200
#define XPLM210
#define XPLM300
#define XPLM301

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMNavigation.h"
#include "XPLMUtilities.h"

#include <acfutils/assert.h>
#include <acfutils/airportdb.h>


/* Constants */
static const float D2R=M_PI/180.0;
static const float F2M=0.3048;	/* 1 ft [m] */

/* Capture distances [m] (to stand) */
static const float CAP_X = 10;
static const float CAP_Z = 80;	/* (50-80 in Safedock2 flier) */
static const float GOOD_Z= 0.5;

/* DGS distances [m]     (to stand) */
static const float AZI_X = 5;	/* Azimuth guidance */
static const float AZI_Z = 50;	/* Azimuth guidance */
static const float REM_Z = 12;	/* Distance remaining */

/* place DGS at this dist from stop position */
static const float DGS_RAMP_DIST_DEFAULT = 25.0;
static float dgs_ramp_dist = DGS_RAMP_DIST_DEFAULT;

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

static char xpdir[512];
static const char *psep;

static XPLMCommandRef cycle_dgs_cmdr, move_dgs_closer_cmdr, activate_cmdr, toggle_jetway_cmdr;

/* Datarefs */
static XPLMDataRef ref_plane_x, ref_plane_y, ref_plane_z;
static XPLMDataRef ref_plane_lat, ref_plane_lon, ref_plane_elevation, ref_plane_true_psi;
static XPLMDataRef ref_gear_fnrml, ref_acf_cg_z, ref_gear_z;
static XPLMDataRef ref_beacon, ref_parkbrake, ref_acf_icao, ref_total_running_time_sec;
static XPLMProbeRef ref_probe;

/* Published DataRef values */
static int status, track, lr;
static int icao[4];
static float azimuth, distance, distance2;

/* Internal state */
static float now;           /* current timestamp */
static int beacon_state, beacon_last_pos;   /* beacon state, last switch_pos, ts of last switch action */
static float beacon_off_ts, beacon_on_ts;
static airportdb_t airportdb;
static const airport_t *arpt;
static int on_ground = 1;
static float on_ground_ts;
static float stand_x, stand_y, stand_z, stand_dir_x, stand_dir_z, stand_hdg;
static float dgs_pos_x, dgs_pos_y, dgs_pos_z;
static float plane_ref_z;   // z value of plane's reference point
static const ramp_start_t *nearest_ramp;
static int update_dgs_log_ts;   // throttling of logging
static int dgs_type = 0;
static XPLMObjectRef dgs_obj[2];

enum _DGS_DREF {
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_AZIMUTH,
    DGS_DR_DISTANCE,
    DGS_DR_DISTANCE2,
    DGS_DR_ICAO_0,
    DGS_DR_ICAO_1,
    DGS_DR_ICAO_2,
    DGS_DR_ICAO_3,
    DGS_DR_NUM             // # of drefs
};

// keep exactly the same order as list above
static const char *dgs_dref_list[] = {
    "hotbso/dgs/status",
    "hotbso/dgs/lr",
    "hotbso/dgs/track",
    "hotbso/dgs/azimuth",
    "hotbso/dgs/distance",
    "hotbso/dgs/distance2",
    "hotbso/dgs/icao_0",
    "hotbso/dgs/icao_1",
    "hotbso/dgs/icao_2",
    "hotbso/dgs/icao_3",
    NULL
};

static XPLMInstanceRef dgs_inst_ref;

/* new_state <= INACTIVE */
static void
reset_state(state_t new_state)
{
    if (state != new_state)
        logMsg("setting state to %s", state_str[new_state]);

    state = new_state;
    status = lr = track = 0;
    azimuth = distance = distance2 = 0;

    beacon_state = beacon_last_pos = XPLMGetDatai(ref_beacon);
    beacon_on_ts = beacon_off_ts = -10.0;

    nearest_ramp = NULL;
    dgs_ramp_dist = DGS_RAMP_DIST_DEFAULT;

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }
}

/* set mode to arrival */
static void
set_active()
{
    if (! on_ground) {
        logMsg("can't set active when not on ground");
        return;
    }

    if (state > INACTIVE)
        return;

    float lat = XPLMGetDataf(ref_plane_lat);
    float lon = XPLMGetDataf(ref_plane_lon);
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
        if (arpt) {
            logMsg("found in DGS cache: %s, new state: ACTIVE", arpt->icao);
            state = ACTIVE;
        } else
            logMsg("not a global + IFR airport, sorry no DGS");
    }
}

static int check_beacon()
{
    /* when checking the beacon guard against power transitions when switching
       to the APU generator (e.g. for the ToLiss fleet).
       Report only state transitions when the new state persisted for 3 seconds */

    int beacon_state_prev = beacon_state;

    int beacon = XPLMGetDatai(ref_beacon);
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

    if (beacon_state != beacon_state_prev)
        logMsg("beacon transition to: %d", beacon_state);

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
    XPLMProbeInfo_t probeinfo;
    probeinfo.structSize = sizeof(XPLMProbeInfo_t);

    dgs_pos_x = stand_x + dgs_ramp_dist * stand_dir_x;
    dgs_pos_z = stand_z + dgs_ramp_dist * stand_dir_z;

    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(ref_probe, dgs_pos_x, stand_y, dgs_pos_z, &probeinfo)) {
        logMsg("XPLMProbeTerrainXYZ failed");
        reset_state(ACTIVE);
        return;
    }

    dgs_pos_y = probeinfo.locationY;
}

static void
find_nearest_ramp()
{
    assert(arpt != NULL);

    double dist = 1.0E10;
    const ramp_start_t *min_ramp = NULL;

    float plane_x = XPLMGetDataf(ref_plane_x);
    float plane_z = XPLMGetDataf(ref_plane_z);

    float plane_elevation = XPLMGetDataf(ref_plane_elevation);
    float plane_hdgt = XPLMGetDataf(ref_plane_true_psi);

    XPLMProbeInfo_t probeinfo;
    probeinfo.structSize = sizeof(XPLMProbeInfo_t);

    if (ref_probe == NULL)
        ref_probe = XPLMCreateProbe(xplm_ProbeY);

    for (const ramp_start_t *ramp = avl_first(&arpt->ramp_starts); ramp != NULL;
        ramp = AVL_NEXT(&arpt->ramp_starts, ramp)) {

        if (fabs(rel_angle(plane_hdgt, ramp->hdgt)) > 90.0)
            continue;   // not looking to ramp

        double s_x, s_y, s_z;
        XPLMWorldToLocal(ramp->pos.lat, ramp->pos.lon, plane_elevation, &s_x, &s_y, &s_z);

        float dx = s_x - plane_x;
        float dz = s_z - plane_z;
        float d = sqrt(dx * dx + dz * dz);
        if (d > 170.0) // fast exit
            continue;

        /* transform into gate local coordinate system */
        float s_dir_x =  sinf(D2R * ramp->hdgt);
        float s_dir_z = -cosf(D2R * ramp->hdgt);
        float local_z = dx * s_dir_x + dz * s_dir_z - plane_ref_z;
        float local_x = dx * s_dir_z - dz * s_dir_x;

        //logMsg("stand: %s, z: %2.1f, x: %2.1f", ramp->name, local_z, local_x);

        // behind
        if (local_z < -4.0) {
            //logMsg("behind: %s", ramp->name);
            continue;
        }

        // check whether plane is in a +-50° sector relative to stand
        if (local_z > 10.0) {
            float angle = 0.0;
            angle = atan(local_x / local_z) / D2R;
            //logMsg("angle: %s, %3.1f", ramp->name, angle);
            if (fabsf(angle) > 50.0)
                continue;
        }

        if (d < dist) {
            //logMsg("new min: %s, z: %2.1f, x: %2.1f", ramp->name, local_z, local_x);
            dist = d;
            min_ramp = ramp;
            stand_x = s_x;
            stand_y = s_y;
            stand_z = s_z;
            stand_dir_x = s_dir_x;
            stand_dir_z = s_dir_z;
        }
    }

    if (min_ramp != NULL && min_ramp != nearest_ramp) {
        logMsg("ramp: %s, %f, %f, %f, dist: %f", min_ramp->name, min_ramp->pos.lat, min_ramp->pos.lon,
               min_ramp->hdgt, dist);

        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(ref_probe, stand_x, stand_y, stand_z, &probeinfo)) {
            logMsg("XPLMProbeTerrainXYZ failed");
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
    float loop_delay = 2.0;

    if (state <= INACTIVE)
        return loop_delay;

    find_nearest_ramp();

    if (nearest_ramp == NULL) {
        state = ACTIVE;
        return loop_delay;
    }

    state_t old_state = state;

    // xform plane pos into stand local coordinate system
    float dx = stand_x - XPLMGetDataf(ref_plane_x);
    float dz = stand_z - XPLMGetDataf(ref_plane_z);
    float local_z = dx * stand_dir_x + dz * stand_dir_z - plane_ref_z;
    float local_x = dx * stand_dir_z - dz * stand_dir_x;

    int locgood = (fabsf(local_x)<=AZI_X && fabsf(local_z)<=GOOD_Z);
    int beacon = check_beacon();

    status = lr = track = 0;
    azimuth = distance = distance2 = 0;

    switch (state) {
        case ACTIVE:
            break;

        case ENGAGED:
            loop_delay = 1.0;
            if (beacon) {
                if ((local_z-GOOD_Z <= CAP_Z)
                    && (fabsf(local_x) <= CAP_X)) {
                    state = TRACK;
                    /* FALLTHROUGH */
                } else
                   break;
            } else { // not beacon
                state = PARKED;
                break;
            }

        case TRACK:
            loop_delay = 0.2;
            if (locgood) {
                state=GOOD;
                timestamp=now;
            } else if (local_z<-GOOD_Z) {
                timestamp = now;
                state=BAD;
            } else {
                status=1;	/* plane id */

                if (local_z-GOOD_Z > AZI_Z ||
                    fabsf(local_x) > AZI_X)
                    track=1;	/* lead-in only */
                else {
                    // round to multiples of 0.5m
                    distance=((float)((int)((local_z - GOOD_Z)*2))) / 2;
                    azimuth=((float)((int)(local_x*2))) / 2;
                    if (azimuth>4)	azimuth=4;
                    if (azimuth<-4) azimuth=-4;

                    if (azimuth <= -0.5f)
                        lr=1;
                    else if (azimuth >= 0.5f)
                        lr=2;
                    else
                        lr=0;

                    if (local_z-GOOD_Z <= REM_Z/2) {
                        track=3;
                        distance2=distance;
                        loop_delay = 0.1;
                    } else {
                        if (local_z-GOOD_Z > REM_Z)
                            /* azimuth only */
                            distance=REM_Z;
                        track = 2;
                        distance2 = distance - REM_Z/2;
                    }
                }
            }
            break;

        case GOOD:
            loop_delay = 0.2;
            int parkbrake_set = (XPLMGetDataf(ref_parkbrake) > 0.5);
            if (!locgood)
                state = TRACK;
            else if (! parkbrake_set) {
                /* Stop */
                status = 2;
                lr = 3;
            } else {
                /* brake set */
                state = PARKED;
                status = 3;
                lr = track = 0;
            }
            break;

        case BAD:
            if (!beacon
                && (now > timestamp + 5.0)) {
                reset_state(INACTIVE);
                break;
            }

            if (local_z >= -GOOD_Z)
                state=TRACK;
            else {
                /* Too far */
                lr=3;
                status=4;
            }
            break;

        case PARKED:
            /* wait for beacon off */
            if (! beacon) {
                timestamp = now;
                state = DONE;
                if (operation_mode == MODE_AUTO)
                    XPLMCommandOnce(toggle_jetway_cmdr);
            }
            break;

        case DONE:
            if (now > timestamp + 5.0)
                reset_state(INACTIVE);
            break;

        default:
            break;
    }

    if (state > ACTIVE) {
        // don't flood the log
        if (state != old_state || now > update_dgs_log_ts + 3.0) {
            update_dgs_log_ts = now;
            logMsg("ramp: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, distance2: %0.2f, azimuth: %0.2f",
                   nearest_ramp->name, state_str[state], status, track, lr, distance, distance2, azimuth);
            logMsg("acf position local z, x: %f, %f", local_z, local_x);
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
            dgs_inst_ref = XPLMCreateInstance(dgs_obj[dgs_type], dgs_dref_list);
            if (dgs_inst_ref == NULL) {
                logMsg("error creating instance");
                state = DISABLED;
                return 0.0;
            }
        }

        drefs[DGS_DR_STATUS] = status;
        drefs[DGS_DR_TRACK] = track;
        drefs[DGS_DR_DISTANCE] = distance;
        drefs[DGS_DR_DISTANCE2] = distance2;
        drefs[DGS_DR_AZIMUTH] = azimuth;
        drefs[DGS_DR_LR] = lr;

        if (state == TRACK) {
            for (int i = 0; i < 4; i++)
                drefs[DGS_DR_ICAO_0 + i] = icao[i];

            if (isalpha(icao[3]))
                drefs[DGS_DR_ICAO_3] += 0.8;    // bug in VDGS
        }

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

    now = XPLMGetDataf(ref_total_running_time_sec);
    int og = (XPLMGetDataf(ref_gear_fnrml) != 0.0);

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
            if (ref_probe) {
                XPLMDestroyProbe(ref_probe);
                ref_probe = NULL;
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

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }

    dgs_type = (dgs_type + 1) % 2;
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

/* call back for menu */
static void
menu_cb(void *menu_ref, void *item_ref)
{
    XPLMCommandOnce((XPLMCommandRef)item_ref);
}

/* Convert path to posix style in-place */
void posixify(char *path)
{
#if IBM
    char *c;
    for (c=path; *c; c++) if (*c=='\\') *c='/';
#endif
}


/* =========================== plugin entry points ===============================================*/
PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    sprintf(outName, "%s v%s", pluginName, VERSION);
    strcpy(outSig,  pluginSig);
    strcpy(outDesc, pluginDesc);

	log_init(XPLMDebugString, "AutoDGS");

    logMsg("startup " VERSION);

    /* Refuse to initialise if Fat plugin has been moved out of its folder */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);			/* Get paths in posix format under X-Plane 10+ */

    psep = XPLMGetDirectorySeparator();
	XPLMGetSystemPath(xpdir);


    char cache_path[512];
    sprintf(cache_path, "%sOutput%scaches%sAutoDGS.cache", xpdir, psep, psep);
    fix_pathsep(cache_path);                        /* libacfutils requires a canonical path sep */
    airportdb_create(&airportdb, xpdir, cache_path);
    airportdb.ifr_only = B_TRUE;
    airportdb.global_airports_only = B_TRUE;
    if (!recreate_cache(&airportdb)) {
        logMsg("init failure: recreate_cache failed");
        return 0;
    }

    /* Datarefs */
    ref_plane_x        = XPLMFindDataRef("sim/flightmodel/position/local_x");
    ref_plane_y        = XPLMFindDataRef("sim/flightmodel/position/local_y");
    ref_plane_z        = XPLMFindDataRef("sim/flightmodel/position/local_z");
    ref_gear_fnrml     = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    ref_plane_lat      = XPLMFindDataRef("sim/flightmodel/position/latitude");
    ref_plane_lon      = XPLMFindDataRef("sim/flightmodel/position/longitude");
    ref_plane_elevation= XPLMFindDataRef("sim/flightmodel/position/elevation");
    ref_plane_true_psi = XPLMFindDataRef("sim/flightmodel2/position/true_psi");
    ref_parkbrake      = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");
    ref_beacon         = XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    ref_acf_icao       = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    ref_acf_cg_z       = XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    ref_gear_z         = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
    ref_total_running_time_sec = XPLMFindDataRef("sim/time/total_running_time_sec");

    /* Published scalar datarefs, as we draw with the instancing API the accessors will never be called */
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dref_list[i], xplmType_Float, 0, NULL, NULL, getdgsfloat,
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

    XPLMRegisterDataAccessor("AutoDGS/state_str", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_STATE_STR, NULL);

    XPLMRegisterDataAccessor("AutoDGS/ramp", xplmType_Data, 0, NULL, NULL, NULL,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, api_getbytes, NULL,
                             (void *)API_RAMP, NULL);

    dgs_obj[0] = XPLMLoadObject("Resources/plugins/AutoDGS/resources/Marshaller.obj");
    if (dgs_obj[0] == NULL) {
        logMsg("error loading Marshaller");
        return 0;
    }

    dgs_obj[1] = XPLMLoadObject("Resources/plugins/AutoDGS/resources/SafedockT2-6m-pole.obj");
    if (dgs_obj[1] == NULL) {
        logMsg("error loading SafedockT2");
        return 0;
    }

    /* own commands */
    cycle_dgs_cmdr = XPLMCreateCommand("AutoDGS/cycle_dgs", "Cycle DGS between Marshaller, VDGS");
    XPLMRegisterCommandHandler(cycle_dgs_cmdr, cmd_cycle_dgs_cb, 0, NULL);

    move_dgs_closer_cmdr = XPLMCreateCommand("AutoDGS/move_dgs_closer", "Move DGS closer by 2m");
    XPLMRegisterCommandHandler(move_dgs_closer_cmdr, cmd_move_dgs_closer, 0, NULL);

    activate_cmdr = XPLMCreateCommand("AutoDGS/activate", "Manually activate searching for stands");
    XPLMRegisterCommandHandler(activate_cmdr, cmd_activate_cb, 0, NULL);

    /* menu */
    XPLMMenuID menu = XPLMFindPluginsMenu();
    int sub_menu = XPLMAppendMenuItem(menu, "AutoDGS", NULL, 1);
    XPLMMenuID adgs_menu = XPLMCreateMenu("AutoDGS", menu, sub_menu, menu_cb, NULL);

    XPLMAppendMenuItem(adgs_menu, "Manually activate", &activate_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Cycle DGS", &cycle_dgs_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Move DGS closer by 2m", &move_dgs_closer_cmdr, 0);

    /* foreign commands */
    toggle_jetway_cmdr = XPLMFindCommand("sim/ground_ops/jetway");

    XPLMRegisterFlightLoopCallback(flight_loop_cb, 2.0, NULL);
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    XPLMUnregisterFlightLoopCallback(flight_loop_cb, NULL);
    if (ref_probe)
        XPLMDestroyProbe(ref_probe);

    for (int i = 0; i < 2; i++)
        if (dgs_obj[i])
            XPLMUnloadObject(dgs_obj[i]);
}

PLUGIN_API int XPluginEnable(void)
{
    state = INACTIVE;
    return 1;
}

PLUGIN_API void XPluginDisable(void)
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
        if (ref_acf_icao)
            XPLMGetDatab(ref_acf_icao, acf_icao, 0, 40);

        for (int i=0; i<4; i++)
            icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';

        if (1 == XPLMGetDatavf(ref_gear_z, &plane_ref_z, 0, 1))      // nose wheel
            plane_ref_z = - plane_ref_z;
        else
            plane_ref_z = F2M * XPLMGetDataf(ref_acf_cg_z);         // fall back to CG

        logMsg("plane loaded: %c%c%c%c, plane_ref_z: %1.2f", icao[0], icao[1], icao[2], icao[3], plane_ref_z);
    }
}
