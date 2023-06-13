/*
 * AutoVDGS
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

/* Capture distances [m] (to stand) */
//static const float CAP_X = 10;
//static const float CAP_Z = 70;	/* (50-80 in Safedock2 flier) */
static const float GOOD_Z= 0.5;

/* DGS distances [m]     (to stand) */
static const float AZI_X = 5;	/* Azimuth guidance */
static const float AZI_Z = 50;	/* Azimuth guidance */
static const float REM_Z = 12;	/* Distance remaining */

static const float DGS_RAMP_DIST = 25.0;

/* types */
typedef enum
{
    DISABLED=0, IDLE, TRACK, GOOD, BAD, PARKED
} state_t;

const char * const statestr[] = { "Disabled", "Idle", "Track", "Good", "Bad", "Parked" };

typedef struct {
    const char *key;
    const float lng, lat, vert;
    const int type;
} db_t;

typedef struct {
    const char *key;
    const int type;
} icao_t;


/* Globals */
static const char pluginName[]="AutoDGS";
static const char pluginSig[] ="hotbso.AutoDGS";
static const char pluginDesc[]="Automatically provides DGS for gateway airports";

static state_t state = DISABLED;
static float timestamp;

static char xpdir[512];
static const char *psep;

static XPLMCommandRef cycle_dgs_cmdr;

/* Datarefs */
static XPLMDataRef ref_plane_x, ref_plane_y, ref_plane_z, ref_plane_psi;
static XPLMDataRef ref_plane_lat, ref_plane_lon, ref_plane_elevation, ref_gear_fnrml;
static XPLMDataRef ref_beacon;
static XPLMDataRef ref_acf_icao;
static XPLMDataRef ref_total_running_time_sec;
static XPLMProbeRef ref_probe;

/* Published DataRef values */
static int status, track, lr;
static int icao[4];
static float azimuth, distance, distance2;

/* Internal state */
static float now;           /* current timestamp */
static float running_state, beacon_last_pos, beacon_off_ts, beacon_on_ts;  /* running state, last switch_pos, ts of last switch action */
static airportdb_t airportdb;
static airport_t *arpt;
static int on_ground;
static float on_ground_ts;
static float stand_x, stand_z, stand_dir_x, stand_dir_z, stand_hdg;
static vect3_t dgs_pos;  // position + vector
static const ramp_start_t *nearest_ramp;

static int dgs_type = 1;
static XPLMObjectRef dgs_obj[2];

enum _DGS_DREF {
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_AZIMUTH,
    DGS_DR_DISTANCE,
    DGS_DR_DISTANCE2,
    DGS_DR_ICAO,
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
    "hotbso/dgs/icao",          // that's an array of 4
    NULL
};

static XPLMInstanceRef dgs_inst_ref;


static void resetidle(void)
{
    state = IDLE;
    status = lr = track = 0;
    azimuth = distance = distance2 = 0;

    running_state = beacon_last_pos = XPLMGetDatai(ref_beacon);
    beacon_on_ts = beacon_off_ts = -10.0;

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }
}

static void newplane(void)
{
    char acf_icao[41];

    resetidle();

    memset(acf_icao, 0, sizeof(acf_icao));
    if (ref_acf_icao)
        XPLMGetDatab(ref_acf_icao, acf_icao, 0, 40);

    for (int i=0; i<4; i++)
        icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';
}

static int check_running()
{
    /* when checking the beacon guard against power transition when switching
       to the APU generator (e.g. for the ToLiss fleet.
       Report only state transitions when the new state persisted for 3 seconds */
    int beacon = XPLMGetDatai(ref_beacon);
    if (beacon) {
        if (! beacon_last_pos) {
            beacon_on_ts = now;
            beacon_last_pos = 1;
        } else if (now > beacon_on_ts + 3.0)
            running_state = 1;
    } else {
        if (beacon_last_pos) {
            beacon_off_ts = now;
            beacon_last_pos = 0;
        } else if (now > beacon_off_ts + 3.0)
            running_state = 0;
   }

   return running_state;
}

// dummy accessor routines
static float getdgsfloat(XPLMDataRef inRefcon)
{
    return -1.0;
}

// the will obviously be called even with the instancing interface
static int getdgsicao(XPLMDataRef inRefcon, int *outValues, int inOffset, int inMax)
{
    //logMsg("getdgsintv outValues %p, inOffset %d, inMax %d", outValues, inOffset, inMax);
    int i;

    if (outValues==NULL)
        return 4;
    else if (inMax<=0 || inOffset<0 || inOffset>=4)
        return 0;

    if (inMax+inOffset > 4)
        inMax = 4-inOffset;

    /* We get called a lot, so don't bother checking for updated values */
    if (state != TRACK)	/* Only report when active and tracking */
        for (i=0; i<inMax; outValues[i++] = 0);
    else
        for (i=0; i<inMax; i++) outValues[i] = icao[inOffset+i];

    return inMax;
}

static void find_nearest_ramp()
{
    if (arpt == NULL)
        return;

    double dist = 1.0E10;
    const ramp_start_t *min_ramp = NULL;

    float plane_x = XPLMGetDataf(ref_plane_x);
    float plane_z = XPLMGetDataf(ref_plane_z);
    float plane_psi = XPLMGetDataf(ref_plane_psi);

    float plane_elevation = XPLMGetDataf(ref_plane_elevation);
    float stand_y = 0.0;    // to avoid compiler warning

    XPLMProbeInfo_t probeinfo;
    probeinfo.structSize = sizeof(XPLMProbeInfo_t);

    if (ref_probe == NULL)
        ref_probe    =XPLMCreateProbe(xplm_ProbeY);

    for (const ramp_start_t *ramp = avl_first(&arpt->ramp_starts); ramp != NULL;
        ramp = AVL_NEXT(&arpt->ramp_starts, ramp)) {

        if (fabs(plane_psi - ramp->hdgt > 70.0))  // stands in front of plane only
            continue;

        double x, y, z;
        XPLMWorldToLocal(ramp->pos.lat, ramp->pos.lon, plane_elevation, &x, &y, &z);
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(ref_probe, x, y, z, &probeinfo)) {
            logMsg("probe failed");
            return;
        }

        float dx = plane_x - x;
        float dz = plane_z - z;

        float d = sqrt(dx * dx + dz * dz);
        if (d < dist) {
            dist = d;
            min_ramp = ramp;
            stand_x = x;
            stand_y = probeinfo.locationY;
            stand_z = z;
        }
    }

    if (min_ramp != nearest_ramp) {
        nearest_ramp = min_ramp;
        logMsg("ramp: %s, %f, %f, %f, dist: %f", min_ramp->name, min_ramp->pos.lat, min_ramp->pos.lon,
               min_ramp->hdgt, dist);

        stand_hdg = min_ramp->hdgt;
        stand_dir_x = sinf(D2R * (stand_hdg)); stand_dir_z = - cosf(D2R * stand_hdg);

        // move dgs some distance away
        dgs_pos = VECT3(stand_x + DGS_RAMP_DIST * stand_dir_x, stand_y, stand_z + DGS_RAMP_DIST * stand_dir_z);
        state = TRACK;
    }
}

static float update_dgs()
{
    float loop_delay = 2.0;

    if (nearest_ramp == NULL) {
        state = IDLE;
        return loop_delay;
    }

    state_t old_state = state;

    XPLMDrawInfo_t drawinfo;
    float drefs[DGS_DR_NUM];
    memset(drefs, 0, sizeof(drefs));

    drawinfo.structSize = sizeof(drawinfo);
    drawinfo.x = dgs_pos.x;
    drawinfo.y = dgs_pos.y;
    drawinfo.z = dgs_pos.z;
    drawinfo.heading = stand_hdg;
    drawinfo.pitch = drawinfo.roll = 0.0;

    // xform plane pos into stand local coordinate system
    float dx = stand_x - XPLMGetDataf(ref_plane_x);
    float dz = stand_z - XPLMGetDataf(ref_plane_z);
    float local_z = dx * stand_dir_x + dz * stand_dir_z;
    float local_x = dx * stand_dir_z - dz * stand_dir_x;

    int locgood=(fabsf(local_x)<=AZI_X && fabsf(local_z)<=GOOD_Z);
    int running = check_running();

    status = lr = track = 0;
    azimuth = distance = distance2 = 0;

    switch (state) {
        case IDLE:
            if (running
                && (local_z-GOOD_Z <= AZI_Z)
                && (fabsf(local_x) <= AZI_X))
                track=1;
            /* FALLTHROUGH */

        case TRACK:
            loop_delay = 0.5;
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
                        loop_delay = 0.25;
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
            loop_delay = 0.5;
            if (!locgood)
                state = TRACK;
            else if (running) {
                /* Stop */
                lr=3;
                status=2;
            } else {
                state = PARKED;
                timestamp = now;
                status = 3;
                lr = track = 0;
            }
            break;

        case BAD:
            if (!running
                && (now > timestamp + 5.0)) {
                resetidle();
                break;
            }

            if (local_z>=-GOOD_Z)
                state=TRACK;
            else {
                /* Too far */
                lr=3;
                status=4;
            }
            break;

        case PARKED:
            if (now > timestamp + 5.0)
                resetidle();
            break;

        default:
            /* FALLTHROUGH */
    }

    if (old_state != state) {
        logMsg("ramp: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, distance2: %0.2f, azimuth: %0.2f",
               nearest_ramp->name, statestr[state], status, track, lr, distance, distance2, azimuth);
        logMsg("local z, x %f, %f", local_z, local_x);
    }

    if (state > IDLE) {
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

        XPLMInstanceSetPosition(dgs_inst_ref, &drawinfo, drefs);
    }

    return loop_delay;
}

static float flight_loop_cb(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,
                void *inRefcon)
{
    if (state == DISABLED)
        return 0.0;

    float loop_delay = 2.0;

    now = XPLMGetDataf(ref_total_running_time_sec);
    int og = (XPLMGetDataf(ref_gear_fnrml) != 0.0);

    if (og != on_ground && now > on_ground_ts + 10.0) {
        on_ground = og;
        on_ground_ts = now;
        logMsg("transition to on_ground: %d", on_ground);

        if (on_ground) {
            float lat = XPLMGetDataf(ref_plane_lat);
            float lon = XPLMGetDataf(ref_plane_lon);
            char airport_id[50];

            XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
            if (XPLM_NAV_NOT_FOUND != ref) {
                XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, airport_id,
                        NULL, NULL);
                arpt = adb_airport_lookup_by_ident(&airportdb, airport_id);
                if (arpt) {
                    logMsg("now on airport: %s", arpt->icao);
                }
            } else {
                arpt = NULL;
                nearest_ramp = NULL;
                state = IDLE;
            }

        } else {
            // transition to airborne
            if (ref_probe) {
                XPLMDestroyProbe(ref_probe);
                ref_probe = NULL;
            }
        }
    }

    if (on_ground) {
        find_nearest_ramp();
        loop_delay = update_dgs();
    }

    return loop_delay;
}

/* call back for cycle cmd */
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

/* call back for menu */
static void
menu_cb(void *menu_ref, void *item_ref)
{
    if (item_ref == &cycle_dgs_cmdr)
        XPLMCommandOnce(cycle_dgs_cmdr);
}

/* Convert path to posix style in-place */
void posixify(char *path)
{
#if APL
    if (*path!='/')
    {
        /* X-Plane 9 - screw around with HFS paths FFS */
        int isfolder = (path[strlen(path)-1]==':');
        CFStringRef hfspath = CFStringCreateWithCString(NULL, path, kCFStringEncodingMacRoman);
        CFURLRef url = CFURLCreateWithFileSystemPath(NULL, hfspath, kCFURLHFSPathStyle, 0);
        CFStringRef posixpath = CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle);
        CFStringGetCString(posixpath, path, PATH_MAX, kCFStringEncodingUTF8);
        CFRelease(hfspath);
        CFRelease(url);
        CFRelease(posixpath);
        if (isfolder && path[strlen(path)-1]!='/') { strcat(path, "/"); }	/* converting from HFS loses trailing separator */
    }
#elif IBM
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
    sprintf(cache_path, "%sOutput%scaches%sAutoVDGS.cache", xpdir, psep, psep);
    fix_pathsep(cache_path);                        /* libacfutils requires a canonical path sep */
    airportdb_create(&airportdb, xpdir, cache_path);
    airportdb.ifr_only = B_TRUE;
    airportdb.global_airports_only = B_TRUE;
    if (!recreate_cache(&airportdb)) {
        logMsg("init failure: recreate_cache failed");
        return 0;
    }

    /* Datarefs */
    ref_plane_x        =XPLMFindDataRef("sim/flightmodel/position/local_x");
    ref_plane_y        =XPLMFindDataRef("sim/flightmodel/position/local_y");
    ref_plane_z        =XPLMFindDataRef("sim/flightmodel/position/local_z");
    ref_plane_psi      =XPLMFindDataRef("sim/flightmodel/position/psi");
    ref_gear_fnrml     =XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    ref_plane_lat      =XPLMFindDataRef("sim/flightmodel/position/latitude");
    ref_plane_lon      =XPLMFindDataRef("sim/flightmodel/position/longitude");
    ref_plane_elevation=XPLMFindDataRef("sim/flightmodel/position/elevation");
    ref_beacon         =XPLMFindDataRef("sim/cockpit2/switches/beacon_on");
    ref_acf_icao       =XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    ref_total_running_time_sec=XPLMFindDataRef("sim/time/total_running_time_sec");

    /* Published scalar datarefs, as we draw with the instancing API the accessors will never be called */
    for (int i = 0; i < DGS_DR_ICAO; i++)
        XPLMRegisterDataAccessor(dgs_dref_list[i], xplmType_Float, 0, NULL, NULL, getdgsfloat,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);

    // this is an array and it will be called
    XPLMRegisterDataAccessor(dgs_dref_list[DGS_DR_ICAO], xplmType_IntArray, 0, NULL, NULL,
                             NULL, NULL, NULL, NULL, getdgsicao,
                             NULL, NULL, NULL, NULL, NULL, NULL, 0);


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

    XPLMMenuID menu = XPLMFindPluginsMenu();
    int sub_menu = XPLMAppendMenuItem(menu, "AutoDGS", NULL, 1);
    XPLMMenuID adgs_menu = XPLMCreateMenu("AutoDGS", menu, sub_menu, menu_cb, NULL);
    XPLMAppendMenuItem(adgs_menu, "Cycle DGS", &cycle_dgs_cmdr, 0);

    cycle_dgs_cmdr = XPLMCreateCommand("AutoDGS/cycle_dgs", "Cycle DGS between Marshaller, VDGS");
    XPLMRegisterCommandHandler(cycle_dgs_cmdr, cmd_cycle_dgs_cb, 0, NULL);

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
    newplane();
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    resetidle();
    state=DISABLED;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void *inParam)
{
    if (state!=DISABLED && inMessage==XPLM_MSG_AIRPORT_LOADED)
        newplane();
}
