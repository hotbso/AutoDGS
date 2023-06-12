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
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMNavigation.h"
#include "XPLMUtilities.h"

#include <acfutils/assert.h>
#include <acfutils/airportdb.h>


/* Constants */
static const float D2R=M_PI/180.0;

/* Capture distances [m] (to stand) */
static const float CAP_X = 10;
static const float CAP_Z = 70;	/* (50-80 in Safedock2 flier) */
static const float GOOD_Z= 0.5;

/* DGS distances [m]     (to stand) */
static const float AZI_X = 5;	/* Azimuth guidance */
static const float AZI_Z = 50;	/* Azimuth guidance */
static const float REM_Z = 12;	/* Distance remaining */

static const float VDGS_RAMP_DIST = 25.0;

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
static const char pluginName[]="AutoVDGS";
static const char pluginSig[] ="hotbso.AutoVDGS";
static const char pluginDesc[]="Automatically provides VDGS for gateway airports";

static state_t state = DISABLED;
static float timestamp;
static int plane_type;

static char xpdir[512];
static const char *psep;

/* Datarefs */
static XPLMDataRef ref_plane_x, ref_plane_y, ref_plane_z, ref_plane_psi;
static XPLMDataRef ref_plane_lat, ref_plane_lon, ref_plane_elevation, ref_gear_fnrml;
static XPLMDataRef ref_beacon;
static XPLMDataRef ref_acf_icao;
static XPLMDataRef ref_total_running_time_sec;
static XPLMProbeRef ref_probe;

/* Published DataRefs */
static XPLMDataRef ref_status, ref_icao, ref_id1, ref_id2, ref_id3, ref_id4, ref_lr, ref_track;
static XPLMDataRef ref_azimuth, ref_distance, ref_distance2;

/* Published DataRef values */
static int status, id1, id2, id3, id4, lr, track;
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
static vect3_t vdgs_pos;  // position + vector
static const ramp_start_t *nearest_ramp;

static int vdgs_type = 0;
static XPLMObjectRef vdgs_obj[2];

enum _VDGS_DREF {
    VDGS_DR_STATUS,
    VDGS_DR_ID1,
    VDGS_DR_ID2,
    VDGS_DR_ID3,
    VDGS_DR_ID4,
    VDGS_DR_LR,
    VDGS_DR_TRACK,
    VDGS_DR_AZIMUTH,
    VDGS_DR_DISTANCE,
    VDGS_DR_DISTANCE2,
    VDGS_DR_ICAO_0,
    VDGS_DR_ICAO_1,
    VDGS_DR_ICAO_2,
    VDGS_DR_ICAO_3,
    VDGS_DR_NUM             // # of drefs
};

// keep exactly the same order as list above
static const char *vdgs_dref_list[] = {
    "hotbso/dgs/status",
    "hotbso/dgs/id1",
    "hotbso/dgs/id2",
    "hotbso/dgs/id3",
    "hotbso/dgs/id4",
    "hotbso/dgs/lr",
    "hotbso/dgs/track",
    "hotbso/dgs/azimuth",
    "hotbso/dgs/distance",
    "hotbso/dgs/distance2",
    "hotbso/dgs/icao",          // that's an array of 4
    NULL
};

static XPLMInstanceRef vdgs_inst_ref;


/* Known plane ICAOs */
static const icao_t icaodb[]={
    {"A30",  0},
    {"A3ST", 0},
    {"A318", 2},
    {"A319", 2},
    {"A32",  2},
    {"A310", 1},	/* Note after A318/A319 */
    {"A33",  3},
    {"A34",  4},
    {"A35",  5},
    {"A38",  6},
    {"B71",  7},
    {"MD8",  7},
    {"MD9",  7},
    {"B73",  8},
    {"E737", 8},
    {"B74",  9},
    {"BSCA", 9},
    {"B75",  10},
    {"B76",  11},
    {"E767", 11},
    {"B77",  12},
    {"B78",  13},
    {"RJ",   14},
    {"B46",  14},
};

/* Canonical ICAOs for known planes */
static char canonical[16][5] = {
    "A300",
    "A310",
    "A320",
    "A330",
    "A340",
    "A350",
    "A380",
    "B717",
    "B737",
    "B747",
    "B757",
    "B767",
    "B777",
    "B787",
    "AVRO",
    "APRH",
};


static void resetidle(void)
{
    state=IDLE;
    status=id1=id2=id3=id4=lr=track=0;
    azimuth=distance=distance2=0;

    running_state = beacon_last_pos = XPLMGetDatai(ref_beacon);
    beacon_on_ts = beacon_off_ts = -10.0;

    if (vdgs_inst_ref) {
        XPLMDestroyInstance(vdgs_inst_ref);
        vdgs_inst_ref = NULL;
    }
}

static void newplane(void)
{
    char acf_icao[41];
    int i;

    resetidle();
    acf_icao[40]=0;		/* Not sure if XPLMGetDatab NULL terminates */

    /* Find ICAO code */
    plane_type=15;	/* unknown */
    if (ref_acf_icao!=NULL && XPLMGetDatab(ref_acf_icao, acf_icao, 0, 40))
        for (i=0; i<sizeof(icaodb)/sizeof(icao_t); i++)
            if (!strncmp(acf_icao, icaodb[i].key, strlen(icaodb[i].key)))
            {
                plane_type=icaodb[i].type;
                break;
            }

    if (isupper(acf_icao[0]) || isdigit(acf_icao[0]))
        /* DGS objects fall back to using id1-4 datarefs if first character of ICAO field is null */
        for (i=0; i<4; i++)
            icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';
    else
        /* Display canonical ICAO type */
        for (i=0; i<4; i++)
            icao[i] = canonical[plane_type][i];
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

static int getdgs(void)
{
    return -1;
}

static float getdgsfloat(XPLMDataRef inRefcon)
{
    return getdgs() ? *(float*)inRefcon : 0;
}

static int getdgsint(XPLMDataRef inRefcon)
{
    return getdgs() ? *(int*)inRefcon : 0;
}

static int getdgsicao(XPLMDataRef inRefcon, int *outValues, int inOffset, int inMax)
{
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


static XPLMDataRef floatref(char *inDataName, XPLMGetDataf_f inReadFloat, float *inRefcon)
{
    return XPLMRegisterDataAccessor(inDataName, xplmType_Float, 0, NULL, NULL, inReadFloat, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, inRefcon, 0);
}

static XPLMDataRef intref(char *inDataName, XPLMGetDatai_f inReadInt, int *inRefcon)
{
    return XPLMRegisterDataAccessor(inDataName, xplmType_Int, 0, inReadInt, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, inRefcon, 0);
}

static XPLMDataRef intarrayref(char *inDataName, XPLMGetDatavi_f inReadIntArray, int *inRefcon)
{
    return XPLMRegisterDataAccessor(inDataName, xplmType_IntArray, 0, NULL, NULL, NULL, NULL, NULL, NULL, inReadIntArray, NULL, NULL, NULL, NULL, NULL, inRefcon, 0);
}

static void find_nearest_ramp()
{
    if (arpt == NULL)
        return;

    double dist = 1.0E10;
    const ramp_start_t *min_ramp = NULL;

    float plane_x = XPLMGetDataf(ref_plane_x);
    float plane_z = XPLMGetDataf(ref_plane_z);

    float plane_elevation = XPLMGetDataf(ref_plane_elevation);
    float stand_y = 0.0;    // to avoid compiler warning

    XPLMProbeInfo_t probeinfo;
    probeinfo.structSize = sizeof(XPLMProbeInfo_t);

    for (const ramp_start_t *ramp = avl_first(&arpt->ramp_starts); ramp != NULL;
        ramp = AVL_NEXT(&arpt->ramp_starts, ramp)) {

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

        // move vdgs some distance away
        vdgs_pos = VECT3(stand_x + VDGS_RAMP_DIST * stand_dir_x, stand_y, stand_z + VDGS_RAMP_DIST * stand_dir_z);
        state = TRACK;
    }
}

static void update_vdgs()
{
    if (nearest_ramp == NULL) {
        state = IDLE;
        return;
    }

    XPLMDrawInfo_t drawinfo;
    float drefs[VDGS_DR_NUM];
    memset(drefs, 0, sizeof(drefs));

    drawinfo.structSize = sizeof(drawinfo);
    drawinfo.x = vdgs_pos.x;
    drawinfo.y = vdgs_pos.y;
    drawinfo.z = vdgs_pos.z;
    drawinfo.heading = stand_hdg;
    drawinfo.pitch = drawinfo.roll = 0.0;

    // xform plane pos into stand local coordinate system
    float dx = stand_x - XPLMGetDataf(ref_plane_x);
    float dz = stand_z - XPLMGetDataf(ref_plane_z);
    float local_z = dx * stand_dir_x + dz * stand_dir_z;
    float local_x = dx * stand_dir_z - dz * stand_dir_x;
    logMsg("local z, x %f, %f", local_z, local_x);

    int locgood=(fabsf(local_x)<=AZI_X && fabsf(local_z)<=GOOD_Z);
    int running = check_running();

    status=id1=id2=id3=id4=lr=track=0;
    azimuth=distance=distance2=0;

    switch (state) {
        case IDLE:
            if (running
                && (local_z-GOOD_Z <= AZI_Z)
                && (fabsf(local_x) <= AZI_X))
                track=1;
            /* FALLTHROUGH */

        case TRACK:
            if (locgood)
            {
                state=GOOD;
                timestamp=now;
            }
            else if (local_z<-GOOD_Z) {
                timestamp = now;
                state=BAD;
            } else {
                status=1;	/* plane id */
                if (plane_type<4)
                    id1=plane_type+1;
                else if (plane_type<8)
                    id2=plane_type-3;
                else if (plane_type<12)
                    id3=plane_type-7;
                else
                    id4=plane_type-11;

                if (local_z-GOOD_Z > AZI_Z ||
                    fabsf(local_x) > AZI_X)
                    track=1;	/* lead-in only */
                else
                {
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

                    if (local_z-GOOD_Z <= REM_Z/2){
                        track=3;
                        distance2=distance;
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

    logMsg("ramp: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, distance2: %0.2f, azimuth: %0.2f",
           nearest_ramp->name, statestr[state], status, track, lr, distance, distance2, azimuth);

    if (state > IDLE) {
        if (vdgs_inst_ref == NULL) {
            vdgs_inst_ref = XPLMCreateInstance(vdgs_obj[vdgs_type], vdgs_dref_list);
            if (vdgs_inst_ref == NULL) {
                logMsg("error creating instance");
                state = DISABLED;
                return;
            }
        }

        drefs[VDGS_DR_STATUS] = status;
        drefs[VDGS_DR_TRACK] = track;
        drefs[VDGS_DR_DISTANCE] = distance;
        drefs[VDGS_DR_DISTANCE2] = distance2;
        drefs[VDGS_DR_AZIMUTH] = azimuth;
        drefs[VDGS_DR_LR] = lr;

        for (int i = 0; i < 4; i++)
            drefs[VDGS_DR_ICAO_0 +i] = icao[i];

        XPLMInstanceSetPosition(vdgs_inst_ref, &drawinfo, drefs);
    }
}

static float flight_loop_cb(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,
                void *inRefcon)
{
    if (state == DISABLED)
        return 0.0;

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

        }
    }

    if (on_ground) {
        find_nearest_ramp();
        update_vdgs();
    }

    return 2.0;
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

	log_init(XPLMDebugString, "AutoVDGS");

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
    ref_probe    =XPLMCreateProbe(xplm_ProbeY);

    /* Published Datarefs */
    ref_status   =  intref("hotbso/dgs/status",    getdgsint, &status);
    ref_icao  =intarrayref("hotbso/dgs/icao",      getdgsicao, icao);
    ref_id1      =  intref("hotbso/dgs/id1",       getdgsint, &id1);
    ref_id2      =  intref("hotbso/dgs/id2",       getdgsint, &id2);
    ref_id3      =  intref("hotbso/dgs/id3",       getdgsint, &id3);
    ref_id4      =  intref("hotbso/dgs/id4",       getdgsint, &id4);
    ref_lr       =  intref("hotbso/dgs/lr",        getdgsint, &lr);
    ref_track    =  intref("hotbso/dgs/track",     getdgsint, &track);
    ref_azimuth  =floatref("hotbso/dgs/azimuth",   getdgsfloat, &azimuth);
    ref_distance =floatref("hotbso/dgs/distance",  getdgsfloat, &distance);
    ref_distance2=floatref("hotbso/dgs/distance2", getdgsfloat, &distance2);

    vdgs_obj[0] = XPLMLoadObject("Resources/plugins/AutoVDGS/resources/Marshaller.obj");
    if (vdgs_obj[0] == NULL) {
        logMsg("error loading Marshaller");
        return 0;
    }

    vdgs_obj[1] = XPLMLoadObject("Resources/plugins/AutoVDGS/resources/SafedockT2-6m-pole.obj");
    if (vdgs_obj[1] == NULL) {
        logMsg("error loading SafedockT2");
        return 0;
    }

    XPLMRegisterFlightLoopCallback(flight_loop_cb, 2.0, NULL);
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    XPLMUnregisterFlightLoopCallback(flight_loop_cb, NULL);
    XPLMDestroyProbe(ref_probe);

    for (int i = 0; i < 2; i++)
        if (vdgs_obj[i])
            XPLMUnloadObject(vdgs_obj[i]);
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
