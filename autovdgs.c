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
#include "XPLMCamera.h"
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMNavigation.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"

#include <acfutils/assert.h>
#include <acfutils/airportdb.h>



/* Constants */
static const float F2M=0.3048;	/* 1 ft [m] */
static const float D2R=M_PI/180.0;
static const float DEG2M = 111195.0;    // deg lat to m

/* Rest location [m] of bridge door floor in AutoGate.obj */
static const float OBJ_X= -7.5;
static const float OBJ_Y=  4;

/* Capture distances [m] (to door location, not ref point) */
static const float CAP_X = 10;
static const float CAP_Z = 70;	/* (50-80 in Safedock2 flier) */
static const float GOOD_Z= 0.5;
static const float NEW_Z = 20;	/* Max distance to fudge new Ramp Start */

/* DGS distances [m]     (to door location, not ref point) */
static const float AZI_X = 5;	/* Azimuth guidance */
static const float AZI_Z = 50;	/* Azimuth guidance */
static const float REM_Z = 12;	/* Distance remaining */

/* Permissable distance [m] of DGS from gate origin */
static const float DGS_X = 10;
static const float DGS_Z =-50;		/* s2.2 of Safedock Manual says 50m from nose */
static const float DGS_H = 0.2f;	/* ~11 degrees. s2.3 of Safedock Manual says 9 or 12 degrees */

static const float WAITTIME=1;	/* Time to wait before engaging */
static const float DURATION=15;	/* Time to engage/disengage */

static const float POLLTIME=5;	/* How often to check we're still in range of our gate */

static const float VDGS_RAMP_DIST = 10.0;
extern float gate_x, gate_y, gate_z, gate_h;		/* active gate */
extern float lat, vert, moving;

/* types */
typedef enum
{
    DISABLED=0, NEWPLANE, IDLE, IDFAIL, TRACK, GOOD, BAD, ENGAGE, DOCKED, DISENGAGE, DISENGAGED
} state_t;

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
static const char pluginDesc[]="Automatically provides";

static XPLMWindowID windowId = NULL;
static state_t state = DISABLED;
static float timestamp;
static int plane_type;
static float door_x, door_y, door_z;		/* door offset relative to ref point */

static char xpdir[512];
static const char *psep;

/* Datarefs */
static XPLMDataRef ref_plane_x, ref_plane_y, ref_plane_z, ref_plane_psi;
static XPLMDataRef ref_plane_lat, ref_plane_lon, ref_plane_elevation, ref_gear_fnrml;
static XPLMDataRef ref_beacon;
static XPLMDataRef ref_draw_object_x, ref_draw_object_y, ref_draw_object_z, ref_draw_object_psi;
static XPLMDataRef ref_acf_descrip, ref_acf_icao;
static XPLMDataRef ref_acf_cg_y, ref_acf_cg_z;
static XPLMDataRef ref_acf_door_x, ref_acf_door_y, ref_acf_door_z;
static XPLMDataRef ref_total_running_time_sec;
static XPLMProbeRef ref_probe;

/* Published DataRefs */
static XPLMDataRef ref_vert, ref_lat, ref_moving;
static XPLMDataRef ref_status, ref_icao, ref_id1, ref_id2, ref_id3, ref_id4, ref_lr, ref_track;
static XPLMDataRef ref_azimuth, ref_distance, ref_distance2;

/* Published DataRef values */
float lat, vert, moving;
static int status, id1, id2, id3, id4, lr, track;
static int icao[4];
static float azimuth, distance, distance2;

/* Internal state */
static float last_gate_x, last_gate_y, last_gate_z;	/* last gate object examined */
static float last_gate_update = 0;		/* and the time we examined it */
float gate_x, gate_y, gate_z, gate_h;		/* active gate */
static float gate_update=0;			/* and the time we examined it */
int gate_AutoVDGS;				/* active gate is an AutoVDGS, not a standalone dummy */
static float last_dgs_x, last_dgs_y, last_dgs_z;	/* last dgs object examined */
static float last_dgs_update = 0;		/* and the time we examined it */
static float dgs_x, dgs_y, dgs_z;		/* active DGS */
static float running_state, beacon_last_pos, beacon_off_ts, beacon_on_ts;  /* running state, last switch_pos, ts of last switch action */
static airportdb_t airportdb;
static airport_t *arpt;
static int on_ground;
static float on_ground_ts;
static geo_pos3_t cur_pos;

static const ramp_start_t *nearest_ramp;
static float vdgs_lat, vdgs_lon, vdgs_alt, vdgs_hdg;

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
    VDGS_DR_NUM             // # of drefs
};

// keep exactly the same order
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
    NULL
};

static XPLMInstanceRef vdgs_inst_ref;

/* Known plane descriptions */
static const db_t planedb[]={/* lng   lat  vert  type */
    {"A300",	21.0, -8.0, -1.0,  0},
    {"A310",	18.0, -8.0, -1.0,  1},
    {"A318",	16.7, -6.0, -1.5,  2},
    {"A319",	21.7, -6.0, -1.5,  2},	/* xpfw */
    {"A320-200",16.7, -6.0, -1.5,  2},	/* xpfw */
    {"A320",	18.3, -6.0, -1.4,  2},
    {"A321-200", 2.5, -6.0, -1.5,  2},	/* xpfw */
    {"A321",	17.5, -6.0, -1.4,  2},
    {"A330",	   0,    0,    0,  3},
    {"A340",	19.6, -8.0, -1.2,  4},
    {"A350",	   0,    0,    0,  5},
    {"A380",	23.0, -9.7, -6.0,  6}, /* first door */
    //	{"A380",	56.5, -11,  -6.0,  6}, /* second door */
    {"717",	   0,    0,    0,  7},
    {"737-700",	15.0, -6.0, -1.2,  8},
    {"737 800",	16.8, -5.2, -1.5,  8},	/* xpfw b26*/
    {"737-800",	17.4, -6.0, -1.4,  8},
    {"738", 	17.4, -6.0, -1.4,  8},
    {"737", 	17.4, -6.0, -1.4,  8},
    {"747 400", 30.9, -9.6, -2.2,  9},	/* xpfw */
    {"747", 	31.8, -9.4, -3.8,  9},	/* XP840b6 first door */
    {"757",	   0,    0,    0, 10},
    {"767", 	18.2, -7.5, -1.5, 11},
    {"777 200",	39.8, -9.0, -1.4, 12},	/* xpfw 777-200 ER & LR - note nose is at 18.39 */
    {"777", 	21.7, -9.0, -2.4, 12},
    {"787",	   0,    0,    0, 13},
    {"RJ70",	 6.6, -5.3, -3.0, 14},
    {"RJ 70",	 6.6, -5.3, -3.0, 14},
    {"RJ85",	 5.3, -5.7, -2.0, 14},
    {"RJ 85",	 5.3, -5.7, -2.0, 14},
    {"RJ100",	 0.9, -5.7, -2.0, 14},
    {"RJ 100",	 0.9, -5.7, -2.0, 14},
    {"md-11",	15.9, -7.6, -1.6, 15},
    {"MD11",	17.0, -7.7, -1.4, 15},
};

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
    gate_x=gate_y=gate_z=gate_h=gate_update=0;
    gate_AutoVDGS=0;
    dgs_x=dgs_y=dgs_z=0;
    last_gate_x = last_gate_y = last_gate_z = last_gate_update = 0;
    last_dgs_x = last_dgs_y = last_dgs_z = last_dgs_update = 0;
    vert=lat=moving=0;
    status=id1=id2=id3=id4=lr=track=0;
    azimuth=distance=distance2=0;

    running_state = beacon_last_pos = XPLMGetDatai(ref_beacon);
    beacon_on_ts = beacon_off_ts = -10.0;
}

/* Reset new plane state after drawing */
static float newplanecallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    if (state == NEWPLANE) state = IDLE;
    return 0;	/* Don't call again */
}

static void newplane(void)
{
    char acf_descrip[129];
    char acf_icao[41];
    int i;

    resetidle();
    acf_descrip[128]=0;		/* Not sure if XPLMGetDatab NULL terminates */
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

    if (ref_acf_door_x!=NULL)
    {
        door_x=XPLMGetDataf(ref_acf_door_x);	/* 0 if unset */
        door_y=XPLMGetDataf(ref_acf_door_y);
        door_z=XPLMGetDataf(ref_acf_door_z);
    }
    else
        door_x=door_y=door_z=0;

    if ((!door_x || plane_type==15) && XPLMGetDatab(ref_acf_descrip, acf_descrip, 0, 128))
        /* Try descriptions */
        for (i=0; i<sizeof(planedb)/sizeof(db_t); i++)
            if (strstr(acf_descrip, planedb[i].key) && (door_x || planedb[i].lat))
            {
                if (plane_type==15)
                    plane_type=planedb[i].type;
                if (!door_x)
                {
                    door_x = F2M * planedb[i].lat;
                    door_y = F2M * (planedb[i].vert - XPLMGetDataf(ref_acf_cg_y));	/* Adjust relative to static cog */
                    door_z = F2M * (planedb[i].lng  - XPLMGetDataf(ref_acf_cg_z));	/* Adjust relative to static cog */
                }
                break;
            }

    if (!door_x)
    {
        state = IDLE;
        icao[0]=icao[1]=icao[2]=icao[3]=0;
    }
    else
    {
        int i;
        state = NEWPLANE;	/* Check for alignment at a gate during next frame */

        if (isupper(acf_icao[0]) || isdigit(acf_icao[0]))
            /* DGS objects fall back to using id1-4 datarefs if first character of ICAO field is null */
            for (i=0; i<4; i++)
                icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';
        else
            /* Display canonical ICAO type */
            for (i=0; i<4; i++)
                icao[i] = canonical[plane_type][i];
    }
}

static int check_running()
{
    float now = XPLMGetDataf(ref_total_running_time_sec);

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


/* Calculate location of plane's centreline opposite door in this object's space */
static int localpos(float object_x, float object_y, float object_z, float object_h, float *local_x, float *local_y, float *local_z)
{
    float plane_x, plane_y, plane_z, plane_h;
    float x, y, z;
    float object_hcos, object_hsin;

    plane_x=XPLMGetDataf(ref_plane_x);
    plane_y=XPLMGetDataf(ref_plane_y);
    plane_z=XPLMGetDataf(ref_plane_z);
    plane_h=XPLMGetDataf(ref_plane_psi) * D2R;

    /* Location of plane's centreline opposite door */
    /* Calculation assumes plane is horizontal */
    x=plane_x-door_z*sinf(plane_h);
    y=plane_y+door_y;
    z=plane_z+door_z*cosf(plane_h);

    /* Location of centreline opposite door in this gate's space */
    object_hcos = cosf(object_h);
    object_hsin = sinf(object_h);
    *local_x=object_hcos*(x-object_x)+object_hsin*(z-object_z);
    *local_y=y-object_y;
    *local_z=object_hcos*(z-object_z)-object_hsin*(x-object_x);

    return 0;	/* Return value has no meaning */
}

/* Update published data used by gate and dgs */
static void updaterefs(float now, float local_x, float local_y, float local_z)
{
    int locgood=(fabsf(local_x)<=AZI_X && fabsf(local_z)<=GOOD_Z);
    int running = check_running();

    status=id1=id2=id3=id4=lr=track=0;
    azimuth=distance=distance2=0;

    switch (state)
    {
    case IDFAIL:
        lr=3;	/* Stop */
        status=5;
        break;

    case TRACK:
        if (locgood)
        {
            state=GOOD;
            timestamp=now;
        }
        else if (local_z<-GOOD_Z)
            state=BAD;
        else
        {
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
                if (azimuth<=-0.5f)
                    lr=1;
                else if (azimuth>=0.5f)
                    lr=2;
                else
                    lr=0;
                if (local_z-GOOD_Z <= REM_Z/2)
                {
                    track=3;
                    distance2=distance;
                }
                else
                {
                    if (local_z-GOOD_Z > REM_Z)
                        /* azimuth only */
                        distance=REM_Z;
                    track=2;
                    distance2=distance - REM_Z/2;
                }
            }
        }
        break;

    case GOOD:
        if (!locgood)
            state=TRACK;
        else if (running)
        {
            /* Stop */
            lr=3;
            status=2;
        }
        else
        {
            state=ENGAGE;
            timestamp=now;
        }
        break;

    case BAD:
        if (local_z>=-GOOD_Z)
            state=TRACK;
        else
        {
            /* Too far */
            lr=3;
            status=4;
        }
        break;

    case ENGAGE:
        lr=3;
        if (running)
        {
            /* abort - reverse animation */
            state=DISENGAGE;
            timestamp=now-(timestamp+WAITTIME+DURATION-now);
        }
        else if (now>timestamp+WAITTIME+DURATION)
            state=DOCKED;
        else if (now>timestamp+WAITTIME)
        {
            float ratio=(now-(timestamp+WAITTIME))/DURATION;
            status=3;	/* OK */
            lat =(door_x-OBJ_X) * ratio;
            vert=(local_y-OBJ_Y) * ratio;
            moving=1;
        }
        else
            status=2;	/* Stop */
        break;

    case DOCKED:
        /* Blank */
        if (running)
        {
            state=DISENGAGE;
            timestamp=now;
        }
        else
        {
            lat =door_x-OBJ_X;
            vert=local_y-OBJ_Y;
            moving=0;
        }
        break;

    case DISENGAGE:
        /* Blank */
        if (now>timestamp+DURATION)
        {
            state=DISENGAGED;
            lat=vert=moving=0;
        }
        else
        {
            float ratio=1 - (now-timestamp)/DURATION;
            lat =(door_x-OBJ_X) * ratio;
            vert=(local_y-OBJ_Y) * ratio;
            moving=1;
        }
        break;

    case DISENGAGED:
        /* Blank */
        if (local_z-GOOD_Z > AZI_Z || fabsf(local_x) > AZI_X)
            /* Go back to lead-in */
            state=TRACK;
        break;

    default:
        /* Shouldn't be here if state<=IDLE */
        assert(0);
    }
}

static float getgatefloat(XPLMDataRef inRefcon)
{
    float now;
    float plane_x, plane_z;
    float object_x, object_y, object_z, object_h;
    float local_x, local_y, local_z;

    now = XPLMGetDataf(ref_total_running_time_sec);
    object_x = XPLMGetDataf(ref_draw_object_x);
    object_y = XPLMGetDataf(ref_draw_object_y);
    object_z = XPLMGetDataf(ref_draw_object_z);

    if (last_gate_update==now && last_gate_x==object_x && last_gate_y==object_y && last_gate_z==object_z)
    {
        /* Same frame and object as last calculation */
        return (gate_x==object_x && gate_y==object_y && gate_z==object_z) ? *(float*)inRefcon : 0;
    }
    else
    {
        last_gate_update = now;
        last_gate_x = object_x;
        last_gate_y = object_y;
        last_gate_z = object_z;
    }

    if (state>IDLE && (gate_x!=object_x || gate_y!=object_y || gate_z!=object_z))
    {
        /* We're tracking and it's not by this gate */

        if (now-gate_update > POLLTIME)
        {
            /* We haven't seen our gate in a while - check we're still in range in case we've been moved across the airport */
            localpos(gate_x, gate_y, gate_z, gate_h, &local_x, &local_y, &local_z);
            if (fabsf(local_x)>CAP_X || local_z<DGS_Z || local_z>CAP_Z)
                resetidle();	/* Just gone out of range of the tracking gate */
                /* Fall through */
            else
                return 0;
        }
        return 0;
    }

    plane_x=XPLMGetDataf(ref_plane_x);
    plane_z=XPLMGetDataf(ref_plane_z);
    object_h=XPLMGetDataf(ref_draw_object_psi) * D2R;

    if (((object_x-plane_x) * (object_x-plane_x) + (object_z-plane_z) * (object_z-plane_z)) > (CAP_Z * CAP_Z) ||
        localpos(object_x, object_y, object_z, object_h, &local_x, &local_y, &local_z) ||
        fabsf(local_x)>CAP_X || local_z<DGS_Z || local_z>CAP_Z)
    {
        /* Not in range of this gate */

        if (gate_x==object_x && gate_y==object_y && gate_z==object_z)
            resetidle();	/* Just gone out of range of the tracking gate */

        if (state == NEWPLANE)
            XPLMSetFlightLoopCallbackInterval(newplanecallback, -1, 1, NULL);	/* Reset newplane state before next frame */

        return 0;
    }

    /* In range of this gate */

    if (gate_x!=object_x || gate_y!=object_y || gate_z!=object_z)
    {
        /* Just come into range */

        if (state == NEWPLANE && fabsf(local_z) < NEW_Z)
        {
            /* Fudge plane's position to line up with this gate */
            float object_hcos, object_hsin;

            object_hcos = cosf(object_h);
            object_hsin = sinf(object_h);
            XPLMSetDataf(ref_plane_x, plane_x + local_z * object_hsin - local_x * object_hcos);
            XPLMSetDataf(ref_plane_z, plane_z - local_z * object_hcos - local_x * object_hsin);
            localpos(object_x, object_y, object_z, object_h, &local_x, &local_y, &local_z);	/* recalc */

            int running = check_running();
            state = running ? TRACK : DOCKED;
        }
        else if (!door_x)
        {
            state = IDFAIL;
        }
        else
        {
            /* Approaching gate */
            state = TRACK;
        }
        gate_x=object_x;
        gate_y=object_y;
        gate_z=object_z;
        gate_h=object_h;

        if ((float*)inRefcon == &lat)	/* Standalone DGS dummy gate only uses vert */
            gate_AutoVDGS = -1;		/* Relies on lat occurring in the .obj file before vert */
    }

    gate_update = now;
    updaterefs(now, local_x, local_y, local_z);
    return *(float*)inRefcon;
}


static int getdgs(void)
{
    float now, object_x, object_y, object_z;
    float local_x, local_y, local_z;

    if (state <= IDLE) return 0;	/* Only interested in DGSs if we're in range of a gate */

    now = XPLMGetDataf(ref_total_running_time_sec);
    object_x = XPLMGetDataf(ref_draw_object_x);
    object_y = XPLMGetDataf(ref_draw_object_y);
    object_z = XPLMGetDataf(ref_draw_object_z);

    if (last_dgs_update==now && last_dgs_x==object_x && last_dgs_y==object_y && last_dgs_z==object_z)
    {
        /* Same frame and object as last calculation */
        return (dgs_x==object_x && dgs_y==object_y && dgs_z==object_z);
    }
    else
    {
        last_dgs_update = now;
        last_dgs_x = object_x;
        last_dgs_y = object_y;
        last_dgs_z = object_z;
    }

    if (!(dgs_x || dgs_y || dgs_z))
    {
        /* Haven't yet identified the active dgs */
        float x, z;
        float gate_hcos, gate_hsin;

        float object_h = XPLMGetDataf(ref_draw_object_psi) * D2R;

        /* Check DGS is pointing in the same direction as the gate, and within desired radius */
        if (fabsf(object_h - gate_h) <= DGS_H &&
            ((object_x-gate_x) * (object_x-gate_x) + (object_z-gate_z) * (object_z-gate_z)) <= (DGS_Z * DGS_Z))
        {
            /* Location of this DGS in the active gate's space */
            gate_hcos = cosf(gate_h);
            gate_hsin = sinf(gate_h);
            x = gate_hcos*(object_x-gate_x) + gate_hsin*(object_z-gate_z);
            z = gate_hcos*(object_z-gate_z) - gate_hsin*(object_x-gate_x);

            if (fabsf(x)<=DGS_X && z<=0 && z>=DGS_Z)
            {
                dgs_x=object_x;
                dgs_y=object_y;
                dgs_z=object_z;
            }
            else
                return 0;
        }
        else
            return 0;
    }
    else if (dgs_x!=object_x || dgs_y!=object_y || dgs_z!=object_z)
        /*  Have identified the active dgs and this isn't it */
        return 0;

    /* Re-calculate plane location - can't rely on values from getgate() since that will not be being called if gate no longer in view */
    localpos(gate_x, gate_y, gate_z, gate_h, &local_x, &local_y, &local_z);

    updaterefs(now, local_x, local_y, local_z);
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

    for (const ramp_start_t *ramp = avl_first(&arpt->ramp_starts); ramp != NULL;
        ramp = AVL_NEXT(&arpt->ramp_starts, ramp)) {

        float dlat = 111195.0 * (cur_pos.lat - ramp->pos.lat);
        float dlon = 111195.0 * (cur_pos.lon - ramp->pos.lon) * cos(0.01745* ramp->pos.lat);

        double d = sqrt(dlat * dlat + dlon * dlon);
        if (d < dist) {
            dist = d;
            min_ramp = ramp;
        }
    }

    if (min_ramp != nearest_ramp) {
        nearest_ramp = min_ramp;
        logMsg("ramp: %s, %f, %f, %f, dist: %f", min_ramp->name, min_ramp->pos.lat, min_ramp->pos.lon,
               min_ramp->hdgt, dist);
        double x, y, z, foo, alt;
        XPLMProbeInfo_t probeinfo;
        probeinfo.structSize = sizeof(XPLMProbeInfo_t);
        XPLMWorldToLocal(min_ramp->pos.lat, min_ramp->pos.lon, cur_pos.elev - 50.0, &x, &y, &z);
        if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(ref_probe, x, y, z, &probeinfo)) {
            logMsg("probe failed");
            return;
        }

        XPLMLocalToWorld(probeinfo.locationX, probeinfo.locationY, probeinfo.locationZ, &foo, &foo, &alt);
        logMsg("ramp alt: %f", alt);
        vdgs_hdg = min_ramp->hdgt;
        vdgs_lat = min_ramp->pos.lat;// + VDGS_RAMP_DIST * cosf(D2R * vdgs_hdg) / DEG2M;
        vdgs_lon = min_ramp->pos.lon;// + VDGS_RAMP_DIST * sinf(D2R * vdgs_hdg) * cosf(D2R * vdgs_lat) / DEG2M;
        vdgs_alt = alt;
        logMsg("vdgs pos is : %f, %f, %f, hdg: %f", vdgs_lat, vdgs_lon = min_ramp->pos.lon, vdgs_alt, vdgs_hdg);
    }
}

static void update_vdgs()
{
    if (nearest_ramp == NULL)
        return;

    float drefs[VDGS_DR_NUM];
    memset(drefs, 0, sizeof(drefs));
    drefs[VDGS_DR_STATUS] = 1;
    drefs[VDGS_DR_TRACK] = 2;
    drefs[VDGS_DR_DISTANCE] = 20;
    drefs[VDGS_DR_LR] = 2;

    double x, y, z;
    XPLMWorldToLocal(vdgs_lat, vdgs_lon, vdgs_alt, &x, &y, &z);

    XPLMDrawInfo_t drawinfo;
    drawinfo.structSize = sizeof(drawinfo);
    drawinfo.x = x + VDGS_RAMP_DIST * sinf(D2R * vdgs_hdg);
    drawinfo.y = y;
    drawinfo.z = z - VDGS_RAMP_DIST * cosf(D2R * vdgs_hdg);;
    drawinfo.heading = vdgs_hdg;
    drawinfo.pitch = drawinfo.roll = 0.0;
    XPLMInstanceSetPosition(vdgs_inst_ref, &drawinfo, drefs);
    logMsg("XPLMInstanceSetPosition");
}

static float flight_loop_cb(float inElapsedSinceLastCall,
                float inElapsedTimeSinceLastFlightLoop, int inCounter,
                void *inRefcon)
{
    float now = XPLMGetDataf(ref_total_running_time_sec);
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
                    vdgs_inst_ref = XPLMCreateInstance(vdgs_obj[0], vdgs_dref_list);
                    if (vdgs_inst_ref == NULL) {
                        logMsg("error creating instance");
                    }
                }
            } else {
                arpt = NULL;
                nearest_ramp = NULL;
            }

        }
    }

    if (on_ground) {
        cur_pos = GEO_POS3(XPLMGetDataf(ref_plane_lat), XPLMGetDataf(ref_plane_lon), XPLMGetDataf(ref_plane_elevation));
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

#ifdef DEBUG
static void drawdebug(XPLMWindowID inWindowID, void *inRefcon)
{
    char buf[128];
    int left, top, right, bottom;
    float color[] = { 1.0, 1.0, 1.0 };	/* RGB White */
    float pos[3], gain;
    int running = check_running();

    XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);

    char *statestr[] = { "Disabled", "NewPlane", "Idle", "IDFail", "Track", "Good", "Bad", "Engage", "Docked", "Disengage", "Disengaged" };
    sprintf(buf, "State: %s %s %s", statestr[state], plane_type==15 ? "Unknown" : canonical[plane_type], running ? "Running" : "Parked");
    XPLMDrawString(color, left + 5, top - 10, buf, 0, xplmFont_Basic);
    sprintf(buf, "Door : %10.3f %10.3f %10.3f",       XPLMGetDataf(ref_acf_door_x), XPLMGetDataf(ref_acf_door_y), XPLMGetDataf(ref_acf_door_z));
    XPLMDrawString(color, left + 5, top - 30, buf, 0, xplmFont_Basic);
    sprintf(buf, "Plane: %10.3f %10.3f %10.3f %6.2f", XPLMGetDataf(ref_plane_x), XPLMGetDataf(ref_plane_y), XPLMGetDataf(ref_plane_z), XPLMGetDataf(ref_plane_psi));
    XPLMDrawString(color, left + 5, top - 40, buf, 0, xplmFont_Basic);
    sprintf(buf, "Gate : %10.3f %10.3f %10.3f %6.2f", gate_x, gate_y, gate_z, gate_h/D2R);
    XPLMDrawString(color, left + 5, top - 50, buf, 0, xplmFont_Basic);
    sprintf(buf, "DGS  : %10.3f %10.3f %10.3f", dgs_x, dgs_y, dgs_z);
    XPLMDrawString(color, left + 5, top - 60, buf, 0, xplmFont_Basic);
    sprintf(buf, "Time : %10.3f", timestamp);
    XPLMDrawString(color, left + 5, top - 70, buf, 0, xplmFont_Basic);
    sprintf(buf, "ID   : %1d %1d %1d %1d %c%c%c%c", id1, id2, id3, id4, icao[0], icao[1], icao[2], icao[3]);
    XPLMDrawString(color, left + 5, top - 90, buf, 0, xplmFont_Basic);
    sprintf(buf, "Gate : lat=%6.3f vert=%6.3f moving=%1.0f", lat, vert, moving);
    XPLMDrawString(color, left + 5, top -100, buf, 0, xplmFont_Basic);
    sprintf(buf, "DGS  : status=%1d track=%1d lr=%1d %4.1f %4.1f %4.1f", status, track, lr, azimuth, distance, distance2);
    XPLMDrawString(color, left + 5, top -110, buf, 0, xplmFont_Basic);
}
#endif

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
    ref_draw_object_x  =XPLMFindDataRef("sim/graphics/animation/draw_object_x");
    ref_draw_object_y  =XPLMFindDataRef("sim/graphics/animation/draw_object_y");
    ref_draw_object_z  =XPLMFindDataRef("sim/graphics/animation/draw_object_z");
    ref_draw_object_psi=XPLMFindDataRef("sim/graphics/animation/draw_object_psi");
    ref_acf_descrip    =XPLMFindDataRef("sim/aircraft/view/acf_descrip");
    ref_acf_icao       =XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    ref_acf_cg_y       =XPLMFindDataRef("sim/aircraft/weight/acf_cgY_original");
    ref_acf_cg_z       =XPLMFindDataRef("sim/aircraft/weight/acf_cgZ_original");
    ref_acf_door_x     =XPLMFindDataRef("sim/aircraft/view/acf_door_x");
    ref_acf_door_y     =XPLMFindDataRef("sim/aircraft/view/acf_door_y");
    ref_acf_door_z     =XPLMFindDataRef("sim/aircraft/view/acf_door_z");
    ref_total_running_time_sec=XPLMFindDataRef("sim/time/total_running_time_sec");
    ref_probe    =XPLMCreateProbe(xplm_ProbeY);

    /* Published Datarefs */
    ref_vert     =floatref("hotbso/AutoVDGS/vert", getgatefloat, &vert);
    ref_lat      =floatref("hotbso/AutoVDGS/lat",  getgatefloat, &lat);
    ref_moving   =floatref("hotbso/AutoVDGS/moving", getgatefloat, &moving);

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

#ifdef DEBUG
    XPLMCreateWindow_t cw = {
        .structSize = sizeof(XPLMCreateWindow_t),
        .left = 10, .top = 750, .right = 310, .bottom = 600,
        .visible = 1,
        .drawWindowFunc = drawdebug,
        .decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle
    };

    windowId = XPLMCreateWindowEx(&cw);
#endif

    XPLMRegisterFlightLoopCallback(newplanecallback, 0, NULL);		/* For checking gate alignment on new location */

    XPLMRegisterFlightLoopCallback(flight_loop_cb, 2.0, NULL);
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    if (windowId) XPLMDestroyWindow(windowId);
    XPLMUnregisterFlightLoopCallback(newplanecallback, NULL);
    XPLMUnregisterFlightLoopCallback(flight_loop_cb, NULL);
    XPLMDestroyProbe(ref_probe);
    if (vdgs_obj[0])
        XPLMUnloadObject(vdgs_obj[0]);
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
