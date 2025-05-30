//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
//    Copyright (C) 2006-2013 Jonathan Harris
//    Copyright (C) 2023, 2025 Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

// This code loosely follows
// Google's style guide: https://google.github.io/styleguide/cppguide.html

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cassert>
#include <fstream>
#include <map>
#include <algorithm>
#include <filesystem>

#include "autodgs.h"
#include "airport.h"
#include "plane.h"

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"

const char * const opmode_str[] = { "Automatic", "Manual" };

// Globals
bool error_disabled;

std::string xp_dir;
std::string base_dir;       // base directory of AutoDGS
std::string sys_cfg_dir;    // <base_dir>/cfg
std::string user_cfg_dir;   // <xp_dir>/Output/AutoDGS

opmode_t operation_mode = MODE_AUTO;
XPLMCommandRef cycle_dgs_cmdr, move_dgs_closer_cmdr, activate_cmdr,
    toggle_ui_cmdr, toggle_jetway_cmdr;

XPLMDataRef plane_x_dr, plane_y_dr, plane_z_dr, is_helicopter_dr, y_agl_dr;
XPLMDataRef plane_lat_dr, plane_lon_dr, plane_elevation_dr, plane_true_psi_dr;
XPLMDataRef gear_fnrml_dr, acf_cg_y_dr, acf_cg_z_dr, gear_z_dr;
XPLMDataRef beacon_dr, parkbrake_dr, acf_icao_dr, total_running_time_sec_dr;
XPLMDataRef percent_lights_dr, xp_version_dr, eng_running_dr, sin_wave_dr;
XPLMDataRef vr_enabled_dr, ground_speed_dr;
static XPLMDataRef zulu_time_minutes_dr, zulu_time_hours_dr;
XPLMProbeRef probe_ref;
XPLMObjectRef dgs_obj[2];

float now;           // current timestamp
int on_ground;

std::unique_ptr<Airport> arpt;

// keep exactly the same order as in enum _DGS_DREF
const char *dgs_dlist_dr[] = {
    "AutoDGS/dgs/status",
    "AutoDGS/dgs/lr",
    "AutoDGS/dgs/track",
    "AutoDGS/dgs/azimuth",
    "AutoDGS/dgs/distance",
    "AutoDGS/dgs/distance_0",
    "AutoDGS/dgs/distance_01",
    "AutoDGS/dgs/icao_0",
    "AutoDGS/dgs/icao_1",
    "AutoDGS/dgs/icao_2",
    "AutoDGS/dgs/icao_3",
    "AutoDGS/dgs/r1c0",
    "AutoDGS/dgs/r1c1",
    "AutoDGS/dgs/r1c2",
    "AutoDGS/dgs/r1c3",
    "AutoDGS/dgs/r1c4",
    "AutoDGS/dgs/r1c5",
    "AutoDGS/dgs/boarding",
    "AutoDGS/dgs/paxno_0",
    "AutoDGS/dgs/paxno_1",
    "AutoDGS/dgs/paxno_2",
    NULL
};

static float time_utc_m0, time_utc_m1, time_utc_h0, time_utc_h1, vdgs_brightness;

static float FlightLoopCb(float inElapsedSinceLastCall,
               float inElapsedTimeSinceLastFlightLoop, int inCounter,
               void *inRefcon);

static XPLMCreateFlightLoop_t flight_loop_ctx = {
    sizeof(XPLMCreateFlightLoop_t),
    xplm_FlightLoop_Phase_BeforeFlightModel,
    FlightLoopCb,
    nullptr
};

static XPLMFlightLoopID flight_loop_id;
static bool pending_plane_loaded_cb = false;    // delayed init

//------------------------------------------------------------------------------------

// set mode to arrival
void
Activate(void)
{
    if (! on_ground) {
        LogMsg("can't set active when not on ground");
        return;
    }

    if (arpt && arpt->state() > Airport::INACTIVE)
        return;

    plane.ResetBeacon();

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);

    // find and load airport I'm on now
    XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL, xplm_Nav_Airport);
    if (XPLM_NAV_NOT_FOUND != ref) {
        char buffer[50];
        XPLMGetNavAidInfo(ref, NULL, &lat, &lon, NULL, NULL, NULL, buffer,
                NULL, NULL);
        std::string airport_id(buffer);
        LogMsg("now on airport: %s", airport_id.c_str());
        if (arpt == nullptr || arpt->name() != airport_id) {   // don't reload same
            arpt = Airport::LoadAirport(airport_id);
        }
    } else {
        LogMsg("airport could not be identified at %0.8f,%0.8f", lat, lon);
        arpt = nullptr;
    }

    if (arpt == nullptr)
        return;

    arpt->ResetState(plane.BeaconOn() ? Airport::ARRIVAL : Airport::INACTIVE);
    LogMsg("airport loaded: '%s', new state: %s", arpt->name().c_str(), Airport::state_str[arpt->state()]);
    UpdateUI();
}

// Dataref accessor, only called for the _utc_ datarefs
static float
GetDgsFloat(void *ref)
{
    if (ref == nullptr)
        return -1.0f;

    return *(float *)ref;
}

static float
FlightLoopCb(float inElapsedSinceLastCall,
               float inElapsedTimeSinceLastFlightLoop, int inCounter,
               void *inRefcon)
{
    static float on_ground_ts;  // debounce ground contact

    try {
        if (pending_plane_loaded_cb) {
            plane.PlaneLoadedCb();
            pending_plane_loaded_cb = false;
        }

        float loop_delay = 2.0;

        now = XPLMGetDataf(total_running_time_sec_dr);
        int og;
        if (plane.is_helicopter)
            og = (XPLMGetDataf(y_agl_dr) < 10.0);
        else
            og = (XPLMGetDataf(gear_fnrml_dr) != 0.0);

        if (og != on_ground && now > on_ground_ts + 10.0) {
            on_ground = og;
            on_ground_ts = now;
            LogMsg("transition to on_ground: %d", on_ground);

            if (on_ground) {
                if (operation_mode == MODE_AUTO)
                    Activate();
            } else {
                // transition to airborne
                arpt = nullptr;
            }
        }

        if (arpt)
            loop_delay = arpt->StateMachine();

        // update global dataref values
        static constexpr float min_brightness = 0.025;   // relativ to 1
        vdgs_brightness = min_brightness + (1 - min_brightness) * powf(1 - XPLMGetDataf(percent_lights_dr), 1.5);
        int zm = XPLMGetDatai(zulu_time_minutes_dr);
        int zh = XPLMGetDatai(zulu_time_hours_dr);
        time_utc_m0 = zm % 10;
        time_utc_m1 = zm / 10;
        time_utc_h0 = zh % 10;
        time_utc_h1 = zh / 10;

        return loop_delay;
    } catch (const std::exception& ex) {
        LogMsg("fatal error: '%s'", ex.what());  // hopefully LogMsg is still alive
        error_disabled = true;
        return 0;
    }
}

// call backs for commands
static int
CmdCb(XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void *ref)
{
    if (xplm_CommandBegin != phase)
        return 0;

    if (cmdr == cycle_dgs_cmdr) {
        if (arpt)
            arpt->CycleDgsType();
        UpdateUI();
    } else if (cmdr == activate_cmdr) {
        LogMsg("cmd manually_activate");
        Activate();
    } else if (cmdr == move_dgs_closer_cmdr) {
        if (arpt)
            arpt->DgsMoveCloser();
    } else if (cmdr == toggle_ui_cmdr) {
        LogMsg("cmd toggle_ui");
        ToggleUI();
    }

    return 0;
}

// call back for menu
static void
MenuCb([[maybe_unused]] void *menu_ref, void *item_ref)
{
    XPLMCommandOnce(*(XPLMCommandRef *)item_ref);
}

// =========================== plugin entry points ===============================================
PLUGIN_API int
XPluginStart(char *outName, char *outSig, char *outDesc)
{
    strcpy(outName, "AutoDGS " VERSION);
    strcpy(outSig,  "hotbso.AutoDGS");
    strcpy(outDesc, "Automatically provides DGS for gateway airports");

    LogMsg("startup " VERSION);

    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    XPLMEnableFeature("XPLM_USE_NATIVE_WIDGET_WINDOWS", 1);

    char buffer[2048];
	XPLMGetSystemPath(buffer);
    xp_dir = std::string(buffer);

    // set plugin's base dir
    base_dir = xp_dir + "Resources/plugins/AutoDGS/";
    sys_cfg_dir = base_dir + "cfg/";
    user_cfg_dir = xp_dir + "Output/AutoDGS/";
    std::filesystem::create_directories(user_cfg_dir);

    if (!AptAirport::CollectAirports(xp_dir)) {
        LogMsg("init failure: Can't load airports");
        return 0;
    }

    // Datarefs
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
    ground_speed_dr   = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    zulu_time_minutes_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_minutes");
    zulu_time_hours_dr = XPLMFindDataRef("sim/cockpit2/clock_timer/zulu_time_hours");

    // // these are served via instancing
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL, NULL, GetDgsFloat,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, (void *)0, 0);

    // these are served globally
    XPLMRegisterDataAccessor("AutoDGS/dgs/time_utc_m0", xplmType_Float, 0, NULL, NULL, GetDgsFloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_m0, 0);
    XPLMRegisterDataAccessor("AutoDGS/dgs/time_utc_m1", xplmType_Float, 0, NULL, NULL, GetDgsFloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_m1, 0);
    XPLMRegisterDataAccessor("AutoDGS/dgs/time_utc_h0", xplmType_Float, 0, NULL, NULL, GetDgsFloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_h0, 0);
    XPLMRegisterDataAccessor("AutoDGS/dgs/time_utc_h1", xplmType_Float, 0, NULL, NULL, GetDgsFloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &time_utc_h1, 0);
    XPLMRegisterDataAccessor("AutoDGS/dgs/vdgs_brightness", xplmType_Float, 0, NULL, NULL, GetDgsFloat,
                             NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &vdgs_brightness, 0);

    create_api_drefs();
    int is_XP11 = (XPLMGetDatai(xp_version_dr) < 120000);
    const char *obj_name[2];

    if (is_XP11) {
        LogMsg("XP11 detected");
        obj_name[0] = "Marshaller_XP11.obj";
        obj_name[1] = "Safedock-T2-24-pole_XP11.obj";
    } else {
        obj_name[0] = "Marshaller.obj";
        obj_name[1] = "Safedock-T2-24-pole.obj";
    }

    for (int i = 0; i < 2; i++) {
        std::string on = base_dir + "/resources/" + obj_name[i];
        dgs_obj[i] = XPLMLoadObject(on.c_str());

        if (dgs_obj[i] == NULL) {
            LogMsg("error loading obj: %s", on.c_str());
            return 0;
        }
    }

    // own commands
    cycle_dgs_cmdr = XPLMCreateCommand("AutoDGS/cycle_dgs", "Cycle DGS between Marshaller, VDGS");
    XPLMRegisterCommandHandler(cycle_dgs_cmdr, CmdCb, 0, NULL);

    move_dgs_closer_cmdr = XPLMCreateCommand("AutoDGS/move_dgs_closer", "Move DGS closer by 2m");
    XPLMRegisterCommandHandler(move_dgs_closer_cmdr, CmdCb, 0, NULL);

    activate_cmdr = XPLMCreateCommand("AutoDGS/activate", "Manually activate searching for stands_");
    XPLMRegisterCommandHandler(activate_cmdr, CmdCb, 0, NULL);

    toggle_ui_cmdr = XPLMCreateCommand("AutoDGS/toggle_ui", "Open UI");
    XPLMRegisterCommandHandler(toggle_ui_cmdr, CmdCb, 0, NULL);

    // menu
    XPLMMenuID menu = XPLMFindPluginsMenu();
    int sub_menu = XPLMAppendMenuItem(menu, "AutoDGS", NULL, 1);
    XPLMMenuID adgs_menu = XPLMCreateMenu("AutoDGS", menu, sub_menu, MenuCb, NULL);

    XPLMAppendMenuItem(adgs_menu, "Manually activate", &activate_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Cycle DGS", &cycle_dgs_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Move DGS closer by 2m", &move_dgs_closer_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Toggle UI", &toggle_ui_cmdr, 0);

    // foreign commands
    toggle_jetway_cmdr = XPLMFindCommand("sim/ground_ops/jetway");

    flight_loop_id = XPLMCreateFlightLoop(&flight_loop_ctx);
    return 1;
}

PLUGIN_API void
XPluginStop(void)
{
    XPLMUnregisterFlightLoopCallback(FlightLoopCb, NULL);
    for (int i = 0; i < 2; i++)
        if (dgs_obj[i])
            XPLMUnloadObject(dgs_obj[i]);
}

PLUGIN_API int
XPluginEnable(void)
{
    if (error_disabled)
        return 0;

    probe_ref = XPLMCreateProbe(xplm_ProbeY);
    UpdateUI(false);     // in case we reenable
    return 1;
}

PLUGIN_API void
XPluginDisable(void)
{
    arpt = nullptr;
    if (probe_ref)
        XPLMDestroyProbe(probe_ref);
}

PLUGIN_API void
XPluginReceiveMessage([[maybe_unused]] XPLMPluginID in_from, long in_msg, void *in_param)
{
    if (error_disabled)
        return;

    // my plane loaded
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        arpt = nullptr;
        XPLMScheduleFlightLoop(flight_loop_id, 0, 0);
        pending_plane_loaded_cb = true;
        XPLMScheduleFlightLoop(flight_loop_id, 5.0, 1);
    }
}
