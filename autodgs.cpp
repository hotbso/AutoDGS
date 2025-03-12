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

// This code is still mostly plain C.
// Refactored or new code loosely follows
// Google's style guide: https://google.github.io/styleguide/cppguide.html

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cassert>
#include <fstream>
#include <algorithm>

#include "autodgs.h"
#include "flat_earth_math.h"

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"
#include "XPLMPlanes.h"

// Constants
static const float F2M=0.3048;	// 1 ft [m]

// DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand)
static const float CAP_A = 15;  // Capture
static const float CAP_Z = 100;	// (50-80 in Safedock2 flier)

static const float AZI_A = 15;	    // provide azimuth guidance
static const float AZI_DISP_A = 10; // max value for display
static const float AZI_Z = 90;

static const float GOOD_Z= 0.5;     // stop position for nw
static const float GOOD_X = 2.0;    // for mw

static const float REM_Z = 12;	// Distance remaining from here on

// place DGS at this dist from stop position, exported as dataref
static float dgs_ramp_dist_default = 25.0;
static float dgs_stand_dist;
static bool dgs_stand_dist_set;

const char * const state_str[] = {
    "DISABLED", "INACTIVE", "ACTIVE", "ENGAGED",
    "TRACK", "GOOD", "BAD", "PARKED", "DONE" };

const char * const opmode_str[] = { "Automatic", "Manual" };

// Globals
std::string xp_dir;
std::string base_dir; // base directory of AutoDGS

opmode_t operation_mode = MODE_AUTO;
state_t state = DISABLED;

XPLMCommandRef cycle_dgs_cmdr;
static XPLMCommandRef move_dgs_closer_cmdr, activate_cmdr, toggle_ui_cmdr, toggle_jetway_cmdr;

// Datarefs
static XPLMDataRef plane_x_dr, plane_y_dr, plane_z_dr, is_helicopter_dr, y_agl_dr;
static XPLMDataRef plane_lat_dr, plane_lon_dr, plane_elevation_dr, plane_true_psi_dr;
static XPLMDataRef gear_fnrml_dr, acf_cg_y_dr, acf_cg_z_dr, gear_z_dr;
static XPLMDataRef beacon_dr, parkbrake_dr, acf_icao_dr, total_running_time_sec_dr;
static XPLMDataRef percent_lights_dr, xp_version_dr, eng_running_dr, sin_wave_dr;
XPLMDataRef vr_enabled_dr;
static XPLMProbeRef probe_ref;

static int icao[4];     // as int for the datatrefs

// Internal state
static float now;           // current timestamp
static int beacon_state, beacon_last_pos;   // beacon state, last switch_pos, ts of last switch actions
static float beacon_off_ts, beacon_on_ts;
static bool use_engine_running;              // instead of beacon, e.g. MD11
static bool dont_connect_jetway;             // e.g. for ZIBO with own ground service

std::unique_ptr<Airport> arpt;

int on_ground = 1;
static float plane_nw_z, plane_mw_z;   // z value of plane's 0 to fw, mw
static float pe_y_plane_0;        // pilot eye y to plane's 0 point
static bool pe_y_plane_0_valid;
static bool is_helicopter;

static std::string selected_stand;

bool dgs_type_auto = true;
int dgs_type = 0;
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

Stand::Stand(const AptStand& as, float elevation) : as_(as)
{
    XPLMWorldToLocal(as_.lat, as_.lon, elevation, &x_, &y_, &z_);
    // TODO: terrain probe
    sin_hdgt_ = sinf(kD2R * as_.hdgt);
    cos_hdgt_ = cosf(kD2R * as_.hdgt);
}

// move dgs some distance away
void
Stand::SetDgsPos(void)
{
    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (!dgs_stand_dist_set) {
        // determine dgs_ramp_dist_default depending on pilot eye height agl
        if (pe_y_plane_0_valid) {
            float plane_x = XPLMGetDataf(plane_x_dr);
            float plane_y = XPLMGetDataf(plane_y_dr);
            float plane_z = XPLMGetDataf(plane_z_dr);

            // get terrain y below plane y

            if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, plane_x, plane_y, plane_z, &probeinfo)) {
                LogMsg("XPLMProbeTerrainXYZ failed");
                arpt->ResetState(INACTIVE);
                return;
            }

            // pilot eye above agl
            float pe_agl = plane_y - probeinfo.locationY + pe_y_plane_0;

            // 4.3 ~ 1 / tan(13°) -> 13° down look
            dgs_ramp_dist_default = std::max(8.0, std::min(4.3 * pe_agl, 30.0));
            LogMsg("setting DGS default distance, pe_agl: %0.2f, dist: %0.1f", pe_agl, dgs_ramp_dist_default);
        }

        dgs_stand_dist = dgs_ramp_dist_default;
        dgs_stand_dist_set = true;
    }

    // xform (0, -dgs_stand_dist) into global frame
    dgs_pos_x_ = x_ + -sin_hdgt_ * (-dgs_stand_dist);
    dgs_pos_z_ = z_ +  cos_hdgt_ * (-dgs_stand_dist);

    if (xplm_ProbeHitTerrain
        != XPLMProbeTerrainXYZ(probe_ref, dgs_pos_x_, y_, dgs_pos_z_, &probeinfo)) {
        LogMsg("XPLMProbeTerrainXYZ failed");
        arpt->ResetState(ACTIVE);
        return;
    }

    dgs_pos_y_ = probeinfo.locationY;
}

Airport::Airport(const AptAirport* apt_airport)
{
    apt_airport_ = apt_airport;
    stands_.reserve(apt_airport_->stands_.size());
    float arpt_elevation = XPLMGetDataf(plane_elevation_dr);    // best guess
    for (auto const & as : apt_airport->stands_)
        stands_.emplace_back(as, arpt_elevation);
}

Airport::~Airport()
{
    state = INACTIVE;
    LogMsg("Airport '%s' destructed", name().c_str());
}

std::unique_ptr<Airport>
Airport::LoadAirport(const std::string& icao)
{
    auto arpt = AptAirport::LookupAirport(icao);
    if (arpt == nullptr)
        return nullptr;

    return std::make_unique<Airport>(arpt);
}

void
Airport::ResetState(state_t new_state)
{
    if (state != new_state)
        LogMsg("setting state to %s", state_str[new_state]);

    state = new_state;
    active_stand_ = nullptr;
    dgs_stand_dist = dgs_ramp_dist_default;
    if (state == INACTIVE) {
        selected_stand.resize(0);
        stands_.clear();
        arpt = nullptr;
        update_ui(1);
    }

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }
}

// set mode to arrival
static void
SetActive(void)
{
    if (! on_ground) {
        LogMsg("can't set active when not on ground");
        return;
    }

    if (state > INACTIVE)
        return;

    beacon_state = beacon_last_pos = XPLMGetDatai(beacon_dr);
    beacon_on_ts = beacon_off_ts = -10.0;

    float lat = XPLMGetDataf(plane_lat_dr);
    float lon = XPLMGetDataf(plane_lon_dr);

    // can be a teleportation so play it safe
    if (arpt)
        arpt->ResetState(INACTIVE);

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
    }

    if (arpt == nullptr)
        return;
    LogMsg("airport activated: %s, new state: ACTIVE", arpt->name().c_str());
    state = ACTIVE;
    dgs_stand_dist_set = false;
    dgs_type_auto = true;
    update_ui(1);
}

static int
CheckBeacon(void)
{
    if (use_engine_running) {
        int er[8];
        int n = XPLMGetDatavi(eng_running_dr, er, 0, 8);
        for (int i = 0; i < n; i++)
            if (er[i])
                return 1;

        return 0;
    }

    // when checking the beacon guard against power transitions when switching
    // to the APU generator (e.g. for the ToLiss fleet).
    // Report only state transitions when the new state persisted for 3 seconds

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

// hooks for the ui
void
SetSelectedStand(const std::string& ui_selected_stand)
{
    assert(arpt != nullptr);
    LogMsg("SetSelectedStand to '%s'", ui_selected_stand.c_str());
    if (ui_selected_stand == "Automatic") {
        selected_stand.resize(0);
        if (state > ACTIVE)
            arpt->ResetState(ACTIVE);
    } else {
        selected_stand = ui_selected_stand;
        if (state > ACTIVE)
            arpt->ResetState(ACTIVE);
    }
}

void
SetDgsType(int new_dgs_type)
{
    if (new_dgs_type == dgs_type)
        return;

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }

    dgs_type = new_dgs_type;
}

void
Airport::SetDgsTypeAuto()
{
    dgs_type_auto = true;
    if (active_stand_ == nullptr)
        return;

    int new_dgs_type = (active_stand_->has_jw() ? 1 : 0);

    if (new_dgs_type == dgs_type)
        return;

    if (dgs_inst_ref) {
        XPLMDestroyInstance(dgs_inst_ref);
        dgs_inst_ref = NULL;
    }

    dgs_type = new_dgs_type;
}

void
Airport::FindNearestStand()
{
    // check whether we already have a selected stand
    if (active_stand_ && !selected_stand.empty())
        return;

    double dist = 1.0E10;
    Stand *min_stand = nullptr;

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);

    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (probe_ref == NULL)
        probe_ref = XPLMCreateProbe(xplm_ProbeY);

    for (auto & s : stands_) {
        // heading in local system
        float local_hdgt = RA(plane_hdgt - s.hdgt());

        if (fabsf(local_hdgt) > 90.0f)
            continue;   // not looking to stand

        // transform into gate local coordinate system

        if (selected_stand.empty()) {
            // xlate + rotate into stand frame
            float dx = plane_x - s.x_;
            float dz = plane_z - s.z_;

            float local_x =  s.cos_hdgt_ * dx + s.sin_hdgt_ * dz;
            float local_z = -s.sin_hdgt_ * dx + s.cos_hdgt_ * dz;

            // nose wheel
            float nw_z = local_z - plane_nw_z;
            float nw_x = local_x + plane_nw_z * sinf(kD2R * local_hdgt);

            float d = sqrt(SQR(nw_x) + SQR(nw_z));
            if (d > CAP_Z + 50) // fast exit
                continue;

            //LogMsg("stand: %s, z: %2.1f, x: %2.1f", s.name(), nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                //LogMsg("behind: %s", s.cname());
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / kD2R;
                //LogMsg("angle to plane: %s, %3.1f", s.cname(), angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = RA(-angle - local_hdgt);
                //LogMsg("rel_to_stand: %s, nw_x: %0.1f, local_hdgt %0.1f, rel_to_stand: %0.1f",
                //       s.cname(), nw_x, local_hdgt, rel_to_stand);
                if ((nw_x > 10.0 && rel_to_stand < -60.0)
                    || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    //LogMsg("drive by %s", s.cname());
                    continue;
                }
            }

            // for the final comparison give azimuth a higher weight
            static const float azi_weight = 4.0;
            d = sqrt(SQR(azi_weight * nw_x)+ SQR(nw_z));

            if (d < dist) {
                //LogMsg("new min: %s, z: %2.1f, x: %2.1f", s.cname(), nw_z, nw_x);
                dist = d;
                min_stand = &s;
            }

        } else if (s.name() == selected_stand) {
            dist = 0.0;
            min_stand = &s;
            break;
        }
    }

    if (min_stand != NULL && min_stand != active_stand_) {
        if (xplm_ProbeHitTerrain
            != XPLMProbeTerrainXYZ(probe_ref, min_stand->x_, min_stand->y_, min_stand->z_, &probeinfo)) {
            LogMsg("XPLMProbeTerrainXYZ failed");
            ResetState(ACTIVE);
            return;
        }

        LogMsg("stand: %s, %f, %f, %f, dist: %f, is_wet: %d", min_stand->cname(), min_stand->lat(), min_stand->lon(),
               min_stand->hdgt(), dist, probeinfo.is_wet);

        if (probeinfo.is_wet) {
            ResetState(ACTIVE);
            return;
        }

        min_stand->y_ = probeinfo.locationY;
        active_stand_ = min_stand;
        active_stand_->SetDgsPos();
        if (dgs_type_auto)
            SetDgsTypeAuto();
        state = ENGAGED;
    }
}

float
Airport::StateMachine()
{
    // values that must survive a single run of the state_machine
    static int status, track, lr;
    static float timestamp, distance, sin_wave_prev;
    static float nearest_stand_ts, update_dgs_log_ts;

    if (state <= INACTIVE)
        return 2.0;

    // throttle costly search
    if (now > nearest_stand_ts + 2.0) {
        FindNearestStand();
        nearest_stand_ts = now;
    }

    if (active_stand_ == nullptr) {
        state = ACTIVE;
        return 2.0;
    }

    int lr_prev = lr;
    int track_prev = track;
    float distance_prev = distance;

    float loop_delay = 0.2;
    state_t new_state = state;

    // xform plane pos into stand local coordinate system
    float dx = XPLMGetDataf(plane_x_dr) - active_stand_->x_;
    float dz = XPLMGetDataf(plane_z_dr) - active_stand_->z_;
    float local_x =  active_stand_->cos_hdgt_ * dx + active_stand_->sin_hdgt_ * dz;
    float local_z = -active_stand_->sin_hdgt_ * dx + active_stand_->cos_hdgt_ * dz;

    // relative reading to stand +/- 180
    float local_hdgt = RA(XPLMGetDataf(plane_true_psi_dr) - active_stand_->hdgt());

    // nose wheel
    float nw_z = local_z - plane_nw_z;
    float nw_x = local_x + plane_nw_z * sinf(kD2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - plane_mw_z;
    float mw_x = local_x + plane_mw_z * sinf(kD2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = std::clamp((nw_z - 6.0) / 20.0, 0.0, 1.0);
    float plane_z_dr = (1.0 - a) * plane_nw_z + a * plane_mw_z;
    float z_dr = local_z - plane_z_dr;
    float x_dr = local_x + plane_z_dr * sinf(kD2R * local_hdgt);

    float azimuth;
    if (fabs(x_dr) > 0.5 && z_dr > 0)
        azimuth = atanf(x_dr / (z_dr + 0.5 * dgs_stand_dist)) / kD2R;
    else
        azimuth = 0.0;

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 0.5 * dgs_stand_dist)) / kD2R;
    else
        azimuth_nw = 0.0;

    int locgood = (fabsf(mw_x) <= GOOD_X && fabsf(nw_z) <= GOOD_Z);
    int beacon_on = CheckBeacon();

    status = lr = track = 0;
    distance = nw_z - GOOD_Z;

    // catch the phase ~180° point -> the Marshaller's arm is straight
    float sin_wave = XPLMGetDataf(sin_wave_dr);
    int phase180 = (sin_wave_prev > 0.0) && (sin_wave <= 0.0);
    sin_wave_prev = sin_wave;

    // set drefs according to *current* state
    switch (state) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance <= CAP_Z) && (fabsf(azimuth_nw) <= CAP_A))
                    new_state = TRACK;
            } else { // not beacon_on
                new_state = DONE;
            }
            break;

        case TRACK: {
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

                status = 1;	// plane id
                if (distance > AZI_Z || fabsf(azimuth_nw) > AZI_A) {
                    track=1;	// lead-in only
                    break;
                }

                // compute distance and guidance commands
                azimuth = std::clamp(azimuth, -AZI_A, AZI_A);
                float req_hdgt = -3.5 * azimuth;        // to track back to centerline
                float d_hdgt = req_hdgt - local_hdgt;   // degrees to turn

                if (now > update_dgs_log_ts + 2.0)
                    LogMsg("azimuth: %0.1f, mw: (%0.1f, %0.1f), nw: (%0.1f, %0.1f), ref: (%0.1f, %0.1f), "
                           "x: %0.1f, local_hdgt: %0.1f, d_hdgt: %0.1f",
                           azimuth, mw_x, mw_z, nw_x, nw_z,
                           x_dr, z_dr,
                           local_x, local_hdgt, d_hdgt);

                if (d_hdgt < -1.5)
                    lr = 2;
                else if (d_hdgt > 1.5)
                    lr = 1;

                // xform azimuth to values required ob OBJ
                azimuth = std::clamp(azimuth, -AZI_DISP_A, AZI_DISP_A) * 4.0 / AZI_DISP_A;
                azimuth=((float)((int)(azimuth * 2))) / 2;  // round to 0.5 increments

                if (distance <= REM_Z/2) {
                    track = 3;
                    loop_delay = 0.03;
                } else // azimuth only
                    track = 2;

                if (! phase180) { // no wild oscillation
                    lr = lr_prev;

                    // sync transition with Marshaller's arm movement
                    if (dgs_type == 0 && track == 3 && track_prev == 2) {
                        track = track_prev;
                        distance = distance_prev;
                    }
                }
            }
            break;

        case GOOD:  {
                // @stop position
                status = 2; lr = 3;

                int parkbrake_set = (XPLMGetDataf(parkbrake_dr) > 0.5);
                if (!locgood)
                    new_state = TRACK;
                else if (parkbrake_set || !beacon_on)
                    new_state = PARKED;
            }
            break;

        case BAD:
            if (!beacon_on
                && (now > timestamp + 5.0)) {
                ResetState(INACTIVE);
                return loop_delay;
            }

            if (nw_z >= -GOOD_Z)
                new_state = TRACK;
            else {
                // Too far
                status = 4;
                lr = 3;
            }
            break;

        case PARKED:
            status = 3;
            lr = 0;
            // wait for beacon off
            if (! beacon_on) {
                new_state = DONE;
                if (operation_mode == MODE_AUTO && ! dont_connect_jetway)
                    XPLMCommandOnce(toggle_jetway_cmdr);
            }
            break;

        case DONE:
            if (now > timestamp + 5.0) {
                ResetState(INACTIVE);
                return loop_delay;
            }
            break;

        default:
            break;
    }

    if (new_state != state) {
        LogMsg("state transition %s -> %s, beacon: %d", state_str[state], state_str[new_state], beacon_on);
        state = new_state;
        timestamp = now;
        return -1;  // see you on next frame
    }

    if (state > ACTIVE) {
        // xform drefs into required constraints for the OBJs
        if (track == 0 || track == 1) {
            distance = 0;
            azimuth = 0.0;
        }

        distance = std::clamp(distance, -GOOD_Z, REM_Z);

        // is not necessary for Marshaller + SafedockT2
        // distance=((float)((int)((distance)*2))) / 2;    // multiple of 0.5m

        // don't flood the log
        if (now > update_dgs_log_ts + 2.0) {
            update_dgs_log_ts = now;
            LogMsg("stand: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, azimuth: %0.1f",
                   active_stand_->name().c_str(), state_str[state], status, track, lr, distance, azimuth);
        }

        XPLMDrawInfo_t drawinfo;
        float drefs[DGS_DR_NUM];
        memset(drefs, 0, sizeof(drefs));

        drawinfo.structSize = sizeof(drawinfo);
        drawinfo.x = active_stand_->dgs_pos_x_;
        drawinfo.y = active_stand_->dgs_pos_y_;
        drawinfo.z = active_stand_->dgs_pos_z_;
        drawinfo.heading = active_stand_->hdgt();
        drawinfo.pitch = drawinfo.roll = 0.0;

        if (dgs_inst_ref == nullptr) {
            dgs_inst_ref = XPLMCreateInstance(dgs_obj[dgs_type], dgs_dlist_dr);
            if (dgs_inst_ref == nullptr) {
                LogMsg("error creating instance");
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
    static float on_ground_ts;  // debounce ground contact

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
        LogMsg("transition to on_ground: %d", on_ground);

        if (on_ground) {
            if (operation_mode == MODE_AUTO)
                SetActive();
        } else {
            // transition to airborne
            arpt = nullptr;
            if (probe_ref) {
                XPLMDestroyProbe(probe_ref);
                probe_ref = nullptr;
            }
        }
    }

    if (state >= ACTIVE)
        loop_delay = arpt->StateMachine();

    return loop_delay;
}

// call backs for commands
static int
CmdCb(XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void *ref)
{
    if (xplm_CommandBegin != phase)
        return 0;

    if (cmdr == cycle_dgs_cmdr) {
        SetDgsType(!dgs_type);
        dgs_type_auto = false;
        update_ui(1);
    } else if (cmdr == activate_cmdr) {
        LogMsg("cmd manually_activate");
        SetActive();
    } else if (cmdr == move_dgs_closer_cmdr && state >= ENGAGED && dgs_stand_dist > 12.0) {
        dgs_stand_dist -= 2.0;
        LogMsg("dgs_stand_dist reduced to %0.1f", dgs_stand_dist);
        arpt->SetDgsPos();
    } else if (cmdr == toggle_ui_cmdr) {
        LogMsg("cmd toggle_ui");
        toggle_ui();
    }

    return 0;
}

// call back for menu
static void
menu_cb([[maybe_unused]] void *menu_ref, void *item_ref)
{
    XPLMCommandOnce(*(XPLMCommandRef *)item_ref);
}

static bool
FindIcaoInFile(const std::string& acf_icao, const std::string& fn)
{
    std::ifstream f(fn);
    if (!f.is_open()) {
        LogMsg("Can't open '%s'", fn.c_str());
        return false;
    }

    LogMsg("check whether acf '%s' is in exception file %s", acf_icao.c_str(), fn.c_str());
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() > 0 && line.back() == '\r') // just in case
            line.pop_back();

        if (line == acf_icao) {
            LogMsg("found acf %s in %s", acf_icao.c_str(), fn.c_str());
            return true;
        }
    }

    return false;
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

    // Published scalar datarefs, as we draw with the instancing API the accessors will never be called
    for (int i = 0; i < DGS_DR_NUM; i++)
        XPLMRegisterDataAccessor(dgs_dlist_dr[i], xplmType_Float, 0, NULL, NULL, getdgsfloat,
                                 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);

    create_api_drefs();
    int is_XP11 = (XPLMGetDatai(xp_version_dr) < 120000);
    const char *obj_name[2];

    if (is_XP11) {
        LogMsg("XP11 detected");
        obj_name[0] = "Marshaller_XP11.obj";
        obj_name[1] = "SafedockT2-6m-pole_XP11.obj";
    } else {
        obj_name[0] = "Marshaller.obj";
        obj_name[1] = "SafedockT2-6m-pole.obj";
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
    XPLMMenuID adgs_menu = XPLMCreateMenu("AutoDGS", menu, sub_menu, menu_cb, NULL);

    XPLMAppendMenuItem(adgs_menu, "Manually activate", &activate_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Cycle DGS", &cycle_dgs_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Move DGS closer by 2m", &move_dgs_closer_cmdr, 0);
    XPLMAppendMenuItem(adgs_menu, "Toggle UI", &toggle_ui_cmdr, 0);

    // foreign commands
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
    arpt = nullptr;
    state = DISABLED;
}

PLUGIN_API void
XPluginReceiveMessage([[maybe_unused]] XPLMPluginID in_from, long in_msg, void *in_param)
{
    // my plane loaded
    if (in_msg == XPLM_MSG_PLANE_LOADED && in_param == 0) {
        char acf_icao[41];

        arpt = nullptr;
        memset(acf_icao, 0, sizeof(acf_icao));
        XPLMGetDatab(acf_icao_dr, acf_icao, 0, 40);

        for (int i=0; i<4; i++)
            icao[i] = (isupper(acf_icao[i]) || isdigit(acf_icao[i])) ? acf_icao[i] : ' ';

        acf_icao[4] = '\0';

        float plane_cg_z = F2M * XPLMGetDataf(acf_cg_z_dr);

        float gear_z[2];
        if (2 == XPLMGetDatavf(gear_z_dr, gear_z, 0, 2)) {      // nose + main wheel
            plane_nw_z = -gear_z[0];
            plane_mw_z = -gear_z[1];
        } else
            plane_nw_z = plane_mw_z = plane_cg_z;         // fall back to CG

        is_helicopter = XPLMGetDatai(is_helicopter_dr);

        pe_y_plane_0_valid = false;
        pe_y_plane_0 = 0.0;

        if (! is_helicopter) {
            // unfortunately the *default* pilot eye y coordinate is not published in
            // a dataref, only the dynamic values.
            // Therefore we pull it from the acf file.

            char acf_path[512], acf_file[256];
            XPLMGetNthAircraftModel(XPLM_USER_AIRCRAFT, acf_file, acf_path);
            LogMsg("acf_path: '%s'", acf_path);

            std::ifstream acf(acf_path);
            if (acf.is_open()) {
                std::string line;
                while (std::getline(acf, line)) {
                    if (line.starts_with("P acf/_pe_xyz/1 ")) {
                        if (1 == sscanf(line.c_str() + 16, "%f", &pe_y_plane_0)) {
                            pe_y_plane_0 -= XPLMGetDataf(acf_cg_y_dr);
                            pe_y_plane_0 *= F2M;
                            pe_y_plane_0_valid = true;
                        }
                        break;
                    }
                }
            }
        }

        // check whether acf is listed in exception files
        use_engine_running = FindIcaoInFile(acf_icao, base_dir + "acf_use_engine_running.txt");
        dont_connect_jetway = FindIcaoInFile(acf_icao, base_dir + "acf_dont_connect_jetway.txt");

        LogMsg("plane loaded: %c%c%c%c, plane_cg_z: %1.2f, plane_nw_z: %1.2f, plane_mw_z: %1.2f, "
               "pe_y_plane_0_valid: %d, pe_y_plane_0: %0.2f, is_helicopter: %d",
               icao[0], icao[1], icao[2], icao[3], plane_cg_z, plane_nw_z, plane_mw_z,
               pe_y_plane_0_valid, pe_y_plane_0, is_helicopter);
    }
}
