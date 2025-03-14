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
#include "flat_earth_math.h"

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"
#include "XPLMGraphics.h"

// DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand)
static const float CAP_A = 15;      // Capture
static const float CAP_Z = 100;	    // (50-80 in Safedock2 flier)

static const float AZI_A = 15;	    // provide azimuth guidance
static const float AZI_DISP_A = 10; // max value for display
static const float AZI_Z = 90;

static const float GOOD_Z= 0.5;     // stop position for nw
static const float GOOD_X = 2.0;    // for mw

static const float REM_Z = 12;	    // Distance remaining from here on

class Marshaller {
    XPLMInstanceRef inst_ref_;

  public:
    Marshaller();
    ~Marshaller();
    void SetPos(const XPLMDrawInfo_t *drawinfo, int status, int track, int lr, float distance);
};

// place DGS at this dist from stop position
static float marshaller_dist_default = 25.0;
static bool marshaller_dist_default_set;        // according to pilot's eye AGL
static float vdgs_dist_default = 25.0;

const char * const opmode_str[] = { "Automatic", "Manual" };

// Globals
std::string xp_dir;
std::string base_dir;       // base directory of AutoDGS
std::string sys_cfg_dir;    // <base_dir>/cfg
std::string user_cfg_dir;   // <xp_dir>/Output/AutoDGS

opmode_t operation_mode = MODE_AUTO;
static bool error_disabled;

static XPLMCommandRef cycle_dgs_cmdr, move_dgs_closer_cmdr, activate_cmdr,
    toggle_ui_cmdr, toggle_jetway_cmdr;

XPLMDataRef plane_x_dr, plane_y_dr, plane_z_dr, is_helicopter_dr, y_agl_dr;
XPLMDataRef plane_lat_dr, plane_lon_dr, plane_elevation_dr, plane_true_psi_dr;
XPLMDataRef gear_fnrml_dr, acf_cg_y_dr, acf_cg_z_dr, gear_z_dr;
XPLMDataRef beacon_dr, parkbrake_dr, acf_icao_dr, total_running_time_sec_dr;
XPLMDataRef percent_lights_dr, xp_version_dr, eng_running_dr, sin_wave_dr;
XPLMDataRef vr_enabled_dr;
XPLMProbeRef probe_ref;

float now;           // current timestamp
int on_ground = 1;

std::unique_ptr<Airport> arpt;

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
    "AutoDGS/dgs/status",
    "AutoDGS/dgs/lr",
    "AutoDGS/dgs/track",
    "AutoDGS/dgs/azimuth",
    "AutoDGS/dgs/distance",
    "AutoDGS/dgs/icao_0",
    "AutoDGS/dgs/icao_1",
    "AutoDGS/dgs/icao_2",
    "AutoDGS/dgs/icao_3",
    "AutoDGS/dgs/vdgs_brightness",
    NULL
};

// there is exactly none or one Marshaller
static std::unique_ptr<Marshaller> marshaller;

#define SQR(x) ((x) * (x))

//------------------------------------------------------------------------------------
Marshaller::Marshaller()
{
    inst_ref_ = XPLMCreateInstance(dgs_obj[kMarshaller], dgs_dlist_dr);
}

Marshaller::~Marshaller()
{
    if (inst_ref_)
        XPLMDestroyInstance(inst_ref_);
}

void
Marshaller::SetPos(const XPLMDrawInfo_t *drawinfo, int status, int track, int lr, float distance)
{
    float drefs[DGS_DR_NUM]{};
    drefs[DGS_DR_STATUS] = status;
    drefs[DGS_DR_DISTANCE] = distance;
    drefs[DGS_DR_TRACK] = track;
    drefs[DGS_DR_LR] = lr;
    XPLMInstanceSetPosition(inst_ref_, drawinfo, drefs);
}

//------------------------------------------------------------------------------------
Stand::Stand(int idx, const AptStand& as, float elevation, int dgs_type, float dist_adjust) : as_(as), idx_(idx)
{
    XPLMWorldToLocal(as_.lat, as_.lon, elevation, &x_, &y_, &z_);
    // TODO: terrain probe
    sin_hdgt_ = sinf(kD2R * as_.hdgt);
    cos_hdgt_ = cosf(kD2R * as_.hdgt);
    drawinfo_.structSize = sizeof(drawinfo_);
    drawinfo_.heading = as_.hdgt;
    drawinfo_.pitch = drawinfo_.roll = 0.0f;
    vdgs_inst_ref_ = nullptr;
    dgs_dist_ = vdgs_dist_default;
    dist_adjust_ = dist_adjust;
    dgs_type_ = kMarshaller;
    SetDgsType(dgs_type);
    LogMsg("Stand '%s', type: %d, dist_adjust: %0.1f constructed", cname(), dgs_type_, dist_adjust_);
}

Stand::~Stand()
{
    if (vdgs_inst_ref_)
        XPLMDestroyInstance(vdgs_inst_ref_);
}

void
Stand::SetDgsType(int dgs_type)
{
    LogMsg("Stand::SetDgsType: Stand '%s', type: %d, new_type: %d", cname(), dgs_type_, dgs_type);

    if (dgs_type == kAutomatic)
        dgs_type = as_.has_jw ? kVDGS : kMarshaller;

    if (dgs_type_ == dgs_type)
        return;

    dgs_type_ = dgs_type;

    if (dgs_type_ == kMarshaller) {
        if (vdgs_inst_ref_)
            XPLMDestroyInstance(vdgs_inst_ref_);
        vdgs_inst_ref_ = nullptr;
        SetDgsDist();
    } else {
        marshaller = nullptr;
        SetDgsDist();
        vdgs_inst_ref_ = XPLMCreateInstance(dgs_obj[kVDGS], dgs_dlist_dr);
        Deactivate();
    }
}

void
Stand::CycleDgsType()
{
    int new_dgs_type = (dgs_type_ == kMarshaller ? kVDGS : kMarshaller);
    SetDgsType(new_dgs_type);
}

void
Stand::SetState(int status, int track, int lr, float azimuth, float distance,
                bool state_track, float brightness)
{
    assert(dgs_type_ == kVDGS);

    float drefs[DGS_DR_NUM]{};
    drefs[DGS_DR_STATUS] = status;
    drefs[DGS_DR_TRACK] = track;
    drefs[DGS_DR_DISTANCE] = distance;
    drefs[DGS_DR_AZIMUTH] = azimuth;
    drefs[DGS_DR_LR] = lr;

    if (state_track)
        for (int i = 0; i < 4; i++)
            drefs[DGS_DR_ICAO_0 + i] = (int)plane.acf_icao[i];

    drefs[DGS_DR_BRIGHTNESS] = brightness;
    XPLMInstanceSetPosition(vdgs_inst_ref_, &drawinfo_, drefs);
}

void
Stand::Deactivate()
{
    if (vdgs_inst_ref_ == nullptr)
        return;

    LogMsg("Deactivate stand: '%s'", cname());

    float drefs[DGS_DR_NUM]{};
    XPLMInstanceSetPosition(vdgs_inst_ref_, &drawinfo_, drefs);
}

// compute the DGS position
// adjust may be negative to move it closer
void
Stand::SetDgsDist(float adjust)
{
    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};
    dist_adjust_ = adjust;

    if (dgs_type_ == kVDGS)
        dgs_dist_ = vdgs_dist_default + adjust;
    else {
        if (!marshaller_dist_default_set) {
            // determine marshaller_dist_default depending on pilot eye height agl
            if (plane.pe_y_0_valid) {
                float plane_x = XPLMGetDataf(plane_x_dr);
                float plane_y = XPLMGetDataf(plane_y_dr);
                float plane_z = XPLMGetDataf(plane_z_dr);

                // get terrain y below plane y

                if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, plane_x, plane_y, plane_z, &probeinfo)) {
                    LogMsg("XPLMProbeTerrainXYZ failed");
                    return;
                }

                // pilot eye above agl
                float pe_agl = plane_y - probeinfo.locationY + plane.pe_y_0;

                // 4.3 ~ 1 / tan(13°) -> 13° down look
                marshaller_dist_default = std::max(8.0, std::min(4.3 * pe_agl, 30.0));
                LogMsg("setting DGS default distance, pe_agl: %0.2f, dist: %0.1f", pe_agl, marshaller_dist_default);
            }

            marshaller_dist_default_set = true;
        }
        dgs_dist_ = marshaller_dist_default + adjust;
    }

    // xform (0, -dgs_dist) into global frame
    drawinfo_.x = x_ + -sin_hdgt_ * (-dgs_dist_);
    drawinfo_.z = z_ +  cos_hdgt_ * (-dgs_dist_);

    if (xplm_ProbeHitTerrain
        != XPLMProbeTerrainXYZ(probe_ref, drawinfo_.x, y_, drawinfo_.z, &probeinfo)) {
        LogMsg("XPLMProbeTerrainXYZ failed");
        arpt->ResetState(Airport::ACTIVE);
        return;
    }

    drawinfo_.x = probeinfo.locationX;
    drawinfo_.y = probeinfo.locationY;
    drawinfo_.z = probeinfo.locationZ;
}

//------------------------------------------------------------------------------------
const char * const Airport::state_str[] = {
    "INACTIVE", "ACTIVE", "ENGAGED",
    "TRACK", "GOOD", "BAD", "PARKED", "DONE"
};


void LoadCfg(const std::string& pathname,
             std::unordered_map<std::string, std::tuple<int, float>>& cfg);

Airport::Airport(const AptAirport* apt_airport)
{
    apt_airport_ = apt_airport;
    stands_.reserve(apt_airport_->stands_.size());
    float arpt_elevation = XPLMGetDataf(plane_elevation_dr);    // best guess

    std::unordered_map<std::string, std::tuple<int, float>> cfg;
    LoadCfg(user_cfg_dir + name() + ".cfg", cfg);
    if (cfg.empty())
        LoadCfg(sys_cfg_dir + name() + ".cfg", cfg);

    int idx = 0;
    for (auto const & as : apt_airport->stands_) {
        int dgs_type = kAutomatic;
        float dist_adjust = 0.0f;
        try {
            std::tie<int, float>(dgs_type, dist_adjust) = cfg.at(as.name);
            LogMsg("found '%s', %d, %0.1f", as.name.c_str(), dgs_type, dist_adjust);
        } catch (const std::out_of_range& ex) {
            // nothing
        }

        stands_.emplace_back(idx++, as, arpt_elevation, dgs_type, dist_adjust);
    }

    state_ = INACTIVE;
    active_stand_ = nullptr;
    selected_stand_ = -1;
    user_cfg_changed_ = false;

    timestamp_ = distance_ = sin_wave_prev_ = 0.0f;
    nearest_stand_ts_ = update_dgs_log_ts_ = 0.0f;
}

Airport::~Airport()
{
    FlushUserCfg();
    LogMsg("Airport '%s' destructed", name().c_str());
}

void
LoadCfg(const std::string& pathname, std::unordered_map<std::string, std::tuple<int, float>>& cfg)
{
    std::ifstream f(pathname);
    if (f.is_open()) {
        LogMsg("Loading config from '%s'", pathname.c_str());

        std::string line;
        while (std::getline(f, line)) {
            if (line.size() == 0 || line[0] == '#')
                continue;

            if (line.back() == '\r')
                line.pop_back();

            int ofs;
            float dist_adjust;
            char type;
            int n = sscanf(line.c_str(), "%c,%f, %n", &type, &dist_adjust, &ofs);
            if (n != 2 || ofs >= (int)line.size()       // distrust user input
                || !(type == 'V' || type == 'M')
                || dist_adjust < -20.0f || dist_adjust > 10.0f) {
                LogMsg("invalid line: '%s' %d", line.c_str(), n);
                continue;
            }

            cfg[line.substr(ofs)]
                = std::make_tuple((type == 'M' ? kMarshaller : kVDGS), dist_adjust);
        }
    }
}

void
Airport::FlushUserCfg()
{
    if (!user_cfg_changed_)
        return;

    std::string fn = user_cfg_dir + name() + ".cfg";
    std::ofstream f(fn);
    if (!f.is_open()) {
        LogMsg("Can't create '%s'", fn.c_str());
        return;
    }

    // The apt.dat spec demands that the stand names must be unique but usually they are not.
    // Hence we build an ordered map first and write that out.
    // Last entry wins.
    std::map<std::string, std::string> cfg;
    for (auto const & s : stands_) {
        char line[200];
        snprintf(line, sizeof(line), "%c, %5.1f, %s\n", (s.dgs_type_ == kMarshaller ? 'M' : 'V'),
                 s.dist_adjust_, s.name().c_str());
        cfg[s.name()] = line;
    }

    f << "# type, dist_adjust, stand_name\n";
    f << "# type = M or V, dist_adjust = delta to default position, negative = closer to stand\n";

    for (auto const & kv : cfg)
        f << kv.second;

    LogMsg("cfg written to '%s'", fn.c_str());
}

std::unique_ptr<Airport>
Airport::LoadAirport(const std::string& icao)
{
    auto arpt = AptAirport::LookupAirport(icao);
    if (arpt == nullptr)
        return nullptr;

    return std::make_unique<Airport>(arpt);
}

std::tuple<int, const std::string>
Airport::GetStand(int idx) const
{
    assert(0 <= idx && idx < (int)stands_.size());
    const Stand& s = stands_[idx];
    std::string name{s.name()};
    int dgs_type = s.dgs_type_;
    return std::make_tuple(dgs_type, name);
}

void
Airport::SetSelectedStand(int selected_stand)
{
    assert(-1 <= selected_stand && selected_stand < (int)stands_.size());
    if (selected_stand_ == selected_stand)
        return;
    selected_stand_ = selected_stand;

    if (arpt->state() > Airport::ACTIVE)
        arpt->ResetState(Airport::ACTIVE);
    user_cfg_changed_ = true;
}

void
Airport::SetDgsDistAdjust(float adjust)
{
    if (selected_stand_ >= 0)
        stands_[selected_stand_].SetDgsDist(adjust);
    else if (active_stand_)
        active_stand_->SetDgsDist(adjust);

    user_cfg_changed_ = true;
}

void
Airport::SetDgsType(int dgs_type)
{
    if (selected_stand_ >= 0)
        stands_[selected_stand_].SetDgsType(dgs_type);
    else if (active_stand_)
        active_stand_->SetDgsType(dgs_type);

    user_cfg_changed_ = true;
}

int
Airport::GetDgsType() const
{
    if (selected_stand_ >= 0)
        return stands_[selected_stand_].dgs_type_;
    else if (active_stand_)
        return active_stand_->dgs_type_;

    return kMarshaller;
}

void
Airport::ResetState(state_t new_state)
{
    if (state_ != new_state)
        LogMsg("setting state to %s", state_str[new_state]);

    state_ = new_state;
    if (active_stand_)
        active_stand_->Deactivate();
    active_stand_ = nullptr;
    marshaller = nullptr;
    if (new_state == INACTIVE) {
        selected_stand_ = -1;
        FlushUserCfg();
    }

    marshaller_dist_default_set = false;
    UpdateUI();
}

void
Airport::CycleDgsType()
{
    if (selected_stand_ >= 0)
        stands_[selected_stand_].CycleDgsType();
    else if (active_stand_)
        active_stand_->CycleDgsType();

    user_cfg_changed_ = true;
}

void
Airport::FindNearestStand()
{
    // check whether we already have an active  selected stand
    if (active_stand_ && active_stand_->idx_ == selected_stand_)
        return;

    double dist = 1.0E10;
    Stand *min_stand = nullptr;

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);

    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (selected_stand_ >= 0) {
        dist = 0.0;
        min_stand = &stands_[selected_stand_];
    } else {
        for (auto & s : stands_) {
            // heading in local system
            float local_hdgt = RA(plane_hdgt - s.hdgt());

            if (fabsf(local_hdgt) > 90.0f)
                continue;   // not looking to stand

            // transform into gate local coordinate system

            // xlate + rotate into stand frame
            float dx = plane_x - s.x_;
            float dz = plane_z - s.z_;

            float local_x =  s.cos_hdgt_ * dx + s.sin_hdgt_ * dz;
            float local_z = -s.sin_hdgt_ * dx + s.cos_hdgt_ * dz;

            // nose wheel
            float nw_z = local_z - plane.nw_z;
            float nw_x = local_x + plane.nw_z * sinf(kD2R * local_hdgt);

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
        if (active_stand_)
            active_stand_->Deactivate();

        active_stand_ = min_stand;
        active_stand_->SetDgsDist();
        state_ = ENGAGED;
    }
}

float
Airport::StateMachine()
{
    if (error_disabled)
        return 0.0f;

    if (state_ == INACTIVE)
        return 2.0f;

    // throttle costly search
    if (now > nearest_stand_ts_ + 2.0) {
        FindNearestStand();
        nearest_stand_ts_ = now;
    }

    if (active_stand_ == nullptr) {
        state_ = ACTIVE;
        return 2.0;
    }

    int lr_prev = lr_;
    int track_prev = track_;
    float distance_prev = distance_;

    float loop_delay = 0.2;
    state_t new_state = state_;

    // xform plane pos into stand local coordinate system
    float dx = XPLMGetDataf(plane_x_dr) - active_stand_->x_;
    float dz = XPLMGetDataf(plane_z_dr) - active_stand_->z_;
    float local_x =  active_stand_->cos_hdgt_ * dx + active_stand_->sin_hdgt_ * dz;
    float local_z = -active_stand_->sin_hdgt_ * dx + active_stand_->cos_hdgt_ * dz;

    // relative reading to stand +/- 180
    float local_hdgt = RA(XPLMGetDataf(plane_true_psi_dr) - active_stand_->hdgt());

    // nose wheel
    float nw_z = local_z - plane.nw_z;
    float nw_x = local_x + plane.nw_z * sinf(kD2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - plane.mw_z;
    float mw_x = local_x + plane.mw_z * sinf(kD2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = std::clamp((nw_z - 6.0) / 20.0, 0.0, 1.0);
    float plane_z_dr = (1.0 - a) * plane.nw_z + a * plane.mw_z;
    float z_dr = local_z - plane_z_dr;
    float x_dr = local_x + plane_z_dr * sinf(kD2R * local_hdgt);

    float azimuth;
    if (fabs(x_dr) > 0.5 && z_dr > 0)
        azimuth = atanf(x_dr / (z_dr + 0.5 * active_stand_->dgs_dist_)) / kD2R;
    else
        azimuth = 0.0;

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 0.5 * active_stand_->dgs_dist_)) / kD2R;
    else
        azimuth_nw = 0.0;

    int locgood = (fabsf(mw_x) <= GOOD_X && fabsf(nw_z) <= GOOD_Z);
    int beacon_on = plane.BeaconState();

    status_ = lr_ = track_ = 0;
    distance_ = nw_z - GOOD_Z;

    // catch the phase ~180° point -> the Marshaller's arm is straight
    float sin_wave = XPLMGetDataf(sin_wave_dr);
    int phase180 = (sin_wave_prev_ > 0.0) && (sin_wave <= 0.0);
    sin_wave_prev_ = sin_wave;

    // set drefs according to *current* state
    switch (state_) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance_ <= CAP_Z) && (fabsf(azimuth_nw) <= CAP_A))
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

                if ((distance_ > CAP_Z) || (fabsf(azimuth_nw) > CAP_A)) {
                    new_state = ENGAGED;    // moving away from current gate
                    break;
                }

                status_ = 1;	// plane id
                if (distance_ > AZI_Z || fabsf(azimuth_nw) > AZI_A) {
                    track_=1;	// lead-in only
                    break;
                }

                // compute distance_ and guidance commands
                azimuth = std::clamp(azimuth, -AZI_A, AZI_A);
                float req_hdgt = -3.5 * azimuth;        // to track back to centerline
                float d_hdgt = req_hdgt - local_hdgt;   // degrees to turn

                if (now > update_dgs_log_ts_ + 2.0)
                    LogMsg("azimuth: %0.1f, mw: (%0.1f, %0.1f), nw: (%0.1f, %0.1f), ref: (%0.1f, %0.1f), "
                           "x: %0.1f, local_hdgt: %0.1f, d_hdgt: %0.1f",
                           azimuth, mw_x, mw_z, nw_x, nw_z,
                           x_dr, z_dr,
                           local_x, local_hdgt, d_hdgt);

                if (d_hdgt < -1.5)
                    lr_ = 2;
                else if (d_hdgt > 1.5)
                    lr_ = 1;

                // xform azimuth to values required ob OBJ
                azimuth = std::clamp(azimuth, -AZI_DISP_A, AZI_DISP_A) * 4.0 / AZI_DISP_A;
                azimuth=((float)((int)(azimuth * 2))) / 2;  // round to 0.5 increments

                if (distance_ <= REM_Z/2) {
                    track_ = 3;
                    loop_delay = 0.03;
                } else // azimuth only
                    track_ = 2;

                if (! phase180) { // no wild oscillation
                    lr_ = lr_prev;

                    // sync transition with Marshaller's arm movement
                    if (active_stand_->dgs_type_ == kMarshaller && track_ == 3 && track_prev == 2) {
                        track_ = track_prev;
                        distance_ = distance_prev;
                    }
                }
            }
            break;

        case GOOD:  {
                // @stop position
                status_ = 2; lr_ = 3;

                int parkbrake_set = (XPLMGetDataf(parkbrake_dr) > 0.5);
                if (!locgood)
                    new_state = TRACK;
                else if (parkbrake_set || !beacon_on)
                    new_state = PARKED;
            }
            break;

        case BAD:
            if (!beacon_on
                && (now > timestamp_ + 5.0)) {
                ResetState(INACTIVE);
                return loop_delay;
            }

            if (nw_z >= -GOOD_Z)
                new_state = TRACK;
            else {
                // Too far
                status_ = 4;
                lr_ = 3;
            }
            break;

        case PARKED:
            status_ = 3;
            lr_ = 0;
            // wait for beacon off
            if (! beacon_on) {
                new_state = DONE;
                if (operation_mode == MODE_AUTO && ! plane.dont_connect_jetway)
                    XPLMCommandOnce(toggle_jetway_cmdr);
            }
            break;

        case DONE:
            if (now > timestamp_ + 5.0) {
                ResetState(INACTIVE);
                return loop_delay;
            }
            break;

        default:
            break;
    }

    if (new_state != state_) {
        LogMsg("state transition %s -> %s, beacon: %d", state_str[state_], state_str[new_state], beacon_on);
        state_ = new_state;
        timestamp_ = now;
        return -1;  // see you on next frame
    }

    if (state_ > ACTIVE) {
        // xform drefs into required constraints for the OBJs
        if (track_ == 0 || track_ == 1) {
            distance_ = 0;
            azimuth = 0.0;
        }

        distance_ = std::clamp(distance_, -GOOD_Z, REM_Z);

        // is not necessary for Marshaller + SafedockT2
        // distance_=((float)((int)((distance_)*2))) / 2;    // multiple of 0.5m

        // don't flood the log
        if (now > update_dgs_log_ts_ + 2.0) {
            update_dgs_log_ts_ = now;
            LogMsg("stand: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, azimuth: %0.1f",
                   active_stand_->name().c_str(), state_str[state_], status_, track_, lr_, distance_, azimuth);
        }

        if (active_stand_->dgs_type_ == kMarshaller) {
            if (marshaller == nullptr)
                marshaller = std::make_unique<Marshaller>();
            marshaller->SetPos(&active_stand_->drawinfo_, status_, track_, lr_, distance_);
        } else {
            static const float min_brightness = 0.025;   // relativ to 1
            float brightness = min_brightness + (1 - min_brightness) * powf(1 - XPLMGetDataf(percent_lights_dr), 1.5);
            active_stand_->SetState(status_, track_, lr_, azimuth, distance_, (state_ == TRACK), brightness);
        }
    }

    return loop_delay;
}

//------------------------------------------------------------------------------------

// set mode to arrival
static void
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
    LogMsg("airport activated: %s, new state: ACTIVE", arpt->name().c_str());
    arpt->ResetState(Airport::ACTIVE);
    UpdateUI();
}

// dummy accessor routine
static float
getdgsfloat(XPLMDataRef inRefcon)
{
    return -1.0;
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

    if (arpt && arpt->state() >= Airport::ACTIVE)
        loop_delay = arpt->StateMachine();

    return loop_delay;
}

// call backs for commands
static int
CmdCb(XPLMCommandRef cmdr, XPLMCommandPhase phase, [[maybe_unused]] void *ref)
{
    static float dgs_dist_adjust;                   // for moving closer

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
        dgs_dist_adjust -= 2.0f;
        if (dgs_dist_adjust <= -13.0)
            dgs_dist_adjust = 0.0f;

        LogMsg("dgs_dist_adjust set to %0.1f", dgs_dist_adjust);
        if (arpt)
            arpt->SetDgsDistAdjust(dgs_dist_adjust);
    } else if (cmdr == toggle_ui_cmdr) {
        LogMsg("cmd toggle_ui");
        ToggleUI();
    }

    return 0;
}

// call back for menu
static void
menu_cb([[maybe_unused]] void *menu_ref, void *item_ref)
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
        plane.PlaneLoadedCb();
    }
}
