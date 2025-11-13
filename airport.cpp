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

#include <cassert>
#include <cmath>
#include <fstream>
#include <map>
#include <algorithm>

#include "autodgs.h"
#include "airport.h"
#include "plane.h"
#include "simbrief.h"

#include "XPLMGraphics.h"

namespace fem = flat_earth_math;

// DGS _A = angles [°] (to centerline), _X, _Z = [m] (to stand)
static constexpr float kCapA = 15;   // Capture
static constexpr float kCapZ = 105;  // (50-80 in Safedock2 flier)

static constexpr float kAziA = 15;  // °, provide azimuth guidance
static constexpr float kAziZ = 85;  // m, from this distance

static constexpr float kAziCrossover = 6;  // m, switch from azimuth to xtrack guidance

static constexpr float kGoodZ_p = 0.2;   // stop position for nw / to stop
static constexpr float kGoodZ_m = -0.5;  // stop position for nw / beyond stop

static constexpr float kGoodX = 2.0;  // for mw

static constexpr float kCrZ = 12;  // Closing Rate starts here VDGS, Marshaller uses 0.5 * kCrZ

static constexpr int kTurnRight = 1;  // arrow on left side
static constexpr int kTurnLeft = 2;   // arrow on right side

static constexpr float kVdgsDefaultDist = 15.0;  // m
static constexpr float kMarshallerDefaultDist = 25.0;
static constexpr float kVdgsDefaultHeight = 5.0;  // m AGL

static constexpr float kDgsMinDist = 8.0;
static constexpr float kDgsMaxDist = 30.0;
static constexpr float kDgsMoveDeltaMin = 1.0;  // min/max for 'move closer' cmd
static constexpr float kDgsMoveDeltaMax = 3.0;

static bool marshaller_pe_dist_updated;  // according to pilot's eye AGL
static float marshaller_pe_dist = kMarshallerDefaultDist;

static std::unique_ptr<Ofp> ofp;
static int ofp_seqno;
static float ofp_ts;

class Marshaller;

// there is exactly none or one Marshaller
static std::unique_ptr<Marshaller> marshaller;

#define SQR(x) ((x) * (x))

class Marshaller {
    XPLMInstanceRef inst_ref_;

  public:
    Marshaller();
    ~Marshaller();
    void SetPos(const XPLMDrawInfo_t *drawinfo, int status, int track, int lr, float distance);
};

//------------------------------------------------------------------------------------
Marshaller::Marshaller() {
    inst_ref_ = XPLMCreateInstance(dgs_obj[kMarshaller], dgs_dlist_dr);
}

Marshaller::~Marshaller() {
    if (inst_ref_)
        XPLMDestroyInstance(inst_ref_);
}

void Marshaller::SetPos(const XPLMDrawInfo_t* drawinfo, int status, int track, int lr, float distance) {
    float drefs[DGS_DR_NUM]{};
    drefs[DGS_DR_STATUS] = status;
    drefs[DGS_DR_DISTANCE] = distance;
    drefs[DGS_DR_TRACK] = track;
    drefs[DGS_DR_LR] = lr;
    XPLMInstanceSetPosition(inst_ref_, drawinfo, drefs);
}

//------------------------------------------------------------------------------------
ScrollTxt::ScrollTxt(const std::string& txt) {
    txt_ = txt;
    dr_scroll_ = 10;  // right most
    chars_[kR1Nchar - 1] = txt_[0];
    char_pos_ = 0;
}

void ScrollTxt::Tick(float* drefs) {
    if (txt_.size() == 0)
        return;

    dr_scroll_ -= 2;
    if (dr_scroll_ < 0) {
        dr_scroll_ = 10;
        char_pos_++;
        if (char_pos_ >= (int)txt_.size())
            char_pos_ = 0;

        for (int i = 1; i < kR1Nchar; i++)
            chars_[i - 1] = chars_[i];
        chars_[kR1Nchar - 1] = txt_[char_pos_];
    }

    drefs[DGS_DR_R1_SCROLL] = dr_scroll_;
    for (int i = 0; i < kR1Nchar; i++)
        drefs[DGS_DR_R1C0 + i] = chars_[i];
};

//------------------------------------------------------------------------------------
Stand::Stand(const AptStand& as, float elevation, int dgs_type, float dgs_dist) : as_(as) {
    // create display name
    // a stand name can be anything between "1" and "Gate A 40 (Class C, Terminal 3)"
    // we try to extract the net name "A 40" in the latter case
    const std::string& asn = as_.name;

    if (asn.starts_with("Stand"))
        display_name_ = asn.substr(6);
    else if (asn.starts_with("Gate"))
        display_name_ = asn.substr(5);
    else
        display_name_ = as_.name;

    // delete stuff following and including a "(,;"
    if (display_name_.length() > kR1Nchar) {
        const auto i = display_name_.find_first_of("(,;");
        if (i != std::string::npos) {
            display_name_.resize(i);
            display_name_.erase(display_name_.find_last_not_of(" ") + 1);
        }
    }

    // trim whitespace
    display_name_.erase(0, display_name_.find_first_not_of(" "));

    if (display_name_.length() > kR1Nchar)
        display_name_.clear();  // give up

    double x, y, z;
    XPLMWorldToLocal(as_.lat, as_.lon, elevation, &x, &y, &z);

    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};
    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y, z, &probeinfo))
        throw std::runtime_error("XPLMProbeTerrainXYZ failed");

    is_wet_ = probeinfo.is_wet;
    x_ = probeinfo.locationX;
    y_ = probeinfo.locationY;
    z_ = probeinfo.locationZ;

    sin_hdgt_ = sinf(kD2R * as_.hdgt);
    cos_hdgt_ = cosf(kD2R * as_.hdgt);
    drawinfo_.structSize = sizeof(drawinfo_);
    drawinfo_.heading = as_.hdgt;
    drawinfo_.pitch = drawinfo_.roll = 0.0f;
    vdgs_inst_ref_ = nullptr;
    pole_base_inst_ref_ = nullptr;

    marshaller_max_dist_ = kDgsMaxDist;

    dgs_dist_ = dgs_dist;
    dgs_type_ = -1;  // invalidate to ensure that SetDgsType's code does something
    SetDgsType(dgs_type);

    // LogMsg("Stand '%s', disp: '%s', is_wet: %d, type: %d, dgs_dist: %0.1f constructed", cname(),
    // display_name_.c_str(),
    //        is_wet_, dgs_type_, dgs_dist_);
}

Stand::~Stand() {
    if (vdgs_inst_ref_)
        XPLMDestroyInstance(vdgs_inst_ref_);
    if (pole_base_inst_ref_)
        XPLMDestroyInstance(pole_base_inst_ref_);
}

void Stand::SetDgsType(int dgs_type) {
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

        if (pole_base_inst_ref_)
            XPLMDestroyInstance(pole_base_inst_ref_);
        pole_base_inst_ref_ = nullptr;

        SetDgsDist();
    } else {
        static const char* null_dlist[] = {nullptr};
        marshaller = nullptr;
        vdgs_inst_ref_ = XPLMCreateInstance(dgs_obj[kVDGS], dgs_dlist_dr);
        pole_base_inst_ref_ = XPLMCreateInstance(pole_base_obj, null_dlist);
        SetDgsDist();
        SetIdle();
    }
}

void Stand::CycleDgsType() {
    int new_dgs_type = (dgs_type_ == kMarshaller ? kVDGS : kMarshaller);
    SetDgsType(new_dgs_type);
}

void Stand::SetState(int status, int track, int lr, float xtrack, float distance, bool slow) {
    assert(dgs_type_ == kVDGS);

    int d_0 = 0;
    int d_01 = 0;
    // according to Safegate_SDK_UG_Pilots_v1.10_s.pdf
    // > 3m: 1.0 m decrements, <= 3m 0.2m decrements
    if (0.0f <= distance && distance < 10.0f) {
        d_0 = distance;
        if (d_0 < 3) {
            int d = (distance - d_0) * 10.0f;
            d &= ~1;  // make it even = 0.2m increments
            d_01 = d;
        }
    }

    distance = ((float)((int)((distance) * 2))) / 2;  // multiple of 0.5m

    float drefs[DGS_DR_NUM]{};
    drefs[DGS_DR_STATUS] = status;
    drefs[DGS_DR_TRACK] = track;
    drefs[DGS_DR_DISTANCE] = distance;
    drefs[DGS_DR_DISTANCE_0] = d_0;
    drefs[DGS_DR_DISTANCE_01] = d_01;
    drefs[DGS_DR_XTRACK] = xtrack;
    drefs[DGS_DR_LR] = lr;

    if (slow) {
        drefs[DGS_DR_ICAO_0] = 'S';
        drefs[DGS_DR_ICAO_1] = 'L';
        drefs[DGS_DR_ICAO_2] = 'O';
        drefs[DGS_DR_ICAO_3] = 'W';
    } else
        for (int i = 0; i < 4; i++)
            drefs[DGS_DR_ICAO_0 + i] = (int)plane.acf_icao[i];

    XPLMInstanceSetPosition(vdgs_inst_ref_, &drawinfo_, drefs);
}

float Stand::SetState(int pax_no) {
    assert(dgs_type_ == kVDGS);

    float delay = 1.0f;

    float drefs[DGS_DR_NUM]{};

    if (scroll_txt_) {
        scroll_txt_->Tick(drefs);
        delay = 0.05f;
    } else {
        int n = display_name_.length();
        for (int i = 0; i < n; i++)
            drefs[DGS_DR_R1C0 + i] = display_name_[i];
        drefs[DGS_DR_R1_SCROLL] = (5 * 16 - (n * 12 - 2)) / 2;  // center
    }

    if (pax_no > 0) {
        int pn[3]{-1, -1, -1};
        for (int i = 0; i < 3; i++) {
            pn[i] = pax_no % 10;
            pax_no /= 10;
            if (pax_no == 0)
                break;
        }
        drefs[DGS_DR_BOARDING] = 1;
        for (int i = 0; i < 3; i++)
            drefs[DGS_DR_PAXNO_0 + i] = pn[i];
    }

    XPLMInstanceSetPosition(vdgs_inst_ref_, &drawinfo_, drefs);
    return delay;
}

void Stand::SetIdle() {
    if (vdgs_inst_ref_ == nullptr)
        return;

    LogMsg("SetIdle stand: '%s'", cname());

    scroll_txt_ = nullptr;

    float drefs[DGS_DR_NUM]{};

    int n = display_name_.length();
    for (int i = 0; i < n; i++)
        drefs[DGS_DR_R1C0 + i] = display_name_[i];
    drefs[DGS_DR_R1_SCROLL] = (5 * 16 - (n * 12 - 2)) / 2;  // center

    XPLMInstanceSetPosition(vdgs_inst_ref_, &drawinfo_, drefs);
}

// compute the DGS position
void Stand::SetDgsDist() {
    XPLMProbeInfo_t probeinfo = {.structSize = sizeof(XPLMProbeInfo_t)};

    if (dgs_type_ == kMarshaller) {
        if (!marshaller_pe_dist_updated) {
            // determine marshaller_dist_default depending on pilot eye height agl
            if (plane.pe_y_0_valid) {
                float plane_x = XPLMGetDataf(plane_x_dr);
                float plane_y = XPLMGetDataf(plane_y_dr);
                float plane_z = XPLMGetDataf(plane_z_dr);

                // get terrain y below plane y

                if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, plane_x, plane_y, plane_z, &probeinfo))
                    throw std::runtime_error("XPLMProbeTerrainXYZ failed");

                // pilot eye above agl
                float pe_agl = plane_y - probeinfo.locationY + plane.pe_y_0;

                // 4.3 ~ 1 / tan(13°) -> 13° down look
                marshaller_pe_dist = std::max(kDgsMinDist, std::min(4.3f * pe_agl, kDgsMaxDist));
                marshaller_pe_dist_updated = true;
                LogMsg("setting Marshaller PE distance, pe_agl: %0.2f, dist: %0.1f", pe_agl, marshaller_pe_dist);
            }
        }

        dgs_dist_ = std::min(marshaller_pe_dist, dgs_dist_);
    }

    // xform vector (0, -dgs_dist) into global frame
    float x = x_ + -sin_hdgt_ * (-dgs_dist_);
    float z = z_ +  cos_hdgt_ * (-dgs_dist_);

    if (xplm_ProbeHitTerrain != XPLMProbeTerrainXYZ(probe_ref, x, y_, z, &probeinfo))
        throw std::runtime_error("XPLMProbeTerrainXYZ failed");

    drawinfo_.x = probeinfo.locationX;
    drawinfo_.y = probeinfo.locationY;
    drawinfo_.z = probeinfo.locationZ;
    if (dgs_type_ == kVDGS) {
        assert(pole_base_inst_ref_ != nullptr);
        XPLMInstanceSetPosition(pole_base_inst_ref_, &drawinfo_, nullptr);
        drawinfo_.y += kVdgsDefaultHeight;
    }
}

// adjust may be negative to move it closer
void Stand::DgsMoveCloser() {
    float delta = std::clamp(0.1f * dgs_dist_, kDgsMoveDeltaMin, kDgsMoveDeltaMax);
    dgs_dist_ -= delta;
    if (dgs_dist_ < kDgsMinDist)  // wrap around
        dgs_dist_ = kDgsMaxDist;
    marshaller_max_dist_ = dgs_dist_;
    SetDgsDist();
    LogMsg("stand' '%s', new dgs_dist: %0.1f", cname(), dgs_dist_);
}

//------------------------------------------------------------------------------------
const char* const Airport::state_str[] = {"INACTIVE", "DEPARTURE", "BOARDING", "ARRIVAL", "ENGAGED", "TRACK",
                                          "GOOD",     "BAD",       "PARKED",   "CHOCKS",  "DONE"};

void LoadCfg(const std::string& pathname, std::unordered_map<std::string, std::tuple<int, float>>& cfg);

Airport::Airport(const AptAirport& apt_airport) {
    name_ = apt_airport.icao_;
    stands_.reserve(apt_airport.stands_.size());
    float arpt_elevation = XPLMGetDataf(plane_elevation_dr);  // best guess

    std::unordered_map<std::string, std::tuple<int, float>> cfg;
    LoadCfg(user_cfg_dir + name() + ".cfg", cfg);
    if (cfg.empty())
        LoadCfg(sys_cfg_dir + name() + ".cfg", cfg);

    for (auto const& as : apt_airport.stands_) {
        int dgs_type = kAutomatic;
        float dgs_dist;
        if (as.has_jw)
            dgs_dist = kVdgsDefaultDist;
        else
            dgs_dist = kMarshallerDefaultDist;

        // override with user defined config
        if (const auto it = cfg.find(as.name); it != cfg.end()) {
            std::tie(dgs_type, dgs_dist) = it->second;
            LogMsg("found in config '%s', %d, %0.1f", as.name.c_str(), dgs_type, dgs_dist);
        }

        stands_.emplace_back(as, arpt_elevation, dgs_type, dgs_dist);
    }

    state_ = INACTIVE;
    active_stand_ = selected_stand_ = departure_stand_ = -1;
    user_cfg_changed_ = false;

    timestamp_ = distance_ = sin_wave_prev_ = 0.0f;
    departure_stand_ts_ = nearest_stand_ts_ = update_dgs_log_ts_ = 0.0f;
}

Airport::~Airport() {
    FlushUserCfg();
    LogMsg("Airport '%s' destructed", name().c_str());
}

void LoadCfg(const std::string& pathname, std::unordered_map<std::string, std::tuple<int, float>>& cfg) {
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
            float dgs_dist;
            char type;
            int n = sscanf(line.c_str(), "%c,%f, %n", &type, &dgs_dist, &ofs);
            if (n != 2 || ofs >= (int)line.size()  // distrust user input
                || !(type == 'V' || type == 'M') || dgs_dist < kDgsMinDist || dgs_dist > kDgsMaxDist) {
                LogMsg("invalid line: '%s' %d", line.c_str(), n);
                continue;
            }

            cfg[line.substr(ofs)] = std::make_tuple((type == 'M' ? kMarshaller : kVDGS), dgs_dist);
        }
    }
}

void Airport::FlushUserCfg() {
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
    for (auto const& s : stands_) {
        char line[200];
        float dist = s.dgs_type_ == kMarshaller ? s.marshaller_max_dist_ : s.dgs_dist_;
        snprintf(line, sizeof(line), "%c, %5.1f, %s\n", (s.dgs_type_ == kMarshaller ? 'M' : 'V'), dist,
                 s.name().c_str());
        cfg[s.name()] = line;
    }

    f << "# type, dgs_dist, stand_name\n";
    f << "# type = M or V, dgs_dist = dist from parking pos in m\n";

    for (auto const& kv : cfg)
        f << kv.second;

    LogMsg("cfg written to '%s'", fn.c_str());
}

std::unique_ptr<Airport> Airport::LoadAirport(const std::string& icao) {
    auto arpt = AptAirport::LookupAirport(icao);
    if (arpt == nullptr)
        return nullptr;

    return std::make_unique<Airport>(*arpt);
}

std::tuple<int, const std::string> Airport::GetStand(int idx) const {
    assert(0 <= idx && idx < (int)stands_.size());
    const Stand& s = stands_[idx];
    std::string name{s.name()};
    int dgs_type = s.dgs_type_;
    return std::make_tuple(dgs_type, name);
}

void Airport::SetSelectedStand(int selected_stand) {
    assert(-1 <= selected_stand && selected_stand < (int)stands_.size());
    if (selected_stand_ == selected_stand)
        return;
    selected_stand_ = selected_stand;

    if (arpt->state() > Airport::ARRIVAL)
        arpt->ResetState(Airport::ARRIVAL);
}

void Airport::DgsMoveCloser() {
    if (active_stand_ >= 0) {
        stands_[active_stand_].DgsMoveCloser();
        user_cfg_changed_ = true;
    }
}

void Airport::SetDgsType(int dgs_type) {
    if (active_stand_ >= 0) {
        stands_[active_stand_].SetDgsType(dgs_type);
        user_cfg_changed_ = true;
    }
}

int Airport::GetDgsType() const {
    // called by the ui and the selected_stand may not already be the active stand
    if (selected_stand_ >= 0)
        return stands_[selected_stand_].dgs_type_;
    if (active_stand_ >= 0)
        return stands_[active_stand_].dgs_type_;

    return kMarshaller;
}

void Airport::ResetState(state_t new_state) {
    if (state_ != new_state)
        LogMsg("setting state to %s", state_str[new_state]);

    state_ = new_state;
    if (active_stand_ >= 0)
        stands_[active_stand_].SetIdle();
    active_stand_ = -1;

    marshaller = nullptr;
    if (new_state == INACTIVE) {
        selected_stand_ = -1;
        FlushUserCfg();
    }

    marshaller_pe_dist_updated = false;
    marshaller_pe_dist = kMarshallerDefaultDist;
    UpdateUI();
}

void Airport::CycleDgsType() {
    if (active_stand_ >= 0) {
        stands_[active_stand_].CycleDgsType();
        user_cfg_changed_ = true;
    }
}

void Airport::FindNearestStand() {
    // Check if the currently active stand is also the selected stand
    if (active_stand_ >= 0 && active_stand_ == selected_stand_)
        return;

    double dist = 1.0E10;
    int min_stand = -1;

    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);

    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    if (selected_stand_ >= 0) {
        dist = 0.0;
        min_stand = selected_stand_;
    } else {
        for (int i = 0; i < (int)stands_.size(); i++) {
            const Stand& s = stands_[i];
            if (s.is_wet_)
                continue;

            // heading in local system
            float local_hdgt = fem::RA(plane_hdgt - s.hdgt());

            if (fabsf(local_hdgt) > 90.0f)
                continue;  // not looking to stand

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
            if (d > kCapZ + 50)  // fast exit
                continue;

            // LogMsg("stand: %s, z: %2.1f, x: %2.1f", s.name(), nw_z, nw_x);

            // behind
            if (nw_z < -4.0) {
                // LogMsg("behind: %s", s.cname());
                continue;
            }

            if (nw_z > 10.0) {
                float angle = atan(nw_x / nw_z) / kD2R;
                // LogMsg("angle to plane: %s, %3.1f", s.cname(), angle);

                // check whether plane is in a +-60° sector relative to stand
                if (fabsf(angle) > 60.0)
                    continue;

                // drive-by and beyond a +- 60° sector relative to plane's direction
                float rel_to_stand = fem::RA(-angle - local_hdgt);
                // LogMsg("rel_to_stand: %s, nw_x: %0.1f, local_hdgt %0.1f, rel_to_stand: %0.1f",
                //        s.cname(), nw_x, local_hdgt, rel_to_stand);
                if ((nw_x > 10.0 && rel_to_stand < -60.0) || (nw_x < -10.0 && rel_to_stand > 60.0)) {
                    // LogMsg("drive by %s", s.cname());
                    continue;
                }
            }

            // for the final comparison give xtrack a higher weight
            static const float xtrack_weight = 4.0;
            d = sqrt(SQR(xtrack_weight * nw_x) + SQR(nw_z));

            if (d < dist) {
                // LogMsg("new min: %s, z: %2.1f, x: %2.1f", s.cname(), nw_z, nw_x);
                dist = d;
                min_stand = i;
            }
        }
    }

    if (min_stand >= 0 && min_stand != active_stand_) {
        Stand& ms = stands_[min_stand];
        LogMsg("stand: %s, lat: %f, lon: %f, hdgt: %f, dist: %f", ms.cname(), ms.lat(), ms.lon(), ms.hdgt(), dist);

        if (active_stand_ >= 0)
            stands_[active_stand_].SetIdle();

        ms.SetDgsDist();
        active_stand_ = min_stand;
        state_ = ENGAGED;
    }
}

// find the stand the plane is parked on
int Airport::FindDepartureStand() {
    float plane_x = XPLMGetDataf(plane_x_dr);
    float plane_z = XPLMGetDataf(plane_z_dr);
    float plane_hdgt = XPLMGetDataf(plane_true_psi_dr);

    // nose wheel
    float nw_z = plane_z - plane.nw_z * cosf(kD2R * plane_hdgt);
    float nw_x = plane_x + plane.nw_z * sinf(kD2R * plane_hdgt);

    for (int i = 0; i < (int)stands_.size(); i++) {
        const Stand& s = stands_[i];
        if (s.dgs_type_ != kVDGS)
            continue;

        if (fabsf(fem::RA(plane_hdgt - s.hdgt())) > 3.0f)
            continue;

        float dx = nw_x - s.x_;
        float dz = nw_z - s.z_;
        // LogMsg("stand: %s, z: %2.1f, x: %2.1f", s.cname(), dz, dx);
        if (dx * dx + dz * dz < 1.0f)
            // Return the first matching stand found
            return i;
    }

    return -1;
}

float Airport::StateMachine() {
    if (error_disabled)
        return 0.0f;

    state_t state_prev = state_;

    // DEPARTURE and friends ...
    // that's all low freq stuff
    if (INACTIVE <= state_ && state_ <= BOARDING) {
        if (now > departure_stand_ts_ + 2.0f) {
            departure_stand_ts_ = now;
            // on beacon or engine or teleportation -> INACTIVE
            if (plane.BeaconOn() || plane.EnginesOn()) {
                if (departure_stand_ >= 0)
                    stands_[departure_stand_].SetIdle();
                departure_stand_ = -1;
                state_ = INACTIVE;
                return 2.0f;
            }

            // check for stand (new or changed)
            int dsi = FindDepartureStand();
            // LogMsg("departure stand: %s, dsi: %d", dsi >= 0 ? stands_[dsi].cname() : "*none*", dsi);
            if (dsi != departure_stand_) {
                if (departure_stand_ >= 0)
                    stands_[departure_stand_].SetIdle();
                LogMsg("Departure stand now '%s'", dsi >= 0 ? stands_[dsi].cname() : "*none*");
                if (dsi >= 0) {
                    Stand& ds = stands_[dsi];
                    if (ds.display_name_.empty())
                        ds.scroll_txt_ = std::make_unique<ScrollTxt>(name() + "   ");
                    else
                        ds.scroll_txt_ = std::make_unique<ScrollTxt>(name() + " STAND " + ds.display_name_ + "   ");
                }
                departure_stand_ = dsi;
            }
        }

        if (departure_stand_ < 0) {
            state_ = INACTIVE;
            return 4.0f;
        }

        Stand& ds = stands_[departure_stand_];

        if (plane.PaxNo() <= 0) {
            state_ = DEPARTURE;
            if (state_ != state_prev)
                LogMsg("New state %s", state_str[state_]);
            // FALLTHROUGH
        }

        if (state_ == INACTIVE) {
            return std::min(4.0f, ds.SetState(0));
        }

        // cdm data may come in late during boarding
        if (state_ == DEPARTURE || state_ == BOARDING) {
            // although LoadIfNewer is cheap throttling it is even cheaper
            if (now > ofp_ts + 5.0f) {
                ofp_ts = now;
                ofp = Ofp::LoadIfNewer(ofp_seqno);  // fetch ofp
                if (ofp) {
                    ofp_seqno = ofp->seqno;
                    std::string ofp_str = ofp->GenDepartureStr();
                    if (ds.display_name_.empty())
                        ds.scroll_txt_ = std::make_unique<ScrollTxt>(name() + "   " + ofp_str + "   ");
                    else
                        ds.scroll_txt_ = std::make_unique<ScrollTxt>(name() + " STAND " + ds.display_name_ + "   " +
                                                                     ofp_str + "   ");
                }
            }
        }

        if (state_ == DEPARTURE) {
            if (plane.PaxNo() > 0) {
                state_ = BOARDING;
                LogMsg("New state %s", state_str[state_]);
                // FALLTHROUGH
            } else
                return ds.SetState(0);  // just scroll the text
        }

        if (state_ == BOARDING) {
            int pax_no = plane.PaxNo();
            // LogMsg("boarding PaxNo: %d", pax_no);
            return ds.SetState(pax_no);
        }
    }

    // ARRIVAL and friends ...
    // this can be high freq stuff

    // throttle costly search...
    // ... but if we have a new selected stand activate it immediately
    if (now > nearest_stand_ts_ + 2.0 || (selected_stand_ >= 0 && selected_stand_ != active_stand_)) {
        FindNearestStand();
        nearest_stand_ts_ = now;
    }

    if (active_stand_ < 0) {
        state_ = ARRIVAL;
        return 2.0;
    }

    state_t new_state = state_;

    int lr_prev = lr_;
    int track_prev = track_;
    float distance_prev = distance_;

    float loop_delay = 0.2;

    Stand& as = stands_[active_stand_];

    // xform plane pos into stand local coordinate system
    float dx = XPLMGetDataf(plane_x_dr) - as.x_;
    float dz = XPLMGetDataf(plane_z_dr) - as.z_;
    float local_x =  as.cos_hdgt_ * dx + as.sin_hdgt_ * dz;
    float local_z = -as.sin_hdgt_ * dx + as.cos_hdgt_ * dz;

    // relative reading to stand +/- 180
    float local_hdgt = fem::RA(XPLMGetDataf(plane_true_psi_dr) - as.hdgt());

    // nose wheel
    float nw_z = local_z - plane.nw_z;
    float nw_x = local_x + plane.nw_z * sinf(kD2R * local_hdgt);

    // main wheel pos on logitudinal axis
    float mw_z = local_z - plane.mw_z;
    float mw_x = local_x + plane.mw_z * sinf(kD2R * local_hdgt);

    // ref pos on logitudinal axis of acf blending from mw to nw as we come closer
    // should be nw if dist is below 6 m
    float a = std::clamp((nw_z - kAziCrossover) / 20.0, 0.0, 1.0);
    float plane_ref_z = (1.0 - a) * plane.nw_z + a * plane.mw_z;
    float ref_z = local_z - plane_ref_z;
    float ref_x = local_x + plane_ref_z * sinf(kD2R * local_hdgt);

    float xtrack = 0.0;  // xtrack for VDGS, set later if needed

    float azimuth_nw;
    if (nw_z > 0)
        azimuth_nw = atanf(nw_x / (nw_z + 5.0f)) / kD2R;
    else
        azimuth_nw = 0.0;

    bool locgood = (fabsf(mw_x) <= kGoodX && kGoodZ_m <= nw_z && nw_z <= kGoodZ_p);
    bool beacon_on = plane.BeaconOn();

    status_ = lr_ = track_ = 0;
    distance_ = nw_z;
    bool slow = false;

   // set drefs according to *current* state
    switch (state_) {
        case ENGAGED:
            if (beacon_on) {
                if ((distance_ <= kCapZ) && (fabsf(azimuth_nw) <= kCapA))
                    new_state = TRACK;
            } else {  // not beacon_on
                new_state = DONE;
            }

            // always light up the VDGS or signal "this way" for the selected stand
            if (active_stand_ == selected_stand_) {
                status_ = 1;  // plane id
                track_ = 1;   // lead-in
            }
            break;

        case TRACK: {
            if (!beacon_on) {  // don't get stuck in TRACK
                new_state = DONE;
                break;
            }

            if (locgood) {
                new_state = GOOD;
                break;
            }

            if (nw_z < kGoodZ_m) {
                new_state = BAD;
                break;
            }

            if ((distance_ > kCapZ) || (fabsf(azimuth_nw) > kCapA)) {
                new_state = ENGAGED;  // moving away from current gate
                break;
            }

            status_ = 1;  // plane id
            if (distance_ > kAziZ || fabsf(azimuth_nw) > kAziA) {
                track_ = 1;  // lead-in only
                break;
            }

            // compute distance_ and guidance commands

            // xform xtrack distance to values required by the OBJ
            xtrack = std::clamp(ref_x, -4.0f, 4.0f);     // in m, 4 is hardcoded in the OBJ
            xtrack = std::roundf(xtrack * 2.0f) / 2.0f;  // round to 0.5 increments

            // compute left/right command
            if (ref_z > kAziCrossover) {
                // far, aim to an intermediate point between ref point and stand
                float req_hdgt = atanf(-ref_x / (0.3f * ref_z)) / kD2R;  // required hdgt
                float d_hdgt = req_hdgt - local_hdgt;  // degrees to turn
                if (d_hdgt < -1.5f)
                    lr_ = kTurnLeft;
                else if (d_hdgt > 1.5f)
                    lr_ = kTurnRight;

                if (now > update_dgs_log_ts_ + 2.0)
                    LogMsg(
                        "req_hdgt: %0.1f, local_hdgt: %0.1f, d_hdgt: %0.1f, mw: (%0.1f, %0.1f), nw: (%0.1f, %0.1f), "
                        "ref: (%0.1f, %0.1f), "
                        "x: %0.1f, ",
                        req_hdgt, local_hdgt, d_hdgt, mw_x, mw_z, nw_x, nw_z, ref_x, ref_z, local_x);

            } else {
                // close, use xtrack
                if (ref_x < -0.25f)
                    lr_ = kTurnRight;
                else if (ref_x > 0.25f)
                    lr_ = kTurnLeft;
            }

            // decide whether to show the SLOW indication
            // depends on distance and ground speed
            float gs = XPLMGetDataf(ground_speed_dr);
            slow = (distance_ > 20.0f && gs > 4.0f) || (10.0f < distance_ && distance_ <= 20.0f && gs > 3.0f) ||
                   (distance_ <= 10.0f && gs > 2.0f);

            if (distance_ <= kCrZ / 2) {
                track_ = 3;
                loop_delay = 0.03;
            } else  // azimuth only
                track_ = 2;

            // For the Marshaller sync change of straight ahead / turn commands with arm position
            if (as.dgs_type_ == kMarshaller) {
                // catch the phase ~180° point -> the Marshaller's arm is straight
                float sin_wave = XPLMGetDataf(sin_wave_dr);
                bool phase180 = (sin_wave_prev_ > 0.0) && (sin_wave <= 0.0);
                sin_wave_prev_ = sin_wave;

                if (!phase180) {
                    lr_ = lr_prev;

                    if (track_ == 3 && track_prev == 2) {
                        track_ = track_prev;
                        distance_ = distance_prev;
                    }
                }
            }
        } break;

        case GOOD: {
            // @stop position
            status_ = 2;
            lr_ = 3;

            int parkbrake_set = (XPLMGetDataf(parkbrake_dr) > 0.5);
            if (!locgood)
                new_state = TRACK;
            else if (parkbrake_set || !beacon_on)
                new_state = PARKED;
        } break;

        case BAD:
            if (!beacon_on && (now > timestamp_ + 5.0)) {
                ResetState(INACTIVE);
                return loop_delay;
            }

            if (nw_z >= kGoodZ_m)  // moving backwards
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
            if (!beacon_on) {
                new_state = DONE;
                if (operation_mode == MODE_AUTO && !plane.dont_connect_jetway) {
                    XPLMCommandOnce(toggle_jetway_cmdr);
                    // check whether it's a ToLiss, then set chocks
                    XPLMDataRef tls_chocks = XPLMFindDataRef("AirbusFBW/Chocks");
                    if (tls_chocks) {
                        XPLMSetDatai(tls_chocks, 1);
                        if (as.dgs_type() == kVDGS)
                            new_state = CHOCKS;
                    }
                }
            }
            break;

        case CHOCKS:
            status_ = 6;
            if (now > timestamp_ + 7.0)
                new_state = DONE;
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

    if (state_ > ARRIVAL) {
        // don't flood the log
        if (now > update_dgs_log_ts_ + 2.0) {
            update_dgs_log_ts_ = now;
            LogMsg("stand: %s, state: %s, status: %d, track: %d, lr: %d, distance: %0.2f, xtrack: %0.1f m",
                   as.name().c_str(), state_str[state_], status_, track_, lr_, distance_, xtrack);
        }

        // xform drefs into required constraints for the OBJs
        if (track_ == 0 || track_ == 1) {
            distance_ = 0;
            xtrack = 0.0;
        }

        distance_ = std::clamp(distance_, kGoodZ_m, kCrZ);

        if (as.dgs_type_ == kMarshaller) {
            if (marshaller == nullptr)
                marshaller = std::make_unique<Marshaller>();

            marshaller->SetPos(&as.drawinfo_, status_, track_, lr_, distance_);
        } else
            as.SetState(status_, track_, lr_, xtrack, distance_, slow);
    }

    return loop_delay;
}
