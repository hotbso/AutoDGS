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

#include <cmath>
#include <string>
#include <memory>
#include <numbers>
#include <vector>
#include <unordered_map>

#define XPLM200
#define XPLM210
#define XPLM300
#define XPLM301

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMInstance.h"

static constexpr float kD2R = std::numbers::pi/180.0;
static constexpr float kF2M = 0.3048;               // 1 ft [m]
static constexpr float kJw2Stand = 25.0;            // m, max dist jw to stand

// DGS types per stand
static constexpr int kMarshaller = 0;
static constexpr int kVDGS = 1;
static constexpr int kAutomatic = 2;

typedef enum
{
    MODE_AUTO,
    MODE_MANUAL
} opmode_t;

extern const char * const opmode_str[];

enum _DGS_DREF {
    DGS_DR_STATUS,
    DGS_DR_LR,
    DGS_DR_TRACK,
    DGS_DR_AZIMUTH,
    DGS_DR_DISTANCE,
    DGS_DR_DISTANCE_0,      // if distance < 10: full meters digit
    DGS_DR_DISTANCE_01,     // first decimal digit
    DGS_DR_ICAO_0,
    DGS_DR_ICAO_1,
    DGS_DR_ICAO_2,
    DGS_DR_ICAO_3,
    DGS_DR_R1C0,            // top row (=1), char #
    DGS_DR_R1C1,
    DGS_DR_R1C2,
    DGS_DR_R1C3,
    DGS_DR_R1C4,
    DGS_DR_R1C5,
    DGS_DR_BOARDING,        // boarding state 0/1
    DGS_DR_PAXNO_0,         // 3 digits
    DGS_DR_PAXNO_1,
    DGS_DR_PAXNO_2,
    DGS_DR_NUM              // # of drefs
};

static constexpr int kR1Nchar = 6;

extern const char *dgs_dlist_dr[];

// The airport database as loaded at plugin start and then stays unmodified.
// Pointers to AptStand and AptAirport therefore never become dangling.
// Hence we use raw pointers here.
struct AptStand {
	std::string name;
	double lon, lat;
	float hdgt;
    bool has_jw{false};
};

class AptAirport {
  public:
    static bool CollectAirports(const std::string& xp_dir);
    static const AptAirport *LookupAirport(const std::string& airport_id);
    std::string icao_;

    bool has_app_dep_{false};
    bool has_twr_{false};
    bool ignore_{false};		// e.g. sam or no_autodgs marker present
    std::vector<AptStand> stands_;
    AptAirport(const std::string& name) : icao_(name) {}
    void dump() const;
};

extern bool error_disabled;

extern std::string xp_dir;
extern std::string base_dir; // base directory of AutoDGS
extern std::string sys_cfg_dir;
extern std::string user_cfg_dir;

extern XPLMDataRef plane_x_dr, plane_y_dr, plane_z_dr, is_helicopter_dr, y_agl_dr;
extern XPLMDataRef plane_lat_dr, plane_lon_dr, plane_elevation_dr, plane_true_psi_dr;
extern XPLMDataRef gear_fnrml_dr, acf_cg_y_dr, acf_cg_z_dr, gear_z_dr;
extern XPLMDataRef beacon_dr, parkbrake_dr, acf_icao_dr, total_running_time_sec_dr;
extern XPLMDataRef percent_lights_dr, xp_version_dr, eng_running_dr, sin_wave_dr;
extern XPLMDataRef vr_enabled_dr, ground_speed_dr;
extern XPLMCommandRef cycle_dgs_cmdr, move_dgs_closer_cmdr, activate_cmdr,
    toggle_ui_cmdr, toggle_jetway_cmdr;
extern XPLMProbeRef probe_ref;
extern XPLMObjectRef dgs_obj[2];

extern opmode_t operation_mode;
extern float now;           // current timestamp
extern int on_ground;

extern void LogMsg(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
extern void create_api_drefs();
extern void ToggleUI(void);
extern void UpdateUI(bool only_if_visible = true);
extern void Activate(void);
