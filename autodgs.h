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

static constexpr float kD2R = std::numbers::pi/180.0;
static constexpr float kF2M = 0.3048;               // 1 ft [m]
static constexpr float kJw2Stand = 18.0;            // m, max dist jw to stand

// types
typedef enum
{
    DISABLED=0, INACTIVE, ACTIVE, ENGAGED, TRACK, GOOD, BAD, PARKED, DONE
} state_t;

extern const char * const state_str[];

typedef enum
{
    MODE_AUTO,
    MODE_MANUAL
} opmode_t;

extern const char * const opmode_str[];

struct Stand {
	std::string name;
	double lon, lat;
	float hdgt;
    bool has_jw{false};
};

class Airport {
  public:
    std::string icao_;

	bool has_app_dep_{false};
	bool has_twr_{false};
	bool ignore_{false};		// e.g. sam or no_autodgs marker present
	std::vector<Stand> stands_;
	Airport(const std::string& name) : icao_(name) {}
	void dump();
};

extern std::string xp_dir;
extern std::string base_dir; // base directory of AutoDGS

extern void LogMsg(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

extern std::unordered_map<std::string, std::shared_ptr<Airport>> airports;
extern std::shared_ptr<Airport> arpt;
extern bool CollectAirports(const std::string& xp_dir);

extern XPLMCommandRef cycle_dgs_cmdr;
extern XPLMDataRef vr_enabled_dr;

extern opmode_t operation_mode;
extern state_t state;
extern int on_ground;
extern float dgs_ramp_dist_default;
extern int dgs_ramp_dist_override;

extern int dgs_type;
extern bool dgs_type_auto;

extern const Stand *active_stand;

extern void create_api_drefs();
extern void toggle_ui(void);
extern void update_ui(int only_if_visible);

extern void set_selected_stand(const std::string& ui_selected_stand);
extern void set_dgs_type(int new_dgs_type);
extern void set_dgs_type_auto();

