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

// AptStand augmented
class Stand {
	const AptStand& as_;

  protected:
    friend class Airport;

    double x_, y_, z_;
    float sin_hdgt_, cos_hdgt_;
    XPLMDrawInfo_t drawinfo_;

  public:
    Stand(Stand&&) = default;
    Stand& operator=(Stand&&) = delete;

    Stand(const AptStand& as, float elevation);
    const std::string& name() const { return as_.name; };
    const char *cname() const { return as_.name.c_str(); };
    bool has_jw() const { return as_.has_jw; }
    float hdgt() const { return as_.hdgt; }
    double lat() const { return as_.lat; }
    double lon() const { return as_.lon; }

    void SetDgsPos(void);
};

// AptAirport augmented
class Airport {
    const AptAirport *apt_airport_;
    std::vector<Stand> stands_;
    Stand *active_stand_ = nullptr;

    void FindNearestStand();

    friend void update_ui(int only_if_visible); // TODO

  public:

    Airport() = delete;
    Airport(const AptAirport*);
    ~Airport();
    static std::unique_ptr<Airport> LoadAirport(const std::string& icao);

    const std::string& name() const { return apt_airport_->icao_; }

    void ResetState(state_t new_state);
    void SetDgsPos() { if (active_stand_) active_stand_->SetDgsPos(); } ;
    void SetDgsTypeAuto();
    float StateMachine();
};

extern std::string xp_dir;
extern std::string base_dir; // base directory of AutoDGS

extern std::unique_ptr<Airport> arpt;

extern XPLMCommandRef cycle_dgs_cmdr;
extern XPLMDataRef vr_enabled_dr;

extern opmode_t operation_mode;
extern state_t state;
extern int on_ground;

extern int dgs_type;
extern bool dgs_type_auto;

extern void LogMsg(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
extern void create_api_drefs();
extern void toggle_ui(void);
extern void update_ui(int only_if_visible);

extern void SetSelectedStand(const std::string& ui_selected_stand);
extern void SetDgsType(int new_dgs_type);
