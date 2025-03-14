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

static constexpr int kMarshaller = 0;
static constexpr int kVDGS = 1;
static constexpr int kAutomatic = 2;

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
    int idx_;               // index into stands_ vector

    double x_, y_, z_;
    float sin_hdgt_, cos_hdgt_;
    int dgs_type_;
    XPLMDrawInfo_t drawinfo_;
    XPLMInstanceRef vdgs_inst_ref_;
    float dgs_dist_;

  public:
    Stand(Stand&&) = default;
    Stand& operator=(Stand&&) = delete;

    Stand(int idx, const AptStand& as, float elevation);
    ~Stand();

    void SetDgsType(int dgs_type);
    void SetDgsDist(float adjust = 0.0f);
    void SetState(int status, int track, int lr, float azimuth, float distance,
                  bool state_track, float brightness);
    void Deactivate();

    // accessors
    const std::string& name() const { return as_.name; };
    const char *cname() const { return as_.name.c_str(); };
    bool has_jw() const { return as_.has_jw; }
    float hdgt() const { return as_.hdgt; }
    double lat() const { return as_.lat; }
    double lon() const { return as_.lon; }
    int dgs_type() const { return dgs_type_; }
};

// AptAirport augmented
class Airport {
  public:
    typedef enum {
        INACTIVE = 0, ACTIVE, ENGAGED, TRACK, GOOD, BAD, PARKED, DONE
    } state_t;

    static const char * const state_str[];

  private:
    state_t state_;
    const AptAirport *apt_airport_;
    std::vector<Stand> stands_;
    Stand *active_stand_;
    int selected_stand_;

    // values that must survive a single run of the state_machine
    int status_, track_, lr_;
    float timestamp_, distance_, sin_wave_prev_;
    float nearest_stand_ts_, update_dgs_log_ts_;

    void FindNearestStand();

    friend void UpdateUI(bool only_if_visible); // TODO

  public:
    static std::unique_ptr<Airport> LoadAirport(const std::string& icao);

    Airport() = delete;
    Airport(const AptAirport*);
    ~Airport();

    void ResetState(state_t new_state);

    void SetSelectedStand(int selected_stand);

    // these act onto the selected of active stand
    void SetDgsDistAdjust(float adjust);
    void SetDgsType(int dgs_type);
    void CycleDgsType();

    float StateMachine();

    // accessors
    const std::string& name() const { return apt_airport_->icao_; }
    state_t state() const { return state_; }
};

extern std::string xp_dir;
extern std::string base_dir; // base directory of AutoDGS

extern std::unique_ptr<Airport> arpt;

extern XPLMDataRef vr_enabled_dr;

extern opmode_t operation_mode;
extern int on_ground;

extern void LogMsg(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
extern void create_api_drefs();
extern void ToggleUI(void);
extern void UpdateUI(bool only_if_visible = true);
