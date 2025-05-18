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

#ifndef _AIRPORT_H_
#define _AIRPORT_H_

#include <memory>
#include <tuple>

// AptStand augmented
class Stand {
	const AptStand& as_;
    std::string display_name_;   // for use in the VDGS

  protected:
    friend class Airport;

    float x_, y_, z_;
    float sin_hdgt_, cos_hdgt_;
    int dgs_type_;
    bool is_wet_;
    XPLMDrawInfo_t drawinfo_;
    XPLMInstanceRef vdgs_inst_ref_;
    float dgs_dist_;            // distance to dgs
    float marshaller_max_dist_; // max distance, actual can be lower according to PE
    void SetDgsDist();

  public:
    Stand(Stand&&) = default;
    Stand& operator=(Stand&&) = delete;

    Stand(const AptStand& as, float elevation, int dgs_type, float dist_adjust);
    ~Stand();

    void SetDgsType(int dgs_type);
    void CycleDgsType();
    void DgsMoveCloser();           // with wrap around
    void SetState(int status, int track, int lr, float azimuth,
                  float distance, float distance_0, float distance_01);
    void SetIdle();

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
        INACTIVE = 0, ACTIVE, ENGAGED, TRACK, GOOD, BAD, PARKED, CHOCKS, DONE
    } state_t;

    static const char * const state_str[];

  private:
    std::string name_;
    state_t state_;

    std::vector<Stand> stands_;
    int active_stand_;      // -1 or index into stands_
    int selected_stand_;

    bool user_cfg_changed_;

    // values that must survive a single run of the state_machine
    int status_, track_, lr_;
    float timestamp_, distance_, sin_wave_prev_;
    float nearest_stand_ts_, update_dgs_log_ts_;

    void FindNearestStand();
    void FlushUserCfg();

  public:
    static std::unique_ptr<Airport> LoadAirport(const std::string& icao);

    Airport() = delete;
    Airport(const AptAirport&);
    ~Airport();

    int nstands() const { return stands_.size(); }
    std::tuple<int, const std::string> GetStand(int idx) const;  // dgs_type, name

    void ResetState(state_t new_state);
    void SetSelectedStand(int selected_stand);

    // these act onto the selected of active stand
    void DgsMoveCloser();
    void SetDgsType(int dgs_type);
    int GetDgsType() const;
    void CycleDgsType();

    float StateMachine();

    // accessors
    const std::string& name() const { return name_; }
    state_t state() const { return state_; }
};

extern std::unique_ptr<Airport> arpt;

#endif