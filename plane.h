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

#ifndef _PLANE_H_
#define _PLANE_H_

// a container of flags
// and a BeaconState debounced of power transitions

class Plane {
    int beacon_state_, beacon_last_pos_;   // beacon state, last switch_pos, ts of last switch actions
    float beacon_off_ts_, beacon_on_ts_;

  public:
    std::string acf_icao;
    bool use_engine_running;            // instead of beacon, e.g. MD11
    bool dont_connect_jetway;           // e.g. for ZIBO with own ground service
    float nw_z, mw_z;                   // z value of plane's 0 to nose wheel, main wheel

    float pe_y_0;                       // pilot eye y to plane's 0 point
    bool pe_y_0_valid;

    bool is_helicopter;

    void PlaneLoadedCb();               // callback for XPLM_MSG_PLANE_LOADED
    bool BeaconState();                 // debounced state
    void ResetBeacon();                 // e.g. after a teleportation
};

extern Plane plane;
#endif