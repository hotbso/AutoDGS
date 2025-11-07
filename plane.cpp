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

#include <fstream>
#include "autodgs.h"
#include "plane.h"

#include "XPLMPlanes.h"

Plane plane;

bool Plane::EnginesOn() {
    int er[8];
    int n = XPLMGetDatavi(eng_running_dr, er, 0, 8);
    for (int i = 0; i < n; i++)
        if (er[i])
            return true;

    return false;
}

void Plane::ResetBeacon() {
    beacon_state_ = beacon_last_pos_ = XPLMGetDatai(beacon_dr);
    beacon_on_ts_ = beacon_off_ts_ = -10.0;
}

bool Plane::BeaconOn(void) {
    if (use_engine_running_)
        return EnginesOn();

    // when checking the beacon guard against power transients when switching
    // to the APU generator (e.g. for the ToLiss fleet).
    // Report only state transitions if the new (off) state persisted for 3 seconds

    int beacon = XPLMGetDatai(beacon_dr);
    if (beacon) {
        if (!beacon_last_pos_) {
            beacon_on_ts_ = now;
            beacon_last_pos_ = 1;
        } else if (now > beacon_on_ts_ + 0.5)
            beacon_state_ = 1;
    } else {
        if (beacon_last_pos_) {
            beacon_off_ts_ = now;
            beacon_last_pos_ = 0;
        } else if (now > beacon_off_ts_ + 3.0)
            beacon_state_ = 0;
    }

    return beacon_state_;
}

int Plane::PaxNo() {
    if (pax_no_dr_ == NULL)
        return -1;
    return XPLMGetDataf(pax_no_dr_) + 0.5f;  // round upwards
}

static bool FindIcaoInFile(const std::string& acf_icao, const std::string& fn) {
    std::ifstream f(fn);
    if (!f.is_open()) {
        LogMsg("Can't open '%s'", fn.c_str());
        return false;
    }

    LogMsg("check whether acf '%s' is in exception file %s", acf_icao.c_str(), fn.c_str());
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() > 0 && line.back() == '\r')  // just in case
            line.pop_back();

        if (line == acf_icao) {
            LogMsg("found acf %s in %s", acf_icao.c_str(), fn.c_str());
            return true;
        }
    }

    return false;
}

void Plane::PlaneLoadedCb() {
    char buffer[41]{};
    XPLMGetDatab(acf_icao_dr, buffer, 0, 40);

    for (int i = 0; i < 4; i++)
        buffer[i] = (isupper(buffer[i]) || isdigit(buffer[i])) ? buffer[i] : ' ';
    buffer[4] = '\0';
    acf_icao = buffer;

    // the VDGS object does not like letters in the last position
    if (acf_icao == "A20N")
        acf_icao = "A320";
    else if (acf_icao == "A21N")
        acf_icao = "A321";

    float plane_cg_z = kF2M * XPLMGetDataf(acf_cg_z_dr);

    float gear_z[2];
    if (2 == XPLMGetDatavf(gear_z_dr, gear_z, 0, 2)) {  // nose + main wheel
        nw_z = -gear_z[0];
        mw_z = -gear_z[1];
    } else
        nw_z = mw_z = plane_cg_z;  // fall back to CG

    is_helicopter = XPLMGetDatai(is_helicopter_dr);

    pe_y_0_valid = false;
    pe_y_0 = 0.0;

    if (!is_helicopter) {
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
                    if (1 == sscanf(line.c_str() + 16, "%f", &pe_y_0)) {
                        pe_y_0 -= XPLMGetDataf(acf_cg_y_dr);
                        pe_y_0 *= kF2M;
                        pe_y_0_valid = true;
                    }
                    break;
                }
            }
        }
    }

    // check whether acf is listed in exception files
    use_engine_running_ = FindIcaoInFile(acf_icao, base_dir + "acf_use_engine_running.txt");
    dont_connect_jetway = FindIcaoInFile(acf_icao, base_dir + "acf_dont_connect_jetway.txt");

    pax_no_dr_ = XPLMFindDataRef("AirbusFBW/NoPax");  // currently only ToLiss
    if (pax_no_dr_) {
        LogMsg("ToLiss detected");
        int pax_no = PaxNo();
        if (pax_no > 0)  // warn on common user error
            LogMsg("WARNING: plane is already boarded with initial # of pax: %d", pax_no);
    }

    LogMsg(
        "plane loaded: %s, plane_cg_z: %1.2f, nw_z: %1.2f, mw_z: %1.2f, "
        "pe_y_0_valid: %d, pe_y_0: %0.2f, is_helicopter: %d",
        acf_icao.c_str(), plane_cg_z, nw_z, mw_z, pe_y_0_valid, pe_y_0, is_helicopter);
}
