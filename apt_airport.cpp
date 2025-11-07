//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
//    Copyright (C) 2025  Holger Teutsch
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

#include <cstring>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <unordered_map>
#include <algorithm>

#include "autodgs.h"

namespace fem = flat_earth_math;

struct SceneryPacks {
    std::vector<std::string> sc_paths;
    SceneryPacks(const std::string& xp_dir);
};

struct Jetway {
	fem::LLPos pos;
	float hdgt;
	float length;
    fem::LLPos cabin;    // = pos + length * dir(hdgt)
};

std::unordered_map<std::string, AptAirport*> apt_airports;

// SceneryPacks constructor
SceneryPacks::SceneryPacks(const std::string& xp_dir)
{
    std::string scpi_name(xp_dir + "/Custom Scenery/scenery_packs.ini");

    std::ifstream scpi(scpi_name);
    if (scpi.fail()) {
        LogMsg("Can't open '%s'", scpi_name.c_str());
        return;
    }

    sc_paths.reserve(500);
    std::string line;

    while (std::getline(scpi, line)) {
        size_t i;
        if ((i = line.find('\r')) != std::string::npos)
            line.resize(i);

        if (!line.starts_with("SCENERY_PACK ")
            || line.find("*GLOBAL_AIRPORTS*") != std::string::npos                  // XP12
            || line.find("Custom Scenery/Global Airports/") != std::string::npos)   // XP11
            continue;

        // autoortho pretends every file exists but
        // reads give errors
        if (line.find("/z_ao_") != std::string::npos)
            continue;

        line.erase(0, 13);
        std::string sc_path;
        bool is_absolute = (line[0] == '/' || line.find(':') != std::string::npos);
        if (is_absolute)
            sc_path = line;
        else
            sc_path = xp_dir + line;

        // posixify
        for (unsigned i = 0; i < sc_path.size(); i++)
            if (sc_path[i] == '\\')
                sc_path[i] = '/';

        sc_paths.push_back(sc_path);
    }

    scpi.close();
    sc_paths.shrink_to_fit();
}

static bool
operator<(const AptStand& a, const AptStand& b)
{
    return a.name < b.name;
}

void
AptAirport::dump() const
{
    LogMsg("Dump of airport: %s", icao_.c_str());

    for (auto const & s : stands_)
        LogMsg("'%s', %0.6f, %0.6f, %0.6f, has_jw: %d", s.name.c_str(), s.lat, s.lon, s.hdgt, s.has_jw);

#if 0
    for (auto & jw : jetways_)
         LogMsg("%0.8f, %0.8f, %0.2f, %0.1f, end: %0.8f,%0.8f",
                jw.pos.lat, jw.pos.lon, jw.hdgt, jw.length, jw.cabin.lat, jw.cabin.lon);
#endif

    for (auto const & rwy : rwys_) {
        LogMsg("Runway: '%s', %0.8f, %0.8f, %0.8f, %0.8f, len: %0.1f, width: %0.1f",
               rwy.name.c_str(), rwy.end1.lat, rwy.end1.lon, rwy.end2.lat, rwy.end2.lon,
               rwy.len, rwy.width);
    }
}

void AptAirport::ComputeBBox() {
    static constexpr double kDlat = 150.0 / fem::kLat2m;    // 150 m grace distance

    bbox_max_ = {-1000.0, -1000.0};
    bbox_min_ = {+1000.0, +1000.0};

    for (const auto& s : stands_) {
        const double dlon = kDlat * cosf(s.lat * kD2R);
        bbox_min_.lon = std::min(bbox_min_.lon, fem::RA(s.lon - dlon));
        bbox_max_.lon = std::max(bbox_max_.lon, fem::RA(s.lon + dlon));
        bbox_min_.lat = std::min(bbox_min_.lat, s.lat - kDlat);
        bbox_max_.lat = std::max(bbox_max_.lat, s.lat + kDlat);
    }

    for (const auto& r : rwys_) {
        const double dlon = kDlat * cosf(r.end1.lat * kD2R);
        bbox_min_.lon = std::min(bbox_min_.lon, fem::RA(r.end1.lon - dlon));
        bbox_max_.lon = std::max(bbox_max_.lon, fem::RA(r.end1.lon + dlon));
        bbox_min_.lat = std::min(bbox_min_.lat, r.end1.lat - kDlat);
        bbox_max_.lat = std::max(bbox_max_.lat, r.end1.lat + kDlat);

        bbox_min_.lon = std::min(bbox_min_.lon, fem::RA(r.end2.lon - dlon));
        bbox_max_.lon = std::max(bbox_max_.lon, fem::RA(r.end2.lon + dlon));
        bbox_min_.lat = std::min(bbox_min_.lat, r.end2.lat - kDlat);
        bbox_max_.lat = std::max(bbox_max_.lat, r.end2.lat + kDlat);
    }

    //LogMsg("BBox for airport %s: min: %0.8f,%0.8f, max: %0.8f,%0.8f",
    //       icao_.c_str(), bbox_min_.lat, bbox_min_.lon, bbox_max_.lat, bbox_max_.lon);
}

static int n_stands;

// go through apt.dat and collect stands
static bool
ParseAptDat(const std::string& fn, bool ignore)
{
    std::ifstream apt(fn);
    if (apt.fail())
        return false;

    LogMsg("Processing '%s'", fn.c_str());
    std::string line;
    line.reserve(2000);          // can be quite long

    AptAirport *arpt = nullptr;
    std::string arpt_name;
	std::vector<Jetway> jetways;

    // save arpt if it has a tower frequency and stands
    auto save_arpt = [&] () {
        if (arpt == nullptr)
            return;

        // LogMsg("Save ---> '%s', %d, %d", arpt->icao_.c_str(), arpt->has_twr_, (int)arpt->stands_.size());
        if (arpt->has_twr_ && arpt->stands_.size() > 0) {
            for (auto & s : arpt->stands_)
                for (auto & jw : jetways)
                    if (fem::len(jw.cabin - fem::LLPos{s.lon, s.lat}) < kJw2Stand) {
                        s.has_jw = true;
                        break;
                    }

            n_stands += arpt->stands_.size();
            arpt->stands_.shrink_to_fit();
            std::sort(arpt->stands_.begin(), arpt->stands_.end());
            apt_airports[arpt->icao_] = arpt;
            jetways.clear();
            arpt->ComputeBBox(); // compute bounding box for this airport
        } else
            delete(arpt);

        arpt = nullptr;
        arpt_name.clear();
    };

    while (std::getline(apt, line)) {
        // ignore helipads + seaplane bases
        //17      0 0 0 EKAR [H] South Arne Helideck
        if (line.starts_with("17 ") || line.starts_with("16 ")) {
            save_arpt();
            continue;
        }

        //1    681 0 0 ENGM Oslo Gardermoen
        if (line.starts_with("1 ")) {
			//LogMsg("%s", line.c_str());
            save_arpt();

            if (line.back() == '\r')
                line.pop_back();

            int ofs;
			sscanf(line.c_str(), "%*d %*d %*d %*d %n", &ofs);
			if (ofs < (int)line.size()) {
                size_t bpos = std::min(line.find(' ', ofs), line.size());
                int len = bpos - ofs;
                arpt_name = line.substr(ofs, len);
            } else {
                arpt_name.clear();
                LogMsg("could not locate airport id '%s'", line.c_str());
            }

			continue;
		}

        if (arpt_name.empty())
            continue;

        // after 1 comes the 1302 block

        // 1302 icao_code ENRM
        if (line.starts_with("1302 icao_code ")) {
            arpt_name = line.substr(15, 4);
            continue;
        }

        if (line.starts_with("1302"))   // ignore
            continue;

        if (arpt == nullptr) {          // after leaving 1302 block ...
            if (arpt_name.length() > 4 || arpt_name.find_first_of("0123456789") != std::string::npos) {
                arpt_name.clear();
                continue;   // can't be an icao airport
            }

            try {
                apt_airports.at(arpt_name);
            } catch(const std::out_of_range& ex) {
                // does not yet exist
                arpt = new AptAirport(arpt_name);
                if (ignore) {
                    // LogMsg("Saving '%s' with ignore", arpt->icao_.c_str());
                    arpt->ignore_ = true;
                    apt_airports[arpt->icao_] = arpt;
                    arpt = nullptr;
                    arpt_name.clear();
                } else
                    arpt->stands_.reserve(50);
           }
        }

        if (arpt == nullptr)
            continue;

        if (line.starts_with("1054 ") || line.starts_with("54 ")) {
            arpt->has_twr_ = true;
            continue;
        }

        // stand
		//1300 50.030069 8.557858 159.4 tie_down jets|turboprops|props S403
        if (line.starts_with("1300 ")) {
            if (line.back() == '\r')
                line.pop_back();

            AptStand st;
            int ofs;
			sscanf(line.c_str(), "%*d %lf %lf %f %*s %*s %n", &st.lat, &st.lon, &st.hdgt, &ofs);
			if (ofs < (int)line.size())
                st.name = line.substr(ofs, line.size() - ofs);
            arpt->stands_.push_back(st);
            continue;
		}

        // jetway
        // 1500 60.3161845 24.9597493 234.4 2 1 234.4 16.17 253.2
        if (line.starts_with("1500 ")) {
            Jetway jw;
			sscanf(line.c_str(), "%*d %lf %lf %f %*d %*d %*f %f", &jw.pos.lat, &jw.pos.lon, &jw.hdgt, &jw.length);
            fem::Vec2 dir{cosf((90.0f - jw.hdgt) * kD2R), sinf((90.0f - jw.hdgt) * kD2R)};
            jw.cabin = jw.pos + jw.length * dir;
            jetways.push_back(jw);
            continue;
        }

        // runway
        // 100 45.11 1 0 0.25 0 2 0  17 -15.64371363 -056.12159961 0 55 3 0 0 0 35 -15.66223638 -056.11174395 0 62 3 0 0 0
        if (line.starts_with("100 ")) {
            AptRunway rwy;
            char name1[10], name2[10];
			int n = sscanf(line.c_str(), "%*d %f %*d %*d %*f %*d %*d %*d %9s %lf %lf %*f %*f %*d %*d %*d %*d %9s %lf %lf",
                           &rwy.width, name1, &rwy.end1.lat, &rwy.end1.lon, name2, &rwy.end2.lat, &rwy.end2.lon);
            if (n == 7) {
                rwy.name = std::string(name1) + "/" + std::string(name2);
                rwy.cl = rwy.end2 - rwy.end1;  // center line vector
                rwy.len = fem::len(rwy.cl);
                if (rwy.len < 1.0) {
                    LogMsg("Runway '%s' too short: %0.1f", rwy.name.c_str(), rwy.len);
                    continue;
                }
                rwy.cl = (1/rwy.len) * rwy.cl; // normalize
                arpt->rwys_.push_back(rwy);
            }
            continue;
        }


    }

    save_arpt();
    apt.close();
    return true;
}

bool
AptAirport::CollectAirports(const std::string& xp_dir)
{
    const std::clock_t c_start = std::clock();
    auto t_start = std::chrono::high_resolution_clock::now();

    SceneryPacks scp(xp_dir);
    if (scp.sc_paths.size() == 0) {
        LogMsg("Can't collect scenery_packs.ini");
        return false;
    }

    apt_airports.reserve(5000);
    n_stands = 0;

	for (auto & path : scp.sc_paths) {
        bool ignore = (std::filesystem::exists(path + "no_autodgs") || std::filesystem::exists(path + "no_autodgs.txt"));
        if (std::filesystem::exists(path + "sam.xml")
            && !(std::filesystem::exists(path + "use_autodgs") || std::filesystem::exists(path + "use_autodgs.txt")))
            ignore = true;

        // don't check return code here, maybe meshes etc...
        ParseAptDat(path + "Earth nav data/apt.dat", ignore);
	}

    if (!(ParseAptDat(xp_dir + "Global Scenery/Global Airports/Earth nav data/apt.dat", false)      // XP12
          || ParseAptDat(xp_dir + "Custom Scenery/Global Airports/Earth nav data/apt.dat", false))) // XP11
        return false;

    const std::clock_t c_end = std::clock();
    auto t_end = std::chrono::high_resolution_clock::now();

    LogMsg("CollectAirports: # of airports: %d, # of stands: %d, CPU: %1.3fs, elapsed: %1.3fs",
           (int)apt_airports.size(), n_stands, (double)(c_end - c_start) / CLOCKS_PER_SEC,
           std::chrono::duration<double>(t_end - t_start).count());

    return true;
}

const AptAirport* AptAirport::LookupAirport(const std::string& airport_id) {
    const AptAirport* arpt = nullptr;
    auto it = apt_airports.find(airport_id);
    if (it != apt_airports.end()) {
        arpt = it->second;
        if (arpt->ignore_)
            arpt = nullptr;
    }

    if (arpt == nullptr)
        LogMsg("sorry, '%s' is not an AutoDGS airport", airport_id.c_str());

    return arpt;
}

// Locate airport from position -> id
const std::string AptAirport::LocateAirport(const fem::LLPos& pos) {

    for (const auto& [n, a] : apt_airports) {
        if (a->ignore_)
            continue;
#if 0
        // keep in case we want to use runways
        // check if we are on a runway
        for (const auto& r : a->rwys_) {
            fem::Vec2 pos_end1 = pos - r.end1;
            auto proj = pos_end1 * r.cl;
            //LogMsg("proj: %f, runway: %s", proj, r.name.c_str());
            if (proj < 0.0 || proj > r.len)
                continue;   // not on runway

            float dist = fem::len(pos_end1 - proj * r.cl);
            if (dist < 0.6f * r.width) {   // be gracious with width
                LogMsg("Found runway '%s' '%s' at %0.8f,%0.8f", n.c_str(), r.name.c_str(), pos.lat, pos.lon);
                return n;   // found runway, so this is the airport
            }
        }
#endif
        if (fem::InRect(pos, a->bbox_min_, a->bbox_max_)) {
            LogMsg("Found airport '%s' at %0.8f,%0.8f", n.c_str(), pos.lat, pos.lon);
            return n;   // found airport, so return id
        }
    }

    LogMsg("sorry, %0.8f,%0.8f is not on an AutoDGS airport", pos.lat, pos.lon);
    return ""; // not found
}
