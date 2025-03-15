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
#include "flat_earth_math.h"

struct SceneryPacks {
    std::vector<std::string> sc_paths;
    SceneryPacks(const std::string& xp_dir);
};

struct Jetway {
	LLPos pos;
	float hdgt;
	float length;
    LLPos cabin;    // = pos + length * dir(hdgt)
};

static std::unordered_map<std::string, AptAirport*> airports;

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

        if (!line.starts_with("SCENERY_PACK ") || line.find("*GLOBAL_AIRPORTS*") != std::string::npos)
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
	std::vector<Jetway> jetways;

    // save arpt if it has a tower frequency and stands
    auto save_arpt = [&] () {
        if (arpt && arpt->has_twr_ && arpt->stands_.size() > 0) {
            for (auto & s : arpt->stands_)
                for (auto & jw : jetways)
                    if (len(jw.cabin - LLPos(s.lon, s.lat)) < kJw2Stand) {
                        s.has_jw = true;
                        break;
                    }

            n_stands += arpt->stands_.size();
            arpt->stands_.shrink_to_fit();
            std::sort(arpt->stands_.begin(), arpt->stands_.end());
            airports[arpt->icao_] = arpt;
            jetways.resize(0);
        } else {
            delete(arpt);
            arpt = nullptr;
        }
    };

    while (std::getline(apt, line)) {
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
                if (len > 4)
                    continue;
                std::string name = line.substr(ofs, len);
                if (name.find_first_of("0123456789") != std::string::npos)
                    continue;   // can't be an icao airport

                try {
                    airports.at(name);
                } catch(const std::out_of_range& ex) {
                    // does not yet exist
                    arpt = new AptAirport(name);
                    if (ignore) {
                        arpt->ignore_ = true;
                        airports[arpt->icao_] = arpt;
                        arpt = nullptr;
                    } else
                        arpt->stands_.reserve(50);
               }
            } else
                LogMsg("could not locate airport id '%s'", line.c_str());
			continue;
		}

        if (arpt == nullptr)
            continue;

        // 1302 icao_code ENRM
        if (line.starts_with("1302 icao_code ")) {
            arpt->icao_ = line.substr(15, 4);
            continue;
        }

        // check for APP or DEP frequency
        if (line.starts_with("1055 ") || line.starts_with("1056 ") || line.starts_with("55 ") || line.starts_with("56 ")) {
            arpt->has_app_dep_ = true;
            continue;
        }

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
            Vec2 dir{cosf((90.0f - jw.hdgt) * kD2R), sinf((90.0f - jw.hdgt) * kD2R)};
            jw.cabin = jw.pos + jw.length * dir;
            jetways.push_back(jw);
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

    airports.reserve(5000);
    n_stands = 0;

	for (auto & path : scp.sc_paths) {
        bool ignore = (std::filesystem::exists(path + "no_autodgs") || std::filesystem::exists(path + "no_autodgs.txt"));
        if (std::filesystem::exists(path + "sam.xml")
            && !(std::filesystem::exists(path + "use_autodgs") || std::filesystem::exists(path + "use_autodgs.txt")))
            ignore = true;

        // don't check return code here, maybe meshes etc...
        ParseAptDat(path + "Earth nav data/apt.dat", ignore);
	}

    if (!ParseAptDat(xp_dir + "Global Scenery/Global Airports/Earth nav data/apt.dat", false))
        return false;

    const std::clock_t c_end = std::clock();
    auto t_end = std::chrono::high_resolution_clock::now();

    LogMsg("CollectAirports: # of airports: %d, # of stands: %d, CPU: %1.3fs, elapsed: %1.3fs",
           (int)airports.size(), n_stands, (double)(c_end - c_start) / CLOCKS_PER_SEC,
           std::chrono::duration<double>(t_end - t_start).count());

    return true;
}

const AptAirport *
AptAirport::LookupAirport(const std::string& airport_id)
{
    try {
        const AptAirport *arpt = airports.at(airport_id);
        if (arpt->ignore_)
            return nullptr;
        return arpt;
    } catch(const std::out_of_range& ex) {
        LogMsg("sorry, '%s' is not an AutoDGS airport", airport_id.c_str());
   }

   return nullptr;
}

#ifdef TEST_AIRPORTS
// g++ --std=c++20 -DIBM=1 -DTEST_AIRPORTS -DLOCAL_DEBUGSTRING -I../SDK/CHeaders/XPLM  -O apt_airport.cpp log_msg.cpp

const AptAirport *arpt;

static
void find_and_dump(const std::string& name)
{
    arpt = AptAirport::LookupAirport(name);
    if (arpt)
        arpt->dump();
    else
        LogMsg("sorry, '%s' is not an AutoDGS airport", name.c_str());
}

int main()
{
	bool res = AptAirport::CollectAirports("e:/X-Plane-12-test/");

    for (auto & a : airports) {
        auto const arpt = a.second;

        if (arpt->ignore_) {
            LogMsg("Ignored: %s", arpt->icao_.c_str());
            continue;
        }

        //arpt->dump();
    }

    find_and_dump("EKBI");
    find_and_dump("EKBIx");
    find_and_dump("EDDV");
    find_and_dump("EDDF");
    find_and_dump("EHAM");
}
#endif
