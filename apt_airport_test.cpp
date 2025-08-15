//
//    AutoDGS: Show Marshaller or VDGS at default apt_airports
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

#include "autodgs.h"

namespace fem = flat_earth_math;

const char* log_msg_prefix = "apt_airport: ";
const AptAirport* arpt;
extern std::unordered_map<std::string, AptAirport*> apt_airports;

[[maybe_unused]] static void find_and_dump(const std::string& name) {
    arpt = AptAirport::LookupAirport(name);
    if (arpt)
        arpt->dump();
}

static void LocateAndDump(const fem::LLPos& pos) {
    std::string id = AptAirport::LocateAirport(pos);
    if (!id.empty()) {
        arpt = AptAirport::LookupAirport(id);
        if (arpt)
            arpt->dump();
    } else {
        LogMsg("No airport found at %0.8f,%0.8f", pos.lat, pos.lon);
    }
}

int main() {
    AptAirport::CollectAirports("e:/X-Plane-12-test/");

    for (auto& a : apt_airports) {
        auto const arpt = a.second;

        if (arpt->ignore_) {
            LogMsg("Ignored: %s", arpt->icao_.c_str());
            continue;
        }

        // arpt->dump();
    }

    // find_and_dump("EDDB");
    // find_and_dump("EIDW");

    LocateAndDump({-6.280610, 53.437163});    // Dublin
    LocateAndDump({-122.393487, 37.619167});  // SFO
    LocateAndDump({-73.778889, 40.641389});   // JFK

#if 0
    find_and_dump("EKBI");
    find_and_dump("EKBIx");
    find_and_dump("EDDV");
    find_and_dump("ZUTF");
#endif
}
#
