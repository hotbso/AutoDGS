//
//    AutoDGS: Show Marshaller or VDGS at default airports
//
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

#include "simbrief.h"

#include <cassert>
#include <ctime>

#ifndef XPLM210
#error "need at least XPLM210"
#endif

#include "XPLMDataAccess.h"

#include "log_msg.h"

static bool drefs_loaded, sbh_unavail;

#define DEF_DR(f) static XPLMDataRef f ## _dr;
DEF_DR(icao_airline);
DEF_DR(flight_number);
DEF_DR(aircraft_icao);
DEF_DR(destination);
DEF_DR(pax_count);
DEF_DR(est_out);
DEF_DR(est_off);
DEF_DR(est_on);
DEF_DR(est_in);
#undef DEF_DREF_DR

static XPLMDataRef seqno_dr, stale_dr;

// fetch byte data into a string
static void
FetchDref(std::string& str, XPLMDataRef dr)
{
    auto n = XPLMGetDatab(dr, nullptr, 0, 0);
    if (n == 0)
        return;
    str.resize(n);
    auto n1 = XPLMGetDatab(dr, (void *)str.data(), 0, n);
    assert(n == n1);
}

#define FIND_DREF(f)  f ## _dr = XPLMFindDataRef("sbh/" #f)
#define GET_DREF(f) FetchDref(ofp->f, f ## _dr)
#define LOG_DREF(f) LogMsg(" " #f ": '%s'", ofp->f.c_str())

std::unique_ptr<Ofp>
Ofp::LoadIfNewer(int cur_seqno)
{
    if (sbh_unavail)
        return nullptr;

    if (!drefs_loaded) {
        stale_dr = XPLMFindDataRef("sbh/stale");
        if (stale_dr == nullptr) {
            sbh_unavail = true;
            LogMsg("simbrief_hub plugin is not loaded, bye!");
            return nullptr;
        }

        seqno_dr = XPLMFindDataRef("sbh/seqno");

        FIND_DREF(icao_airline);
        FIND_DREF(flight_number);
        FIND_DREF(aircraft_icao);
        FIND_DREF(destination);
        FIND_DREF(pax_count);
        FIND_DREF(est_out);
        FIND_DREF(est_off);
        FIND_DREF(est_on);
        FIND_DREF(est_in);
    }

    int seqno = XPLMGetDatai(seqno_dr);
    if (seqno <= cur_seqno)
        return nullptr;

    int stale = XPLMGetDatai(stale_dr);
    if (stale)
        LogMsg("simbrief_hub data may be stale");

    auto ofp = std::make_unique<Ofp>();

    ofp->seqno = seqno;
    GET_DREF(icao_airline);
    GET_DREF(flight_number);
    GET_DREF(aircraft_icao);
    GET_DREF(destination);
    GET_DREF(pax_count);
    GET_DREF(est_out);
    GET_DREF(est_off);
    GET_DREF(est_on);
    GET_DREF(est_in);

    LogMsg("From simbrief_hub: Seqno: %d", seqno);
    LOG_DREF(icao_airline);
    LOG_DREF(flight_number);
    LOG_DREF(aircraft_icao);
    LOG_DREF(destination);
    LOG_DREF(pax_count);
    LOG_DREF(est_out);
    LOG_DREF(est_off);
    LOG_DREF(est_on);
    LOG_DREF(est_in);

    return ofp;
}

const std::string
Ofp::GenDepartureStr() const
{
    std::string str;
    str = icao_airline + flight_number + " " + aircraft_icao + " TO " +
        destination;

    time_t out_time = atol(est_out.c_str());
    time_t off_time = atol(est_off.c_str());

    auto out_tm = *std::gmtime(&out_time);
    auto off_tm = *std::gmtime(&off_time);
    char out[20], off[20];
    strftime(out, sizeof(out), " OUT %H:%M", &out_tm);
    strftime(off, sizeof(off), " OFF %H:%M", &off_tm);
    str.append(out);
    str.append(off);
    return str;
}
