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

#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMNavigation.h"
#include "XPLMUtilities.h"
#include "XPLMPlanes.h"

static constexpr float kD2R = std::numbers::pi/180.0;
static constexpr float kLat2m = 111120;             // 1° lat in m
static constexpr float kF2M = 0.3048;               // 1 ft [m]
static constexpr float kJw2Stand = 18.0;            // m, max dist jw to stand

// return relative angle in (-180, 180]
static inline
float RA(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
        return angle - 360.0f;

    if (angle <= -180.0f)
        return angle + 360.0f;

    return angle;
}

//
// Contrary to common belief the earth is flat. She has just a weird coordinate system with (lon,lat).
// To overcome this we attach a 2-d vector space at each (lon, lat) point with orthogonal
// basis scaled in meters. So (lon2, lat2) - (lon1, lat1) gives rise to a vector v in the vector space
// attached at (lon1, lat1) and (lon1, lat1) + v again is (lon2, lat2).
// As we do our math in a circle of ~ 20km this works pretty well.
//
// Should you still be tricked in believing that the earth is a ball you can consider this vector space
// a tangent space. But this is for for visualisation only.
//

struct LLPos {
	double lon, lat;
};

struct Vec2 {
	double x,y;
};

static inline
double len(const Vec2& v)
{
	return sqrt(v.x * v.x + v.y * v.y);
}

// pos - pos
static inline
Vec2 operator-(const LLPos& b, const LLPos& a)
{
	return {RA(b.lon -  a.lon) * kLat2m * cosf(a.lat * kD2R),
		    RA(b.lat -  a.lat) * kLat2m};
}

// pos + vec
static inline
LLPos operator+(const LLPos &p, const Vec2& v)
{
	return {RA(p.lon + v.x / (kLat2m * cosf(p.lat * kD2R))),
			RA(p.lat + v.y / kLat2m)};
}

// vec - vec
static inline
Vec2 operator-(const Vec2& b, const Vec2& a)
{
	return {b.x - a.x, b.y - a.y};
}

// vec + vec
static inline
Vec2 operator+(const Vec2& a, const Vec2& b)
{
	return {a.x + b.x, a.y + b.y};
}

// c * vec
static inline
Vec2 operator*(double c, const Vec2& v)
{
	return {c * v.x, c * v.y};
}

struct Stand {
	std::string name;
	LLPos pos;
	float hdgt;
    bool has_jw{false};
};

class Airport {
  public:
    const std::string name_;

	bool has_app_dep_{false};
	bool has_twr_{false};
	bool ignore_{false};		// e.g. sam or no_autodgs marker present
	std::vector<Stand> stands_;
	Airport(const std::string& name) : name_(name) {}
	void dump();
};

#define RAMP_NAME_LEN 15
#define AIRPORTDB_ICAO_LEN 6

extern std::string xp_dir;
extern std::string base_dir; // base directory of AutoDGS

extern void LogMsg(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

extern std::unordered_map<std::string, std::shared_ptr<Airport>> airports;
extern std::shared_ptr<Airport> arpt;
extern bool CollectAirports(const std::string& xp_dir);

extern XPLMCommandRef cycle_dgs_cmdr;
extern XPLMDataRef vr_enabled_dr;

extern int dgs_type;
extern void toggle_ui(void);
extern void update_ui(int only_if_visible);

extern void set_selected_ramp(const char *ui_sr);
extern void set_dgs_type(int new_dgs_type);
