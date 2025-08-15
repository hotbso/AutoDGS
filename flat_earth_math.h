//
//    Some Math for the flat earth.
//
//    Copyright (C) 2024, 2025 Holger Teutsch
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


// Contrary to common belief the earth is flat. She has just a weird coordinate system with (lon,lat).
// To overcome this we attach a 2-d vector space at each (lon, lat) point with orthogonal
// basis scaled in meters. So (lon2, lat2) - (lon1, lat1) gives rise to a vector v in the vector space
// attached at (lon1, lat1) and (lon1, lat1) + v again is (lon2, lat2).
// As we do our math in a circle of ~ 20km this works pretty well.
//
// Should you still be tricked in believing that the earth is a ball you can consider this vector space
// a tangent space. But this is for for visualisation only.
//

#ifndef _FLAT_EARTH_MATH_
#define _FLAT_EARTH_MATH_

namespace flat_earth_math {

#include <cmath>

static constexpr float kLat2m = 111120;             // 1° lat in m

// return relative angle in (-180, 180]
static inline
double RA(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle > 180.0)
        return angle - 360.0;

    if (angle <= -180.0)
        return angle + 360.0;

    return angle;
}


struct LLPos {
	double lon, lat;
};

struct Vec2 {
	double x,y;
};

static inline double len(const Vec2& v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

// pos b - pos a
static inline Vec2 operator-(const LLPos& b, const LLPos& a) {
    return {RA(b.lon - a.lon) * kLat2m * cosf(a.lat * 0.01745329252), RA(b.lat - a.lat) * kLat2m};
}

// pos + vec
static inline LLPos operator+(const LLPos &p, const Vec2& v) {
	return {RA(p.lon + v.x / (kLat2m * cosf(p.lat * 0.01745329252))),
			RA(p.lat + v.y / kLat2m)};
}

// vec b - vec a
static inline Vec2 operator-(const Vec2& b, const Vec2& a) {
    return {b.x - a.x, b.y - a.y};
}

// vec + vec
static inline Vec2 operator+(const Vec2& a, const Vec2& b) {
    return {a.x + b.x, a.y + b.y};
}

// c * vec
static inline Vec2 operator*(double c, const Vec2& v) {
    return {c * v.x, c * v.y};
}

// vec * vec
static inline double operator*(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;  // dot product
}

// pos in boundary box
static inline bool InBB(const LLPos& pos, const LLPos& bbox_min, const LLPos& bbox_max) {
    // cheap test before we do the more expensive RA
    return (pos.lat >= bbox_min.lat && pos.lat <= bbox_max.lat && RA(pos.lon - bbox_min.lon) > 0.0f &&
            RA(pos.lon - bbox_max.lon) < 0.0f);
}

}	// namespace
#endif
