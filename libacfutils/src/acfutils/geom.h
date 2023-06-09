/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_GEOM_H_
#define	_ACF_UTILS_GEOM_H_

#include <stdlib.h>
#include <math.h>

#include "sysmacros.h"
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Geographic (spherical) coordinates.
 */
typedef struct {
	double	lat;	/* degrees, increasing north */
	double	lon;	/* degrees, increasing east */
	double	elev;	/* meters, increasing away from surface */
} geo_pos3_t;

/*
 * Simplified version of geo_pos3_t without an elevation component.
 */
typedef struct {
	double	lat;
	double	lon;
} geo_pos2_t;

/*
 * More compact versions of the geo_pos*_t types. These use single
 * precision floating point to save a bit on memory.
 */
typedef struct {
	float	lat;
	float	lon;
	float	elev;
} geo_pos3_32_t;
typedef struct {
	float	lat;
	float	lon;
} geo_pos2_32_t;

/*
 * Generic 3-space vector. Looking down onto a plane embedded in euclidian
 * 3-space, the axes are:
 * x: left-to-right (increasing right)
 * y: down-to-up (increasing up)
 * z: away-towards viewer (increasing towards viewer)
 *
 *			Y (incr up)
 *			^
 *			|
 *			|
 *			|
 *			x-------->
 *			Z	 X (incr right)
 *	(incr towards us)
 */
typedef struct {
	double	x;
	double	y;
	double	z;
} vect3_t;

typedef struct {
	long double	x;
	long double	y;
	long double	z;
} vect3l_t;

/*
 * Generic 2-space vector. On euclidian 2-space, axes are:
 * x: left-to-right (increasing right)
 * y: down-to-up (increasing up)
 *
 *	Y (incr up)
 *	^
 *	|
 *	|
 *	|
 *	 -------->
 *		 X (incr right)
 */
typedef struct vect2_s {
	double	x;
	double	y;
} vect2_t;

typedef struct {
	double	a;	/* semi-major axis of the ellipsoid in meters */
	double	b;	/* semi-minor axis of the ellipsoid in meters */
	double	f;	/* flattening */
	double	ecc;	/* first eccentricity */
	double	ecc2;	/* first eccentricity squared */
	double	r;	/* mean radius in meters */
} ellip_t;

typedef struct {
	size_t	n_pts;
	vect2_t	*pts;
} bezier_t;

/*
 * Unit conversions
 */
#define	RAD2DEG_RATIO	(M_PI / 180)		/* 1 rad / 180 deg */
#define	DEG2RAD_RATIO	(180 / M_PI)		/* 180 deg / 1 rad */
#define	DEG2RAD(d)	((d) * RAD2DEG_RATIO)	/* degrees to radians */
#define	RAD2DEG(r)	((r) * DEG2RAD_RATIO)	/* radians to degrees */

/*
 * Coordinate constructors.
 */
#define	GEO_POS2(lat, lon)		((geo_pos2_t){(lat), (lon)})
#define	GEO_POS3(lat, lon, elev)	((geo_pos3_t){(lat), (lon), (elev)})
#define	GEO_POS3_F32(lat, lon, elev)	((geo_pos3_f32_t){(lat), (lon), (elev)})
#define	GEO_POS2_F32(lat, lon)		((geo_pos2_f32_t){(lat), (lon)})
#define	TO_GEO3(geo3)		\
	((geo_pos3_t){(geo3).lat, (geo3).lon, (geo3).elev})
#define	TO_GEO2(geo2)		\
	((geo_pos2_t){(geo2).lat, (geo2).lon})
#define	TO_GEO3_32(geo3)	\
	((geo_pos3_32_t){(geo3).lat, (geo3).lon, (geo3).elev})
#define	TO_GEO2_32(geo2)	\
	((geo_pos3_32_t){(geo3).lat, (geo3).lon})
#define	VECT2(x, y)			((vect2_t){(x), (y)})
#define	VECT3(x, y, z)			((vect3_t){(x), (y), (z)})
#define	VECT3L(x, y, z)			((vect3l_t){(x), (y), (z)})
#define	VECT2_EQ(a, b)			((a).x == (b).x && (a).y == (b).y)
#define	VECT3_EQ(a, b)	\
	((a).x == (b).x && (a).y == (b).y && (a).z == (b).z)
#define	VECT2_PARALLEL(a, b)	\
	(((a).y == 0 && (b).y == 0) || (((a).x / (a).y) == ((b).x / (b).y)))

/*
 * Special coordinate values and tests for these special values.
 */
#define	ZERO_VECT2		((vect2_t){0.0, 0.0})
#define	ZERO_VECT3		((vect3_t){0.0, 0.0, 0.0})
#define	ZERO_VECT3L		((vect3l_t){0.0, 0.0, 0.0})
#define	NULL_VECT2		((vect2_t){NAN, NAN})
#define	NULL_VECT3		((vect3_t){NAN, NAN, NAN})
#define	NULL_VECT3L		((vect3l_t){NAN, NAN, NAN})
#define	NULL_GEO_POS3		((geo_pos3_t){NAN, NAN, NAN})
#define	NULL_GEO_POS2		((geo_pos2_t){NAN, NAN})
#define	IS_NULL_VECT(a)		(isnan((a).x))
#define	IS_NULL_VECT2(a)	(isnan((a).x) || isnan((a).y))
#define	IS_NULL_VECT3(a)	(isnan((a).x) || isnan((a).y) || isnan((a).z))
#define	IS_FINITE_VECT2(a)	(isfinite((a).x) && isfinite((a).y))
#define	IS_FINITE_VECT3(a)	\
	(isfinite((a).x) && isfinite((a).y) && isfinite((a).z))
#define	IS_NULL_GEO_POS(a)	(isnan((a).lat))
#define	IS_NULL_GEO_POS2(a)	\
	(isnan((a).lat) || isnan((a).lon))
#define	IS_NULL_GEO_POS3(a)	\
	(isnan((a).lat) || isnan((a).lon) || isnan((a).elev))
#define	IS_ZERO_VECT2(a)	((a).x == 0.0 && (a).y == 0.0)
#define	IS_ZERO_VECT3(a)	((a).x == 0.0 && (a).y == 0.0 && (a).z == 0.0)

#define	VECT2_TO_VECT3(v, z)	((vect3_t){(v).x, (v).y, (z)})
#define	VECT3_TO_VECT2(v)	((vect2_t){(v).x, (v).y})
#define	VECT3L_TO_VECT3(v)	((vect3_t){(v).x, (v).y, (v).z})
#define	VECT3_TO_VECT3L(v)	((vect3l_t){(v).x, (v).y, (v).z})

#define	GEO2_TO_GEO3(v, a)	((geo_pos3_t){(v).lat, (v).lon, (a)})
#define	GEO3_TO_GEO2(v)		((geo_pos2_t){(v).lat, (v).lon})
#define	GEO3_FT2M(g)		GEO_POS3((g).lat, (g).lon, FEET2MET((g).elev))
#define	GEO3_M2FT(g)		GEO_POS3((g).lat, (g).lon, MET2FEET((g).elev))
#define	GEO3_EQ(p1, p2)	\
	((p1).lat == (p2).lat && (p1).lon == (p2).lon && \
	(p1).elev == (p2).elev)
#define	GEO2_EQ(p1, p2)		((p1).lat == (p2).lat && (p1).lon == (p2).lon)

#define	EARTH_MSL		6371200		/* meters */
#ifndef	ABS
#define	ABS(x)	((x) > 0 ? (x) : -(x))
#endif

/* Math debugging */
#if	1
#define	PRINT_VECT2(v)	printf(#v "(%f, %f)\n", v.x, v.y)
#define	PRINT_VECT3(v)	printf(#v "(%f, %f, %f)\n", v.x, v.y, v.z)
#define	PRINT_VECT3L(v)	printf(#v "(%Lf, %Lf, %Lf)\n", v.x, v.y, v.z)
#define	PRINT_GEO2(p)	printf(#p "(%f, %f)\n", p.lat, p.lon)
#define	PRINT_GEO3(p)	printf(#p "(%f, %f, %f)\n", p.lat, p.lon, p.elev)
#define	DEBUG_PRINT(...)	printf(__VA_ARGS__)
#else
#define	PRINT_VECT2(v)
#define	PRINT_VECT3(v)
#define	PRINT_GEO2(p)
#define	PRINT_GEO3(p)
#define	DEBUG_PRINT(...)
#endif

/*
 * The standard WGS84 ellipsoid.
 */
#define	wgs84	ACFSYM(wgs84)
API_EXPORT_DATA const ellip_t wgs84;

/*
 * Small helpers.
 */
#define	is_on_arc	ACFSYM(is_on_arc)
API_EXPORT bool_t is_on_arc(double angle_x, double angle1, double angle2,
    bool_t cw);

/*
 * Angle util functions.
 */
API_EXPORT double rel_angle(double a1, double a2) PURE_ATTR;

/*
 * Vector math.
 */
API_EXPORT double vect3_abs(vect3_t a) PURE_ATTR;
API_EXPORT long double vect3l_abs(vect3l_t a) PURE_ATTR;
API_EXPORT double vect3_dist(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT long double vect3l_dist(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT double vect2_abs(vect2_t a) PURE_ATTR;
API_EXPORT double vect2_dist(vect2_t a, vect2_t b) PURE_ATTR;
API_EXPORT vect3_t vect3_set_abs(vect3_t a, double abs) PURE_ATTR;
API_EXPORT vect3l_t vect3l_set_abs(vect3l_t a, long double abs) PURE_ATTR;
API_EXPORT vect2_t vect2_set_abs(vect2_t a, double abs) PURE_ATTR;
API_EXPORT vect3_t vect3_unit(vect3_t a, double *l);
API_EXPORT vect2_t vect2_unit(vect2_t a, double *l);

API_EXPORT vect3_t vect3_add(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT vect3l_t vect3l_add(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT vect2_t vect2_add(vect2_t a, vect2_t b) PURE_ATTR;
API_EXPORT vect3_t vect3_sub(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT vect3l_t vect3l_sub(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT vect2_t vect2_sub(vect2_t a, vect2_t b) PURE_ATTR;
API_EXPORT vect3_t vect3_mul(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT vect3l_t vect3l_mul(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT vect2_t vect2_mul(vect2_t a, vect2_t b) PURE_ATTR;
API_EXPORT vect3_t vect3_scmul(vect3_t a, double b) PURE_ATTR;
API_EXPORT vect3l_t vect3l_scmul(vect3l_t a, long double b) PURE_ATTR;
API_EXPORT vect2_t vect2_scmul(vect2_t a, double b) PURE_ATTR;
API_EXPORT double vect3_dotprod(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT long double vect3l_dotprod(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT double vect2_dotprod(vect2_t a, vect2_t b) PURE_ATTR;
API_EXPORT vect3_t vect3_xprod(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT vect3l_t vect3l_xprod(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT vect3_t vect3_mean(vect3_t a, vect3_t b) PURE_ATTR;
API_EXPORT vect3l_t vect3l_mean(vect3l_t a, vect3l_t b) PURE_ATTR;
API_EXPORT vect2_t vect2_mean(vect2_t a, vect2_t b) PURE_ATTR;

API_EXPORT vect2_t vect2_norm(vect2_t v, bool_t right) PURE_ATTR;
API_EXPORT vect3_t vect3_rot(vect3_t v, double angle, unsigned axis) PURE_ATTR;
API_EXPORT vect3l_t vect3l_rot(vect3l_t v, long double angle, unsigned axis)
    PURE_ATTR;
API_EXPORT vect2_t vect2_rot(vect2_t v, double angle) PURE_ATTR;
API_EXPORT vect2_t vect2_rot_inv_y(vect2_t v, double angle) PURE_ATTR;
API_EXPORT vect3_t vect3_neg(vect3_t v) PURE_ATTR;
API_EXPORT vect3l_t vect3l_neg(vect3l_t v) PURE_ATTR;
API_EXPORT vect2_t vect2_neg(vect2_t v) PURE_ATTR;

API_EXPORT vect3_t vect3_local2acf(vect3_t v, double roll, double pitch,
    double hdgt) PURE_ATTR;
API_EXPORT vect3_t vect3_acf2local(vect3_t v, double roll, double pitch,
    double hdgt) PURE_ATTR;

/*
 * Spherical, geodesic, ECEF and ECmI coordinate conversion.
 */
API_EXPORT ellip_t ellip_init(double semi_major, double semi_minor,
    double flattening);
API_EXPORT geo_pos3_t geo2sph(geo_pos3_t pos, const ellip_t *ellip) PURE_ATTR;
API_EXPORT vect3_t geo2ecef_mtr(geo_pos3_t pos, const ellip_t *ellip) PURE_ATTR;
API_EXPORT vect3_t geo2ecef_ft(geo_pos3_t pos, const ellip_t *ellip) PURE_ATTR;
API_EXPORT geo_pos3_t ecef2geo(vect3_t pos, const ellip_t *ellip) PURE_ATTR;
API_EXPORT geo_pos3_t ecef2sph(vect3_t v) PURE_ATTR;
API_EXPORT vect3_t sph2ecef(geo_pos3_t pos) PURE_ATTR;
/*
 * Converts between ECEF/ECMI coordinates and OpenGL coordinates. OpenGL
 * uses X to represent the lateral aixs, Y to represent the vertical axis
 * and Z to represent the axis going towards the camera.
 */
API_EXPORT vect3_t ecef2gl(vect3_t ecmi) PURE_ATTR;
API_EXPORT vect3_t gl2ecef(vect3_t opengl) PURE_ATTR;
API_EXPORT vect3l_t ecef2gl_l(vect3l_t ecmi) PURE_ATTR;
API_EXPORT vect3l_t gl2ecef_l(vect3l_t opengl) PURE_ATTR;
/*
 * ECmI stands for Earth-Centered-modified-Inertial. Unlike ECEF, it is
 * an inertial frame that doesn't rotate together with the earth. However,
 * unlike plain ECI coordinates, ECmI is aligned with the Earth equatorial
 * plane, not the Earth's orbital plane around the Sun. This makes it a
 * bit easier to use for inertial calculations for objects moving around
 * on the Earth (like airplanes, duh). In ECmI, the X and Y axes are
 * aligned with the 0 and 90 degree meridians respectively ONLY at the
 * coordinate reference time (delta_t=0). Thus, to convert between
 * geographic or ECEF coordinates and ECmI, we need to know the exact time
 * as a delta from the reference time (which can be arbitrary).
 */
API_EXPORT vect3_t geo2ecmi(geo_pos3_t pos, double delta_t,
    const ellip_t *ellip) PURE_ATTR;
API_EXPORT geo_pos3_t ecmi2geo(vect3_t pos, double delta_t,
    const ellip_t *ellip) PURE_ATTR;
API_EXPORT vect3_t sph2ecmi(geo_pos3_t pos, double delta_t) PURE_ATTR;
API_EXPORT geo_pos3_t ecmi2sph(vect3_t pos, double delta_t) PURE_ATTR;
API_EXPORT vect3_t ecef2ecmi(vect3_t ecef, double delta_t) PURE_ATTR;
API_EXPORT vect3_t ecmi2ecef(vect3_t ecmi, double delta_t) PURE_ATTR;

/*
 * Interesections.
 */
API_EXPORT unsigned vect2sph_isect(vect3_t v, vect3_t o, vect3_t c, double r,
    bool_t confined, vect3_t i[2]);
API_EXPORT unsigned vect2circ_isect(vect2_t v, vect2_t o, vect2_t c, double r,
    bool_t confined, vect2_t i[2]);
API_EXPORT vect2_t vect2vect_isect(vect2_t da, vect2_t oa, vect2_t db,
    vect2_t ob, bool_t confined) PURE_ATTR;
API_EXPORT unsigned circ2circ_isect(vect2_t ca, double ra, vect2_t cb,
    double rb, vect2_t i[2]);
API_EXPORT unsigned vect2poly_isect_get(vect2_t a, vect2_t oa,
    const vect2_t *poly, vect2_t *isects, unsigned cap);
API_EXPORT unsigned vect2poly_isect(vect2_t a, vect2_t oa, const vect2_t *poly)
    PURE_ATTR;
API_EXPORT bool_t point_in_poly(vect2_t pt, const vect2_t *poly) PURE_ATTR;

/*
 * Converting between headings and direction vectors on a 2D plane.
 */
#define	hdg2dir	ACFSYM(hdg2dir)
API_EXPORT vect2_t hdg2dir(double truehdg) PURE_ATTR;
#define	dir2hdg	ACFSYM(dir2hdg)
API_EXPORT double dir2hdg(vect2_t dir) PURE_ATTR;

/*
 * Calculating coordinate displacement & radial intersection.
 */
API_EXPORT geo_pos2_t geo_displace(const ellip_t *ellip, geo_pos2_t pos,
    double truehdg, double dist) PURE_ATTR;
API_EXPORT geo_pos2_t geo_displace_dir(const ellip_t *ellip, geo_pos2_t pos,
    vect2_t dir, double dist) PURE_ATTR;

/*
 * Geometry parser & validator helpers.
 */
API_EXPORT bool_t geo_pos2_from_str(const char *lat, const char *lon,
    geo_pos2_t *pos);
API_EXPORT bool_t geo_pos3_from_str(const char *lat, const char *lon,
    const char *elev, geo_pos3_t *pos);

/*
 * Spherical coordinate system translation.
 */
typedef struct {
	double	sph_matrix[3 * 3];
	double	rot_matrix[2 * 2];
	bool_t	inv;
} sph_xlate_t;

API_EXPORT sph_xlate_t sph_xlate_init(geo_pos2_t displacement,
    double rotation, bool_t inv);
API_EXPORT geo_pos2_t sph_xlate(geo_pos2_t pos, const sph_xlate_t *xlate)
    PURE_ATTR;
API_EXPORT vect3_t sph_xlate_vect(vect3_t pos, const sph_xlate_t *xlate)
    PURE_ATTR;

/*
 * Great circle functions.
 */
#define	gc_distance	ACFSYM(gc_distance)
API_EXPORT double gc_distance(geo_pos2_t start, geo_pos2_t end);
#define	gc_point_hdg	ACFSYM(gc_point_hdg)
API_EXPORT double gc_point_hdg(geo_pos2_t start, geo_pos2_t end);

/*
 * Generic spherical - to - flat-plane projections.
 */
typedef struct {
	const ellip_t	*ellip;
	sph_xlate_t	xlate;
	sph_xlate_t	inv_xlate;
	bool_t		allow_inv;
	double		dist;
	vect2_t		scale;
} fpp_t;

API_EXPORT fpp_t fpp_init(geo_pos2_t center, double rot, double dist,
    const ellip_t *ellip, bool_t allow_inv);
API_EXPORT fpp_t ortho_fpp_init(geo_pos2_t center, double rot,
    const ellip_t *ellip, bool_t allow_inv);
API_EXPORT fpp_t gnomo_fpp_init(geo_pos2_t center, double rot,
    const ellip_t *ellip, bool_t allow_inv);
API_EXPORT fpp_t stereo_fpp_init(geo_pos2_t center, double rot,
    const ellip_t *ellip, bool_t allow_inv);
API_EXPORT vect2_t geo2fpp(geo_pos2_t pos, const fpp_t *fpp) PURE_ATTR;
API_EXPORT geo_pos2_t fpp2geo(vect2_t pos, const fpp_t *fpp) PURE_ATTR;
API_EXPORT void fpp_set_scale(fpp_t *fpp, vect2_t scale);
API_EXPORT vect2_t fpp_get_scale(const fpp_t *fpp);

/*
 * Lambert conformal conic projection
 */
typedef struct {
	double	reflat;
	double	reflon;
	double	n;
	double	F;
	double	rho0;
} lcc_t;

API_EXPORT lcc_t lcc_init(double reflat, double reflon, double stdpar1,
    double stdpar2);
API_EXPORT vect2_t geo2lcc(geo_pos2_t pos, const lcc_t *lcc) PURE_ATTR;

/*
 *  Bezier curve functions.
 */
API_EXPORT bezier_t *bezier_alloc(size_t num_pts);
API_EXPORT void bezier_free(bezier_t *curve);
API_EXPORT double quad_bezier_func(double x, const bezier_t *func);
API_EXPORT double *quad_bezier_func_inv(double y, const bezier_t *func,
    size_t *n_xs);

/*
 * Matrix math.
 */
typedef struct {
	double	_mat4_data[16];
} mat4_t;

typedef struct {
	double	_mat3_data[9];
} mat3_t;

#define	MAT4(mat, col, row)	((mat)->_mat4_data[(col) * 4 + (row)])
#define	MAT3(mat, col, row)	((mat)->_mat3_data[(col) * 3 + (row)])
#define	MAT4_DATA(mat)		((mat)->_mat4_data)
#define	MAT3_DATA(mat)		((mat)->_mat3_data)

API_EXPORT void mat4_ident(mat4_t *mat);
API_EXPORT void mat3_ident(mat3_t *mat);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_GEOM_H_ */
