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
 * Copyright 2020 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_MATH_H_
#define	_ACF_UTILS_MATH_H_

#include "assert.h"
#include "math_core.h"
#include "geom.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	POW4(x)	((x) * (x) * (x) * (x))
#define	POW3(x)	((x) * (x) * (x))
#define	POW2(x)	((x) * (x))
#define	ROUND_ERROR	1e-10
#ifndef	ABS
/** @return The absolute value of `x`. */
#define	ABS(x)	((x) > 0 ? (x) : -(x))
#endif

struct vect2_s;

#define	quadratic_solve		ACFSYM(quadratic_solve)
API_EXPORT unsigned quadratic_solve(double a, double b, double c, double x[2]);
#define	fx_lin			ACFSYM(fx_lin)
API_EXPORT double fx_lin(double x, double x1, double y1, double x2, double y2)
    PURE_ATTR;
#define	fx_lin_multi		ACFSYM(fx_lin_multi)
API_EXPORT double fx_lin_multi(double x, const struct vect2_s *points,
    bool_t extrapolate) PURE_ATTR;
#define	fx_lin_multi_inv	ACFSYM(fx_lin_multi_inv)
API_EXPORT double *fx_lin_multi_inv(double y, const struct vect2_s *points,
    size_t *num_out);
#define	fx_lin_multi_inv2	ACFSYM(fx_lin_multi_inv2)
API_EXPORT double *fx_lin_multi_inv2(double y, const struct vect2_s *points,
    bool_t extrapolate, size_t *num_out);

/*
 * Weighted avg, 'w' is weight fraction from 0.0 = all of x to 1.0 = all of y.
 */
static inline double
wavg_impl(double x, double y, double w, const char *file, int line)
{
	UNUSED(file);
	UNUSED(line);
	ASSERT_MSG(w >= 0.0, "%f < 0.0: called from: %s:%d", w, file, line);
	ASSERT_MSG(w <= 1.0, "%f > 1.0: called from: %s:%d", w, file, line);
	return (x + (y - x) * w);
}
#define wavg(x, y, w)	wavg_impl((x), (y), (w), __FILE__, __LINE__)

static inline double
wavg2(double x, double y, double w)
{
	return (x + (y - x) * w);
}

/*
 * Given two values min_val and max_val, returns how far between min_val
 * and max_val a third value 'x' lies. If `clamp_output' is true, 'x' is
 * clamped such that it always lies between min_val and max_val. In essence,
 * this function computes:
 *
 *      ^
 *      |
 *    1 -------------------+
 *      |               /  |
 *      |             /    |
 * f(x) ------------+      |
 *      |         / |      |
 *      |       /   |      |
 *    0 ------+     |      |
 *      |     |     |      |
 *      +-----|-----|------|----->
 *         min_val  x   max_val
 */
static inline double
iter_fract(double x, double min_val, double max_val, bool_t clamp_output)
{
	ASSERT3F(min_val, !=, max_val);
	x = (x - min_val) / (max_val - min_val);
	if (clamp_output)
		x = clamp(x, 0, 1);
	return (x);
}

/*
 * This is a generic polynomial interpolator/extrapolator. Given a series
 * of X-Y coordinates, pn_interp_init constructs a polynomial interpolation
 * that smoothly passes through all of the input points. Please note that
 * this function is limited to a maximum number of inputs points (mainly
 * to make memory management easy by not requiring dynamic allocation).
 */
#define	MAX_PN_INTERP_ORDER	64
typedef struct {
	unsigned	order;
	double		coeff[MAX_PN_INTERP_ORDER];
} pn_interp_t;

/*
 * Given a series of X-Y coordinates, this function initializes a polynomial
 * interpolator that smoothly passes through all the input points. When you
 * are done with the interpolator, you DON'T have to free it. The pn_interp_t
 * structure is entirely self-contained.
 *
 * @param interp Interpolator that needs to be initialized.
 * @param points Input points that the interpolator needs to pass through.
 * @param npts Number points in `points'. This must be GREATER than 0.
 */
#define	pn_interp_init	ACFSYM(pn_interp_init)
API_EXPORT void pn_interp_init(pn_interp_t *interp, const vect2_t *points,
    unsigned npts);

/*
 * Given an initialized pn_interp_t (see above), calculates the Y value
 * at a given point.
 * @param x The X point for which to calculate the interpolated Y value.
 * @param interp An initialized interpolator (as initialized by pn_interp_init).
 */
static inline double
pn_interp_run(double x, const pn_interp_t *interp)
{
	double y = 0, power = 1;

	ASSERT(interp != NULL);
	ASSERT(interp->order != 0);
	for (unsigned i = 0; i < interp->order; i++) {
		y += interp->coeff[i] * power;
		power *= x;
	}

	return (y);
}

/*
 * Implements the smoothstep function from GLSL. See Wikipedia
 * for more info: https://en.wikipedia.org/wiki/Smoothstep
 */
static inline double
smoothstep(double x, double edge0, double edge1)
{
	ASSERT(!isnan(x));
	ASSERT3F(edge1, >, edge0);
	x = clamp((x - edge0) / (edge1 - edge0), 0, 1);
	return (POW2(x) * (3 - 2 * x));
}

/*
 * Inverse of smoothstep. The returned value is always in the range of 0-1.
 */
static inline double
smoothstep_inv(double x)
{
	ASSERT(!isnan(x));
	return (0.5 - sin(asin(1 - 2 * x) / 3));
}

#define	HROUND2(oldval, newval, step, hyst_rng) \
	do { \
		double tmpval = round((newval) / (step)) * (step); \
		if (isnan((oldval))) { \
			(oldval) = tmpval; \
		} else if ( \
		    (newval) > (oldval) + (step) * (0.5 + (hyst_rng)) || \
		    (newval) < (oldval) - (step) * (0.5 + (hyst_rng))) { \
			(oldval) = tmpval; \
		} \
	} while (0)

#define	HROUND(oldval, newval, step) \
	HROUND2((oldval), (newval), (step), 0.35)

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_GEOM_H_ */
