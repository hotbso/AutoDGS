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

#ifndef	_ACF_UTILS_MATH_CORE_H_
#define	_ACF_UTILS_MATH_CORE_H_

#include <math.h>

#include "assert.h"
#include "core.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	DEFN_CLAMP(name, type, assert_chk) \
static inline type \
name(type x, type min_val, type max_val) \
{ \
	if (COND_LIKELY(min_val < max_val)) { \
		if (x < min_val) \
			return (min_val); \
		if (x > max_val) \
			return (max_val); \
	} else { \
		if (x > min_val) \
			return (min_val); \
		if (x < max_val) \
			return (max_val); \
	} \
	return (x); \
}

DEFN_CLAMP(clamp, double, ASSERT3F)
DEFN_CLAMP(clampl, long, ASSERT3S)
DEFN_CLAMP(clampi, int, ASSERT3S)

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_GEOM_H_ */
