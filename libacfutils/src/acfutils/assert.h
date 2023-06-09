/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2015 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_ASSERT_H_
#define	_ACF_UTILS_ASSERT_H_

#include <assert.h>
#include <stdlib.h>

#include "log.h"
#include "sysmacros.h"

#ifdef	__cplusplus
extern "C" {
#endif

#if	LIN || APL
#define	LACF_CRASH()	abort()
#else	/* !LIN && !APL */
#define	EXCEPTION_ASSERTION_FAILED	0x8000
#define	LACF_CRASH()	\
	do { \
		RaiseException(EXCEPTION_ASSERTION_FAILED, \
		    EXCEPTION_NONCONTINUABLE, 0, NULL); \
		/* Needed to avoid no-return-value warnings */ \
		abort(); \
	} while (0)
#endif	/* !LIN && !APL */

/*
 * ASSERT() and VERIFY() are assertion test macros. If the condition
 * expression provided as the argument to the macro evaluates as non-true,
 * the program prints a debug message specifying exactly where and what
 * condition was violated, a stack backtrace and a dumps core by
 * calling abort().
 *
 * The difference between ASSERT and VERIFY is that ASSERT compiles to
 * a no-op unless -DDEBUG is provided to the compiler. VERIFY always
 * checks its condition and dumps if it is non-true.
 */

#define	VERIFY_MSG(x, fmt, ...) \
	do { \
		if (COND_UNLIKELY(!(x))) { \
			log_impl(log_basename(__FILE__), __LINE__, \
			    "assertion \"%s\" failed: " fmt, #x, __VA_ARGS__); \
			LACF_CRASH(); \
		} \
	} while (0)

#define	VERIFY(x)	VERIFY_MSG(x, "%s", "")

#define	VERIFY3_impl(x, op, y, type, fmt) \
	do { \
		type tmp_x = (type)(x); \
		type tmp_y = (type)(y); \
		if (COND_UNLIKELY(!(tmp_x op tmp_y))) { \
			log_impl(log_basename(__FILE__), __LINE__, \
			    "assertion %s %s %s failed (" fmt " %s " \
			    fmt ")", #x, #op, #y, tmp_x, #op, tmp_y); \
			LACF_CRASH(); \
		} \
	} while (0)
#define	VERIFY3S(x, op, y)	VERIFY3_impl(x, op, y, long, "%lu")
#if	IBM
#define	VERIFY3U(x, op, y)	\
	VERIFY3_impl(x, op, y, unsigned long long, "0x%I64x")
#else	/* !IBM */
#define	VERIFY3U(x, op, y)	\
	VERIFY3_impl(x, op, y, unsigned long long, "0x%llx")
#endif	/* !IBM */
#define	VERIFY3F(x, op, y)	VERIFY3_impl(x, op, y, double, "%f")
#define	VERIFY3P(x, op, y)	VERIFY3_impl(x, op, y, void *, "%p")
#define	VERIFY0(x)		VERIFY3S((x), ==, 0)
#define	VERIFY_FAIL()		\
	do { \
		log_impl(log_basename(__FILE__), __LINE__, "Internal error"); \
		LACF_CRASH(); \
	} while (0)

#ifdef	DEBUG
#define	ASSERT(x)		VERIFY(x)
#define	ASSERT3S(x, op, y)	VERIFY3S(x, op, y)
#define	ASSERT3U(x, op, y)	VERIFY3U(x, op, y)
#define	ASSERT3F(x, op, y)	VERIFY3F(x, op, y)
#define	ASSERT3P(x, op, y)	VERIFY3P(x, op, y)
#define	ASSERT0(x)		VERIFY0(x)
#define	ASSERT_MSG(x, fmt, ...)	VERIFY_MSG(x, fmt, __VA_ARGS__)
#else	/* !DEBUG */
#define	ASSERT(x)		UNUSED(x)
#define	ASSERT3S(x, op, y)	UNUSED((x) op (y))
#define	ASSERT3U(x, op, y)	UNUSED((x) op (y))
#define	ASSERT3F(x, op, y)	UNUSED((x) op (y))
#define	ASSERT3P(x, op, y)	UNUSED((x) op (y))
#define	ASSERT0(x)		UNUSED(x)
#define	ASSERT_MSG(x, fmt, ...)	UNUSED(x)
#endif	/* !DEBUG */

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_ASSERT_H_ */
