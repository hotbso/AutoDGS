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
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_SYSMACROS_H_
#define	_ACF_UTILS_SYSMACROS_H_

#if	IBM
#include <windows.h>
#endif	/* IBM */

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	NO_ACF_TYPE		0
#define	FF_A320_ACF_TYPE	1

#if	IBM
#define	DIRSEP		'\\'
#define	DIRSEP_S	"\\"	/* DIRSEP as a string */
#else	/* !IBM */
#define	DIRSEP		'/'
#define	DIRSEP_S	"/"	/* DIRSEP as a string */
#ifndef	MAX_PATH
#define	MAX_PATH	512
#endif
#endif	/* !IBM */

#if	defined(__GNUC__) || defined(__clang__)
#define	DEPRECATED_FUNC(f)	f __attribute__((deprecated))
#define	PRINTF_ATTR(x)		__attribute__((format(printf, x, x + 1)))
#define	PRINTF_ATTR2(x,y)	__attribute__((format(printf, x, y)))
#define	PRINTF_FORMAT(f)	f
#define	SENTINEL_ATTR		__attribute__((sentinel))
#define	HOT_ATTR		__attribute__((hot))
#define	PURE_ATTR		__attribute__((pure))
#define	ALWAYS_INLINE_ATTR	__attribute__((always_inline))
#define	ALIGN_ATTR(x)		__attribute__((aligned(x)))

#ifndef	BSWAP32
#define	BSWAP16(x)	__builtin_bswap16((x))
#define	BSWAP32(x)	__builtin_bswap32((x))
#define	BSWAP64(x)	__builtin_bswap64((x))
#endif	/* BSWAP32 */

#define	COND_LIKELY(x)		__builtin_expect(x, 1)
#define	COND_UNLIKELY(x)	__builtin_expect(x, 0)

#else	/* !__GNUC__ && !__clang__ */

#define	PRINTF_ATTR(x)
#define	PRINTF_ATTR2(x,y)
#define	SENTINEL_ATTR
#define	HOT_ATTR
#define	PURE_ATTR
#define	ALWAYS_INLINE_ATTR

#define	COND_LIKELY(x)		x
#define	COND_UNLIKELY(x)	x

#if	defined(_MSC_VER)
#define	ALIGN_ATTR(x)		__declspec(align(x))
#else
#define	ALIGN_ATTR(x)
#endif

#if	_MSC_VER >= 1400
# define	DEPRECATED_FUNC(f)	__declspec(deprecated) f
# include <sal.h>
# if	_MSC_VER > 1400
#  define	PRINTF_FORMAT(f)	_Printf_format_string_ f
# else	/* _MSC_VER == 1400 */
#  define	PRINTF_FORMAT(f)	__format_string f
# endif /* FORMAT_STRING */
#else	/* _MSC_VER < 1400 */
# define	PRINTF_FORMAT(f)	f
#endif	/* _MSC_VER */

#ifndef	BSWAP32
#define	BSWAP16(x)	\
	((((x) & 0xff00u) >> 8) | \
	(((x) & 0x00ffu) << 8))
#define	BSWAP32(x)	\
	((((x) & 0xff000000u) >> 24) | \
	(((x) & 0x00ff0000u) >> 8) | \
	(((x) & 0x0000ff00u) << 8) | \
	(((x) & 0x000000ffu) << 24))
#define	BSWAP64(x)	\
	((((x) & 0x00000000000000ffllu) >> 56) | \
	(((x) & 0x000000000000ff00llu) << 40) | \
	(((x) & 0x0000000000ff0000llu) << 24) | \
	(((x) & 0x00000000ff000000llu) << 8) | \
	(((x) & 0x000000ff00000000llu) >> 8) | \
	(((x) & 0x0000ff0000000000llu) >> 24) | \
	(((x) & 0x00ff000000000000llu) >> 40) | \
	(((x) & 0xff00000000000000llu) << 56))
#endif	/* BSWAP32 */
#endif	/* !__GNUC__ && !__clang__ */

#if	__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define	BE64(x)	(x)
#define	BE32(x)	(x)
#define	BE16(x)	(x)
#else	/* __BYTE_ORDER__ != __ORDER_BIG_ENDIAN__ */
#define	BE64(x)	BSWAP64(x)
#define	BE32(x)	BSWAP32(x)
#define	BE16(x)	BSWAP16(x)
#endif	/* __BYTE_ORDER__ != __ORDER_BIG_ENDIAN__ */

#define	DESTROY(x)	do { free(x); (x) = NULL; } while (0)
#define	DESTROY_OP(var, zero_val, destroy_op) \
	do { \
		if ((var) != (zero_val)) { \
			destroy_op; \
			(var) = (zero_val); \
		} \
	} while (0)

#ifdef	WINDOWS
#define	PATHSEP	"\\"
#else
#define	PATHSEP	"/"
#endif

/* Minimum/Maximum allowable elevation AMSL of anything */
#define	MIN_ELEV	-2000.0
#define	MAX_ELEV	30000.0

/* Minimum/Maximum allowable altitude AMSL of anything */
#define	MIN_ALT		-2000.0
#define	MAX_ALT		100000.0

/* Maximum valid speed of anything */
#define	MAX_SPD		1000.0

/* Minimum/Maximum allowable arc radius on any procedure */
#define	MIN_ARC_RADIUS	0.1
#define	MAX_ARC_RADIUS	100.0

/*
 * Compile-time assertion. The condition 'x' must be constant.
 * TYPE_ASSERT is a compile-time assertion, but which checks that
 * the type of `x' is `type'.
 */
#if	__STDC_VERSION__ >= 201112L
#define	CTASSERT(x) _Static_assert((x), #x)
#define	TYPE_ASSERT(x, type) \
	CTASSERT(_Generic((x), type: 1, default: 0) != 0)
#define	NOT_TYPE_ASSERT(x, type) \
	CTASSERT(_Generic((x), type: 0, default: 1) != 0)
#else	/* __STDC_VERSION__ < 201112L */
#define	TYPE_ASSERT(x, type)
#if	defined(__GNUC__) || defined(__clang__)
#define	CTASSERT(x)		_CTASSERT(x, __LINE__)
#define	_CTASSERT(x, y)		__CTASSERT(x, y)
#define	__CTASSERT(x, y)	\
	typedef char __compile_time_assertion__ ## y [(x) ? 1 : -1] \
	    __attribute__((unused))
#else	/* !defined(__GNUC__) && !defined(__clang__) */
#define	CTASSERT(x)
#endif	/* !defined(__GNUC__) && !defined(__clang__) */
#endif	/* __STDC_VERSION__ < 201112L */

#if	defined(__GNUC__) || defined(__clang__)

#define	highbit64(x)	(64 - __builtin_clzll(x) - 1)
#define	highbit32(x)	(32 - __builtin_clzll(x) - 1)

#elif  defined(_MSC_VER)

static inline unsigned
highbit32(unsigned int x)
{
	unsigned long idx;
	_BitScanReverse(&idx, x);
	return (idx);
}

static inline unsigned
highbit64(unsigned long long x)
{
	unsigned long idx;
	_BitScanReverse64(&idx, x);
	return (idx);
}
#else
#error	"Compiler platform unsupported, please add highbit definition"
#endif

#ifndef	MIN
#define	MIN(x, y)	((x) < (y) ? (x) : (y))
#endif
#ifndef	MAX
#define	MAX(x, y)	((x) > (y) ? (x) : (y))
#endif
#ifndef	AVG
#define	AVG(x, y)	(((x) + (y)) / 2.0)
#endif
/*
 * Provides a gradual method of integrating an old value until it approaches
 * a new target value. This is used in iterative processes by calling the
 * FILTER_IN macro repeatedly a certain time intervals (d_t = delta-time).
 * As time progresses, old_val will gradually be made to approach new_val.
 * The lag serves to make the approach slower or faster (e.g. a value of
 * '2' and d_t in seconds makes old_val approach new_val with a ramp that
 * is approximately 2 seconds long).
 */
#define	FILTER_IN(old_val, new_val, d_t, lag) \
	do { \
		__typeof__(old_val) o = (old_val); \
		__typeof__(new_val) n = (new_val); \
		ASSERT(!isnan(o)); \
		(old_val) += (n - o) * ((d_t) / (lag)); \
		/* Prevent an overshoot */ \
		if ((o < n && (old_val) > n) || \
		    (o > n && (old_val) < n)) \
			(old_val) = n; \
	} while (0)
/*
 * Same as FILTER_IN, but handles NAN values for old_val and new_val properly.
 * If new_val is NAN, old_val is set to NAN. Otherwise if old_val is NAN,
 * it is set to new_val directly (without gradual filtering). Otherwise this
 * simply calls the FILTER_IN macro as normal.
 */
#define	FILTER_IN_NAN(old_val, new_val, d_t, lag) \
	do { \
		__typeof__(old_val) o = (old_val); \
		__typeof__(new_val) n = (new_val); \
		if (isnan(n)) \
			(old_val) = NAN; \
		else if (isnan(o)) \
			(old_val) = (new_val); \
		else \
			FILTER_IN(old_val, new_val, d_t, lag); \
	} while (0)

/*
 * Linearly interpolates old_val until it is equal to tgt. The current
 * time delta is d_t (in seconds). The interpolation speed is step/second.
 */
#define	FILTER_IN_LIN(old_val, tgt, d_t, step) \
	do { \
		double o = (old_val); \
		double t = (tgt); \
		double s; \
		if (isnan(o)) \
			o = t; \
		if (o < t) \
			s = (d_t) * (step); \
		else \
			s = (d_t) * (-(step)); \
		if ((o <= t && o + s > t) || (o >= t && o + s < t)) \
			(old_val) = t; \
		else \
			(old_val) += s; \
	} while (0)

#define	SCANF_STR_AUTOLEN_IMPL(_str_)	#_str_
#define	SCANF_STR_AUTOLEN(_str_)	SCANF_STR_AUTOLEN_IMPL(_str_)

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_SYSMACROS_H_ */
