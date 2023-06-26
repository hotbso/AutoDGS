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
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * This is the master assertion checking machinery of libacfutils.
 *
 * The macros in this module are designed to provide error checking and
 * crash log generation. The majority of the time, you will be using the
 * `ASSERT*` family of macros, to create assertion checks. If the condition
 * in the macro argument fails, the check generates a crash, file + line
 * number reference and backtrace, all of which will be logged. After this,
 * the application exits. `ASSERT` macros are only compiled into your code
 * if the `DEBUG` macro is defined during compilation. If you want to
 * generate a an assertion check that is always compiled in, use the
 * `VERIFY*` family of macros.
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

/**
 * ASSERT() and VERIFY() are assertion test macros. If the condition
 * expression provided as the argument to the macro evaluates as non-true,
 * the program prints a debug message specifying exactly where and what
 * condition was violated, a stack backtrace and a dumps core by
 * calling LACF_CRASH() (which calls abort() on macOS/Linux and generates
 * an assertion failure exception on Windows).
 *
 * The difference between ASSERT and VERIFY is that ASSERT compiles to
 * a no-op unless `-DDEBUG` is provided to the compiler. VERIFY always
 * checks its condition and dumps if it is non-true.
 */
#define	VERIFY(x)	VERIFY_MSG(x, "%s", "")

/**
 * Same as the VERIFY() macro, but lets you pass a custom printf-like
 * format string with arguments, to append to the message "assertion
 * <condition> failed:". Use this if you need to provide more context
 * why the assertion check failed.
 *
 * Example usage of VERIFY_MSG():
 *```
 * VERIFY_MSG(is_valid(foo), "foo was set to %d", foo);
 *```
 */
#define	VERIFY_MSG(x, fmt, ...) \
	do { \
		if (COND_UNLIKELY(!(x))) { \
			log_impl(log_basename(__FILE__), __LINE__, \
			    "assertion \"%s\" failed: " fmt, #x, __VA_ARGS__); \
			LACF_CRASH(); \
		} \
	} while (0)

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
/**
 * Provides a more convenient macro for assertions checks of signed integer
 * comparisons ("3S" = 3 arguments, Signed integer). The first and last
 * argument are expected to be integer values, and the middle a comparison
 * operator, such as `==` or `>`, placed between the two operands. If the
 * comparison fails, this macro prints not only the condition that failed,
 * but also what the numerical values of the first and last argument were,
 * to aid in crash analysis. For example:
 * ```
 * int foo = 100, bar = 50;
 * VERIFY3S(foo, <, bar);
 * ```
 * will print "assertion foo < bar failed (100 < 50)".
 */
#define	VERIFY3S(x, op, y)	VERIFY3_impl(x, op, y, long, "%lu")

/**
 * \def VERIFY3U
 * Same as \ref VERIFY3S, but operates on unsigned integer values
 * ("3U" = 3 arguments, Unsigned integer).
 */
#if	IBM
#define	VERIFY3U(x, op, y)	\
	VERIFY3_impl(x, op, y, unsigned long long, "0x%I64x")
#else	/* !IBM */
#define	VERIFY3U(x, op, y)	\
	VERIFY3_impl(x, op, y, unsigned long long, "0x%llx")
#endif	/* !IBM */

/**
 * Same as \ref VERIFY3S, but operates on floating point and double
 * values ("3F" = 3 arguments, Floating point).
 */
#define	VERIFY3F(x, op, y)	VERIFY3_impl(x, op, y, double, "%f")
/**
 * Same as \ref VERIFY3S, but operates on pointer values
 * ("3P" = 3 arguments, Pointer).
 */
#define	VERIFY3P(x, op, y)	VERIFY3_impl(x, op, y, void *, "%p")
/**
 * Similar to \ref VERIFY3S, but only takes a single integer argument
 * and checks that it is zero.
 */
#define	VERIFY0(x)		VERIFY3S((x), ==, 0)
/**
 * Hard-crash generator. This always crashes if it is reached. Use this
 * to mark invalid branches of conditional/case statements. This will
 * generate a log message that says "Internal error".
 */
#define	VERIFY_FAIL()		\
	do { \
		log_impl(log_basename(__FILE__), __LINE__, "Internal error"); \
		LACF_CRASH(); \
	} while (0)

/**
 * \def ASSERT
 * Same as \ref VERIFY, but only active when compiling with `DEBUG` defined.
 * \def ASSERT3S
 * Same as \ref VERIFY3S, but only active when compiling with `DEBUG` defined.
 * \def ASSERT3U
 * Same as \ref VERIFY3U, but only active when compiling with `DEBUG` defined.
 * \def ASSERT3F
 * Same as \ref VERIFY3F, but only active when compiling with `DEBUG` defined.
 * \def ASSERT3P
 * Same as \ref VERIFY3P, but only active when compiling with `DEBUG` defined.
 * \def ASSERT0
 * Same as \ref VERIFY0, but only active when compiling with `DEBUG` defined.
 * \def ASSERT_MSG
 * Same as \ref VERIFY_MSG, but only active when compiling with `DEBUG` defined.
 */

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
#define	ASSERT3S(x, op, y)	do { UNUSED(x); UNUSED(y); } while (0)
#define	ASSERT3U(x, op, y)	do { UNUSED(x); UNUSED(y); } while (0)
#define	ASSERT3F(x, op, y)	do { UNUSED(x); UNUSED(y); } while (0)
#define	ASSERT3P(x, op, y)	do { UNUSED(x); UNUSED(y); } while (0)
#define	ASSERT0(x)		UNUSED(x)
#define	ASSERT_MSG(x, fmt, ...)	UNUSED(x)
#endif	/* !DEBUG */

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_ASSERT_H_ */
