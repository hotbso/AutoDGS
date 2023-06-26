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
/** \file */

#ifndef	_ACFUTILS_CORE_H_
#define	_ACFUTILS_CORE_H_

#include <stdlib.h>

#include "libconfig.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * \def UNUSED_ATTR
 * Attribute which marks a function as unused. This is designed to avoid
 * warnings & errors from compiler code analyzers for unused static
 * functions. Place this ahead of a function declaration or definition,
 * to mark the function as purposely unused.
 */
/**
 * \def UNUSED
 * Attribute which stops a compiler from complaining when a variable
 * isn't used.
 */
/**
 * \def WARN_UNUSED_RES_ATTR
 * Attribute which if used on a function, makes the compiler emit a
 * warning if the return value of a function is ignored.
 */
/**
 * \def PACKED_ATTR
 * Attribute which marks a structure as having a packed memory layout
 * (i.e. the compiler isn't allowed to insert padding). Place at the
 * start of a structure definition,
 * e.g. `PACKED_ATTR struct structName { ... };`
 */
/**
 * \def LACF_DESTROY
 * Calls lacf_free() on `ptr` and also then sets the pointer to `NULL`
 * to force a null-pointer-dereference and hard crash if an attempt is
 * made to use the pointer after the `LACF_DESTROY` operation.
 */
/**
 * \def ARRAY_NUM_ELEM
 * Given a fixed-size array, evaluates to the number of elements in the
 * array. Useful for automatically determining the capacity of an array
 * for functions which take a count-of-elements argument.
 */

#if	defined(__GNUC__) || defined(__clang__)
#define	UNUSED_ATTR		__attribute__((unused))
#define	WARN_UNUSED_RES_ATTR	__attribute__((warn_unused_result))
#if	IBM && defined(__GNUC__) && __GNUC__ < 11
#define	PACKED_ATTR		__attribute__((__packed__, gcc_struct))
#else
#define	PACKED_ATTR		__attribute__((__packed__))
#endif
#else
#define	UNUSED_ATTR
#define	WARN_UNUSED_RES_ATTR
#define	PACKED_ATTR
#endif
#define	UNUSED(x)	(void)(x)

#define	ACFSYM(__sym__)	__libacfutils_ ## __sym__

#if	IBM && (!defined(__MINGW32__) || defined(ACFUTILS_DLL))
#define	API_EXPORT	__declspec(dllexport)
#ifdef	ACFUTILS_BUILD
#define	API_EXPORT_DATA	__declspec(dllexport) extern
#else
#define	API_EXPORT_DATA	__declspec(dllimport) extern
#endif
#else	/* !IBM || (defined(__MINGW32__) && !defined(ACFUTILS_DLL)) */
#define	API_EXPORT
#define	API_EXPORT_DATA	extern
#endif	/* !IBM || (defined(__MINGW32__) && !defined(ACFUTILS_DLL)) */

#ifdef	__cplusplus
# ifndef	restrict
#  if		defined(_MSC_VER)
#   define	restrict	__restrict
#  else
#   define	restrict	__restrict__
#  endif
# endif		/* !defined(restrict) */
#elif	__STDC_VERSION__ < 199901L
# ifndef	restrict
#  if	defined(__GNUC__) || defined(__clang__) || defined(_MSC_VER)
#   define	restrict	__restrict
#  else
#   define	restrict
#  endif
# endif		/* !defined(restrict) */
# ifndef	inline
#  if		defined(_MSC_VER)
#   define	inline	__inline
#  else
#   define	inline
#  endif
# endif		/* !defined(inline) */
#endif	/* __STDC_VERSION__ < 199901L */

API_EXPORT_DATA const char *libacfutils_version;

API_EXPORT void *lacf_malloc(size_t n);
#define	LACF_DESTROY(ptr) \
	do { \
		if ((ptr) != NULL) { \
			lacf_free((ptr)); \
			(ptr) = NULL; \
		} \
	} while (0)
API_EXPORT void lacf_free(void *buf);

/*
 * Simple shorthand for calculating number of elements in an array at
 * compile time. Useful for arrays with flexible sizing.
 */
#define	ARRAY_NUM_ELEM(_array) (sizeof (_array) / sizeof (*(_array)))

#ifdef	__cplusplus
}
#endif

#endif	/* _ACFUTILS_CORE_H_ */
