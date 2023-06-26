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
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/** \file */

#ifndef	_ACF_UTILS_SAFE_ALLOC_H_
#define	_ACF_UTILS_SAFE_ALLOC_H_

#include <string.h>
#include <stdlib.h>

#include "assert.h"
#include "sysmacros.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * Provides a front-end to your compiler's malloc() function, but with
 * automated allocation success checking. In general, you can just replace
 * any calls to malloc() with calls to safe_malloc() and it will work
 * exactly the same, except in cases where the allocation fails. If the
 * allocation fails, this generates an assertion failure crash with a
 * diagnostic message telling you how many byte failed to be allocated.
 *
 * N.B. unlike any allocation that you might get returned from within
 * libacfutils, which you should free using lacf_free(), the pointer
 * returned from this function is allocated your compiler's allocator.
 * Thus you MUST use your normal free() function to free this one.
 */
static inline void *
safe_malloc(size_t size)
{
	void *p = malloc(size);
	if (size > 0) {
		VERIFY_MSG(p != NULL, "Cannot allocate %lu bytes: "
		    "out of memory", (long unsigned)size);
	}
	return (p);
}

/**
 * Same as safe_malloc(), except it calls calloc() on the back-end and
 * behaves exactly the standard calloc() function.
 */
static inline void *
safe_calloc(size_t nmemb, size_t size)
{
	void *p = calloc(nmemb, size);
	if (nmemb > 0 && size > 0) {
		VERIFY_MSG(p != NULL, "Cannot allocate %lu bytes: "
		    "out of memory", (long unsigned)(nmemb * size));
	}
	return (p);
}

/**
 * Same as safe_malloc(), except it calls realloc() on the back-end and
 * behaves exactly the standard realloc() function.
 */
static inline void *
safe_realloc(void *oldptr, size_t size)
{
	void *p = realloc(oldptr, size);
	if (size > 0) {
		VERIFY_MSG(p != NULL, "Cannot allocate %lu bytes: "
		    "out of memory", (long unsigned)size);
	}
	return (p);
}

/**
 * Provides an allocation-safe version of strdup(). If the allocation
 * of the required number of bytes fails, this trips an assertion check
 * and causes the application to crash due to having run out of memory.
 */
static inline char *
safe_strdup(const char *str2)
{
	char *str = strdup(str2);
	if (str2 != NULL) {
		VERIFY_MSG(str != NULL, "Cannot allocate %lu bytes: "
		    "out of memory", (long unsigned)strlen(str2) + 1);
	}
	return (str);
}

/**
 * Concatenates `str` onto the end of `buf`, enlarging it as necessary.
 * If memory cannot be allocated to hold the new string, this crashes
 * with an assertion failure. This is using the safe_malloc() machinery
 * underneath, so all the same rules apply.
 * @param buf Buffer to append to. If this argument is `NULL`, a new
 *	buffer is allocated. Otherwise, the buffer is safe_realloc()d to
 *	contain the new concatenated string.
 * @param str The input string to append onto the end of `buf`. This must
 *	NOT be `NULL`.
 * @return The new combined buffer holding a concatenation of `buf` and
 *	`str`. Please note that since reallocation may occur, you must not
 *	reuse the old `buf` pointer value after calling safe_append_realloc().
 *	You should reassign the pointer using the return value of this
 *	function.
 */
static inline char *
safe_append_realloc(char *buf, const char *str)
{
	char *newbuf;

	ASSERT(str != NULL);
	if (buf == NULL)
		return (safe_strdup(str));
	newbuf = (char *)safe_realloc(buf, strlen(buf) + strlen(str) + 1);
	memcpy(&newbuf[strlen(newbuf)], str, strlen(str) + 1);
	return (newbuf);
}

/**
 * If `ptr` is not a `NULL` pointer, the contained data is first zeroed,
 * and subsequently deallocated using free(). This is useful for making
 * sure nothing remains of a buffer after freeing, thus preventing
 * potentially attempting to read its contents and seeing valid data
 * (use-after-free).
 *
 * N.B. this macro relies on `sizeof` returning the correct size of this
 * data buffer, so it should only be used on single struct buffers, not
 * arrays.
 */
#define	ZERO_FREE(ptr) \
	do { \
		NOT_TYPE_ASSERT(ptr, void *); \
		NOT_TYPE_ASSERT(ptr, char *); \
		if ((ptr) != NULL) \
			memset((ptr), 0, sizeof (*(ptr))); \
		free(ptr); \
	} while (0)
/**
 * Same as ZERO_FREE(), but also sets `ptr` to NULL after freeing.
 */
#define	DESTROY_FREE(ptr) \
	do { \
		ZERO_FREE(ptr); \
		(ptr) = NULL; \
	} while (0)

/**
 * Same as \ref ZERO_FREE, but takes an explicit array element count
 * argument. This is the variant of the \ref ZERO_FREE macro to be used
 * on arrays of objects, rather than single objects.
 */
#define	ZERO_FREE_N(ptr, num) \
	do { \
		NOT_TYPE_ASSERT(ptr, void *); \
		if ((ptr) != NULL) \
			memset((ptr), 0, sizeof (*(ptr)) * (num)); \
		free(ptr); \
	} while (0)
/**
 * Same as ZERO_FREE_N(), but also sets `ptr` to NULL after freeing.
 */
#define	DESTROY_FREE_N(ptr, num) \
	do { \
		ZERO_FREE_N((ptr), (num)); \
		(ptr) = NULL; \
	while (0)

/**
 * Performs a zeroing of the `data` buffer. Please note that `sizeof`
 * must return the correct size of the data to be zeroed. Thus this
 * is intended to be used fixed-size arrays.
 */
#define	BZERO(data)	memset((data), 0, sizeof (*(data)))

/**
 * If `data` is not `NULL`, performs a \ref BZERO on the data.
 */
#define	SAFE_BZERO(data) \
	do { \
		if ((data) != NULL) \
			BZERO(data); \
	} while (0)

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_SAFE_ALLOC_H_ */
