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
 * Copyright 2021 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_SAFE_ALLOC_H_
#define	_ACF_UTILS_SAFE_ALLOC_H_

#include <string.h>
#include <stdlib.h>

#include "assert.h"
#include "sysmacros.h"

#ifdef	__cplusplus
extern "C" {
#endif

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

#define	ZERO_FREE(ptr) \
	do { \
		NOT_TYPE_ASSERT(ptr, void *); \
		NOT_TYPE_ASSERT(ptr, char *); \
		if ((ptr) != NULL) \
			memset((ptr), 0, sizeof (*(ptr))); \
		free(ptr); \
	} while (0)

#define	ZERO_FREE_N(ptr, num) \
	do { \
		NOT_TYPE_ASSERT(ptr, void *); \
		if ((ptr) != NULL) \
			memset((ptr), 0, sizeof (*(ptr)) * (num)); \
		free(ptr); \
	} while (0)

#define	BZERO(data)	memset((data), 0, sizeof (*(data)))

#define	SAFE_BZERO(data) \
	do { \
		if ((data) != NULL) \
			BZERO(data); \
	} while (0)

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_SAFE_ALLOC_H_ */
