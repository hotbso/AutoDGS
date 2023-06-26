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

#include <acfutils/core.h>
#include <acfutils/safe_alloc.h>

#include <stdlib.h>

/**
 * A string holding the current build version of libacfutils.
 * It's just a 7-character git revision number.
 */
const char *libacfutils_version = LIBACFUTILS_VERSION;

/**
 * Allocates `n' bytes of memory using libacfutils' allocator. This is mostly to
 * complete the functionality of `lacf_free'. In general, you won't ever need to
 * call this function.
 */
void *
lacf_malloc(size_t n)
{
	return (safe_malloc(n));
}

/**
 * Whenever libacfutils returns an allocated object (except for the safe_*alloc
 * functions) that you must free, use lacf_free to do so. Otherwise you risk
 * running into troubles with different allocators being used between compilers
 * (thanks Windows!).
 */
void
lacf_free(void *buf)
{
	free(buf);
}
