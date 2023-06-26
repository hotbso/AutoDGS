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
 * Copyright (c) 2005, 2010, Oracle and/or its affiliates. All rights reserved.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * A generic CRC64 implementation from OpenSolaris. Be sure to call
 * crc64_init() before using it. Then just call crc64() and pass it the
 * data you want checksummed. Also includes a fast & light-weight
 * portable pseudo random number generator.
 */

#ifndef	_ACF_UTILS_CRC64_H_
#define	_ACF_UTILS_CRC64_H_

#include <stdlib.h>
#include <stdint.h>

#include "acfutils/assert.h"
#include "acfutils/core.h"

#ifdef	__cplusplus
extern "C" {
#endif

API_EXPORT void crc64_init();

/**
 * Initializes the starting CRC64 value for subsequent calls to crc64_append().
 */
static inline void
crc64_state_init(uint64_t *crc)
{
	ASSERT(crc != NULL);
	*crc = (uint64_t)-1;
}
API_EXPORT uint64_t crc64_append(uint64_t crc, const void *input, size_t sz);
API_EXPORT uint64_t crc64(const void *input, size_t sz);

API_EXPORT void crc64_srand(uint64_t seed);
API_EXPORT uint64_t crc64_rand(void);
API_EXPORT double crc64_rand_fract(void);
API_EXPORT double crc64_rand_normal(double sigma);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_CRC64_H_ */
