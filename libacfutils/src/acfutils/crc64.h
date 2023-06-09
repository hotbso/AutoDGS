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

#ifndef	_ACF_UTILS_CRC64_H_
#define	_ACF_UTILS_CRC64_H_

#include <stdlib.h>
#include <stdint.h>

#include "acfutils/assert.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	crc64_init		ACFSYM(crc64_init)
API_EXPORT void crc64_init();

static inline void
crc64_state_init(uint64_t *crc)
{
	ASSERT(crc != NULL);
	*crc = (uint64_t)-1;
}
#define	crc64_append		ACFSYM(crc64_append)
API_EXPORT uint64_t crc64_append(uint64_t crc, const void *input, size_t sz);
#define	crc64			ACFSYM(crc64)
API_EXPORT uint64_t crc64(const void *input, size_t sz);

#define	crc64_srand		ACFSYM(crc64_srand)
API_EXPORT void crc64_srand(uint64_t seed);
#define	crc64_rand		ACFSYM(crc64_rand)
API_EXPORT uint64_t crc64_rand(void);
#define	crc64_rand_fract	ACFSYM(crc64_rand_fract)
API_EXPORT double crc64_rand_fract(void);
#define	crc64_rand_normal	ACFSYM(crc64_rand_normal)
API_EXPORT double crc64_rand_normal(double sigma);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_CRC64_H_ */
