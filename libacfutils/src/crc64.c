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
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <math.h>

#include <acfutils/crc64.h>

/*
 * A generic CRC64 implementation from OpenSolaris. Be sure to call
 * crc64_init before using it. Then just call crc64() and pass it the
 * data you want checksummed. Also includes a fast & light-weight
 * portable pseudo random number generator.
 */

#define	CRC64_POLY	0xC96C5795D7870F42ULL	/* ECMA-182, reflected form */

static uint64_t crc64_table[256];
static uint64_t rand_seed = 0;

void
crc64_init(void)
{
	for (int i = 0; i < 256; i++) {
		uint64_t *ct;
		int j;
		for (ct = crc64_table + i, *ct = i, j = 8; j > 0; j--)
			*ct = (*ct >> 1) ^ (-(*ct & 1) & CRC64_POLY);
	}
}

/*
 * Computes the CRC64 checksum of a block of input data.
 */
uint64_t
crc64(const void *input, size_t sz)
{
	uint64_t crc;

	crc64_state_init(&crc);
	return (crc64_append(crc, input, sz));
}

uint64_t
crc64_append(uint64_t crc, const void *input, size_t sz)
{
	const uint8_t *in_bytes = input;

	ASSERT3U(crc64_table[128], ==, CRC64_POLY);
	for (size_t i = 0; i < sz; i++)
		crc = (crc >> 8) ^ crc64_table[(crc ^ in_bytes[i]) & 0xFF];

	return (crc);
}

/*
 * Initializes the CRC64-based pseudo random number generator. Pass in some
 * random seed (e.g. current microclock() usually does nicely). Obviously
 * you only want to call this once in your app.
 */
void
crc64_srand(uint64_t seed)
{
	rand_seed = seed;
}

/*
 * Grabs a random 64-bit number from the PRNG. This function isn't
 * thread-safe, so take care not to rely on its output being super-duper
 * unpredictable in multi-threade apps. You shouldn't be relying on it
 * for anything more than lightweight randomness duties which need to be
 * fast above everything else.
 *
 * DO NOT use this for cryptographically secure randomness operations
 * (e.g. generating encryption key material). See osrand.c for a
 * high-quality PRNG.
 */
uint64_t
crc64_rand(void)
{
	rand_seed = crc64(&rand_seed, sizeof (rand_seed));
	return (rand_seed);
}

/*
 * Returns a random number from the PRNG, but instead of returning a 64-bit
 * integer between 0 and UINT64_MAX, this function returns a double value
 * from 0.0 to 1.0 (inclusive).
 */
double
crc64_rand_fract(void)
{
	return (crc64_rand() / (double)UINT64_MAX);
}

/*
 * Returns a random number using a normal distribution with standard
 * deviation equal to sigma.
 */
double
crc64_rand_normal(double sigma)
{
	double x = crc64_rand_fract();
	double y = crc64_rand_fract();
	double z = sqrt(-2 * log(x)) * cos(2 * M_PI * y);
	return (sigma * z);
}
