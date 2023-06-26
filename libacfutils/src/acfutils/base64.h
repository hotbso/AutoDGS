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

#ifndef _ACFUTILS_BASE64_H_
#define _ACFUTILS_BASE64_H_

#include <stdlib.h>
#include <stdint.h>

#include "core.h"

/**
 * Provides a precise computation of how much base64 data will be
 * generated when __raw_size__ many binary bytes a base64-encoded.
 */
#define BASE64_ENC_SIZE(__raw_size__) ((((__raw_size__) + 2) / 3) * 4)

/**
 * Provides an estimate of how much raw data results from decoding of
 * __enc_size__ many base-64 encoded bytes. Unlike the macro above, this
 * macro may slightly (by up to 2 bytes) overestimate the actual size of
 * the decoded data, so be sure to always check the return value of
 * base64_decode to avoid accessing uninitialized data.
 */
#define BASE64_DEC_SIZE(__enc_size__) (((__enc_size__) / 4) * 3)

API_EXPORT size_t lacf_base64_encode(const uint8_t *raw, size_t raw_size,
    uint8_t *encoded);
API_EXPORT size_t lacf_base64_encode2(const uint8_t *raw, size_t raw_size,
    uint8_t *encoded, int mod);
API_EXPORT ssize_t lacf_base64_decode(const uint8_t *encoded,
    size_t encoded_size, uint8_t *raw);
API_EXPORT ssize_t lacf_base64_decode2(const uint8_t *encoded,
    size_t encoded_size, uint8_t *raw, int mod);

#endif /* _ACFUTILS_BASE64_H_ */
