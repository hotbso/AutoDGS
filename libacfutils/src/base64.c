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
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

/*
 * This is a simple and fast Base64 encoder/decoder implementation. In
 * order to encode, call base64_encode. Conversely, to decode, use
 * base64_decode. Be sure to use the base64_[ENC|DEC]_SIZE macros to prepare
 * the buffers into which these functions write.
 */

#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <sys/types.h>

#include "acfutils/base64.h"

/* Base64 encoding table */
static const uint8_t *base64_enc_table = (uint8_t *)
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static const uint8_t *base64_enc_table_mod = (uint8_t *)
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789.~";

/**
 * Base64 decoding table - to be initialized by the first call to
 * base64_decode. We fill it here with 'x' values with the first byte being
 * 'U' to signal that the table is uninitialized.
 */
static uint8_t base64_dec_table[256] = {
  'U', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x'
};

static uint8_t base64_dec_table_mod[256] = {
  'U', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
  'x', 'x', 'x', 'x'
};

/**
 * Front-end to base64_encode2 with mod set to zero.
 */
size_t
lacf_base64_encode(const uint8_t *raw, size_t raw_size, uint8_t *encoded)
{
  return (lacf_base64_encode2(raw, raw_size, encoded, 0));
}

/**
 * Encodes a string of bytes into a Base64 encoding.
 *
 * @param raw The input buffer of raw bytes.
 * @param raw_size Number of bytes in `raw'.
 * @param encoded Buffer where the encoded Base64 bytes will be written.
 *      This buffer must contain at least BASE64_ENC_SIZE(raw_size)
 *      bytes.
 *
 * @return Number of bytes written to `encoded'. This is always equal
 *      to BASE64_ENC_SIZE(raw_size).
 */
size_t
lacf_base64_encode2(const uint8_t *raw, size_t raw_size, uint8_t *encoded,
    int mod)
{
  size_t i, j;
  const uint8_t *enc_table = mod ? base64_enc_table_mod : base64_enc_table;
  const char term_char = mod ? '-' : '=';

  /* first encode all available full 3-byte blocks */
  for (i = j = 0; i + 2 < raw_size; i+= 3, j += 4)
    {
      encoded[j] = enc_table[raw[i] >> 2];
      encoded[j + 1] = enc_table[((raw[i] & 3) << 4) | (raw[i + 1] >> 4)];
      encoded[j + 2] = enc_table[((raw[i + 1] & 0xf) << 2) | raw[i + 2] >> 6];
      encoded[j + 3] = enc_table[(raw[i + 2] & 0x3f)];
    }

  /* now handle the special cases for the last block */
  if (i + 1 < raw_size)
    {
      /* 2 bytes in the last block */
      encoded[j] = enc_table[raw[i] >> 2];
      encoded[j + 1] = enc_table[((raw[i] & 3) << 4) | (raw[i + 1] >> 4)];
      encoded[j + 2] = enc_table[(raw[i + 1] & 0xf) << 2];
      encoded[j + 3] = term_char;
      j += 4;
    }
  else if (i < raw_size)
    {
      /* 1 byte in the last */
      encoded[j] = enc_table[raw[i] >> 2];
      encoded[j + 1] = enc_table[(raw[i] & 3) << 4];
      encoded[j + 2] = term_char;
      encoded[j + 3] = term_char;
      j += 4;
    }
  /*
   * In case the last block contained 3 bytes, it has been already taken
   * care of in the main for loop above.
   */

  return j;
}

/**
 * Internal function called by base64_decode for the first time to
 * initialize the base64_dec_table. Although initialization of this
 * buffer may not be done in a thread-safe manner, this function
 * is structured in such a way that even multiple threads executing
 * it concurrently will always arrive at the correct result (the result
 * of the initialization is always identical), so using base64_decode
 * in multi-threaded programs is ultimately safe.
 */
static inline void
base64_init_dec_tables (void)
{
  int i;

  for (i = 0; i < 64; i++)
    base64_dec_table[base64_enc_table[i]] = i;
  for (i = 0; i < 64; i++)
    base64_dec_table_mod[base64_enc_table_mod[i]] = i;

  // mark the tables as ready
  base64_dec_table[0] = '|';
  base64_dec_table_mod[0] = '|';
}

/**
 * Front-end to base64_decode2 with mod set to zero.
 */
ssize_t
lacf_base64_decode(const uint8_t *encoded, size_t encoded_size, uint8_t *raw)
{
  return (lacf_base64_decode2(encoded, encoded_size, raw, 0));
}

/**
 * Decodes a Base64 encoded string, previously encoded with base64_encode.
 *
 * @param encoded The encoded Base64 bytes.
 * @param encoded_size Number of bytes in `encoded'.
 * @param raw Output buffer to be filled with decoded bytes. This buffer
 *      must contain at least BASE64_DEC_SIZE(encoded_size) bytes.
 *
 * @return The number of actual message that were encoded in the Base64
 *      buffer, or -1 if the input was malformed.
 */
ssize_t
lacf_base64_decode2(const uint8_t *encoded, size_t encoded_size, uint8_t *raw,
    int mod)
{
  const uint8_t *dec_table = mod ? base64_dec_table_mod : base64_dec_table;
  const char term_char = mod ? '-' : '=';

  if (base64_dec_table[0] == 'U')
    base64_init_dec_tables ();

  size_t i, j;

  /* bad size */
  if (encoded_size & 3)
    return -1;

  /* nothing to decode */
  if (!encoded_size)
    return 0;

  /* decode Base64 blocks up to the last one */
  for (i = j = 0; i + 4 < encoded_size; i+= 4, j += 3)
    {
      if (dec_table[encoded[i + 0]] == 'x' ||
          dec_table[encoded[i + 1]] == 'x' ||
          dec_table[encoded[i + 2]] == 'x' ||
          dec_table[encoded[i + 3]] == 'x')
        return -1;

      raw[j] = (dec_table[encoded[i]] << 2) | (dec_table[encoded[i + 1]] >> 4);
      raw[j + 1] = (dec_table[encoded[i + 1]] << 4) |
                   (dec_table[encoded[i + 2]] >> 2);
      raw[j + 2] = (dec_table[encoded[i + 2]] << 6) |
                   (dec_table[encoded[i + 3]]);
    }

  /* now handle the special ending cases for the last Base64 block */
  if (dec_table[encoded[i + 0]] != 'x' &&
      dec_table[encoded[i + 1]] != 'x' &&
      encoded[i + 2] == term_char && encoded[i + 3] == term_char)
    {
      /* single-decoded-byte end block */
      raw[j] = (dec_table[encoded[i]] << 2) |
               (dec_table[encoded[i + 1]] >> 4);
      raw[j + 1] = 0;
      raw[j + 2] = 0;
      j += 1;
    }
  else if (dec_table[encoded[i + 0]] != 'x' &&
           dec_table[encoded[i + 1]] != 'x' &&
           dec_table[encoded[i + 2]] != 'x' && encoded[i + 3] == term_char)
    {
      /* two-decoded-bytes end block */
      raw[j] = (dec_table[encoded[i]] << 2) | (dec_table[encoded[i + 1]] >> 4);
      raw[j + 1] = (dec_table[encoded[i + 1]] << 4) |
                   (dec_table[encoded[i + 2]] >> 2);
      raw[j + 2] = 0;
      j += 2;
    }
  else if (dec_table[encoded[i + 0]] != 'x' &&
           dec_table[encoded[i + 1]] != 'x' &&
           dec_table[encoded[i + 2]] != 'x' &&
           dec_table[encoded[i + 3]] != 'x')
    {
      /* three-decoded-bytes end block */
      raw[j] = (dec_table[encoded[i]] << 2) |
               (dec_table[encoded[i + 1]] >> 4);
      raw[j + 1] = (dec_table[encoded[i + 1]] << 4) |
                   (dec_table[encoded[i + 2]] >> 2);
      raw[j + 2] = (dec_table[encoded[i + 2]] << 6) |
                   (dec_table[encoded[i + 3]]);
      j += 3;
    }
  else
    /* error decoding last block */
    return -1;

  return j;
}
