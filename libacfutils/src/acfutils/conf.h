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
 * This is a general-purpose configuration store. It's really just a
 * key-value pair dictionary that can be read from and written to a file.
 *
 * The file format is very simple, consisting of a simple sequence of
 * lines like the following:
 *
 * key = value
 *
 * In addition to being able to return the full-text values of keys, this
 * set functions also allows you to easily parse the data in a variety of
 * formats (integers, floats, booleans, etc.). The file format also allows
 * for comments, so it is usable as a user-written configuration parser.
 * Lines beginning with "#" or "--" are automatically skipped.
 */

#ifndef	_ACFUTILS_CONF_H_
#define	_ACFUTILS_CONF_H_

#include <stdio.h>
#include <stdint.h>

#if	__STDC_VERSION__ >= 199901L
#include <stdbool.h>
#endif

#include "helpers.h"
#include "types.h"
#include "avl.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct conf conf_t;

API_EXPORT conf_t *conf_create_empty(void);
API_EXPORT conf_t *conf_create_copy(const conf_t *conf2);
API_EXPORT void conf_free(conf_t *conf);

API_EXPORT conf_t *conf_read_file(const char *filename, int *errline);
API_EXPORT conf_t *conf_read(FILE *fp, int *errline);
API_EXPORT conf_t *conf_read2(void *fp, int *errline, bool_t compressed);
API_EXPORT conf_t *conf_read_buf(const void *buf, size_t cap, int *errline);

API_EXPORT bool_t conf_write_file(const conf_t *conf, const char *filename);
API_EXPORT bool_t conf_write_file2(const conf_t *conf, const char *filename,
    bool_t compressed);
API_EXPORT bool_t conf_write(const conf_t *conf, FILE *fp);
API_EXPORT size_t conf_write_buf(const conf_t *conf, void *buf, size_t cap);

API_EXPORT void conf_merge(const conf_t *conf_from, conf_t *conf_to);

API_EXPORT bool_t conf_get_str(const conf_t *conf, const char *key,
    const char **value);
API_EXPORT bool_t conf_get_i(const conf_t *conf, const char *key,
    int *value);
API_EXPORT bool_t conf_get_lli(const conf_t *conf, const char *key,
    long long *value);
API_EXPORT bool_t conf_get_f(const conf_t *conf, const char *key,
    float *value);
API_EXPORT bool_t conf_get_d(const conf_t *conf, const char *key,
    double *value);
API_EXPORT bool_t conf_get_da(const conf_t *conf, const char *key,
    double *value);
API_EXPORT bool_t conf_get_b(const conf_t *conf, const char *key,
    bool_t *value);
API_EXPORT size_t conf_get_data(const conf_t *conf, const char *key,
    void *buf, size_t cap);

API_EXPORT void conf_set_str(conf_t *conf, const char *key, const char *value);
API_EXPORT void conf_set_i(conf_t *conf, const char *key, int value);
API_EXPORT void conf_set_lli(conf_t *conf, const char *key, long long value);
API_EXPORT void conf_set_f(conf_t *conf, const char *key, float value);
API_EXPORT void conf_set_d(conf_t *conf, const char *key, double value);
API_EXPORT void conf_set_da(conf_t *conf, const char *key, double value);
API_EXPORT void conf_set_b(conf_t *conf, const char *key, bool_t value);
API_EXPORT void conf_set_data(conf_t *conf, const char *key,
    const void *buf, size_t sz);

API_EXPORT bool_t conf_get_str_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), const char **value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT bool_t conf_get_i_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), int *value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT bool_t conf_get_lli_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), long long *value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT bool_t conf_get_f_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), float *value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT bool_t conf_get_d_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), double *value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT bool_t conf_get_da_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), double *value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT bool_t conf_get_b_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), bool_t *value, ...) PRINTF_ATTR2(2, 4);
API_EXPORT size_t conf_get_data_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), void *buf, size_t cap, ...)
    PRINTF_ATTR2(2, 5);

API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_str_v(conf_t *conf, PRINTF_FORMAT(const char *fmt),
    const char *value, ...);
API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_i_v(conf_t *conf, PRINTF_FORMAT(const char *fmt),
    int value, ...);
API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_lli_v(conf_t *conf, PRINTF_FORMAT(const char *fmt),
    long long value, ...);
API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_f_v(conf_t *conf, PRINTF_FORMAT(const char *fmt),
    double value, ...);
API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_d_v(conf_t *conf, PRINTF_FORMAT(const char *fmt),
    double value, ...);
API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_da_v(conf_t *conf, PRINTF_FORMAT(const char *fmt),
    double value, ...);
API_EXPORT PRINTF_ATTR2(2, 4) void conf_set_b_v(conf_t *conf,
    PRINTF_FORMAT(const char *fmt), bool_t value, ...);
API_EXPORT PRINTF_ATTR2(2, 5) void conf_set_data_v(conf_t *conf,
    PRINTF_FORMAT(const char *fmt), const void *buf, size_t sz, ...);

#if	__STDC_VERSION__ >= 199901L || defined(__cplusplus)
API_EXPORT bool conf_get_b2(const conf_t *conf, const char *key,
    bool *value);
API_EXPORT void conf_set_b2(conf_t *conf, const char *key, bool value);
API_EXPORT PRINTF_ATTR2(2, 4) bool conf_get_b2_v(const conf_t *conf,
    PRINTF_FORMAT(const char *fmt), bool *value, ...);
/*
 * We can't declare a native bool-type conf_set_b2_v here, because bool
 * isn't a formally declared type. This results in default argument
 * promotion, which is undefined behavior in varargs.
 */
#define	conf_set_b2_v	conf_set_b_v
#endif	/* __STDC_VERSION__ >= 199901L || defined(__cplusplus) */

API_EXPORT bool_t conf_walk(const conf_t *conf, const char **key,
    const char **value, void **cookie);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACFUTILS_CONF_H_ */
