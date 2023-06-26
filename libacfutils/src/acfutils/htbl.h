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
 * This module implements a simple general-purpose hash table.
 * @note The hash table functionality is dependent on the CRC64 subsystem,
 *	so be sure to call crc64_init() before initializing the first hash
 *	table.
 * @see htbl_t
 */

#ifndef	_ACFUTILS_HTBL_H_
#define	_ACFUTILS_HTBL_H_

#include <stdlib.h>
#include <stdint.h>

#include "types.h"
#include "list.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * Hash table structure. This is the object you want to allocate and
 * subsequently initialize using htbl_create(). Use htbl_destroy() to
 * deinitialize a hash table. Use htbl_set(), htbl_remove() and
 * htbl_lookup() to respectively add, remove and look up hash table
 * entries. The hash table supports storing duplicate entries for the
 * same hash value.
 * @see htbl_create()
 * @see htbl_destroy()
 * @see htbl_set()
 * @see htbl_remove()
 * @see htbl_lookup()
 */
typedef struct {
	size_t		tbl_sz;
	size_t		key_sz;
	list_t		*buckets;
	size_t		num_values;
	bool_t		multi_value;
} htbl_t;

typedef struct htbl_multi_value_s htbl_multi_value_t;

API_EXPORT void htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz,
    bool_t multi_value);
void htbl_destroy(htbl_t *htbl);
API_EXPORT void htbl_empty(htbl_t *htbl,
    void (*func)(void *value, void *userinfo), void *userinfo);
API_EXPORT size_t htbl_count(const htbl_t *htbl);

API_EXPORT void htbl_set(htbl_t *htbl, const void *key, void *value);
API_EXPORT void htbl_remove(htbl_t *htbl, const void *key, bool_t nil_ok);
API_EXPORT void htbl_remove_multi(htbl_t *htbl, const void *key,
    htbl_multi_value_t *list_item);

API_EXPORT void *htbl_lookup(const htbl_t *htbl, const void *key);
API_EXPORT const list_t *htbl_lookup_multi(const htbl_t *htbl, const void *key);
API_EXPORT void *htbl_value_multi(htbl_multi_value_t *mv);
/**
 * Legacy backwards compatibility macro that just invokes htbl_value_multi().
 */
#define	HTBL_VALUE_MULTI(x)	htbl_value_multi(x)

API_EXPORT void htbl_foreach(const htbl_t *htbl,
    void (*func)(const void *key, void *value, void *userinfo), void *userinfo);

API_EXPORT char *htbl_dump(const htbl_t *htbl, bool_t printable_keys);

/**
 * Utility function that can be passed in the second argument of
 * htbl_empty() if your values only require a standard C `free()` call
 * using your own heap allocator. You can use this as a shorthand to
 * freeing all the values in a hash table, instead of having to write
 * a custom callback every time.
 * @note This **only** calls `free()` and does no other deinitialization
 *	of the memory pointed to by the hash table value. Do NOT use
 *	this if your values need more complex deinitialization.
 */
UNUSED_ATTR static void
htbl_free(void *obj, void *unused)
{
	UNUSED(unused);
	free(obj);
}

#ifdef	__cplusplus
}
#endif

#endif	/* _ACFUTILS_HTBL_H_ */
