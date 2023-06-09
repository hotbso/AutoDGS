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
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "acfutils/crc64.h"
#include "acfutils/helpers.h"
#include "acfutils/htbl.h"

static inline uint64_t
H(const void *p, size_t l)
{
	return (crc64(p, l));
}

void
htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz, int multi_value)
{
	ASSERT(key_sz != 0);
	ASSERT(tbl_sz != 0);
	ASSERT(tbl_sz <= ((size_t)1 << ((sizeof(size_t) * 8) - 1)));

	memset(htbl, 0, sizeof (*htbl));
	/* round table size up to nearest multiple of 2 */
	htbl->tbl_sz = P2ROUNDUP(tbl_sz);
	htbl->buckets = malloc(sizeof (*htbl->buckets) * htbl->tbl_sz);
	ASSERT(htbl->buckets != NULL);
	for (size_t i = 0; i < htbl->tbl_sz; i++)
		list_create(&htbl->buckets[i], sizeof (htbl_bucket_item_t),
		    offsetof(htbl_bucket_item_t, bucket_node));
	htbl->key_sz = key_sz;
	htbl->multi_value = multi_value;
}

void
htbl_destroy(htbl_t *htbl)
{
	ASSERT(htbl->num_values == 0);
	ASSERT(htbl->buckets != NULL);
	for (size_t i = 0; i < htbl->tbl_sz; i++)
		list_destroy(&htbl->buckets[i]);
	free(htbl->buckets);
}

static void
htbl_empty_multi_item(htbl_bucket_item_t *item, void (*func)(void *, void *),
    void *arg)
{
	for (htbl_multi_value_t *mv = list_head(&item->multi.list); mv;
	    mv = list_head(&item->multi.list)) {
		if (func)
			func(mv->value, arg);
		list_remove_head(&item->multi.list);
		free(mv);
	}
	list_destroy(&item->multi.list);
}

void
htbl_empty(htbl_t *htbl, void (*func)(void *, void *), void *arg)
{
	if (htbl->num_values == 0)
		return;

	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		for (htbl_bucket_item_t *item = list_head(&htbl->buckets[i]);
		    item; item = list_head(&htbl->buckets[i])) {
			if (htbl->multi_value)
				htbl_empty_multi_item(item, func, arg);
			else if (func)
				func(item->value, arg);
			list_remove_head(&htbl->buckets[i]);
			free(item);
		}
	}
	htbl->num_values = 0;
}

size_t
htbl_count(const htbl_t *htbl)
{
	return (htbl->num_values);
}

static void
htbl_multi_value_add(htbl_t *htbl, htbl_bucket_item_t *item, void *value)
{
	htbl_multi_value_t *mv = malloc(sizeof (*mv));
	ASSERT(htbl->multi_value);
	mv->value = value;
	mv->item = item;
	list_insert_head(&item->multi.list, mv);
	item->multi.num++;
	htbl->num_values++;
}

void
htbl_set(htbl_t *htbl, const void *key, void *value)
{
	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
	htbl_bucket_item_t *item;

	ASSERT(key != NULL);
	ASSERT(value != NULL);
	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			if (htbl->multi_value)
				htbl_multi_value_add(htbl, item, value);
			else
				item->value = value;
			return;
		}
	}
	item = calloc(sizeof (*item) + htbl->key_sz - 1, 1);
	memcpy(item->key, key, htbl->key_sz);
	if (htbl->multi_value) {
		list_create(&item->multi.list, sizeof (htbl_multi_value_t),
		    offsetof(htbl_multi_value_t, node));
		htbl_multi_value_add(htbl, item, value);
	} else {
		item->value = value;
	}
	list_insert_head(bucket, item);
	htbl->num_values++;
}

void
htbl_remove(htbl_t *htbl, const void *key, int nil_ok)
{
	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
	htbl_bucket_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			list_remove(bucket, item);
			if (htbl->multi_value) {
				htbl_empty_multi_item(item, NULL, NULL);
				ASSERT(htbl->num_values >= item->multi.num);
				htbl->num_values -= item->multi.num;
			} else {
				free(item);
				ASSERT(htbl->num_values != 0);
				htbl->num_values--;
			}
			return;
		}
	}
	ASSERT(nil_ok != 0);
}

void htbl_remove_multi(htbl_t *htbl, const void *key, void *list_item)
{
	htbl_multi_value_t *mv = list_item;
	htbl_bucket_item_t *item = mv->item;

	ASSERT(htbl->multi_value != 0);
	ASSERT(key != NULL);
	ASSERT(item != NULL);
	ASSERT(item->multi.num != 0);
	ASSERT(htbl->num_values != 0);

	list_remove(&item->multi.list, mv);
	item->multi.num--;
	htbl->num_values--;
	free(mv);
	if (item->multi.num == 0) {
		list_t *bucket =
		    &htbl->buckets[H(key, htbl->key_sz) & (htbl->tbl_sz - 1)];
		list_remove(bucket, item);
		list_destroy(&item->multi.list);
		free(item);
	}
}

static htbl_bucket_item_t *
htbl_lookup_common(const htbl_t *htbl, const void *key)
{
	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
	htbl_bucket_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0)
			return (item);
	}
	return (NULL);
}

void *
htbl_lookup(const htbl_t *htbl, const void *key)
{
	htbl_bucket_item_t *item;
	ASSERT(htbl->multi_value == 0);
	item = htbl_lookup_common(htbl, key);
	return (item != NULL ? item->value : NULL);
}

const list_t *
htbl_lookup_multi(const htbl_t *htbl, const void *key)
{
	htbl_bucket_item_t *item;
	ASSERT(htbl->multi_value != 0);
	item = htbl_lookup_common(htbl, key);
	return (item != NULL ? &item->multi.list : NULL);
}

void
htbl_foreach(const htbl_t *htbl, void (*func)(const void *, void *, void *),
    void *arg)
{
	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		list_t *bucket = &htbl->buckets[i];
		for (const htbl_bucket_item_t *item = list_head(bucket),
		    *next_item = NULL; item; item = next_item) {
			/*
			 * To support current value removal in the callback,
			 * retrieve the next pointer right now.
			 */
			next_item = list_next(bucket, item);
			if (htbl->multi_value) {
				const list_t *ml = &item->multi.list;
				for (htbl_multi_value_t *mv = list_head(ml),
				    *mv_next = NULL; mv != NULL; mv = mv_next) {
					mv_next = list_next(ml, mv);
					func(item->key, mv->value, arg);
				}
			} else {
				func(item->key, item->value, arg);
			}
		}
	}
}

char *
htbl_dump(const htbl_t *htbl, bool_t printable_keys)
{
	char	*result = NULL;
	size_t	result_sz = 0;

	append_format(&result, &result_sz, "(%lu){\n",
	    (long unsigned)htbl->num_values);
	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		list_t *bucket = &htbl->buckets[i];
		append_format(&result, &result_sz, "  [%lu] =",
		    (long unsigned)i);
		if (list_head(bucket) == NULL)
			append_format(&result, &result_sz, " <empty>");
		for (const htbl_bucket_item_t *item = list_head(bucket); item;
		    item = list_next(bucket, item)) {
			if (printable_keys) {
				append_format(&result, &result_sz, " (%s) ",
				    item->key);
			} else {
				append_format(&result, &result_sz, " (#BIN)");
			}
		}
		append_format(&result, &result_sz, "\n");
	}
	append_format(&result, &result_sz, "}");

	return (result);
}
