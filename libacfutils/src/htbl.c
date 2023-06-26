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

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "acfutils/crc64.h"
#include "acfutils/helpers.h"
#include "acfutils/htbl.h"

typedef struct {
	list_node_t	bucket_node;
	union {
		void		*value;
		struct {
			list_t	list;
			size_t	num;
		} multi;
	};
	uint8_t		key[1];	/* variable length, depends on htbl->key_sz */
} htbl_bucket_item_t;

struct htbl_multi_value_s {
	void			*value;
	list_node_t		node;
	htbl_bucket_item_t	*item;
};

static inline uint64_t
H(const void *p, size_t l)
{
	return (crc64(p, l));
}

/**
 * Initializes a new hash table.
 * @note The hash table functionality is dependent on the CRC64 subsystem,
 *	so be sure to call crc64_init() before initializing the first hash
 *	table.
 * @param htbl Pointer to the hash table which is to be initialized.
 * @param tbl_sz Hash table size. Once initialized, hash tables retain a
 *	fixed size and cannot be changed, so pick a size wisely here.
 *	The size affects the hash table's memory-vs-time performance.
 *	The bigger the hash table, the less of a chance of hash collisions
 *	requiring an extensive search, but that also uses more memory.
 *	A good rule of thumb is to size the hash table to double the
 *	maximum number of elements expected to be in the hash table.
 * @param key_sz Size of the hash keys to be used in the hash table.
 *	This will be the size of the `key` objects used in htbl_lookup(),
 *	htbl_set() and similar.
 * @param multi_value Controls whether the hash table will support
 *	duplicate values. This then changes how you need to look up
 *	entries inside of the hash table (see htbl_lookup() and
 *	htbl_lookup_multi()).
 */
void
htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz, bool_t multi_value)
{
	ASSERT(htbl != NULL);
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

/**
 * Deinitializes a hash table which was initialized using htbl_create().
 * The hash table must be empty at the time that htbl_destroy() is called.
 * If you want to quickly empty the entire hash table, see htbl_empty().
 * @note This **doesn't** call free() on the htbl_t argument. If you have
 *	dynamically allocated the htbl_t object, you will need to free it
 *	yourself after calling htbl_destroy(). Also note that simply
 *	free()ing a htbl_t isn't enough to free all memory associated with
 *	a hash table. You **must** call htbl_destroy() on every hash table
 *	which you have initialized using htbl_create().
 * @see htbl_create()
 */
void
htbl_destroy(htbl_t *htbl)
{
	ASSERT(htbl != NULL);
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
		if (func != NULL)
			func(mv->value, arg);
		list_remove_head(&item->multi.list);
		free(mv);
	}
	list_destroy(&item->multi.list);
}

/**
 * Removes all entries from a hash table. This can be used to either reset
 * the hash table to a clean, or to prepare the hash table for
 * deinitialization via htbl_destroy().
 * @param func Optional callback, which will be invoked for every element
 *	in the hash table, as it's being removed. You can use this to
 *	free memory of the table values, or to deinitialize them in some
 *	way. The first argument to the function will be the hash table
 *	value, while the second one will be the `userinfo` you provide in
 *	this call. If you do not want to use this callback, pass NULL here.
 *	If your values only need to be freed using the standard free()
 *	function, you can pass htbl_free() in this this argument and
 *	setting `userinfo` to NULL. This is a simple utility function which
 *	calls free() using your own heap allocator on every value it
 *	receives as part of the htbl_empty() call.
 * @param userinfo Optional argument, which will be passed to the `func`
 *	callback above in the second argument.
 */
void
htbl_empty(htbl_t *htbl, void (*func)(void *value, void *userinfo),
    void *userinfo)
{
	ASSERT(htbl != NULL);

	if (htbl->num_values == 0)
		return;
	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		for (htbl_bucket_item_t *item = list_head(&htbl->buckets[i]);
		    item; item = list_head(&htbl->buckets[i])) {
			if (htbl->multi_value)
				htbl_empty_multi_item(item, func, userinfo);
			else if (func != NULL)
				func(item->value, userinfo);
			list_remove_head(&htbl->buckets[i]);
			free(item);
		}
	}
	htbl->num_values = 0;
}

/**
 * @return The number of values stored in the hash table.
 */
size_t
htbl_count(const htbl_t *htbl)
{
	ASSERT(htbl != NULL);
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

/**
 * Stores a new value in a hash table. If the value already exists,
 * what happens depends on whether the hash table was created with
 * `multi_value` set to `B_TRUE` or `B_FALSE` in htbl_create():
 * - If multi-values were disabled, the old value is transparently
 *	replaced in the hash table.
 * - If multi-values were enable, the new value is stored side-by-side
 *	in the hash table. You can then find both values in the hash
 *	table using htbl_lookup_multi().
 * @param key The hash key under which to store the value. This must
 *	point to a memory buffer of size `key_sz` as passed in the
 *	initial htbl_create() call. The key itself is copied into the
 *	hash table, so you don't have to keep the memory pointed at
 *	by `key` allocated after the call to htbl_set() returns.
 * @param value The value which will be stored under `key`. Hash table
 *	values are stored by reference, so be sure **not** to
 *	de-allocate object pointed to by `value`, if you need to
 *	dereference the stored pointer.
 */
void
htbl_set(htbl_t *htbl, const void *key, void *value)
{
	ASSERT(htbl != NULL);
	ASSERT(key != NULL);

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

/**
 * Removes a value from a hash table. If the hash table was created
 * with `multi_value` set to `B_TRUE` in htbl_create(), then this
 * will remove **all** values stored under `key`. If you want more
 * fine-grained control over which values to remove in a multi-value
 * table, use htbl_remove_multi().
 * @param key The key under which the value was stored. This must
 *	point to a memory buffer of size `key_sz` as passed in the
 *	initial htbl_create() call.
 * @param nil_ok If set to `B_TRUE`, the value not existing in the
 *	hash table is considered OK. If set to `B_FALSE`, if the
 *	value in the hash table doesn't exist, an assertion failure
 *	is triggered.
 */
void
htbl_remove(htbl_t *htbl, const void *key, bool_t nil_ok)
{
	ASSERT(htbl != NULL);
	ASSERT(key != NULL);

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
	ASSERT(nil_ok);
}

/**
 * Removes a single value from a multi-value enabled hash table. The normal
 * htbl_remove() function removes all values paired to a single key, so
 * this function gives you finer control to only remove specific values.
 * @param key The key for which to remove the value.
 * @param list_item The htbl_multi_value_t as contained in the list returned
 *	by htbl_lookup_multi().
 * #### Example
 *```
 * list_t *values = htbl_lookup_multi(my_hash_table, my_key);
 * if (values != NULL) {
 *	for (htbl_multi_value_t *mv = list_head(values), *mv_next = NULL;
 *	    mv != NULL; mv = mv_next) {
 *		// Get the next element in the list early, in case we need
 *		// to remove the current one (which would mean we can no
 *		// longer use it in list_next()).
 *		mv_next = list_next(values, mv);
 *		if (want_to_remove_this_item(htbl_value_multi(mv)) {
 *			htbl_remove_multi(my_hash_table, my_key, mv);
 *		}
 *	}
 * }
 *```
 */
void
htbl_remove_multi(htbl_t *htbl, const void *key, htbl_multi_value_t *list_item)
{
	ASSERT(htbl != NULL);
	ASSERT(htbl->multi_value);
	ASSERT(htbl->num_values != 0);
	ASSERT(list_item != NULL);
	ASSERT(key != NULL);

	htbl_bucket_item_t *item = list_item->item;
	ASSERT(item != NULL);
	ASSERT(item->multi.num != 0);

	list_remove(&item->multi.list, list_item);
	item->multi.num--;
	htbl->num_values--;
	free(list_item);
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
	ASSERT(htbl != NULL);
	ASSERT(key != NULL);

	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
	htbl_bucket_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0)
			return (item);
	}
	return (NULL);
}

/**
 * Performs a hash table lookup. The hash table must have been
 * initialized with `multi_value` set to `B_FALSE` in htbl_create().
 * @param key The key under which the value was stored. This must
 *	point to a memory buffer of size `key_sz` as passed in the
 *	initial htbl_create() call.
 * @return The original `value` which was stored under the key, if
 *	it exists in the hash table. If no value was stored under
 *	`key`, this returns NULL instead.
 */
void *
htbl_lookup(const htbl_t *htbl, const void *key)
{
	htbl_bucket_item_t *item;
	ASSERT(!htbl->multi_value);
	item = htbl_lookup_common(htbl, key);
	return (item != NULL ? item->value : NULL);
}

/**
 * Performs a hash table lookup. The hash table must have been
 * initialized with `multi_value` set to `B_TRUE` in htbl_create().
 * @param key The key under which the value was stored. This must
 *	point to a memory buffer of size `key_sz` as passed in the
 *	initial htbl_create() call.
 * @return If at least one value exists stored under `key`, this
 *	function returns a `list_t` of `htbl_multi_value_t`
 *	structures. You must **not** modify any of the fields of
 *	the returned structure, only access the value stored within
 *	using htbl_value_multi().
 * @return If no values are stored under `key`, returns NULL.
 */
const list_t *
htbl_lookup_multi(const htbl_t *htbl, const void *key)
{
	htbl_bucket_item_t *item;
	ASSERT(htbl->multi_value);
	item = htbl_lookup_common(htbl, key);
	return (item != NULL ? &item->multi.list : NULL);
}

/**
 * Walks the hash table, iterating over each value in the table. It is
 * safe to add or remove entries from the hash table from within the
 * callback, although the walk may not pass over the newly added values.
 * @param htbl The hash table to walk.
 * @param func A callback which will be invoked for every value stored
 *	in the hash table. The first and second arguments to the callback
 *	will be set to the key and value respectively. The third argument
 *	is the value of the `userinfo` parameter.
 * @param userinfo Optional parameter which will be passed on to the
 *	`func` argument in the third parameter.
 */
void
htbl_foreach(const htbl_t *htbl,
    void (*func)(const void *key, void *value, void *userinfo), void *userinfo)
{
	ASSERT(htbl != NULL);
	ASSERT(func != NULL);
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
					func(item->key, mv->value, userinfo);
				}
			} else {
				func(item->key, item->value, userinfo);
			}
		}
	}
}

/**
 * @return The value contained in a multi-value element, which are
 * the contents of the list_t returned from htbl_lookup_multi().
 *
 * #### Example
 *```
 * htbl_t *my_hash_table = ...; // previously initialized hash table
 *
 * list_t *values = htbl_lookup_multi(my_hash_table, my_key);
 * if (values != NULL) {
 *	// key found in hash table, walk the list of values
 *	for (htbl_multi_value_t *mv = list_head(values); mv != NULL;
 *	    mv = list_next(values, mv)) {
 *		my_struct_t *value = htbl_value_multi(mv);
 *		// do something with `value'
 *	}
 * }
 *```
 */
void *
htbl_value_multi(htbl_multi_value_t *mv)
{
	ASSERT(mv != NULL);
	return (mv->value);
}

/**
 * @deprecated
 * Dumps a hash table's context in text form. This has limited usefulness
 * and shouldn't be used too much.
 * @return A string containing a printable description of the contents of
 *	the hash table. Free with lacf_free().
 */
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
