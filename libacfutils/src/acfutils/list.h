/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensolaris.org/os/licensing.
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
 * Copyright 2008 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * This is a general-purpose doubly-linked list system. It is designed
 * to be easy to integrate into pre-existing data structures and has many
 * functions to manipulate and examine the  linked list.
 * @see list_t
 * @see list_create()
 */

#ifndef	_ACF_UTILS_LIST_H_
#define	_ACF_UTILS_LIST_H_

#include "core.h"
#include "list_impl.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * \struct list_node_t
 * You must embed a member of this type somewhere in any data
 * structure which is to be contained within a linked list. The
 * list uses this member to store member references.
 * @see list_create()
 * @see list_next()
 * @see list_link_active()
 */
typedef struct list_node list_node_t;
/**
 * \struct list_t
 * This is the linked list object itself. You want to embed this
 * in your parent structures which contain the linked list, or
 * dynamically allocate it.
 *
 * Before using the linked list, it must be initialized using
 * list_create(). After you are done with the list, you must
 * deinitialize it using list_destroy(). Items in the list can
 * be added using the `list_insert_*` and `list_remove_*`
 * functions. The list can be traversed using list_head(),
 * list_tail(), list_next() and list_prev().
 */
typedef struct list list_t;

API_EXPORT void list_create(list_t *, size_t, size_t);
API_EXPORT void list_destroy(list_t *);

API_EXPORT void list_insert_after(list_t *, void *, void *);
API_EXPORT void list_insert_before(list_t *, void *, void *);
API_EXPORT void list_insert_head(list_t *, void *);
API_EXPORT void list_insert_tail(list_t *, void *);
API_EXPORT void list_remove(list_t *, void *);
API_EXPORT void *list_remove_head(list_t *);
API_EXPORT void *list_remove_tail(list_t *);
API_EXPORT void list_move_tail(list_t *, list_t *);

API_EXPORT void *list_head(const list_t *);
API_EXPORT void *list_tail(const list_t *);
API_EXPORT void *list_next(const list_t *, const void *);
API_EXPORT void *list_prev(const list_t *, const void *);
API_EXPORT void *list_get_i(const list_t *, unsigned);
API_EXPORT int list_is_empty(const list_t *);

API_EXPORT void list_link_init(list_node_t *);
API_EXPORT void list_link_replace(list_node_t *, list_node_t *);

API_EXPORT int list_link_active(const list_node_t *);
API_EXPORT size_t list_count(const list_t *);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_LIST_H_ */
