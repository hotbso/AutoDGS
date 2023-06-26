/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
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
 * Copyright 2003 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */
/**
 * \file
 * @note This is an internal implementation file and shouldn't be
 *	included directly. You want to include list.h instead.
 */

#ifndef	_ACF_UTILS_LIST_IMPL_H_
#define	_ACF_UTILS_LIST_IMPL_H_

#include <sys/types.h>

#ifdef	__cplusplus
extern "C" {
#endif

/** See list_node_t for more information. */
struct list_node {
	struct list_node *list_next;
	struct list_node *list_prev;
};

/** See list_t for more information. */
struct list {
	size_t	list_size;
	size_t	list_offset;
	size_t	list_count;
	struct list_node list_head;
};

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_LIST_IMPL_H_ */
