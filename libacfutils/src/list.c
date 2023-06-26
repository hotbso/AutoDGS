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
 * Copyright (c) 2003, 2010, Oracle and/or its affiliates. All rights reserved.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/*
 * Generic doubly-linked list implementation
 */

#include <acfutils/list.h>
#include <acfutils/list_impl.h>

#include <sys/types.h>
#include <stdlib.h>
#ifdef _KERNEL
#include <sys/debug.h>
#else
#include <assert.h>
#ifdef	DEBUG
#define	ASSERT(a)	assert(a)
#else	/* !DEBUG */
#define	ASSERT(a)
#endif	/* !DEBUG */
#endif

#ifdef lint
extern list_node_t *list_d2l(list_t *list, void *obj);
#else
#define	list_d2l(a, obj) ((list_node_t *)(((char *)obj) + (a)->list_offset))
#endif
#define	list_object(a, node) ((void *)(((char *)node) - (a)->list_offset))
#define	list_empty(a) ((a)->list_head.list_next == &(a)->list_head)

#define	list_insert_after_node(list, node, object) {	\
	list_node_t *lnew = list_d2l(list, object);	\
	lnew->list_prev = (node);			\
	lnew->list_next = (node)->list_next;		\
	(node)->list_next->list_prev = lnew;		\
	(node)->list_next = lnew;			\
	(list)->list_count++;				\
}

#define	list_insert_before_node(list, node, object) {	\
	list_node_t *lnew = list_d2l(list, object);	\
	lnew->list_next = (node);			\
	lnew->list_prev = (node)->list_prev;		\
	(node)->list_prev->list_next = lnew;		\
	(node)->list_prev = lnew;			\
	(list)->list_count++;				\
}

#define	list_remove_node(node)					\
	(node)->list_prev->list_next = (node)->list_next;	\
	(node)->list_next->list_prev = (node)->list_prev;	\
	(node)->list_next = (node)->list_prev = NULL

/**
 * Initializes a new linked list. This must be called before a list_t
 * is used. When you are done with the linked list, you must deinitialize
 * it using list_destroy().
 *
 * Data structures to be held in the linked list must have a list_node_t
 * field added to them. This is where the linked list will keep its
 * lists node references. If you want to hold an object in more than one
 * linked list, you will need a separate list_node_t for each linked list.
 * A single list_node_t cannot be shared at the same time between multiple
 * lists.
 *
 * @param list The linked list to be initialized.
 * @param size The size in bytes of the objects to be referenced in the
 *	linked list. This is mostly meant for error-checking for the
 *	placement of the list node.
 * @param offset Offset within the data structures that will be contained
 *	in the list_t to where the list_node_t field is. You should use
 *	the standard `offsetof()` for this purpose.
 *
 * #### Example:
 *```
 * typedef struct {
 *	int         foo;
 *	float       bar;
 *	list_node_t node;
 * } my_data_t;
 *
 * // initializing a list_t to contain my_data_t structures:
 * list_t my_list;
 * list_create(&my_list, sizeof (my_data_t), offsetof(my_data_t, node));
 *```
 * @see list_destroy()
 */
void
list_create(list_t *list, size_t size, size_t offset)
{
	ASSERT(list);
	ASSERT(size > 0);
	ASSERT(size >= offset + sizeof (list_node_t));

	list->list_size = size;
	list->list_offset = offset;
	list->list_count = 0;
	list->list_head.list_next = list->list_head.list_prev =
	    &list->list_head;
}

/**
 * Destroys a list_t structure which was previously initialized using
 * list_create(). The list MUST be empty before this is done, otherwise
 * an assertion failure is tripped.
 * @note This doesn't call free() on the data structure. It simply cleans
 *	it up and frees any internal resources. If you allocated the
 *	list_t previously using malloc() or similar, you must free() the
 *	data yourself.
 */
void
list_destroy(list_t *list)
{
	list_node_t *node = &list->list_head;

	ASSERT(list);
	ASSERT(list->list_head.list_next == node);
	ASSERT(list->list_head.list_prev == node);
	ASSERT(list->list_count == 0);

	node->list_next = node->list_prev = NULL;
}

/**
 * Inserts a new object immediately after another object into a linked list.
 * @param list The list into which the insert will be performed.
 * @param object The preceding object, after which the new object
 *	will be inserted.
 * @param nobject The new object being inserted. It will be inserted
 *	immediately following `object`.
 * @note The new object being inserted must NOT already be a member of
 *	this list.
 */
void
list_insert_after(list_t *list, void *object, void *nobject)
{
	if (object == NULL) {
		list_insert_head(list, nobject);
	} else {
		list_node_t *lold = list_d2l(list, object);
		list_insert_after_node(list, lold, nobject);
	}
}

/**
 * Inserts a new object immediately in front of another object into
 * a linked list.
 * @param list The list into which the insert will be performed.
 * @param object The following object, in front of which the new object
 *	will be inserted.
 * @param nobject The new object being inserted. It will be inserted
 *	immediately in front of `object`.
 * @note The new object being inserted must NOT already be a member of
 *	this list.
 */
void
list_insert_before(list_t *list, void *object, void *nobject)
{
	if (object == NULL) {
		list_insert_tail(list, nobject);
	} else {
		list_node_t *lold = list_d2l(list, object);
		list_insert_before_node(list, lold, nobject);
	}
}

/**
 * Inserts an object at the head (start) of a linked list.
 * @param list The list into which the insert will be performed.
 * @param object The object to be inserted at the head of the list.
 * @note The new object being inserted must NOT already be a member of
 *	this list.
 */
void
list_insert_head(list_t *list, void *object)
{
	list_node_t *lold = &list->list_head;
	list_insert_after_node(list, lold, object);
}

/**
 * Inserts an object at the tail (end) of a linked list.
 * @param list The list into which the insert will be performed.
 * @param object The object to be inserted at the head of the list.
 * @note The new object being inserted must NOT already be a member of
 *	this list.
 */
void
list_insert_tail(list_t *list, void *object)
{
	list_node_t *lold = &list->list_head;
	list_insert_before_node(list, lold, object);
}

/**
 * Removes an object from the list.
 * @param list The list from which the object is to be removed.
 * @param object The object to be removed from the list.
 * @note The object being removed **must** be a member of this list.
 */
void
list_remove(list_t *list, void *object)
{
	list_node_t *lold = list_d2l(list, object);
	ASSERT(!list_empty(list));
	ASSERT(lold->list_next != NULL);
	list_remove_node(lold);
	list->list_count--;
}

/**
 * Removes an object from the head (start) of the list.
 * @return The object which was removed from the head of the list.
 *	If the list was already empty, returns NULL instead.
 *
 * #### Example
 * This function is commonly used to empty out a list by removing all
 * members, like this:
 *```
 * my_data_t *data;
 * while ((data = list_remove_head(&my_list)) != NULL) {
 *	free(data);
 * }
 *```
 */
void *
list_remove_head(list_t *list)
{
	list_node_t *head = list->list_head.list_next;
	if (head == &list->list_head)
		return (NULL);
	list_remove_node(head);
	list->list_count--;
	return (list_object(list, head));
}

/**
 * Removes an object from the tail (end) of the list.
 * @return The object which was removed from the tail of the list.
 *	If the list was already empty, returns NULL instead.
 *
 * #### Example
 * This function is commonly used to empty out a list by removing all
 * members, like this:
 *```
 * my_data_t *data;
 * while ((data = list_remove_tail(&my_list)) != NULL) {
 *	free(data);
 * }
 *```
 */
void *
list_remove_tail(list_t *list)
{
	list_node_t *tail = list->list_head.list_prev;
	if (tail == &list->list_head)
		return (NULL);
	list_remove_node(tail);
	list->list_count--;
	return (list_object(list, tail));
}

/**
 * @return The first object in the list. If the list is empty,
 *	returns NULL instead.
 */
void *
list_head(const list_t *list)
{
	if (list_empty(list))
		return (NULL);
	return (list_object(list, list->list_head.list_next));
}

/**
 * @return The last object in the list. If the list is empty,
 *	returns NULL instead.
 */
void *
list_tail(const list_t *list)
{
	if (list_empty(list))
		return (NULL);
	return (list_object(list, list->list_head.list_prev));
}

/**
 * Iterates through the list forward (towards the tail) by 1 object.
 * @param list The list being iterated.
 * @param object The object from which the next object will be returned.
 *	This must not be NULL.
 * @return The next object following `object` in the second argument. If
 *	there are no more objects following `object`, returns NULL instead.
 * #### Example
 * Iterating through a list forwards:
 *```
 * for (my_data_t *data = list_head(&my_list); data != NULL;
 *     data = list_next(&my_list, data)) {
 *	// do something with data
 * }
 *```
 * @note If you plan on removing items from the list while iterating
 *	through it, be careful to structure your `for()` loop correctly
 *	so as to avoid using a removed reference for the iteration.
 *	Grab the next item reference early, before potentially removing
 *	the current item:
 *```
 * for (my_data_t *data = list_head(&my_list), *next_data = NULL;
 *     data != NULL; data = next_data) {
 *	// grab next_data now, before we remove the current item
 *	next_data = list_next(&my_list, data);
 *	if (should_remove(data)) {
 *		// now we can safely remove the item from the list
 *		list_remove(&my_list, data);
 *	}
 * }
 *```
 */
void *
list_next(const list_t *list, const void *object)
{
	list_node_t *node = list_d2l(list, object);

	ASSERT(list_link_active(node));
	if (node->list_next != &list->list_head)
		return (list_object(list, node->list_next));

	return (NULL);
}

/**
 * Iterates through the list backward (towards the head) by 1 object.
 * @param list The list being iterated.
 * @param object The object from which the previous object will be returned.
 *	This must not be NULL.
 * @return The nearest object preceding `object` in the second argument. If
 *	there are no more objects preceding `object`, returns NULL instead.
 * #### Example
 * Iterating through a list backwards:
 *```
 * for (my_data_t *data = list_tail(&my_list); data != NULL;
 *     data = list_prev(&my_list, data)) {
 *	// do something with data
 * }
 *```
 * @note If you plan on removing items from the list while iterating
 *	through it, be careful to structure your `for()` loop correctly
 *	so as to avoid using a removed reference for the iteration.
 *	Grab the next item reference early, before potentially removing
 *	the current item:
 *```
 * for (my_data_t *data = list_tail(&my_list), *prev_data = NULL;
 *     data != NULL; data = prev_data) {
 *	// grab prev_data now, before we remove the current item
 *	prev_data = list_prev(&my_list, data);
 *	if (should_remove(data)) {
 *		// now we can safely remove the item from the list
 *		list_remove(&my_list, data);
 *	}
 * }
 *```
 */
void *
list_prev(const list_t *list, const void *object)
{
	list_node_t *node = list_d2l(list, object);

	ASSERT(list_link_active(node));
	if (node->list_prev != &list->list_head)
		return (list_object(list, node->list_prev));

	return (NULL);
}

/**
 * Performs an iterative search through the list to retrieve a particular
 * element in the list in order.
 * @note This is an expensive operation, as it performs an O(n) search
 *	(iterating through the list), which can take long on very large
 *	lists. If you need frequent random access to items in large lists,
 *	you should consider using a simple array instead of a linked list.
 * @param i The index of the item in the list (counting from 0). This
 *	**must** be less than the total number of items in the list as
 *	returned by list_count().
 * @return The i'th object inside of the list.
 */
void *
list_get_i(const list_t *list, unsigned i)
{
	void *obj;

	ASSERT(list != NULL);
	ASSERT(i < list_count(list));

	obj = list_head(list);
	for (unsigned j = 0; j < i; j++)
		obj = list_next(list, obj);

	return (obj);
}

/**
 * Transfers the contents of one list to another, by appending them.
 * This is faster than moving items one by one, as it simply manipulates
 * the list references for the first and last item. Thus this is a
 * constant-time algorithm.
 * @param dst Destination list to which to append all items in `src`.
 * @param src Source list, from which to append all items to the end of `dst`.
 *	Afterwards, the `src` list will be empty.
 */
void
list_move_tail(list_t *dst, list_t *src)
{
	list_node_t *dstnode = &dst->list_head;
	list_node_t *srcnode = &src->list_head;

	ASSERT(dst->list_size == src->list_size);
	ASSERT(dst->list_offset == src->list_offset);

	if (list_empty(src))
		return;

	dstnode->list_prev->list_next = srcnode->list_next;
	srcnode->list_next->list_prev = dstnode->list_prev;
	dstnode->list_prev = srcnode->list_prev;
	srcnode->list_prev->list_next = dstnode;
	dst->list_count += src->list_count;

	/* empty src list */
	srcnode->list_next = srcnode->list_prev = srcnode;
	src->list_count = 0;
}

/**
 * Replaces an object in a linked list with another object, without the
 * need to have a reference to the original list_t.
 * @param lold A pointer to the list_node_t of the old object, which is
 *	to be replaced by the new object. This object MUST be in a list
 *	(list_link_active() returns true).
 * @param lnew A pointer to the list_node_t of the new object, which is
 *	to replace the old object. This object must NOT be in a list
 *	(list_link_active() returns false).
 * @note The two objects must be of the same type and hold their
 *	list_node_t at the exact same offset within the parent structure.
 *	This function is NOT meant to replace one type of object with
 *	a different type of object within the same list.
 * #### Example
 *```
 * typedef struct {
 *	int         foo;
 *	float       bar;
 *	list_node_t node;
 * } my_data_t;
 * my_data_t *old_object;
 * [...]
 * // old_object gets placed into a list and we want to replace it
 * [...]
 * my_data_t *new_object;
 * list_link_replace(&old_object->node, &new_object->node);
 * // now old_object is no longer in its containing list and new_object
 * // took its place
 *```
 */
void
list_link_replace(list_node_t *lold, list_node_t *lnew)
{
	ASSERT(list_link_active(lold));
	ASSERT(!list_link_active(lnew));

	lnew->list_next = lold->list_next;
	lnew->list_prev = lold->list_prev;
	lold->list_prev->list_next = lnew;
	lold->list_next->list_prev = lnew;
	lold->list_next = lold->list_prev = NULL;
}

/**
 * Initializes a list_node_t so that subsequent calls to list_link_active()
 * can determine the activity status of that node correctly. This is
 * equivalent to just setting the contents of the node to NULL, so you
 * can achieve the same effect of this function by simply allocating the
 * structure containing the list_node_t using an allocation function
 * which zeros out the memory contents during allocation (such as calloc()).
 */
void
list_link_init(list_node_t *link)
{
	link->list_next = NULL;
	link->list_prev = NULL;
}

/**
 * @return True if the given list_node_t is active, i.e. the object
 *	containing the node is currently part of a list.
 * @see For this function to work properly, the object containing the
 *	list_node_t must either have been pre-initialized to all
 *	zeros during allocation, or you have explicitly initialized
 *	the list_node_t itself using list_link_init().
 */
int
list_link_active(const list_node_t *link)
{
	return (link->list_next != NULL);
}

/**
 * @return True if the list is empty, otherwise false.
 */
int
list_is_empty(const list_t *list)
{
	return (list_empty(list));
}

/**
 * @return The number of items in the list.
 */
size_t
list_count(const list_t *list)
{
	return (list->list_count);
}
