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
 * Copyright 2021 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_THREAD_H_
#define	_ACF_UTILS_THREAD_H_

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#if	APL || LIN
#include <pthread.h>
#include <stdint.h>
#include <time.h>
#else	/* !APL && !LIN */
#include <windows.h>
#endif	/* !APL && !LIN */

#if	__STDC_VERSION__ >= 201112L && !defined(__STDC_NO_ATOMICS__)
#include <stdatomic.h>
#elif	APL
#include <libkern/OSAtomic.h>
#endif

#if	LIN
#include <sys/syscall.h>
#include <unistd.h>
#endif	/* LIN */

#include "assert.h"
#include "helpers.h"
#include "list.h"
#include "time.h"
#include "tls.h"
#include "safe_alloc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Basic portable multi-threading API. We have 3 kinds of objects and
 * associated manipulation functions here:
 * 1) threat_t - A generic thread identification structure.
 * 2) mutex_t - A generic mutual exclusion lock.
 * 3) condvar_t - A generic condition variable.
 *
 * The actual implementation is all contained in this header and simply
 * works as a set of macro expansions on top of the OS-specific threading
 * API (pthreads on Unix/Linux, winthreads on Win32).
 *
 * Example of how to create a thread:
 *	thread_t my_thread;
 *	if (!thread_create(&my_thread, thread_main_func, thread_arg))
 *		fprintf(stderr, "thread create failed!\n");
 *
 * Example of how to wait for a thread to exit:
 *	thread_t my_thread;
 *	thread_join(my_thread);
 *	... thread disposed of, no need for further cleanup ...
 *
 * Example of how to use a mutex_t:
 *	mutex_t my_lock;		-- the lock object itself
 *	mutex_init(&my_lock);		-- create a lock
 *	mutex_enter(&my_lock);		-- grab a lock
 *	... do some critical, exclusiony-type stuff ...
 *	mutex_exit(&my_lock);		-- release a lock
 *	mutex_destroy(&my_lock);	-- free a lock
 *
 * Example of how to use a condvar_t:
 *	mutex_t my_lock;
 *	condvar_t my_cv;
 *
 *	mutex_init(&my_lock);		-- create a lock to control the CV
 *	cv_init(&my_cv);		-- create the condition variable
 *
 *	-- thread that's going to signal the condition:
 *		mutex_enter(&my_lock);	-- grab the lock
 *		... set up some resource that others might be waiting on ...
 *		cv_broadcast(&my_lock);	-- wake up all waiters
 *		mutex_exit(&my_lock);	-- release the lock
 *
 *	-- thread that's going to wait on the condition:
 *		mutex_enter(&my_lock);			-- grab the lock
 *		while (!condition_met())
 *			cv_wait(&my_cv, &my_lock);	-- wait for the CV
 *							-- to be signalled
 *		... condition fulfilled, use the resource ...
 *		mutex_exit(&my_lock);			-- release the lock
 *
 * You can also performed a "timed" wait on a CV using cv_timedwait. The
 * function will exit when either the condition has been signalled, or the
 * timer has expired. The return value of the function indicates whether
 * the condition was signalled before the timer expired (returns zero),
 * or if the wait timed out (returns ETIMEDOUT) or another error occurred
 * (returns -1).
 *
 *		mutex_enter(&my_lock);			-- grab the lock
 *		-- Wait for the CV to be signalled. Time argument is an
 *		-- absolute time as returned by the 'microclock' function +
 *		-- whatever extra time delay you want to apply.
 *		uint64_t deadline = microclock() + timeout_usecs;
 *		while (!condition_met()) {
 *			if (cv_timedwait(&my_cv, &my_lock, deadline) ==
 *			    ETIMEDOUT) {
 *				-- timed out waiting for CV to signal
 *				break;
 *			}
 *		}
 *		mutex_exit(&my_lock);			-- release the lock
 */

typedef struct {
	void		(*proc)(void *);
	void		*arg;
	const char	*filename;
	int		linenum;
	list_node_t	node;
} lacf_thread_info_t;

/*
 * !!!! CAUTION !!!!
 * MacOS's kernel API contains a function named `thread_create' that'll
 * clash with the naming below. To work around this, we declare the
 * function name to be a macro to our own scoped version. The user must
 * include /usr/include/mach/task.h before us, otherwise we could be
 * rewriting the function name in the header too. The flip side is that
 * the user will get calls to libacfutils' thread_create instead.
 */
#if	!APL || !defined(_task_user_)
#define	thread_create(thrp, start_proc, arg) \
	lacf_thread_create((thrp), (start_proc), (arg), \
	    log_basename(__FILE__), __LINE__)
#endif

#if	__STDC_VERSION__ >= 201112L && !defined(__STDC_NO_ATOMICS__)
#define	atomic32_t		_Atomic int32_t
#define	atomic_inc_32(x)	atomic_fetch_add((x), 1)
#define	atomic_dec_32(x)	atomic_fetch_add((x), -1)
#define	atomic_set_32(x, y)	atomic_store((x), (y))
#define	atomic_add_32(x, y)	atomic_fetch_add((x), (y))
#define	atomic64_t		_Atomic int64_t
#define	atomic_inc_64(x)	atomic_fetch_add((x), 1)
#define	atomic_dec_64(x)	atomic_fetch_add((x), -1)
#define	atomic_set_64(x, y)	atomic_store((x), (y))
#define	atomic_add_64(x, y)	atomic_fetch_add((x), (y))
#elif	IBM
#define	atomic32_t		volatile LONG
#define	atomic_inc_32(x)	InterlockedIncrement((x))
#define	atomic_dec_32(x)	InterlockedDecrement((x))
#define	atomic_add_32(x, y)	InterlockedAdd((x), (y))
#define	atomic_set_32(x, y)	InterlockedExchange((x), (y))
#define	atomic64_t		volatile LONG64
#define	atomic_inc_64(x)	InterlockedIncrement64((x))
#define	atomic_dec_64(x)	InterlockedDecrement64((x))
#define	atomic_add_64(x, y)	InterlockedAdd64((x), (y))
#define	atomic_set_64(x, y)	InterlockedExchange64((x), (y))
#elif	APL
#define	atomic32_t		volatile int32_t
#define	atomic_inc_32(x)	OSAtomicAdd32(1, (x))
#define	atomic_dec_32(x)	OSAtomicAdd32(-1, (x))
#define	atomic_add_32(x, y)	OSAtomicAdd32((y), (x))
#define	atomic_set_32(x, y)	(x) = (y)	/* No such op on OSX */
#define	atomic64_t		volatile int64_t
#define	atomic_inc_64(x)	OSAtomicAdd64(1, (x))
#define	atomic_dec_64(x)	OSAtomicAdd64(-1, (x))
#define	atomic_add_64(x, y)	OSAtomicAdd64((y), (x))
#define	atomic_set_64(x, y)	(x) = (y)	/* No such op on OSX */
#else	/* LIN */
#define	atomic32_t		volatile int32_t
#define	atomic_inc_32(x)	__sync_add_and_fetch((x), 1)
#define	atomic_dec_32(x)	__sync_add_and_fetch((x), -1)
#define	atomic_add_32(x, y)	__sync_add_and_fetch((x), (y))
#define	atomic_set_32(x, y)	__atomic_store_n((x), (y), __ATOMIC_RELAXED)
#define	atomic64_t		volatile int64_t
#define	atomic_inc_64(x)	__sync_add_and_fetch((x), 1)
#define	atomic_dec_64(x)	__sync_add_and_fetch((x), -1)
#define	atomic_add_64(x, y)	__sync_add_and_fetch((x), (y))
#define	atomic_set_64(x, y)	__atomic_store_n((x), (y), __ATOMIC_RELAXED)
#endif	/* LIN */

#if	APL || LIN

#define	thread_t		pthread_t
#define	thread_id_t		pthread_t
#define	mutex_t			pthread_mutex_t
#define	condvar_t		pthread_cond_t
#define	curthread_id		pthread_self()
#define	curthread		pthread_self()

#define	mutex_init(mtx)	\
	do { \
		pthread_mutexattr_t attr; \
		pthread_mutexattr_init(&attr); \
		pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE); \
		pthread_mutex_init((mtx), &attr); \
	} while (0)
#define	mutex_destroy(mtx)	pthread_mutex_destroy((mtx))
#define	mutex_enter(mtx)	pthread_mutex_lock((mtx))
#define	mutex_exit(mtx)		pthread_mutex_unlock((mtx))

#if	LIN
#define	VERIFY_MUTEX_HELD(mtx)	\
	VERIFY((mtx)->__data.__owner == syscall(SYS_gettid))
#define	VERIFY_MUTEX_NOT_HELD(mtx) \
	VERIFY((mtx)->__data.__owner != syscall(SYS_gettid))
#else	/* APL */
#define	VERIFY_MUTEX_HELD(mtx)		(void)1
#define	VERIFY_MUTEX_NOT_HELD(mtx)	(void)1
#endif	/* APL */

#else	/* !APL && !LIN */

#define	thread_t	HANDLE
#define	thread_id_t	DWORD
typedef struct {
	bool_t			inited;
	CRITICAL_SECTION	cs;
} mutex_t;
#define	condvar_t	CONDITION_VARIABLE
#define	curthread_id	GetCurrentThreadId()
#define	curthread	GetCurrentThread()

#define	mutex_init(x) \
	do { \
		(x)->inited = B_TRUE; \
		InitializeCriticalSection(&(x)->cs); \
	} while (0)
#define	mutex_destroy(x) \
	do { \
		ASSERT((x)->inited); \
		DeleteCriticalSection(&(x)->cs); \
		(x)->inited = B_FALSE; \
	} while (0)
#define	mutex_enter(x) \
	do { \
		ASSERT((x)->inited); \
		EnterCriticalSection(&(x)->cs); \
	} while (0)
#define	mutex_exit(x) \
	do { \
		ASSERT((x)->inited); \
		LeaveCriticalSection(&(x)->cs); \
	} while (0)
#define	VERIFY_MUTEX_HELD(mtx)		(void)1
#define	VERIFY_MUTEX_NOT_HELD(mtx)	(void)1

#endif	/* !APL && !LIN */

/*
 * This is the thread cleanup tracking machinery. Due to how the global
 * nature of these variables, this system cannot be used in shared/DLL
 * builds of libacfutils.
 */
#ifndef	ACFUTILS_DLL

extern bool_t	lacf_thread_list_inited;
extern mutex_t	lacf_thread_list_lock;
extern list_t	lacf_thread_list;

static inline void
_lacf_thread_list_init(void)
{
	if (!lacf_thread_list_inited) {
		mutex_init(&lacf_thread_list_lock);
		list_create(&lacf_thread_list, sizeof (lacf_thread_info_t),
		    offsetof(lacf_thread_info_t, node));
		lacf_thread_list_inited = B_TRUE;
	}
}

static inline void
_lacf_thread_list_fini(void)
{
	if (lacf_thread_list_inited) {
		mutex_enter(&lacf_thread_list_lock);
		if (list_count(&lacf_thread_list) == 0) {
			list_destroy(&lacf_thread_list);
			mutex_exit(&lacf_thread_list_lock);
			mutex_destroy(&lacf_thread_list_lock);
			lacf_thread_list_inited = B_FALSE;
		} else {
			mutex_exit(&lacf_thread_list_lock);
		}
	}
}

static inline void
_lacf_thread_list_add(lacf_thread_info_t *ti)
{
	ASSERT(ti != NULL);
	ASSERT(lacf_thread_list_inited);
	mutex_enter(&lacf_thread_list_lock);
	list_insert_tail(&lacf_thread_list, ti);
	mutex_exit(&lacf_thread_list_lock);
}

static inline void
_lacf_thread_list_remove(lacf_thread_info_t *ti)
{
	ASSERT(ti != NULL);
	ASSERT(lacf_thread_list_inited);
	mutex_enter(&lacf_thread_list_lock);
	list_remove(&lacf_thread_list, ti);
	mutex_exit(&lacf_thread_list_lock);
}

/*
 * Checks to see if all threads that were created using thread_create
 * have been properly disposed of. If not, this trips an assertion
 * failure and lists all threads (including filenames and line numbers
 * where they have been spawned) that weren't properly stopped. You
 * should call this just as your plugin is exiting.
 */
static inline void
lacf_threads_fini(void)
{
	if (!lacf_thread_list_inited)
		return;

	mutex_enter(&lacf_thread_list_lock);
	for (lacf_thread_info_t *ti =
	    (lacf_thread_info_t *)list_head(&lacf_thread_list); ti != NULL;
	    ti = (lacf_thread_info_t *)list_next(&lacf_thread_list, ti)) {
		logMsg("Leaked thread, created here %s:%d", ti->filename,
		    ti->linenum);
	}
	VERIFY0(list_count(&lacf_thread_list));
	mutex_exit(&lacf_thread_list_lock);

	_lacf_thread_list_fini();
}

#else	/* defined(ACFUTILS_DLL) */

#define	_lacf_thread_list_init()
#define	_lacf_thread_list_fini()
#define	_lacf_thread_list_add(ti)
#define	_lacf_thread_list_remove(ti)
#define	lacf_threads_fini

#endif	/* defined(ACFUTILS_DLL) */

#if	APL || LIN

static void *_lacf_thread_start_routine(void *arg) UNUSED_ATTR;
static void *
_lacf_thread_start_routine(void *arg)
{
	lacf_thread_info_t *ti = (lacf_thread_info_t *)arg;
	ti->proc(ti->arg);
	_lacf_thread_list_remove(ti);
	free(ti);
	return (NULL);
}

static inline bool_t
lacf_thread_create(thread_t *thrp, void (*proc)(void *), void *arg,
    const char *filename, int linenum)
{
	lacf_thread_info_t *ti =
	    (lacf_thread_info_t *)safe_calloc(1, sizeof (*ti));
	ti->proc = proc;
	ti->arg = arg;
	ti->filename = filename;
	ti->linenum = linenum;
	_lacf_thread_list_init();
	_lacf_thread_list_add(ti);
	if (pthread_create(thrp, NULL, _lacf_thread_start_routine, ti) == 0) {
		/* Start success */
		return (B_TRUE);
	}
	/* Start failure - remove from list and try to destroy the list */
	_lacf_thread_list_remove(ti);
	_lacf_thread_list_fini();
	free(ti);
	return (B_FALSE);
}

#define	thread_join(thrp) \
	do { \
		pthread_join(*(thrp), NULL); \
		_lacf_thread_list_fini(); \
	} while (0)

#if	LIN
#define	thread_set_name(name)	pthread_setname_np(pthread_self(), (name))
#else	/* APL */
#define	thread_set_name(name)	pthread_setname_np((name))
#endif	/* APL */

#define	cv_wait(cv, mtx)	pthread_cond_wait((cv), (mtx))
static inline int
cv_timedwait(condvar_t *cv, mutex_t *mtx, uint64_t limit)
{
	struct timespec ts = { .tv_sec = (time_t)(limit / 1000000),
	    .tv_nsec = (long)((limit % 1000000) * 1000) };
	return (pthread_cond_timedwait(cv, mtx, &ts));
}
#define	cv_init(cv)		pthread_cond_init((cv), NULL)
#define	cv_destroy(cv)		pthread_cond_destroy((cv))
#define	cv_signal(cv)		pthread_cond_signal((cv))
#define	cv_broadcast(cv)	pthread_cond_broadcast((cv))

#if	!APL
#define	THREAD_PRIO_IDLE	sched_get_priority_min()
#define	THREAD_PRIO_VERY_LOW	(THREAD_PRIO_NORM - 2)
#define	THREAD_PRIO_LOW		(THREAD_PRIO_NORM - 1)
#define	THREAD_PRIO_NORM	0	/* Default priority on Linux */
#define	THREAD_PRIO_HIGH	(THREAD_PRIO_NORM + 1)
#define	THREAD_PRIO_VERY_HIGH	(THREAD_PRIO_NORM + 2)
#define	THREAD_PRIO_RT		sched_get_priority_max()
#define	thread_set_prio(thr, prio) \
	do { \
		struct sched_param param = {}; \
		param.sched_priority = (prio); \
		pthread_setschedparam((thr), SCHED_OTHER, &param); \
	} while (0)

#else	/* APL */
/*
 * BIG CAVEAT: Apparently idle thread prioritization is causing massive
 * thread scheduling stability issues on MacOS Monterey with its Rosetta
 * x86 emulation. Threads either don't get scheduled, or they run in
 * "slow mo", gradually speeding up and generally just behave entirely
 * erratically.
 */
#define	THREAD_PRIO_IDLE	0
#define	THREAD_PRIO_VERY_LOW	0
#define	THREAD_PRIO_LOW		0
#define	THREAD_PRIO_NORM	0
#define	THREAD_PRIO_HIGH	0
#define	THREAD_PRIO_VERY_HIGH	0
#define	THREAD_PRIO_RT		0
#define	thread_set_prio(thr, prio)

#endif	/* APL */

#else	/* !APL && !LIN */

static DWORD _lacf_thread_start_routine(void *arg) UNUSED_ATTR;
static DWORD
_lacf_thread_start_routine(void *arg)
{
	lacf_thread_info_t *ti = (lacf_thread_info_t *)arg;
	ti->proc(ti->arg);
	_lacf_thread_list_remove(ti);
	free(ti);
	return (0);
}

static inline bool_t
lacf_thread_create(thread_t *thrp, void (*proc)(void *), void *arg,
    const char *filename, int linenum)
{
	lacf_thread_info_t *ti =
	    (lacf_thread_info_t *)safe_calloc(1, sizeof (*ti));
	ti->proc = proc;
	ti->arg = arg;
	ti->filename = filename;
	ti->linenum = linenum;
	_lacf_thread_list_init();
	_lacf_thread_list_add(ti);
	if ((*(thrp) = CreateThread(NULL, 0, _lacf_thread_start_routine, ti,
	    0, NULL)) != NULL) {
		/* Start success */
		return (B_TRUE);
	}
	/* Start failure - remove from list and try to destroy the list */
	_lacf_thread_list_remove(ti);
	_lacf_thread_list_fini();
	free(ti);
	return (B_FALSE);
}

#define	thread_join(thrp) \
	do { \
		VERIFY3S(WaitForSingleObject(*(thrp), INFINITE), ==, \
		    WAIT_OBJECT_0); \
		_lacf_thread_list_fini(); \
	} while (0)
#define	thread_set_name(name)		UNUSED(name)

#define	cv_wait(cv, mtx) \
	VERIFY(SleepConditionVariableCS((cv), &(mtx)->cs, INFINITE))
static inline int
cv_timedwait(condvar_t *cv, mutex_t *mtx, uint64_t limit)
{
	uint64_t now = microclock();
	if (now < limit) {
		/*
		 * The only way to guarantee that when we return due to a
		 * timeout the full microsecond-accurate quantum has elapsed
		 * is to round-up to the nearest millisecond.
		 */
		if (SleepConditionVariableCS(cv, &mtx->cs,
		    ceil((limit - now) / 1000.0)) != 0) {
			return (0);
		}
		if (GetLastError() == ERROR_TIMEOUT)
			return (ETIMEDOUT);
		return (-1);
	}
	return (ETIMEDOUT);
}
#define	cv_init		InitializeConditionVariable
#define	cv_destroy(cv)	/* no-op */
#define	cv_signal	WakeConditionVariable
#define	cv_broadcast	WakeAllConditionVariable

#define	THREAD_PRIO_IDLE		THREAD_PRIORITY_IDLE
#define	THREAD_PRIO_VERY_LOW		THREAD_PRIORITY_LOWEST
#define	THREAD_PRIO_LOW			THREAD_PRIORITY_BELOW_NORMAL
#define	THREAD_PRIO_NORM		THREAD_PRIORITY_NORMAL
#define	THREAD_PRIO_HIGH		THREAD_PRIORITY_ABOVE_NORMAL
#define	THREAD_PRIO_VERY_HIGH		THREAD_PRIORITY_HIGHEST
#define	THREAD_PRIO_RT			THREAD_PRIORITY_TIME_CRITICAL
#define	thread_set_prio(thr, prio)	SetThreadPriority((thr), (prio))

#endif	/* !APL && !LIN */

API_EXPORT void lacf_mask_sigpipe(void);

typedef struct {
	mutex_t			lock;
	condvar_t		cv;
	bool_t			write_locked;
	thread_id_t		writer;
	unsigned		refcount;
	unsigned		waiters;
} rwmutex_t;

static void rwmutex_init(rwmutex_t *rw) UNUSED_ATTR;
static void rwmutex_destroy(rwmutex_t *rw) UNUSED_ATTR;
static void rwmutex_enter(rwmutex_t *rw, bool_t write) UNUSED_ATTR;
static void rwmutex_exit(rwmutex_t *rw) UNUSED_ATTR;

static void
rwmutex_init(rwmutex_t *rw)
{
	memset(rw, 0, sizeof (*rw));
	mutex_init(&rw->lock);
	cv_init(&rw->cv);
}

static void
rwmutex_destroy(rwmutex_t *rw)
{
	ASSERT3U(rw->refcount, ==, 0);
	cv_destroy(&rw->cv);
	mutex_destroy(&rw->lock);
}

static void
rwmutex_enter(rwmutex_t *rw, bool_t write)
{
	mutex_enter(&rw->lock);

	if (rw->write_locked && rw->writer != curthread_id) {
		rw->waiters++;
		while (rw->write_locked)
			cv_wait(&rw->cv, &rw->lock);
		rw->waiters--;
	}
	if (write) {
		if (rw->refcount > 0 && rw->writer != curthread_id) {
			rw->waiters++;
			while (rw->refcount > 0)
				cv_wait(&rw->cv, &rw->lock);
			rw->waiters--;
		}
		rw->write_locked = B_TRUE;
		rw->writer = curthread_id;
	}
	rw->refcount++;

	mutex_exit(&rw->lock);
}

static void
rwmutex_exit(rwmutex_t *rw)
{
	mutex_enter(&rw->lock);
	ASSERT3U(rw->refcount, >, 0);
	rw->refcount--;
	if (rw->refcount == 0 && rw->write_locked) {
		ASSERT3U(rw->writer, ==, curthread_id);
		rw->write_locked = B_FALSE;
		rw->writer = 0;
	}
	if (rw->waiters > 0)
		cv_broadcast(&rw->cv);
	mutex_exit(&rw->lock);
}

static inline void
rwmutex_upgrade(rwmutex_t *rw)
{
	rwmutex_exit(rw);
	rwmutex_enter(rw, B_TRUE);
}

static inline bool_t
rwmutex_held_write(rwmutex_t *rw)
{
	return (rw->writer == curthread_id);
}

#ifdef	DEBUG
#define	ASSERT_MUTEX_HELD(mtx)		VERIFY_MUTEX_HELD(mtx)
#define	ASSERT_MUTEX_NOT_HELD(mtx)	VERIFY_MUTEX_NOT_HELD(mtx)
#else	/* !DEBUG */
#define	ASSERT_MUTEX_HELD(mtx)
#define	ASSERT_MUTEX_NOT_HELD(mtx)
#endif	/* !DEBUG */

#ifdef __cplusplus
}
#endif

#endif	/* _ACF_UTILS_THREAD_H_ */
