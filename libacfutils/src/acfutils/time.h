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
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_TIME_H_
#define	_ACF_UTILS_TIME_H_

#include <stdint.h>
#include <time.h>

#include "assert.h"
#include "core.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	USEC2SEC(usec)	((usec) / 1000000.0)
#define	SEC2USEC(sec)	((sec) * 1000000ll)
#define	NSEC2SEC(usec)	((usec) / 1000000000.0)
#define	SEC2NSEC(sec)	((sec) * 1000000000ll)

static inline uint64_t
microclock(void)
{
#if	IBM
	LARGE_INTEGER val, freq;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&val);
	return ((val.QuadPart * 1000000llu) / freq.QuadPart);
#else	/* !IBM */
	struct timespec ts;
	VERIFY0(clock_gettime(CLOCK_REALTIME, &ts));
	return ((ts.tv_sec * 1000000llu) + (ts.tv_nsec / 1000llu));
#endif
}

/* Not portable */
static inline uint64_t
nanoclock(void)
{
#if	IBM
	return (microclock() * 1000llu);
#else	/* !IBM */
	struct timespec ts;
	VERIFY0(clock_gettime(CLOCK_MONOTONIC, &ts));
	return (ts.tv_sec * 1000000000llu + ts.tv_nsec);
#endif	/* !IBM */
}

/*
 * Converts a struct tm specified in UTC (not local time) into unixtime.
 * This only considers the following fields from the tm structure, tm_year,
 * tm_yday, tm_hour, tm_min and tm_sec. Any other fields are ignored.
 */
time_t lacf_timegm(const struct tm *tm);

/*
 * Returns time in the system's real time clock as the number of microseconds
 * since UTC 1970-01-01 (unixtime). In essence, this is a microsecond-
 * accurate time_t.
 */
static inline uint64_t
lacf_microtime(void)
{
#if	IBM
	/*
	 * On windows, we emulate this by simply adding the millisecond
	 * portion onto the whole Unixtime seconds provided by time().
	 */
	uint64_t time_micro = ((uint64_t)time(NULL)) * 1000000llu;
	SYSTEMTIME st;
	GetSystemTime(&st);
	return (time_micro + (uint64_t)st.wMilliseconds * 1000llu);
#else	/* !IBM */
	/* On Mac & Linux, microclock already does this */
	return (microclock());
#endif	/* !IBM */
}

/*
 * Takes day-of-year (or an X-Plane "local_date_days" value) and converts
 * it to month + day-of-month in the format used in the "tm" structure
 * (see the C89 standard gmtime, ctime, asctime or localtime functions).
 *
 * @param days The input value of the number of days since January 1.
 *	The function always assumes a non-leap year (consistent with
 *	X-Plane's behavior).
 * @param tm_mon If not NULL, this is filled with the month (0-11).
 * @param tm_mday If not NULL, this is filled with the day-of-month (1-31).
 */
static inline void
lacf_yday_to_mon_mday(unsigned days, int *tm_mon, int *tm_mday)
{
	static const unsigned month2days[12] = {
	    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
	};
	for (int i = 11; i >= 0; i--) {
		if (month2days[i] <= days) {
			if (tm_mon != NULL)
				*tm_mon = i;
			if (tm_mday != NULL)
				*tm_mday = days - month2days[i] + 1;
			return;
		}
	}
	VERIFY_FAIL();
}

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_TIME_H_ */
