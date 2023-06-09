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

#if	IBM
#include <windows.h>
#else	/* !IBM */
#include <sys/time.h>
#endif	/* !IBM */
#include <stdlib.h>

#include <stdint.h>

#include <acfutils/time.h>

static inline int32_t
is_leap(int32_t year)
{
	if (year % 400 == 0)
		return (1);
	if (year % 100 == 0)
		return (0);
	if (year % 4 == 0)
		return (1);
	return (0);
}

static inline int32_t
days_from_0(int32_t year)
{
	year--;
	return (365 * year + (year / 400) - (year / 100) + (year / 4));
}

static inline int32_t
days_from_1970(int32_t year)
{
	int days_from_0_to_1970 = days_from_0(1970);
	return (days_from_0(year) - days_from_0_to_1970);
}

static inline int32_t
days_from_1jan(int32_t year, int32_t month, int32_t day)
{
	static const int32_t days[2][12] = {
	    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
	    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
	};
	return (days[is_leap(year)][month - 1] + day - 1);
}

time_t
lacf_timegm(const struct tm *t)
{
	int year = t->tm_year + 1900;
	int month = t->tm_mon;
	time_t seconds_in_day, result;
	int day_of_year, days_since_epoch;

	if (month > 11) {
		year += month / 12;
		month %= 12;
	} else if (month < 0) {
		int years_diff = (-month + 11) / 12;
		year -= years_diff;
		month += 12 * years_diff;
	}
	month++;
	if (t->tm_mday != 0) {
		int day = t->tm_mday;
		day_of_year = days_from_1jan(year, month, day);
	} else {
		day_of_year = t->tm_yday;
	}
	days_since_epoch = days_from_1970(year) + day_of_year;

	seconds_in_day = 3600 * 24;
	result = seconds_in_day * days_since_epoch + 3600 * t->tm_hour +
	    60 * t->tm_min + t->tm_sec;

	return (result);
}
