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
 * This file contains helper functions mostly concerned with text and
 * string processing.
 */
#ifndef	_ACF_UTILS_HELPERS_H_
#define	_ACF_UTILS_HELPERS_H_

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>
#if	APL || LIN
#include <sys/stat.h>
#include <dirent.h>	/* to bring in DIR, opendir, readdir & friends */
#include <unistd.h>
#endif
#include <time.h>

#if	IBM
#include <windows.h>
#endif	/* IBM */

#define	_LACF_GETLINE_INCLUDED
#include "lacf_getline_impl.h"
#define	_LACF_PARSER_FUNCS_INCLUDED
#include "parser_funcs.h"
#include "math_core.h"
#include "sysmacros.h"
#include "safe_alloc.h"
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * @return True if `pos` is a validate geographic coordinate (the latitude,
 *	longitude and elevation are sensible values). This is using the
 *	is_valid_lat(), is_valid_lon() and is_valid_elev() functions.
 *	Please note that this only accepts elevation values between -2000
 *	and +30000. If your elevations are in, e.g. feet, 30000 is a very
 *	low max elevation value to check against. In that case, you should
 *	use IS_VALID_GEO_POS2(), or write a custom check macro that
 *	uses the is_valid_alt_ft() check function instead.
 * @see is_valid_lat()
 * @see is_valid_lon()
 * @see is_valid_elev()
 * @see is_valid_alt_ft()
 */
#define	IS_VALID_GEO_POS3(pos) \
	(is_valid_lat((pos).lat) && is_valid_lon((pos).lon) && \
	is_valid_elev((pos).elev))
/**
 * Same as IS_VALID_GEO_POS3(), but for 2-space geographic coordinates
 * without elevation.
 * @see is_valid_lat()
 * @see is_valid_lon()
 */
#define	IS_VALID_GEO_POS2(pos) \
	(is_valid_lat((pos).lat) && is_valid_lon((pos).lon))
/**
 * @return True if `lat` is a valid latitude value. That means the value
 *	is not NAN and lay between -90 and +90 (inclusive).
 */
static inline bool_t
is_valid_lat(double lat)
{
	return (!isnan(lat) && fabs(lat) <= 90);
}
/**
 * \deprecated
 * Synonym for is_valid_lat().
 */
static inline bool_t
is_valid_lat_polar(double lat)
{
	return (!isnan(lat) && fabs(lat) <= 90);
}
/**
 * @return True if `lon` is a valid longitude value. That means the value
 *	is not NAN and lay between -180 and +180 (inclusive).
 */
static inline bool_t
is_valid_lon(double lon)
{
	return (!isnan(lon) && fabs(lon) <= 180.0);
}
/**
 * @return True if `elev` is a valid elevation value in meters. That means
 *	the value is not NAN and is between `MIN_ELEV` (-2000) and `MAX_ELEV`
 *	(30000) inclusive. The range check is really just to make sure the
 *	value is within sensible limits.
 * @see is_valid_alt_ft()
 */
static inline bool_t
is_valid_elev(double elev)
{
	return (!isnan(elev) && elev >= MIN_ELEV && elev <= MAX_ELEV);
}

#ifdef	LACF_ENABLE_LEGACY_IS_VALID_ALT
#define	is_valid_alt(alt)	is_valid_alt_ft(alt)
#endif
/**
 * @return True if `alt_ft` is a valid altitude value. That means the value
 *	is not NAN and is between `MIN_ALT` (-2000) and `MAX_ALT` (100000)
 *	inclusive. The range check is really just to make sure the value is
 *	within sensible limits.
 * @see is_valid_elev()
 */
static inline bool_t
is_valid_alt_ft(double alt_ft)
{
	return (!isnan(alt_ft) && alt_ft >= MIN_ALT && alt_ft <= MAX_ALT);
}
/**
 * Variant of is_valid_alt_ft() but expects the input altitude to be in
 * meters.
 * @see is_valid_alt_ft()
 */
static inline bool_t
is_valid_alt_m(double alt_m)
{
	return (!isnan(alt_m) && alt_m >= MIN_ALT / 3.2808398950131 &&
	    alt_m <= MAX_ALT / 3.2808398950131);
}
/**
 * @return True if `spd` is a valid speed value. That means the value
 *	is not NAN and is between 0 and `MAX_SPD` (1000) inclusive. This
 *	expects the speed value to be in knots.
 */
static inline bool_t
is_valid_spd(double spd)
{
	return (!isnan(spd) && spd >= 0.0 && spd <= MAX_SPD);
}
/**
 * @return True if `hdg` is a valid heading value. That means the value
 *	is not NAN and is between 0 and 360 (inclusive).
 */
static inline bool_t
is_valid_hdg(double hdg)
{
	return (!isnan(hdg) && hdg >= 0.0 && hdg <= 360.0);
}
/**
 * Calculates relative heading from `hdg1` to `hdg2`. Both heading values
 * MUST be valid headings (pass the is_valid_hdg() check), otherwise
 * an assertion failure is triggered.
 * @return The number of degrees to turn from `hdg1` to `hdg2` "the shortest
 *	way". That means, if `hdg2` is "to the right" of `hdg1` (i.e. less
 *	than +180 degrees), the return value is in the positive range
 *	of +0 to +180 inclusive. Conversely, if the target is to the left,
 *	the return value will be between -0 and -180 inclusive. Please
 *	note that due to angle wrapping, you cannot simply add the result
 *	of rel_hdg() onto another heading and expect the result to be
 *	valid, such as:
 *```
 * double new_hdg = hdg1 + rel_hdg(hdg1, hdg2);
 *```
 * The value in `new_hdg` could well be >360 or <0 in this case.
 *	To properly add headings together and end up with something that
 *	passes the is_valid_hdg() test again, you always want to
 *	re-normalize the result using normalize_hdg():
 *```
 * double new_hdg = normalize_hdg(hdg1 + rel_hdg(hdg1, hdg2));
 * // new_hdg will now be equivalent to hdg2
 *```
 * @see normalize_hdg()
 */
#define	rel_hdg(hdg1, hdg2)	rel_hdg_impl(hdg1, hdg2, __FILE__, __LINE__)
API_EXPORT double rel_hdg_impl(double hdg1, double hdg2, const char *file,
    int line);

/**
 * Renormalizes a heading value that lies outside of the 0-360 inclusive
 * range. Basically this takes care of undoing "angle wrapping".
 *
 * #### Example:
 *```
 * normalize_hdg(90)  => 90
 * normalize_hdg(-90) => 270
 * normalize_hdg(400) => 40
 * normalize_hdg(NAN) => NAN
 *```
 */
static inline double
normalize_hdg(double hdg)
{
	if (isnan(hdg))
		return (hdg);
	hdg = fmod(hdg, 360);
	/* Flip negative into positive */
	if (hdg < 0.0)
		hdg += 360.0;
	/* Necessary to avoid FP rounding errors */
	if (hdg <= 0.0 || hdg > 360.0) {
		hdg = clamp(hdg, 0, 360);
		/* Avoid negative zero */
		if (hdg == -0.0)
			hdg = 0.0;
	}
	return (hdg);
}
/**
 * Renormalizes a longitude value. This is similar to normalize_hdg(), but
 * instead of resolving angle wrapping into the 0-360 range, this resolves
 * the output to be between -180..+180 (inclusive):
 *```
 * normalize_lon(100) => 100
 * normalize_lon(200) => -160
 * normalize_lon(300) => -60
 * normalize_lon(400) => 40
 *```
 */
static inline double
normalize_lon(double lon)
{
	while (lon > 180.0)
		lon -= 360.0;
	while (lon < -180.0)
		lon += 360.0;
	return (clamp(lon, -180, 180));
}

API_EXPORT bool_t is_valid_icao_code(const char *icao);
API_EXPORT bool_t is_valid_iata_code(const char *iata);
API_EXPORT const char *extract_icao_country_code(const char *icao);

API_EXPORT bool_t is_valid_xpdr_code(int code);
API_EXPORT bool_t is_valid_vor_freq(double freq_mhz);
/**
 * Same as is_valid_vor_freq(), but takes an integer frequency in Hz
 * instead of a floating-point value in MHz.
 * @see is_valid_vor_freq()
 */
static inline bool_t
is_valid_vor_freq_hz(uint32_t freq_hz)
{
	return (is_valid_vor_freq(freq_hz / 1000000.0));
}
/**
 * Same as is_valid_vor_freq(), but takes an integer frequency in kHz
 * instead of a floating-point value in MHz.
 * @see is_valid_vor_freq()
 */
static inline bool_t
is_valid_vor_freq_khz(uint32_t freq_khz)
{
	return (is_valid_vor_freq(freq_khz / 1000.0));
}

API_EXPORT bool_t is_valid_loc_freq(double freq_mhz);
/**
 * Same as is_valid_loc_freq(), but takes an integer frequency in Hz
 * instead of a floating-point value in MHz.
 * @see is_valid_loc_freq()
 */
static inline bool_t
is_valid_loc_freq_hz(uint32_t freq_hz)
{
	return (is_valid_loc_freq(freq_hz / 1000000.0));
}
/**
 * Same as is_valid_loc_freq(), but takes an integer frequency in kHz
 * instead of a floating-point value in MHz.
 * @see is_valid_loc_freq()
 */
static inline bool_t
is_valid_loc_freq_khz(uint32_t freq_khz)
{
	return (is_valid_loc_freq(freq_khz / 1000.0));
}

API_EXPORT bool_t is_valid_ndb_freq(double freq_khz);
/**
 * Same as is_valid_ndb_freq(), but takes an integer frequency in Hz
 * instead of a floating-point value in kHz.
 * @see is_valid_loc_freq()
 */
static inline bool_t is_valid_ndb_freq_hz(uint32_t freq_hz)
{
	return (is_valid_ndb_freq(freq_hz / 1000.0));
}

API_EXPORT bool_t is_valid_tacan_freq(double freq_mhz);
API_EXPORT bool_t is_valid_rwy_ID(const char *rwy_ID);
API_EXPORT void copy_rwy_ID(const char *src, char dst[4]);

/* AIRAC date functions */
API_EXPORT const char *airac_cycle2eff_date(int cycle);
API_EXPORT time_t airac_cycle2eff_date2(int cycle);
API_EXPORT bool_t airac_cycle2exp_date(int cycle, char buf[16],
    time_t *cycle_end_p);
API_EXPORT int airac_time2cycle(time_t t);

/* CSV file & string processing helpers */
/**
 * Grabs the next non-empty, non-comment line from a file, having stripped
 * away all leading and trailing whitespace. Any tab characters are also
 * replaced with spaces. Comments are lines that start with a '#' character.
 * Also, any text following a '#' on a line is stripped away and considered
 * a comment.
 *
 * This function is useful for writing a custom config file parser. It can
 * also be used to consume X-Plane OBJ and similar files, or generally any
 * text-based data file which uses '#' for comments. See also
 * conf_create_empty(), conf_read_file() and the conf.h file for a
 * fully-featured config system already included with libacfutils.
 *
 * @param fp File from which to retrieve the line.
 * @param linep Line buffer which will hold the new line. If the buffer pointer
 *	is set to NULL, it will be allocated. If it is not long enough, it
 *	will be expanded.
 * @param linecap The capacity of *linep. If set to zero a new buffer is
 *	allocated.
 * @param linenum The current line number. Will be advanced by 1 for each
 *	new line read.
 * @return The number of characters in the line (after stripping whitespace)
 *	without the terminating NUL.
 * @see conf_create_empty()
 * @see conf_read_file()
 */
UNUSED_ATTR static ssize_t
parser_get_next_line(FILE *fp, char **linep, size_t *linecap, unsigned *linenum)
{
#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
	return (parser_get_next_line_impl(fp, linep, linecap, linenum,
	    B_FALSE));
#else	/* !defined(ACFUTILS_BUILD) && !defined(ACFUTILS_GZIP_PARSER) */
	return (parser_get_next_line_impl(fp, linep, linecap, linenum));
#endif	/* !defined(ACFUTILS_BUILD) && !defined(ACFUTILS_GZIP_PARSER) */
}

#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
/*
 * Same as parser_get_next_line, but for gzip-compressed files.
 */
UNUSED_ATTR static ssize_t
parser_get_next_gzline(void *gz_fp, char **linep, size_t *linecap,
    unsigned *linenum)
{
	return (parser_get_next_line_impl(gz_fp, linep, linecap, linenum,
	    B_TRUE));
}
#endif	/* defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER) */

/**
 * Legacy bridge to parser_get_next_quoted_str2() function without
 * the optional second argument.
 * @see parser_get_next_quoted_str2()
 */
UNUSED_ATTR static char *
parser_get_next_quoted_str(FILE *fp)
{
	return (parser_get_next_quoted_str2(fp, NULL));
}

API_EXPORT ssize_t explode_line(char *line, char delim, char **comps,
    size_t capacity);
API_EXPORT void append_format(char **str, size_t *sz,
    PRINTF_FORMAT(const char *format), ...) PRINTF_ATTR(3);
/**
 * Converts all whitespace in a string into plain ASCII space characters.
 * This allows for easier splitting of a string at whitespace boundaries
 * using functions such as strsplit(). First run the input to be split
 * through normalize_whitespace() to make sure that any tabs are converted
 * into plain ASCII spaces first and then use strsplit() to separate the
 * line using the " " separator as the field delimeter.
 */
static inline void
normalize_whitespace(char *str)
{
	for (int i = 0, n = strlen(str); i < n; i++) {
		if (isspace(str[i]))
			str[i] = ' ';
	}
}

/* string processing helpers */
API_EXPORT char **strsplit(const char *input, const char *sep,
    bool_t skip_empty, size_t *num);
/**
 * Invokes the free_strlist() function on the macro arguments and
 * then sets both arguments to `NULL` and `0` respectively, to help
 * prevent inadvertent reuse.
 */
#define	DESTROY_STRLIST(comps, num) \
	do { \
		free_strlist((comps), (num)); \
		(comps) = NULL; \
		(num) = 0; \
	} while (0)
API_EXPORT void free_strlist(char **comps, size_t num);
API_EXPORT void unescape_percent(char *str);

API_EXPORT char *mkpathname(const char *comp, ...) SENTINEL_ATTR;
API_EXPORT char *mkpathname_v(const char *comp, va_list ap);
API_EXPORT void fix_pathsep(char *str);

API_EXPORT char *path_last_comp_subst(const char *path, const char *replace);
API_EXPORT char *path_last_comp(const char *path);
API_EXPORT char *path_ext_subst(const char *path, const char *ext);
API_EXPORT void path_normalize(char *path);

API_EXPORT char *file2str(const char *comp, ...) SENTINEL_ATTR;
API_EXPORT char *file2str_ext(long *len_p, const char *comp, ...) SENTINEL_ATTR;
API_EXPORT char *file2str_name(long *len_p, const char *filename);
API_EXPORT void *file2buf(const char *filename, size_t *bufsz);
API_EXPORT ssize_t filesz(const char *filename);

/*
 * strlcpy is a BSD function not available on Windows, so we roll a simple
 * version of it ourselves. Can't inline this function, because GCC's fucked
 * up array bounds checker will squawk endlessly when this is used to copy
 * strings to a fixed-size array.
 */
#if	IBM || LIN
#define	strlcpy				lacf_strlcpy
#endif
void lacf_strlcpy(char *restrict dest, const char *restrict src, size_t cap);

/**
 * Portable version of the POSIX basename() function.
 * @see @see [basename()](https://linux.die.net/man/3/basename)
 */
static inline const char *
lacf_basename(const char *str)
{
	const char *sep = strrchr(str, DIRSEP);

#if	IBM
	const char *sep2 = strrchr(str, '/');
	if (sep2 > sep)
		sep = sep2;
#endif	/* IBM */

	if (sep == NULL)
		return (str);
	return (&sep[1]);
}

/**
 * Portable version of the POSIX getline() function.
 * @see [getline()](https://linux.die.net/man/3/getline)
 */
UNUSED_ATTR static ssize_t
lacf_getline(char **lineptr, size_t *n, FILE *stream)
{
#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
	return (lacf_getline_impl(lineptr, n, stream, B_FALSE));
#else
	return (lacf_getline_impl(lineptr, n, stream));
#endif
}

#if	IBM && !defined(__cplusplus) && \
	(defined(_GNU_SOURCE) || defined(_POSIX_C_SOURCE))
#define	getline				lacf_getline
#endif

API_EXPORT void strtolower(char *str);
API_EXPORT void strtoupper(char *str);

/**
 * Variant of sprintf_alloc(), but which takes a va_list argument list
 * as its second argument, to allow nesting it in other variadic functions.
 * @param fmt Format string conforming to printf() formatting syntax. The
 *	remaining arguments must match the requirements of the format string.
 * @param ap Argument list containing all the arguments required by `fmt`.
 * @return A newly allocated string holding the result of the printf
 *	operation. Please note that the allocation is done using the
 *	caller's heap allocator, so you should use the normal free()
 *	function to free the associated memory, NOT lacf_free().
 */
static inline char *
vsprintf_alloc(const char *fmt, va_list ap)
{
	va_list ap2;
	int l;
	char *str;

	ASSERT(fmt != NULL);

	va_copy(ap2, ap);
	l = vsnprintf(NULL, 0, fmt, ap2);
	va_end(ap2);

	ASSERT(l >= 0);
	str = (char *)safe_malloc(l + 1);
	(void)vsnprintf(str, l + 1, fmt, ap);

	return (str);
}

/**
 * Convenience function which allocates a new string of sufficient storage
 * to hold a printf-formatted string. This removes the need to run the
 * standard library C sprintf once to find out the length of a string,
 * allocate storage, and then run it again to fill the storage.
 * @param fmt Format string conforming to printf() formatting syntax. The
 *	remaining arguments must match the requirements of the format string.
 * @return A newly allocated string holding the result of the printf
 *	operation. Please note that the allocation is done using the
 *	caller's heap allocator, so you should use the normal free()
 *	function to free the associated memory, NOT lacf_free().
 */
PRINTF_ATTR(1) static inline char *
sprintf_alloc(PRINTF_FORMAT(const char *fmt), ...)
{
	va_list ap;
	char *str;

	ASSERT(fmt != NULL);

	va_start(ap, fmt);
	str = vsprintf_alloc(fmt, ap);
	va_end(ap);

	return (str);
}

/**
 * Portable version of BSD & POSIX strcasecmp().
 * This is a case-insensitive variant strcmp().
 * @see [strcmp()](https://linux.die.net/man/3/strcmp)
 * @see [strcasecmp()](https://linux.die.net/man/3/strcasecmp)
 */
static inline int
lacf_strcasecmp(const char *s1, const char *s2)
{
	int l1, l2, res;

	ASSERT(s1 != NULL);
	ASSERT(s2 != NULL);

	l1 = strlen(s1);
	l2 = strlen(s2);

	if (l1 < 4096 && l2 < 4096) {
		char s1_lower[l1 + 1], s2_lower[l2 + 1];

		lacf_strlcpy(s1_lower, s1, l1 + 1);
		lacf_strlcpy(s2_lower, s2, l2 + 1);
		strtolower(s1_lower);
		strtolower(s2_lower);
		res = strcmp(s1_lower, s2_lower);
	} else {
		char *s1_lower = (char *)safe_malloc(l1 + 1);
		char *s2_lower = (char *)safe_malloc(l2 + 1);

		lacf_strlcpy(s1_lower, s1, l1 + 1);
		lacf_strlcpy(s2_lower, s2, l2 + 1);
		strtolower(s1_lower);
		strtolower(s2_lower);
		res = strcmp(s1_lower, s2_lower);
		free(s1_lower);
		free(s2_lower);
	}
	return (res);
}

/**
 * Portable version of BSD & POSIX strcasestr().
 * This is a case-insensitive variant strstr().
 * @see [strstr() and strcasestr()](https://linux.die.net/man/3/strstr)
 */
static inline char *
lacf_strcasestr(const char *haystack, const char *needle)
{
	int l1, l2;
	char *res;

	ASSERT(haystack != NULL);
	ASSERT(needle != NULL);

	l1 = strlen(haystack);
	l2 = strlen(needle);

	if (l1 < 4096 && l2 < 4096) {
		char haystack_lower[l1 + 1], needle_lower[l2 + 1];

		lacf_strlcpy(haystack_lower, haystack, l1 + 1);
		lacf_strlcpy(needle_lower, needle, l2 + 1);
		strtolower(haystack_lower);
		strtolower(needle_lower);
		res = strstr(haystack_lower, needle_lower);
		if (res != NULL)
			res = (char *)haystack + (res - haystack_lower);
	} else {
		char *haystack_lower = (char *)safe_malloc(l1 + 1);
		char *needle_lower = (char *)safe_malloc(l2 + 1);

		lacf_strlcpy(haystack_lower, haystack, l1 + 1);
		lacf_strlcpy(needle_lower, needle, l2 + 1);
		strtolower(haystack_lower);
		strtolower(needle_lower);
		res = strstr(haystack_lower, needle_lower);
		if (res != NULL)
			res = (char *)haystack + (res - haystack_lower);
		free(haystack_lower);
		free(needle_lower);
	}
	return (res);
}

/**
 * Calculates the number of digits after a decimal point for a printf
 * format string to make the number appear with a fixed number of digits
 * as a whole.
 * @param x The input number that is to be formatted. This should be the
 *	same number as that which will be used in the printf format
 *	value.
 * @param digits The number of digits that the final number should have.
 *
 * #### Example:
 *
 * Say we want a number to take up room for 4 digits, adding decimal
 * digits after the point as necessary to padd the number:
 *
 * - 0.001 becomes "0.00" - rounding is applied as appropriate
 * - 0.01 becomes "0.01"
 * - 0.1 becomes "0.10"
 * - 1 becomes "5.00"
 * - 10 becomes "10.0"
 * - 100 becomes "100" - this is rounded to less than 4 digits, since
 *	adding a '.0' at the end would overflow the desired length.
 * - 1000 remains as "1000"
 *
 * For numbers greater than 9999, the final number will have more than
 * 4 digits. Use the fixed_decimals() function to achieve the above
 * effect in a printf format as follows:
 *```
 * double foo = 1.1;
 * printf("The number is %.*f\n", fixed_decimals(foo, 4), foo);
 * // this prints "The number is 1.10"
 *```
 */
static inline int
fixed_decimals(double x, int digits)
{
	if (x > -1e-10 && x < 1e-10)
		return (MAX(digits - 1, 0));
	if (x < 0)
		x *= -1;
	/* This avoids the leading "0." not counting to the digit number */
	if (x < 1)
		digits = MAX(digits - 1, 0);
	return (clampi(digits - ceil(log10(x)), 0, digits));
}

API_EXPORT size_t utf8_char_get_num_bytes(const char *str);
API_EXPORT size_t utf8_get_num_chars(const char *str);

/**
 * @return `x` rounded up to the nearest power-of-2.
 */
#define	P2ROUNDUP(x)	(-(-(x) & -(1 << highbit64(x))))
/** Rounds `x` to the nearest multiple of `y` */
static inline double
roundmul(double x, double y)
{
	return (round(x / y) * y);
}
/** Rounds `x` DOWN to the nearest multiple of `y` */
static inline double
floormul(double x, double y)
{
	return (floor(x / y) * y);
}
/**
 * Helper macro to either set or reset a bit field in an integer.
 * @param out_var The integer variable where the bitfield will be set or reset.
 * @param bit_mask A mask of the bitfield to be manipulated. These are the
 *	bits that will be set or reset.
 * @param bit_value An condition value. If non-zero, the bits in `bit_mask`
 *	will be set to 1 in `out_var`. If zero, the bits in `bit_mask` will
 *	be reset to 0 in `out_var`.
 *
 * #### Example
 *```
 * #define FOO_FEATURE_FLAG 0x1
 * #define BAR_FEATURE_FLAG 0x2
 * int feature_flags = 0;
 *
 * SET_BITFIELD_1(feature_flags, FOO_FEATURE_FLAG, B_TRUE);
 * // feature_flags will now be 0x1 (FOO_FEATURE_FLAG set)
 * SET_BITFIELD_1(feature_flags, BAR_FEATURE_FLAG, B_TRUE);
 * // feature_flags will now be 0x3 (FOO_FEATURE_FLAG | BAR_FEATURE_FLAG)
 * SET_BITFIELD_1(feature_flags, FOO_FEATURE_FLAG, B_FALSE);
 * // feature_flags will now be 0x2 (BAR_FEATURE_FLAG)
 *```
 * A more natural way to use this macro is to use the return value of
 * a check function the third argument:
 *```
 * SET_BITFIELD_1(feature_flags, FOO_FEATURE_FLAG, foo_feature_is_on());
 * // feature_flags will now either have FOO_FEATURE_FLAG bit set or
 * // cleared, depending on the return value of foo_feature_is_on().
 *```
 */
#define	SET_BITFIELD_1(out_var, bit_mask, bit_value) \
	do { \
		if (bit_value) \
			(out_var) |= (bit_mask); \
		else \
			(out_var) &= ~(bit_mask); \
	} while (0)

/* file/directory manipulation */
API_EXPORT bool_t file_exists(const char *path, bool_t *isdir);
API_EXPORT bool_t create_directory(const char *dirname);
API_EXPORT bool_t create_directory_recursive(const char *dirname);
API_EXPORT bool_t remove_directory(const char *dirname);
API_EXPORT bool_t remove_file(const char *filename, bool_t notfound_ok);

API_EXPORT char *lacf_dirname(const char *filename);

#if	IBM && (defined(_GNU_SOURCE) || defined(_POSIX_C_SOURCE))

/* A minimally compatible POSIX-style directory reading implementation */
struct dirent {
	char		d_name[256];
};
typedef struct {
	HANDLE		handle;
	WIN32_FIND_DATA	find_data;
	bool_t		first;
	struct dirent	de;
} DIR;

API_EXPORT DIR *opendir(const char *path);
API_EXPORT struct dirent *readdir(DIR *dirp);
API_EXPORT void closedir(DIR *dirp);

#define	sleep(x)	SleepEx((x) * 1000, FALSE)
#define	usleep(x)	SleepEx((x) / 1000, FALSE)

#endif	/* IBM && (defined(_GNU_SOURCE) || defined(_POSIX_C_SOURCE)) */

#if	IBM

API_EXPORT void win_perror(DWORD err, PRINTF_FORMAT(const char *fmt), ...)
    PRINTF_ATTR(2);

#endif	/* IBM */

API_EXPORT void lacf_qsort_r(void *base, size_t nmemb, size_t size,
    int (*compar)(const void *, const void *, void *), void *arg);

/**
 * This function is a portable and thread-safe version of the gmtime()
 * function. The problem with plain standard C gmtime() is that it is not
 * thread-safe, because the returned argument is a pointer into a shared
 * memory buffer, allocated inside of the standard C library. As such,
 * time calls occurring in other threads might clobber the buffer.
 *
 * The lacf_gmtime_r() function calls the platform-specific thread-safe
 * version of gmtime(). On macOS and Linux, this uses the gmtime_r()
 * function, whereas on Windows, this uses the _gmtime64_s() function.
 *
 * @param tim Pointer to a `time_t`, which is to be converted.
 * @param tm Pointer to a `struct tm` structure, which will be
 *	filled with the broken-down UTC time corresponding to `tim`.
 * @return True if the underlying gmtime_r() call succeeded.
 * @see [gmtime()](https://linux.die.net/man/3/gmtime)
 */
static inline bool_t
lacf_gmtime_r(const time_t *tim, struct tm *tm)
{
#if	defined(__STDC_LIB_EXT1__) || IBM
	return (_gmtime64_s(tm, tim) == 0);
#else	/* !defined(__STDC_LIB_EXT1__) && !IBM */
	return (gmtime_r(tim, tm) != NULL);
#endif	/* !defined(__STDC_LIB_EXT1__) && !IBM */
}


#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_HELPERS_H_ */
