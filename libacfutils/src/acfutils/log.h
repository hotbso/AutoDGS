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
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * This subsystem provides a convenient message logging facility.
 * You must initialize this subsystem prior to using any other part
 * of libacfutils which may emit log messages (which is nearly all
 * of them). Call log_init() with a suitable logging callback in the
 * first argument. On shutdown, call log_fini() to make sure all
 * memory resources of the logging system are freed. To log a
 * message, use the logMsg() macro.
 * @see log_init()
 * @see log_fini()
 * @see logMsg()
 */

#ifndef	_ACF_UTILS_LOG_H_
#define	_ACF_UTILS_LOG_H_

#include <stdarg.h>

#include "sysmacros.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Before using any of the log_* functionality, be sure to properly
 * initialize it and pass it a logging function!
 */
typedef void (*logfunc_t)(const char *);
API_EXPORT void log_init(logfunc_t func, const char *prefix);
API_EXPORT void log_fini(void);
API_EXPORT void log_xplm_cb(const char *str);

#if	defined(__GNUC__) || defined(__clang__)
#define	BUILTIN_STRRCHR	__builtin_strrchr
#else	/* !defined(__GNUC__) && !defined(__clang__) */
#define	BUILTIN_STRRCHR	strrchr
#endif	/* !defined(__GNUC__) && !defined(__clang__) */

/**
 * This lets us chop out the basename (last path component) from __FILE__
 * at compile time. This works on GCC and Clang. The fallback mechanism
 * below just chops it out at compile time.
 *
 * This macro is used in logMsg() to only extract the source filename
 * of the logMsg() call. You can also use it to extract the last path
 * component in any other macros you write.
 *
 * @note This macro requires that the argument be a string literal at
 *	compile time. The underlying implementation is using compiler
 *	built-ins to make sure only the last path component of the
 *	string is stored in the final compiled binary.
 */
#define	log_basename(f)	\
	(BUILTIN_STRRCHR(f, '/') ? BUILTIN_STRRCHR(f, '/') + 1 : \
	    (BUILTIN_STRRCHR(f, '\\') ? BUILTIN_STRRCHR(f, '\\') + 1 : (f)))
/**
 * This macro is the primary logging facility in libacfutils. Its arguments
 * are a printf-like format and optional format arguments, which will be
 * sent to the logging function specified in log_init().
 *
 * @note Before using any logging function of libacfutils, you **must** call
 *	log_init() with a suitable logging callback function.
 *
 * The logging function automatically constructs the string as follows:
 *```
 * YYYY-MM-DD HH:MM:SS PREFIX[FILENAME:LINE]: <your message goes here>
 *```
 * - `PREFIX` is the prefix you provided in the second argument of log_init().
 * - `FILENAME` is the name of the file in which the logMsg() macro was placed.
 * - `LINE` is the line number in the file where the logMsg() macro was placed.
 * - The remainder of the string is formatted using the printf-style arguments
 *	to the logMsg() macro.
 *
 * @see log_init()
 */
#define	logMsg(...) \
	log_impl(log_basename(__FILE__), __LINE__, __VA_ARGS__)
/**
 * Same as logMsg(), but allows you to provide a va_list argument list.
 * This allows you to nest logMsg() invocations inside of your own custom
 * variadic functions.
 */
#define	logMsg_v(fmt, ap) \
	log_impl_v(log_basename(__FILE__), __LINE__, (fmt), (ap))
API_EXPORT void log_impl(const char *filename, int line,
    PRINTF_FORMAT(const char *fmt), ...) PRINTF_ATTR(3);
API_EXPORT void log_impl_v(const char *filename, int line, const char *fmt,
    va_list ap);
API_EXPORT void log_backtrace(int skip_frames);
#if	IBM
API_EXPORT void log_backtrace_sw64(PCONTEXT ctx);
#endif

#ifdef __cplusplus
}
#endif

#endif	/* _ACF_UTILS_LOG_H_ */
