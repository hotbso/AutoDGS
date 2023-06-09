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

#ifndef	_ACF_UTILS_LIBCONFIG_H_
#define	_ACF_UTILS_LIBCONFIG_H_

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef	CURL_STATICLIB
#define	CURL_STATICLIB
#endif

#ifndef	LIBXML_STATIC
#define	LIBXML_STATIC
#endif

#ifndef	PCRE2_STATIC
#define	PCRE2_STATIC
#endif

#ifndef	PCRE2_CODE_UNIT_WIDTH
#define	PCRE2_CODE_UNIT_WIDTH	8
#endif

#ifndef	GLEW_MX
#define	GLEW_MX
#endif

#if	IBM

#ifndef	CAIRO_WIN32_STATIC_BUILD
#define	CAIRO_WIN32_STATIC_BUILD
#endif

#endif	/* IBM */

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_LIBCONFIG_H_ */
