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
 * Copyright 2021 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_LACF_GETLINE_IMPL_H_
#define	_ACF_UTILS_LACF_GETLINE_IMPL_H_

#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
#include <zlib.h>
#endif

#include "core.h"
#include "safe_alloc.h"

#ifndef	_LACF_GETLINE_INCLUDED
#error	"Don't include lacf_getline_impl.h directly. Include acfutils/helpers.h"
#endif

#ifdef	__cplusplus
extern "C" {
#endif

#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
UNUSED_ATTR static ssize_t
lacf_getline_impl(char **line_p, size_t *cap_p, void *fp, bool_t compressed)
#else	/* !defined(ACFUTILS_BUILD) && !defined(ACFUTILS_GZIP_PARSER) */
UNUSED_ATTR static ssize_t
lacf_getline_impl(char **line_p, size_t *cap_p, void *fp)
#endif	/* !defined(ACFUTILS_BUILD) && !defined(ACFUTILS_GZIP_PARSER) */
{
	ASSERT(line_p != NULL);
	ASSERT(cap_p != NULL);
	ASSERT(fp != NULL);

	char *line = *line_p;
	size_t cap = *cap_p, n = 0;

#if	(defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)) && \
    (APL || LIN)
	/* On POSIX we can use the libc version when uncompressed */
	if (!compressed)
		return (getline(line_p, cap_p, (FILE *)fp));
#endif	/* defined(ACFUTILS_BUILD) && (APL || LIN) */
	do {
		char *p;

		if (n + 1 >= cap) {
			cap += 256;
			line = (char *)safe_realloc(line, cap);
		}
		ASSERT(n < cap);
#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
		p = (compressed ? gzgets((gzFile)fp, &line[n], cap - n) :
		    fgets(&line[n], cap - n, (FILE *)fp));
#else	/* !defined(ACFUTILS_BUILD) */
		p = fgets(&line[n], cap - n, (FILE *)fp);
#endif	/* !defined(ACFUTILS_BUILD) */
		if (p == NULL) {
			if (n != 0) {
				break;
			} else {
				*line_p = line;
				*cap_p = cap;
				return (-1);
			}
		}
		n = strlen(line);
	} while (n > 0 && line[n - 1] != '\n');

	*line_p = line;
	*cap_p = cap;

	return (n);
}

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_LACF_GETLINE_IMPL_H_ */
