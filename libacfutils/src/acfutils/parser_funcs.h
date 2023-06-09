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

#ifndef	_ACF_UTILS_PARSER_FUNCS_H_
#define	_ACF_UTILS_PARSER_FUNCS_H_

#include <ctype.h>
#include <string.h>

#include "core.h"
#include "safe_alloc.h"

#ifndef	_LACF_PARSER_FUNCS_INCLUDED
#error	"Don't include parser_funcs.h directly. Include acfutils/helpers.h"
#endif

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Removes all leading & trailing whitespace from a line.
 */
UNUSED_ATTR static void
strip_space(char *line)
{
	char	*p;
	size_t	len = strlen(line);

	/* strip leading whitespace */
	for (p = line; *p != 0 && isspace(*p); p++)
		;
	if (p != line) {
		memmove(line, p, (len + 1) - (p - line));
		len -= (p - line);
	}

	for (p = line + len - 1; p >= line && isspace(*p); p--)
		;
	p[1] = 0;
}

#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
UNUSED_ATTR static ssize_t
parser_get_next_line_impl(void *fp, char **linep, size_t *linecap,
    unsigned *linenum, bool_t compressed)
#else	/* !defined(ACFUTILS_BUILD) && defined(ACFUTILS_GZIP_PARSER) */
UNUSED_ATTR static ssize_t
parser_get_next_line_impl(void *fp, char **linep, size_t *linecap,
    unsigned *linenum)
#endif	/* !defined(ACFUTILS_BUILD) && defined(ACFUTILS_GZIP_PARSER) */
{
	ASSERT(fp != NULL);
	ASSERT(linenum != NULL);

	for (;;) {
#if	defined(ACFUTILS_BUILD) || defined(ACFUTILS_GZIP_PARSER)
		ssize_t len = lacf_getline_impl(linep, linecap, fp, compressed);
#else
		ssize_t len = lacf_getline_impl(linep, linecap, fp);
#endif
		char *hash;
		if (len == -1)
			return (-1);
		(*linenum)++;
		hash = strchr(*linep, '#');
		if (hash != NULL)
		    *hash = '\0';
		strip_space(*linep);
		if (**linep == 0)
			continue;
		len = strlen(*linep);
		/* substitute spaces for tabs */
		for (ssize_t i = 0; i < len; i++) {
			if ((*linep)[i] == '\t')
				(*linep)[i] = ' ';
		}
		return (len);
	}
}

UNUSED_ATTR static char *
parser_get_next_quoted_str2(FILE *fp, int *linep)
{
	char c;
	char *str = (char *)safe_calloc(1, 1);
	size_t len = 0, cap = 0;

	for (;;) {
		do {
			c = fgetc(fp);
			if (c == '\n' && linep != NULL)
				(*linep)++;
		} while (isspace(c));
		if (c == EOF)
			break;
		if (c != '"') {
			ungetc(c, fp);
			break;
		}
		while ((c = fgetc(fp)) != EOF) {
			if (c == '"')
				break;
			if (c == '\\') {
				c = fgetc(fp);
				if (c == 'n') {
					c = '\n';
				} else if (c == 'r') {
					c = '\r';
				} else if (c == 't') {
					c = '\t';
				} else if (c == '\r') {
					/* skip the next LF char as well */
					c = fgetc(fp);
					if (c != '\n' && c != EOF)
						ungetc(c, fp);
					if (linep != NULL)
						(*linep)++;
					continue;
				} else if (c == '\n') {
					/* skip LF char */
					if (linep != NULL)
						(*linep)++;
					continue;
				} else if (c >= '0' && c <= '7') {
					/* 1-3 letter octal codes */
					char num[4];
					int val = 0;

					memset(num, 0, sizeof (num));
					num[0] = c;
					for (int i = 1; i < 3; i++) {
						c = fgetc(fp);
						if (c < '0' || c > '7') {
							ungetc(c, fp);
							break;
						}
						num[i] = c;
					}
					VERIFY(sscanf(num, "%o", &val) == 1);
					c = val;
				}
			}
			if (len == cap) {
				str = (char *)safe_realloc(str, cap + 1 + 128);
				cap += 128;
			}
			str[len++] = c;
		}
		if (c == EOF)
			break;
	}

	str[len] = 0;

	return (str);
}

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_PARSER_FUNCS_H_ */
