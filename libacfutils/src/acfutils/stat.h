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
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_STAT_H_
#define	_ACF_UTILS_STAT_H_

#include <stdint.h>
#include <time.h>

#include "core.h"

#ifdef	__cplusplus
extern "C" {
#endif

#if	IBM

/* A minimally compatible POSIX-style file stat reading implementation */
#define	stat		lacf_stat
struct stat {
	uint64_t	st_size;
	time_t		st_atime;
	time_t		st_mtime;
};
API_EXPORT int stat(const char *pathname, struct stat *buf);

#else	/* !IBM */

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#endif	/* !IBM */

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_STAT_H_ */
