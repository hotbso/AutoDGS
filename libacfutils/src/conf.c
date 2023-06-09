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

#include <ctype.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "acfutils/assert.h"
#include "acfutils/avl.h"
#include "acfutils/base64.h"
#include "acfutils/conf.h"
#include "acfutils/helpers.h"
#include "acfutils/log.h"
#include "acfutils/safe_alloc.h"

/* For conf_get_da and conf_set_da. */
CTASSERT(sizeof (double) == sizeof (unsigned long long));

/*
 * This is a general-purpose configuration store. It's really just a
 * key-value pair dictionary that can be read from and written to a file.
 *
 * The file format is very simple, consisting of a simple sequence of
 * lines like the following:
 *
 * key = value
 *
 * In addition to being able to return the full-text values of keys, this
 * set functions also allows you to easily parse the data in a variety of
 * formats (integers, floats, booleans, etc.). The file format also allows
 * for comments, so it is usable as a user-written configuration parser.
 * Lines beginning with "#" or "--" are automatically skipped.
 */

struct conf {
	avl_tree_t	tree;
};

typedef struct {
	char		*key;
	char		*value;
	avl_node_t	node;
} conf_key_t;

static void conf_set_common(conf_t *conf, const char *key,
    const char *fmt, ...) PRINTF_ATTR(3);

#if 0
static void
strtolower(char *str)
{
	for (; *str != 0; str++)
		*str = tolower(*str);
}
#endif

static int
conf_key_compar(const void *a, const void *b)
{
	const conf_key_t *ka = a, *kb = b;
	int c = strcmp(ka->key, kb->key);
	if (c < 0)
		return (-1);
	else if (c == 0)
		return (0);
	else
		return (1);
}

/*
 * Creates an empty configuration. Set values using conf_set_* and write
 * to a file using conf_write,
 */
conf_t *
conf_create_empty(void)
{
	conf_t *conf = safe_calloc(1, sizeof (*conf));
	avl_create(&conf->tree, conf_key_compar, sizeof (conf_key_t),
	    offsetof(conf_key_t, node));
	return (conf);
}

/*
 * Frees a conf_t object and all of its internal resources.
 */
void
conf_free(conf_t *conf)
{
	void *cookie = NULL;
	conf_key_t *ck;

	while ((ck = avl_destroy_nodes(&conf->tree, &cookie)) != NULL) {
		free(ck->key);
		free(ck->value);
		free(ck);
	}
	avl_destroy(&conf->tree);
	free(conf);
}

/*
 * Same as conf_read, but serves as a shorthand for reading directly from
 * a file path on disk. If the file cannot be opened for reading and errline
 * is not NULL, it will be set to -1.
 */
conf_t *
conf_read_file(const char *filename, int *errline)
{
	FILE *fp = fopen(filename, "rb");
	conf_t *conf;

	if (fp == NULL) {
		if (errline != NULL)
			*errline = -1;
		return (NULL);
	}
	conf = conf_read(fp, errline);
	fclose(fp);

	return (conf);
}

/*
 * Parses a configuration from a file. The file is structured as a
 * series of "key = value" lines. The parser understands "#" and "--"
 * comments.
 *
 * Returns the parsed conf_t object, or NULL in case an error was found.
 * If errline is not NULL, it is set to the line number where the error
 * was encountered.
 *
 * Use conf_free to free the returned structure.
 */
conf_t *
conf_read(FILE *fp, int *errline)
{
	conf_t *conf;
	char *line = NULL;
	size_t linecap = 0;
	int linenum = 0;

	conf = conf_create_empty();

	while (!feof(fp)) {
		char *sep;
		conf_key_t srch;
		conf_key_t *ck;
		avl_index_t where;

		linenum++;
		if (getline(&line, &linecap, fp) <= 0)
			continue;
		strip_space(line);
		if (*line == 0)
			continue;

		if ((sep = strstr(line, "--")) != NULL ||
		    (sep = strstr(line, "#")) != NULL) {
			*sep = 0;
			strip_space(line);
			if (*line == 0)
				continue;
		}

		sep = strstr(line, "=");
		if (sep == NULL) {
			conf_free(conf);
			if (errline != NULL)
				*errline = linenum;
			return (NULL);
		}
		*sep = 0;

		strip_space(line);
		strip_space(&sep[1]);

		srch.key = safe_malloc(strlen(line) + 1);
		strcpy(srch.key, line);
		strtolower(srch.key);	/* keys are case-insensitive */
		ck = avl_find(&conf->tree, &srch, &where);
		if (ck == NULL) {
			/* if the key didn't exist yet, create a new one */
			ck = safe_calloc(1, sizeof (*ck));
			ck->key = srch.key;
			avl_insert(&conf->tree, ck, where);
		} else {
			/* key already exists, free the search one */
			free(srch.key);
		}
		free(ck->value);
		ck->value = safe_malloc(strlen(&sep[1]) + 1);
		strcpy(ck->value, &sep[1]);
	}

	free(line);

	return (conf);
}

/*
 * Same as conf_write, but serves as a shorthand for writing directly to
 * a file path on disk.
 */
bool_t
conf_write_file(const conf_t *conf, const char *filename)
{
	FILE *fp = fopen(filename, "wb");
	bool_t res;

	if (fp == NULL)
		return (B_FALSE);
	res = conf_write(conf, fp);
	fclose (fp);

	return (res);
}

/*
 * Writes a conf_t object to a file. Returns B_TRUE if the write was
 * successful, B_FALSE otherwise.
 */
bool_t conf_write(const conf_t *conf, FILE *fp)
{
	fprintf(fp, "# libacfutils configuration file - DO NOT EDIT!\n");
	for (conf_key_t *ck = avl_first(&conf->tree); ck != NULL;
	    ck = AVL_NEXT(&conf->tree, ck)) {
		if (fprintf(fp, "%s = %s\n", ck->key, ck->value) <= 0)
			return (B_FALSE);
	}
	return (B_TRUE);
}

/*
 * Looks for a pre-existing configuration key-value pair based on key name.
 * Returns the conf_key_t object if found, NULL otherwise.
 */
static conf_key_t *
conf_find(const conf_t *conf, const char *key)
{
	const conf_key_t srch = { .key = (char *)key };
	return (avl_find(&conf->tree, &srch, NULL));
}

/*
 * Retrieves the string value of a configuration key. If found, the value
 * is placed in *value. Returns B_TRUE if the key was found, else B_FALSE.
 */
bool_t
conf_get_str(const conf_t *conf, const char *key, const char **value)
{
	const conf_key_t *ck = conf_find(conf, key);
	if (ck == NULL)
		return (B_FALSE);
	*value = ck->value;
	return (B_TRUE);
}

/*
 * Retrieves the 32-bit int value of a configuration key. If found, the value
 * is placed in *value. Returns B_TRUE if the key was found, else B_FALSE.
 */
bool_t
conf_get_i(const conf_t *conf, const char *key, int *value)
{
	const conf_key_t *ck = conf_find(conf, key);
	if (ck == NULL)
		return (B_FALSE);
	*value = atoi(ck->value);
	return (B_TRUE);
}

/*
 * Retrieves the 64-bit int value of a configuration key. If found, the value
 * is placed in *value. Returns B_TRUE if the key was found, else B_FALSE.
 */
bool_t
conf_get_lli(const conf_t *conf, const char *key, long long *value)
{
	const conf_key_t *ck = conf_find(conf, key);
	if (ck == NULL)
		return (B_FALSE);
	*value = atoll(ck->value);
	return (B_TRUE);
}

/*
 * Retrieves the 64-bit float value of a configuration key. If found, the value
 * is placed in *value. Returns B_TRUE if the key was found, else B_FALSE.
 */
bool_t
conf_get_d(const conf_t *conf, const char *key, double *value)
{
	const conf_key_t *ck = conf_find(conf, key);
	if (ck == NULL)
		return (B_FALSE);
	return (sscanf(ck->value, "%lf", value) == 1);
}

/* Same as conf_get_d, but for float values */
bool_t
conf_get_f(const conf_t *conf, const char *key, float *value)
{
	const conf_key_t *ck = conf_find(conf, key);
	if (ck == NULL)
		return (B_FALSE);
	return (sscanf(ck->value, "%f", value) == 1);
}

/*
 * Retrieves the 64-bit float value previously stored using conf_set_da. If
 * found, the value is placed in *value. Returns B_TRUE if the key was found,
 * else B_FALSE.
 * Due to a limitation in MinGW, we can't use '%a' here. Instead, we directly
 * write the binary representation of the double value. Since IEEE754 is used
 * on all our supported platforms, that makes it portable. As future-proofing
 * for big endian platforms, we always enforce storing the value in LE.
 */
bool_t
conf_get_da(const conf_t *conf, const char *key, double *value)
{
	const conf_key_t *ck = conf_find(conf, key);
	unsigned long long x;
	if (ck == NULL)
		return (B_FALSE);
#if	IBM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"	/* Workaround for MinGW crap */
#pragma GCC diagnostic ignored "-Wformat-extra-args"
#endif	/* IBM */
	if (sscanf(ck->value, "%llx", &x) != 1)
		return (B_FALSE);
#if	IBM
#pragma GCC diagnostic pop
#endif
#if	__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	x = BSWAP64(x);
#endif
	memcpy(value, &x, sizeof (double));
	return (B_TRUE);
}

/*
 * Retrieves the boolean value of a configuration key. If found, the value
 * is placed in *value. Returns B_TRUE if the key was found, else B_FALSE.
 */
bool_t
conf_get_b(const conf_t *conf, const char *key, bool_t *value)
{
	const conf_key_t *ck = conf_find(conf, key);
	if (ck == NULL)
		return (B_FALSE);
	*value = (strcmp(ck->value, "true") == 0 ||
	    strcmp(ck->value, "1") == 0 ||
	    strcmp(ck->value, "yes") == 0);
	return (B_TRUE);
}

/*
 * Sets up a key-value pair in the conf_t structure with a string value.
 * If value = NULL, this instead removes the key-value pair (if present).
 */
void
conf_set_str(conf_t *conf, const char *key, const char *value)
{
	conf_key_t *ck = conf_find(conf, key);

	if (ck == NULL) {
		if (value == NULL)
			return;
		ck = safe_calloc(1, sizeof (*ck));
		ck->key = strdup(key);
		avl_add(&conf->tree, ck);
	}
	free(ck->value);
	if (value == NULL) {
		avl_remove(&conf->tree, ck);
		free(ck->key);
		free(ck);
		return;
	}
	ck->value = strdup(value);
}

/*
 * Common setter back-end for conf_set_{i,d,b}.
 */
static void
conf_set_common(conf_t *conf, const char *key, const char *fmt, ...)
{
	int n;
	conf_key_t *ck = conf_find(conf, key);
	va_list ap1, ap2;

	va_start(ap1, fmt);
	va_copy(ap2, ap1);

	if (ck == NULL) {
		ck = safe_calloc(1, sizeof (*ck));
		ck->key = strdup(key);
		avl_add(&conf->tree, ck);
	}
	free(ck->value);
	n = vsnprintf(NULL, 0, fmt, ap1);
	ASSERT3S(n, >, 0);
	ck->value = safe_malloc(n + 1);
	(void) vsnprintf(ck->value, n + 1, fmt, ap2);
	va_end(ap1);
	va_end(ap2);
}

/*
 * Same as conf_set_str but with an int value. Obviously this cannot
 * remove a key, use conf_set_str(conf, key, NULL) for that.
 */
void
conf_set_i(conf_t *conf, const char *key, int value)
{
	conf_set_common(conf, key, "%i", value);
}

/*
 * Same as conf_set_str but with a long long value. Obviously this
 * cannot remove a key, use conf_set_str(conf, key, NULL) for that.
 */
void
conf_set_lli(conf_t *conf, const char *key, long long value)
{
#if	IBM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"	/* Workaround for MinGW crap */
#pragma GCC diagnostic ignored "-Wformat-extra-args"
#endif	/* IBM */
	conf_set_common(conf, key, "%lld", value);
#if	IBM
#pragma GCC diagnostic pop
#endif
}

/*
 * Same as conf_set_str but with a double value. Obviously this cannot
 * remove a key, use conf_set_str(conf, key, NULL) for that.
 */
void
conf_set_d(conf_t *conf, const char *key, double value)
{
	conf_set_common(conf, key, "%.15f", value);
}

void
conf_set_f(conf_t *conf, const char *key, float value)
{
	conf_set_common(conf, key, "%.12f", value);
}

/*
 * Same as conf_set_d, but able to accurately represent the exact value of
 * a double argument. Obviously this cannot remove a key, use
 * conf_set_str(conf, key, NULL) for that.
 */
void
conf_set_da(conf_t *conf, const char *key, double value)
{
	unsigned long long x;

	memcpy(&x, &value, sizeof (value));
#if	__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	x = BSWAP64(x);
#endif
#if	IBM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"	/* Workaround for MinGW crap */
#pragma GCC diagnostic ignored "-Wformat-extra-args"
#endif	/* IBM */
	conf_set_common(conf, key, "%llx", x);
#if	IBM
#pragma GCC diagnostic pop
#endif
}

/*
 * Same as conf_set_str but with a bool_t value. Obviously this cannot
 * remove a key, use conf_set_str(conf, key, NULL) for that.
 */
void
conf_set_b(conf_t *conf, const char *key, bool_t value)
{
	conf_set_common(conf, key, "%s", value ? "true" : "false");
}

#define	VARIABLE_GET(getfunc) \
	do { \
		va_list ap, ap2; \
		int l; \
		char *key; \
		bool_t res; \
		va_start(ap, value); \
		va_copy(ap2, ap); \
		l = vsnprintf(NULL, 0, fmt, ap); \
		key = safe_malloc(l + 1); \
		vsnprintf(key, l + 1, fmt, ap2); \
		res = getfunc(conf, key, value); \
		free(key); \
		va_end(ap); \
		va_end(ap2); \
		return (res); \
	} while (0)

#define	VARIABLE_SET(setfunc) \
	do { \
		va_list ap, ap2; \
		int l; \
		char *key; \
		va_start(ap, value); \
		va_copy(ap2, ap); \
		l = vsnprintf(NULL, 0, fmt, ap); \
		key = safe_malloc(l + 1); \
		vsnprintf(key, l + 1, fmt, ap2); \
		setfunc(conf, key, value); \
		free(key); \
		va_end(ap); \
		va_end(ap2); \
	} while (0)

/*
 * Same as conf_get_str, but allows passing in a printf-formatted argument
 * string in `fmt' and a variable number of arguments following `value'
 * to construct the configuration key name dynamically.
 */
bool_t
conf_get_str_v(const conf_t *conf, const char *fmt, const char **value, ...)
{
	VARIABLE_GET(conf_get_str);
}

/*
 * Same as conf_get_i, but with dynamic name-construction as conf_get_str_v.
 */
bool_t
conf_get_i_v(const conf_t *conf, const char *fmt, int *value, ...)
{
	VARIABLE_GET(conf_get_i);
}

/*
 * Same as conf_get_lli, but with dynamic name-construction as conf_get_str_v.
 */
bool_t
conf_get_lli_v(const conf_t *conf, const char *fmt, long long *value, ...)
{
	VARIABLE_GET(conf_get_lli);
}

/*
 * Same as conf_get_f, but with dynamic name-construction as conf_get_str_v.
 */
bool_t
conf_get_f_v(const conf_t *conf, const char *fmt, float *value, ...)
{
	VARIABLE_GET(conf_get_f);
}

/*
 * Same as conf_get_d, but with dynamic name-construction as conf_get_str_v.
 */
bool_t
conf_get_d_v(const conf_t *conf, const char *fmt, double *value, ...)
{
	VARIABLE_GET(conf_get_d);
}

/*
 * Same as conf_get_da, but with dynamic name-construction as conf_get_str_v.
 */
bool_t
conf_get_da_v(const conf_t *conf, const char *fmt, double *value, ...)
{
	VARIABLE_GET(conf_get_da);
}

/*
 * Same as conf_get_b, but with dynamic name-construction as conf_get_str_v.
 */
bool_t
conf_get_b_v(const conf_t *conf, const char *fmt, bool_t *value, ...)
{
	VARIABLE_GET(conf_get_b);
}

/*
 * Same as conf_set_str, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_str_v(conf_t *conf, const char *fmt, const char *value, ...)
{
	VARIABLE_SET(conf_set_str);
}

/*
 * Same as conf_set_i, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_i_v(conf_t *conf, const char *fmt, int value, ...)
{
	VARIABLE_SET(conf_set_i);
}

/*
 * Same as conf_set_lli, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_lli_v(conf_t *conf, const char *fmt, long long value, ...)
{
	VARIABLE_SET(conf_set_lli);
}

/*
 * Same as conf_set_f, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_f_v(conf_t *conf, const char *fmt, double value, ...)
{
	VARIABLE_SET(conf_set_f);
}

/*
 * Same as conf_set_d, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_d_v(conf_t *conf, const char *fmt, double value, ...)
{
	VARIABLE_SET(conf_set_d);
}

/*
 * Same as conf_set_da, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_da_v(conf_t *conf, const char *fmt, double value, ...)
{
	VARIABLE_SET(conf_set_da);
}

/*
 * Same as conf_set_b, but with dynamic name-construction as conf_get_str_v.
 */
void
conf_set_b_v(conf_t *conf, const char *fmt, bool_t value, ...)
{
	VARIABLE_SET(conf_set_b);
}

/*
 * Walks all configuration key-value pairs. You must set *cookie to NULL
 * on the first call. The function uses it to know how far it has progressed
 * in the walk. Once the walk is done, the function returns B_FALSE. Otherwise
 * it returns B_TRUE and the next config key & value in the respective args.
 * You may add or remove key-value pairs to the configuration during the walk.
 * Proper usage of this function:
 *
 *	void *cookie = NULL;
 *	const char *key, *value;
 *	while (conf_walk(conf, &key, &value, &cookie)) {
 *		... do something with key & value ...
 *	}
 */
bool_t
conf_walk(const conf_t *conf, const char **key, const char **value,
    void **cookie)
{
	static conf_key_t eol;
	conf_key_t *ck = *cookie;

	if (ck == &eol) {
		/* end of tree */
		return (B_FALSE);
	}
	if (ck == NULL) {
		/* first call */
		ck = avl_first(&conf->tree);
		if (ck == NULL) {
			/* tree is empty */
			*cookie = &eol;
			return (B_FALSE);
		}
	}
	*key = ck->key;
	*value = ck->value;
	*cookie = AVL_NEXT(&conf->tree, ck);
	if (*cookie == NULL) {
		/* end of tree */
		*cookie = &eol;
	}

	return (B_TRUE);
}