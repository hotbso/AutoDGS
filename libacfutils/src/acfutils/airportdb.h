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
 *
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_AIRPORTDB_H_
#define	_AIRPORTDB_H_

#include "avl.h"
#include "geom.h"
#include "list.h"
#include "helpers.h"
#include "htbl.h"
#include "thread.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct airportdb {
	bool_t		inited;
	bool_t		ifr_only;
	bool_t		normalize_gate_names;
	bool_t		override_settings;
	char		*xpdir;
	char		*cachedir;
	int		xp_airac_cycle;
	double		load_limit;

	mutex_t		lock;

	avl_tree_t	apt_dat;
	avl_tree_t	geo_table;
	avl_tree_t	arpt_index;
	htbl_t		icao_index;
	htbl_t		iata_index;
} airportdb_t;

typedef struct airport airport_t;
typedef struct runway runway_t;
typedef struct runway_end runway_end_t;
typedef struct ramp_start ramp_start_t;

typedef enum {
	RWY_SURF_ASPHALT = 1,
	RWY_SURF_CONCRETE = 2,
	RWY_SURF_GRASS = 3,
	RWY_SURF_DIRT = 4,
	RWY_SURF_GRAVEL = 5,
	RWY_SURF_DRY_LAKEBED = 12,
	RWY_SURF_WATER = 13,
	RWY_SURF_SNOWICE = 14,
	RWY_SURF_TRANSPARENT = 15
} rwy_surf_t;

struct runway_end {
	char		id[4];		/* runway ID, nul-terminated */
	geo_pos3_t	thr;		/* threshold position (elev in FEET!) */
	geo_pos3_t	thr_m;		/* same as thr, but elev in meters */
	double		displ;		/* threshold displacement in meters */
	double		blast;		/* stopway/blastpad length in meters */
	double		gpa;		/* glidepath angle in degrees */
	double		tch;		/* threshold clearing height in feet */
	double		tch_m;		/* threshold clearing height in m */

	/* computed on load_airport */
	vect2_t		thr_v;		/* threshold vector coord */
	vect2_t		dthr_v;		/* displaced threshold vector coord */
	double		hdg;		/* true heading */
	vect2_t		*apch_bbox;	/* in-air approach bbox */
	double		land_len;	/* length avail for landing in meters */
};

struct runway {
	airport_t	*arpt;
	double		width;
	runway_end_t	ends[2];
	char		joint_id[8];
	char		rev_joint_id[8];
	rwy_surf_t	surf;

	/* computed on load_airport */
	double		length;		/* meters */
	vect2_t		*prox_bbox;	/* on-ground approach bbox */
	vect2_t		*rwy_bbox;	/* above runway for landing */
	vect2_t		*tora_bbox;	/* on-runway on ground (for tkoff) */
	vect2_t		*asda_bbox;	/* on-runway on ground (for stopping) */

	avl_node_t	node;
};

typedef enum {
	RAMP_START_GATE,
	RAMP_START_HANGAR,
	RAMP_START_TIEDOWN,
	RAMP_START_MISC
} ramp_start_type_t;

struct ramp_start {
	char			name[32];
	geo_pos2_t		pos;
	float			hdgt;	/* true heading, deg */
	ramp_start_type_t	type;
	avl_node_t		node;
};

typedef enum {
	FREQ_TYPE_REC,	/* pre-recorded message ATIS, AWOS or ASOS */
	FREQ_TYPE_CTAF,
	FREQ_TYPE_CLNC,
	FREQ_TYPE_GND,
	FREQ_TYPE_TWR,
	FREQ_TYPE_APP,
	FREQ_TYPE_DEP
} freq_type_t;

typedef struct {
	freq_type_t	type;
	uint64_t	freq;	/* Hz */
	char		name[32];
	list_node_t	node;
} freq_info_t;

#define	AIRPORTDB_IDENT_LEN	8
#define	AIRPORTDB_ICAO_LEN	8
#define	AIRPORTDB_IATA_LEN	4
#define	AIRPORTDB_CC_LEN	4

struct airport {
	/* abstract identifier - only this is guaranteed to be unique */
	char		ident[AIRPORTDB_IDENT_LEN];
	/* 4-letter ICAO code, nul terminated (may not be unique or exist) */
	char		icao[AIRPORTDB_ICAO_LEN];
	/* 3-letter IATA code, nul terminated (may not be unique or exist) */
	char		iata[AIRPORTDB_IATA_LEN];
	/* 2-letter ICAO country/region code, nul terminated */
	char		cc[AIRPORTDB_CC_LEN];
	char		name[24];	/* Airport name, nul terminated */
	char		*name_orig;	/* Non-normalized version of name */
	char		*country;	/* Country name, nul terminated */
	char		*city;		/* City name, nul terminated */
	geo_pos3_t	refpt;		/* airport reference point location */
					/* (^^^ elev in FEET!) */
	geo_pos3_t	refpt_m;	/* same as refpt, but elev in meters */
	bool_t		geo_linked;	/* airport is in geo_table */
	double		TA;		/* transition altitude in feet */
	double		TL;		/* transition level in feet */
	double		TA_m;		/* transition altitude in meters */
	double		TL_m;		/* transition level in meters */
	avl_tree_t	rwys;		/* see runway_t above */
	avl_tree_t	ramp_starts;	/* see ramp_start_t above */
	list_t		freqs;

	bool_t		load_complete;	/* true if we've done load_airport */
	vect3_t		ecef;		/* refpt ECEF coordinates */
	fpp_t		fpp;		/* ortho fpp centered on refpt */
	bool_t		in_navdb;	/* used by recreate_apt_dat_cache */
	bool_t		have_iaps;	/* used by recreate_apt_dat_cache */

	avl_node_t	apt_dat_node;	/* apt_dat tree */
	list_node_t	cur_arpts_node;	/* cur_arpts list */
	avl_node_t	tile_node;	/* tiles in the airport_geo_tree */
};

/*
 * This structure is used in the fast-global-lookup index of airportdb_t.
 * This index is stored entirely in memory and thus doesn't incur any
 * disk time access penalty, but it's also not as fully-featured.
 *
 * This attempts to replicate the most useful fields of ARINC 424 "PA"
 * records in a compact-enough manner that we can hold the entire world-wide
 * database in memory at all times. For more information, lookup the
 * airport using airport_lookup_ident by using the ident field. The other
 * identifier fields may be empty, if the airport lacks this information.
 */
typedef struct {
	char		ident[AIRPORTDB_IDENT_LEN];	/* globally unique */
	char		icao[AIRPORTDB_ICAO_LEN];	/* may be empty */
	char		iata[AIRPORTDB_IATA_LEN];	/* may be empty */
	char		cc[AIRPORTDB_CC_LEN];		/* may be empty */
	geo_pos3_32_t	pos;				/* elev in ft */
	uint16_t	max_rwy_len;			/* ft */
	uint16_t	TA;				/* ft */
	uint16_t	TL;				/* ft */
	avl_node_t	node;
} arpt_index_t;

API_EXPORT void airportdb_create(airportdb_t *db, const char *xpdir,
    const char *cachedir);
API_EXPORT void airportdb_destroy(airportdb_t *db);

API_EXPORT void airportdb_lock(airportdb_t *db);
API_EXPORT void airportdb_unlock(airportdb_t *db);

/*
 * !!!! CAREFUL !!!!
 * This function needs to use setlocale, which means it's not thread-safe.
 * You should only call this on the main thread.
 */
#define	recreate_cache			ACFSYM(recreate_cache)
API_EXPORT bool_t recreate_cache(airportdb_t *db);
API_EXPORT bool_t adb_recreate_cache(airportdb_t *db, int app_version);

#define	find_nearest_airports		ACFSYM(find_nearest_airports)
API_EXPORT list_t *find_nearest_airports(airportdb_t *db, geo_pos2_t my_pos);

#define	free_nearest_airport_list	ACFSYM(free_nearest_airport_list)
API_EXPORT void free_nearest_airport_list(list_t *l);

#define	set_airport_load_limit		ACFSYM(set_airport_load_limit)
API_EXPORT void set_airport_load_limit(airportdb_t *db, double limit);

#define	load_nearest_airport_tiles	ACFSYM(load_nearest_airport_tiles)
API_EXPORT void load_nearest_airport_tiles(airportdb_t *db, geo_pos2_t my_pos);

/*
 * Query functions to look for airports
 */
#define	unload_distant_airport_tiles	ACFSYM(unload_distant_airport_tiles)
API_EXPORT void unload_distant_airport_tiles(airportdb_t *db,
    geo_pos2_t my_pos);
/*
 * Legacy lookup function by ICAO ID. Airports without valid ICAO IDs
 * won't show up here. The search is also restricted to a narrow zone
 * around `pos' (within a 3x3 degree square). Use airport_lookup_global
 * for a search at any arbitrary location.
 */
#define	airport_lookup	ACFSYM(airport_lookup)
API_EXPORT airport_t *airport_lookup(airportdb_t *db, const char *icao,
    geo_pos2_t pos);
/*
 * Legacy lookup function by ICAO ID. Airports without valid ICAO IDs
 * won't show up here.
 */
#define	airport_lookup_global	ACFSYM(airport_lookup_global)
API_EXPORT airport_t *airport_lookup_global(airportdb_t *db, const char *icao);
/*
 * Lookup by non-structured airport ID. This is *usually* the ICAO ID,
 * but many small fields only have a regional ID assigned here.
 */
#define	airport_lookup_by_ident	ACFSYM(airport_lookup_by_ident)
API_EXPORT airport_t *airport_lookup_by_ident(airportdb_t *db,
    const char *ident);
/*
 * Lookup by ICAO ID, with support for duplicates. Rather than returning
 * a pointer to an airport_t, this function returns the number of elements
 * found. If you provide a callback in `found_cb', it is called with the
 * airport_t and the userinfo pointer you provided. Use this for a complex
 * search that is capable of handling duplicates.
 */
#define	airport_lookup_by_icao	ACFSYM(airport_lookup_by_icao)
API_EXPORT size_t airport_lookup_by_icao(airportdb_t *db, const char *icao,
    void (*found_cb)(airport_t *airport, void *userinfo), void *userinfo);
/*
 * Lookup by IATA ID, with support for duplicates. Rather than returning
 * a pointer to an airport_t, this function returns the number of elements
 * found. If you provide a callback in `found_cb', it is called with the
 * airport_t and the userinfo pointer you provided. Use this for a complex
 * search that is capable of handling duplicates.
 */
#define	airport_lookup_by_iata	ACFSYM(airport_lookup_by_iata)
API_EXPORT size_t airport_lookup_by_iata(airportdb_t *db, const char *iata,
    void (*found_cb)(airport_t *airport, void *userinfo), void *userinfo);
/*
 * This performs a complete database index walk. The return value is the
 * total number of entries in the database index. If you provide a callback
 * function, it is called with every arpt_index_t that is in the database,
 * and your userinfo pointer. Although the arpt_index_t structure isn't as
 * fully-featured as a full airport_t, it is kept in memory at all times
 * and thus doesn't incur any disk access penalty. Use this for a fast
 * search to figure out if you're interested in a particular airport
 * (e.g. it's somewhere within a radius of interest for you), then
 * actually load its info using airport_lookup_by_ident.
 */
#define	airport_index_walk	ACFSYM(airport_index_walk)
API_EXPORT size_t airport_index_walk(airportdb_t *db,
    void (*found_cb)(const arpt_index_t *idx, void *userinfo), void *userinfo);
/*
 * Querying information about a particular airport.
 */
#define	airport_find_runway	ACFSYM(airport_find_runway)
API_EXPORT bool_t airport_find_runway(airport_t *arpt, const char *rwy_id,
    runway_t **rwy_p, unsigned *end_p);

#define	matching_airport_in_tile_with_TATL \
    ACFSYM(matching_airport_in_tile_with_TATL)
API_EXPORT airport_t *matching_airport_in_tile_with_TATL(airportdb_t *db,
    geo_pos2_t pos, const char *search_icao);

#define	airportdb_xp11_airac_cycle	ACFSYM(airportdb_xp11_airac_cycle)
API_EXPORT bool_t airportdb_xp11_airac_cycle(const char *xpdir, int *cycle);

#ifdef	__cplusplus
}
#endif

#endif	/* _AIRPORTDB_H_ */
