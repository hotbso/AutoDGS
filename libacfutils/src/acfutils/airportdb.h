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
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */
/**
 * \file
 * The airport database is the primary repository of knowledge about airports,
 * runways and bounding boxes.
 *
 * To start using the airport database, first initialize an \ref airportdb_t
 * using airportdb_create(). This doesn't actually populate the database.
 * You will then want to call adb_recreate_cache() on the initialized database,
 * to actually make sure it is up to date. Subsequently, you can start using
 * the various airport lookup and query functions to interrogate the database.
 * When done, use airportdb_destroy() to free the database and its resources.
 * @see airportdb_create()
 * @see adb_recreate_cache()
 * @see airportdb_destroy()
 * @see adb_airport_lookup_global()
 * @see adb_airport_index_walk()
 *
 * Implementation Details
 * ----------------------
 *
 * The airport database is composed of two data structures:
 *
 * - a global ident -> airport_t AVL tree (apt_dat). This allows us to
 *	quickly locate an airport based on its identifier.
 * - a geo-referenced AVL tree from approximate airport reference point
 *	position (in 1-degree accuracy) to the airport_t (geo_table). This
 *	allows us to quickly sift through the airport database to locate any
 *	airports close to a given point of interest.
 *
 * Of these, apt_dat is the primary repository of knowledge - once an airport
 * is gone from apt_dat, it is freed. An airport may or may not be
 * geo-referenced in the geo_table. Once all loading of an airport is
 * complete, it WILL be geo-referenced.
 *
 * The geo_table is actually comprised of tile_t data structures. A tile_t
 * refers to a 1x1 degree geographical tile at specific coordinates and
 * contains its own private airport_t tree, which is again organized by
 * abstract identifier, allowing us to step through all the airports in a
 * tile or quickly locate one based on identifier.
 *
 * During normal operation, not all airports from all over the world are
 * loaded into memory, as that would use quite a bit of memory and delay
 * startup. Instead, only the closets 9 tiles around the aircraft are
 * present. New tiles are loaded as the aircraft repositions and the old
 * ones are released. Loading a tile first populates the global apt_dat
 * with all its airports, which are then geo-referenced in the newly
 * created tile. Releasing a tile is the converse, ultimately ending in
 * the airports being purged from apt_dat and freed.
 *
 * The 9-tile rule can result in strange behavior close to the poles, where
 * the code might think of close by airports as being very far away and
 * thus not load them. Luckily, there are only about 4 airports beyond 80
 * degrees latitude (north or south), all of which are very special
 * non-regular airports, so we just ignore those.
 *
 * Airport Data Construction Method
 * --------------------------------
 *
 * For each airport, we need to obtain the following pieces of information:
 *
 * 1. The abstract airport identifier.
 *	- Optional ICAO identifier, on a 1302 icao_code line.
 *	- Optional IATA identifier, on a 1302 iata_code line.
 * 2. The airport reference point latitude, longitude and elevation.
 * 3. The airport's transition altitude and transition level (if published).
 * 4. For each runway:
 *	- Runway width.
 *	- Each threshold's geographical position and elevation.
 *	- If the threshold is displaced, the amount of displacement.
 *	- For each end, if available, the optimal glidepath angle and
 *	   threshold clearing height.
 *
 * First we examine all installed scenery. That means going through each
 * apt.dat declared in scenery_packs.ini and the global default apt dat
 * to find these kinds of records:
 *
 * - '1' records identify airports. See parse_apt_dat_1_line.
 * - '21' records identify runway-related lighting fixtures (PAPIs/VASIs).
 *	See parse_apt_dat_21_line.
 * - '50' through '56' and '1050' through '1056' records identify frequency
 *	information. See parse_apt_dat_freq_line.
 * - '100' records identify runways. See parse_apt_dat_100_line.
 * - '1302' records identify airport meta-information, such as ICAO code,
 *	TA, TL, reference point location, etc.
 *
 * Prior to X-Plane 11, apt.dat's didn't necessarily contain the '1302'
 * records, so we had to pull those from the Airports.txt in the navdata
 * directory for the built-in GNS430. Starting from X-Plane 11, Airports.txt
 * is gone and this data has been relocated to the apt.dat.
 *
 * A further complication of the absence of an Airports.txt is that this
 * file contained both the GPA and TCH for each runway and although it did
 * sometimes require some fuzzy matching to account for outdated scenery
 * not exactly matching the navdata, we could obtain this information from
 * one place.
 *
 * So for X-Plane 11, we've implemented a new method of obtaining this
 * information. By default, if a runway has an instrument approach (unless
 * ifr_only=B_FALSE), it will have an entry in CIFP. Runway entries in
 * APPCH-type procedures specify the TCH and GPA in columns 24 and 29 (ARINC
 * 424 fields 4.1.9.1.85-89 and 4.1.9.1.103-106). We only use the first
 * such occurence. If there are multiple approaches to the runway, they
 * should all end up with the same TCH and GPA. This should cover pretty much
 * every case. In the rare case where we *don't* get the TCH and GPA this way,
 * we try a fallback mechanism. Almost every instrument approach runway has
 * some kind of visual glideslope indication (VGSI) right next to it. We can
 * extract the location of those from the apt.dat file. These VGSIs are
 * located in the exact touchdown point and have a fixed GPA. So we simply
 * look for a VGSI close to the runway's centerline and that is aligned with
 * the runway, compute the longitudinal displacement of this indicator from
 * the runway threshold and using the indicator's GPA compute the optimal TCH.
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

typedef struct {
	bool_t		inited;
	bool_t		ifr_only;
    bool_t      global_airports_only;
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

typedef enum {
	RAMP_START_GATE,
	RAMP_START_HANGAR,
	RAMP_START_TIEDOWN,
	RAMP_START_MISC
} ramp_start_type_t;

/**
 * Information about a `ramp start' - an initial airplane spawning location
 * provided by the scenery author.
 */
typedef struct ramp_start {
	char			name[32];	/**< Descriptive name */
	geo_pos2_t		pos;		/**< Position */
	float			hdgt;		/**< True heading, deg */
	ramp_start_type_t	type;		/**< Type of ramp start */
	avl_node_t		node;
} ramp_start_t;

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

/**
 * Describes one end of a \ref runway.
 */
struct runway_end {
	char		id[4];	/**< Runway ID with leading 0, NUL-term'd. */
	geo_pos3_t	thr;	/**< Threshold position (elev in FEET!). */
	geo_pos3_t	thr_m;	/**< Same as thr, but elev in meters. */
	double		displ;	/**< Threshold displacement in meters. */
	double		blast;	/**< Stopway/blastpad length in meters. */
	double		gpa;	/**< Glidepath angle in degrees. */
	double		tch;	/**< Threshold clearing height in feet. */
	double		tch_m;	/**< Threshold clearing height in meters. */

	/* computed on load_airport */
	vect2_t		thr_v;	/**< Threshold vector coord. */
	vect2_t		dthr_v;	/**< Displaced threshold vector coord. */
	double		hdg;	/**< True heading in degrees. */
	vect2_t		*apch_bbox;	/**< In-air approach bbox. */
	double		land_len; /**< Length avail for landing in meters. */
};

/**
 * Describes one runway, consisting of two \ref runway_end structures.
 */
struct runway {
	airport_t	*arpt;		/**< Parent airport. */
	double		width;		/**< Runway width in meters. */
	runway_end_t	ends[2];	/**< Runway ends, lower ID end first. */
	char		joint_id[8];	/**< Concat of the two ends' IDs. */
	char		rev_joint_id[8];/**< Same as joint_id, but reversed. */
	rwy_surf_t	surf;		/**< Type of runway surface. */

	/* computed on load_airport */
	double		length;		/**< Runway length in meters. */
	vect2_t		*prox_bbox;	/**< On-ground approach bbox. */
	vect2_t		*rwy_bbox;	/**< Above runway for landing. */
	vect2_t		*tora_bbox;	/**< On-runway on ground (for t'off). */
	vect2_t		*asda_bbox;	/**< On-runway on ground (for stop). */

	avl_node_t	node;
};

typedef enum {
	FREQ_TYPE_REC,	/**< Pre-recorded message ATIS, AWOS or ASOS */
	FREQ_TYPE_CTAF,	/**< Common Traffic Advisory Frequency */
	FREQ_TYPE_CLNC,	/**< Clearance Delivery */
	FREQ_TYPE_GND,	/**< Ground */
	FREQ_TYPE_TWR,	/**< Tower */
	FREQ_TYPE_APP,	/**< Approach */
	FREQ_TYPE_DEP	/**< Departure */
} freq_type_t;

/**
 * Airport frequency information.
 */
typedef struct {
	freq_type_t	type;		/**< Type of service */
	uint64_t	freq;		/**< Frequency in Hz. */
	char		name[32];	/**< Descriptive name */
	list_node_t	node;
} freq_info_t;

#define	AIRPORTDB_IDENT_LEN	8
#define	AIRPORTDB_ICAO_LEN	8
#define	AIRPORTDB_IATA_LEN	4
#define	AIRPORTDB_CC_LEN	4

/**
 * The master airport data structure.
 */
struct airport {
	/** Abstract identifier - only this is guaranteed to be unique. */
	char		ident[AIRPORTDB_IDENT_LEN];
	/** 4-letter ICAO code, nul terminated (may not be unique or exist). */
	char		icao[AIRPORTDB_ICAO_LEN];
	/** 3-letter IATA code, nul terminated (may not be unique or exist). */
	char		iata[AIRPORTDB_IATA_LEN];
	/** 2-letter ICAO country/region code, nul terminated. */
	char		cc[AIRPORTDB_CC_LEN];
	/** 3-letter ISO-3166 country code, uppercase, nul terminated. */
	char		cc3[AIRPORTDB_CC_LEN];
	char		name[24];	/**< Airport name, nul terminated. */
	char		*name_orig;	/**< Non-normalized version of name. */
	char		*country;	/**< Country name, nul terminated. */
	char		*city;		/**< City name, nul terminated. */
	geo_pos3_t	refpt;		/**< Airport reference point location */
					/**< (N.B. elevation is in FEET!). */
	geo_pos3_t	refpt_m;	/**< Same as refpt, but elev in meters. */
	bool_t		geo_linked;	/**< Airport is in geo_table. */
	double		TA;		/**< Transition altitude in feet. */
	double		TL;		/**< Transition level in feet. */
	double		TA_m;		/**< Transition altitude in meters. */
	double		TL_m;		/**< Transition level in meters. */
	avl_tree_t	rwys;		/**< Tree of \ref runway structures. */
	avl_tree_t	ramp_starts;	/**< Tree of \ref ramp_start structs */
	list_t		freqs;		/**< List of \ref freq_info_t's. */

	bool_t		load_complete;	/**< True if we've done load_airport. */
	vect3_t		ecef;		/**< refpt in ECEF coordinates with elev in ft. */
	fpp_t		fpp;		/**< Orthographic fpp_t centered on refpt. */
	bool_t		in_navdb;	/**< Used by recreate_apt_dat_cache. */
	bool_t		have_iaps;	/**< Used by recreate_apt_dat_cache. */
    bool_t      is_global_arpt; /**< used by recreate_apt_dat_cache */

	avl_node_t	apt_dat_node;	/**< Used by apt_dat tree. */
	list_node_t	cur_arpts_node;	/**< Used by cur_arpts list. */
	avl_node_t	tile_node;	/**< Tiles in the airport_geo_tree. */
};

/**
 * This structure is used in the fast-global-lookup index of \ref airportdb_t.
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
	/** Globally unique name. */
	char		ident[AIRPORTDB_IDENT_LEN];
	/** ICAO code. May be empty. */
	char		icao[AIRPORTDB_ICAO_LEN];
	/** IATA code. May be empty */
	char		iata[AIRPORTDB_IATA_LEN];
	/** 2-letter country code. May be empty. */
	char		cc[AIRPORTDB_CC_LEN];
	geo_pos3_32_t	pos;	/**< Reference point, elevation in feet. */
	uint16_t	max_rwy_len; /**< Length of longest runway in feet. */
	uint16_t	TA;	/**< Transition alt in feet. Zero if unknown. */
	uint16_t	TL;	/**< Transition level in feet. Zero if undef. */
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
#define recreate_cache(__db)	adb_recreate_cache((__db), 0)
API_EXPORT bool_t adb_recreate_cache(airportdb_t *db, int app_version);

#define	find_nearest_airports		adb_find_nearest_airports
API_EXPORT list_t *adb_find_nearest_airports(airportdb_t *db,
    geo_pos2_t my_pos);

#define	free_nearest_airport_list	adb_free_nearest_airport_list
API_EXPORT void adb_free_nearest_airport_list(list_t *l);

#define	set_airport_load_limit		adb_set_airport_load_limit
API_EXPORT void adb_set_airport_load_limit(airportdb_t *db, double limit);

#define	load_nearest_airport_tiles	adb_load_nearest_airport_tiles
API_EXPORT void adb_load_nearest_airport_tiles(airportdb_t *db,
    geo_pos2_t my_pos);

/*
 * Query functions to look for airports
 */
#define	unload_distant_airport_tiles	adb_unload_distant_airport_tiles
API_EXPORT void adb_unload_distant_airport_tiles(airportdb_t *db,
    geo_pos2_t my_pos);

#define	airport_lookup	adb_airport_lookup
API_EXPORT airport_t *adb_airport_lookup(airportdb_t *db, const char *icao,
    geo_pos2_t pos);

#define	airport_lookup_global	adb_airport_lookup_global
API_EXPORT airport_t *adb_airport_lookup_global(airportdb_t *db,
    const char *icao);

#define	airport_lookup_by_ident	adb_airport_lookup_by_ident
API_EXPORT airport_t *adb_airport_lookup_by_ident(airportdb_t *db,
    const char *ident);

#define	airport_lookup_by_icao	adb_airport_lookup_by_icao
API_EXPORT size_t adb_airport_lookup_by_icao(airportdb_t *db, const char *icao,
    void (*found_cb)(airport_t *airport, void *userinfo), void *userinfo);

#define	airport_lookup_by_iata	adb_airport_lookup_by_iata
API_EXPORT size_t adb_airport_lookup_by_iata(airportdb_t *db, const char *iata,
    void (*found_cb)(airport_t *airport, void *userinfo), void *userinfo);

#define	airport_index_walk	adb_airport_index_walk
API_EXPORT size_t adb_airport_index_walk(airportdb_t *db,
    void (*found_cb)(const arpt_index_t *idx, void *userinfo), void *userinfo);
/*
 * Querying information about a particular airport.
 */
#define	airport_find_runway	adb_airport_find_runway
API_EXPORT bool_t adb_airport_find_runway(airport_t *arpt, const char *rwy_id,
    runway_t **rwy_p, unsigned *end_p);

#define	matching_airport_in_tile_with_TATL \
	adb_matching_airport_in_tile_with_TATL
API_EXPORT airport_t *adb_matching_airport_in_tile_with_TATL(airportdb_t *db,
    geo_pos2_t pos, const char *search_icao);

#define	airportdb_xp11_airac_cycle	adb_airportdb_xp_airac_cycle
API_EXPORT bool_t adb_airportdb_xp_airac_cycle(const char *xpdir, int *cycle);

#ifdef	__cplusplus
}
#endif

#endif	/* _AIRPORTDB_H_ */
