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
 * Copyright 2015 Saso Kiselkov. All rights reserved.
 */

#ifndef	_ACF_UTILS_PERF_H_
#define	_ACF_UTILS_PERF_H_

#include "geom.h"

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Temperature unit conversions.
 */
#define	KELVIN2C(k)	((k) - 273.15)
#define	C2KELVIN(c)	((c) + 273.15)
#define	FAH2C(f)	(((f) - 32) * 0.555555)
#define	C2FAH(c)	(((c) * 1.8) + 32)
#define	FAH2KELVIN(f)	(((f) + 459.67) * 0.5555555555)
#define	KELVIN2FAH(k)	(((k) * 1.8) - 459.67)

/*
 * Length and velocity unit conversions.
 */
#define	FEET2MET(x)	((x) * 0.3048)		/* feet to meters */
#define	MET2FEET(x)	((x) * 3.2808398950131)	/* meters to feet */
#define	NM2MET(x)	((x) * 1852)		/* nautical miles to meters */
#define	MET2NM(x)	((x) / 1852.0)		/* meters to nautical miles */
#define	MET2MILES(x)	(MET2FEET(x) / 5280.0)	/* meters to statute miles */
#define	MILES2MET(x)	FEET2MET((x) * 5280.0)	/* statute miles to meters */
#define	KT2MPS(k)	(NM2MET(k) / 3600.0)	/* knots to m/s */
#define	MPS2KT(k)	(MET2NM(k) * 3600.0)	/* m/s to knots */
#define	MPS2KPH(k)	((k) * 3.6)		/* m/s to km/h */
#define	KPH2MPS(k)	((k) / 3.6)		/* km/h to m/s */
#define	MPS2MPH(k)	((k) / 0.44704)		/* m/s to mph */
#define	MPH2MPS(k)	((k) * 0.44704)		/* mph to m/s */
#define	FPM2MPS(f)	FEET2MET((f) / 60.0)	/* ft.min^-1 to m.s^-1 */
#define	MPS2FPM(m)	MET2FEET((m) * 60.0)	/* m.s^-1 to ft.min^-1 */

#define	INHG2PA(p)	((p) * (101325 / 29.92))
#define	PA2INHG(p)	((p) * (29.92 / 101325))

#define	RADSEC2RPM(r)	(((r) / (2.0 * M_PI)) * 60.0)
#define	RPM2RADSEC(r)	(((r) / 60.0) * (2.0 * M_PI))

#define	USG2LIT(usg)	((usg) * 3.785411784)
#define	LIT2USG(lit)	((lit) / 3.785411784)
#define	LBS2KG(lbs)	((lbs) * 0.45359237)
#define	KG2LBS(kg)	((kg) / 0.45359237)

#define	LBF2NEWTON(lb)	(LBS2KG(lb) * EARTH_GRAVITY)
#define	NEWTON2LBF(f)	(KG2LBS((f) / EARTH_GRAVITY))

#define	WATT2HP(W)	((W) * 0.001341022)	/* Watts to horsepower */
#define	HP2WATT(W)	((W) / 0.001341022)	/* horsepower to Watts */

/*
 * Pressure unit conversions
 */
#define	HPA2PA(x)	((x) * 100)
#define	PA2HPA(x)	((x) / 100)
#define	PSI2PA(x)	((x) * 6894.73326075122482308111)
#define	PA2PSI(x)	((x) / 6894.73326075122482308111)

/*
 * ISA (International Standard Atmosphere) parameters.
 */
#define	ISA_SL_TEMP_C		15.0	/* Sea level temperature in degrees C */
#define	ISA_SL_TEMP_K		288.15	/* Sea level temperature in Kelvin */
#define	ISA_SL_PRESS		101325.0/* Sea level pressure in Pa */
#define	ISA_SL_DENS		1.225	/* Sea level density in kg/m^3 */
#define	ISA_TLR_PER_1000FT	1.98	/* Temperature lapse rate per 1000ft */
#define	ISA_TLR_PER_1M		0.0065	/* Temperature lapse rate per 1 meter */
#define	ISA_SPEED_SOUND		340.3	/* Speed of sound at sea level */
#define	ISA_TP_ALT		36089	/* Tropopause altitude in feet */

/*
 * Physical constants.
 */
#define	EARTH_GRAVITY	9.80665		/* Earth surface grav. acceleration */
#define	EARTH_SID_DAY	86164.0905	/* Sidereal day on Earth in seconds */
#define	EARTH_ROT_RATE	(360.0 / EARTH_SID_DAY)	/* deg/sec */
#define	DRY_AIR_MOL	0.02896968	/* Molar mass of dry air */
#define	GAMMA		1.4		/* Specific heat ratio of dry air */
#define	R_univ		8.314462618	/* Universal gas constant */
#define	R_spec		287.058		/* Specific gas constant of dry air */
#define	BOLTZMANN_CONST	5.67E-8		/* Stefan-Boltzmann constant */

/* Calculates gravitational force for mass `m' in kg on Earth */
#define	MASS2GFORCE(m)	((m) * EARTH_GRAVITY)

/*
 * Fuel conversion macros.
 */
#define	JETA_KG2GAL(kg)		((kg) / 3.08447722)
#define	JETA_GAL2KG(gal)	((gal) * 3.08447722)

typedef struct {
	int	spd;
	double	Cd;
} drag_coeff_t;
typedef struct {
	float	kias;	/* knots */
	float	alt_ft;	/* feet */
} flt_spd_lim_t;
/*
 * Serializable in its entirety.
 */
typedef struct {
	float		zfw;		/* kg */
	float		fuel;		/* kg */
	float		clb_ias;	/* knots */
	float		clb_ias_init;	/* knots */
	float		clb_mach;	/* fraction */
	float		crz_ias;	/* knots */
	float		crz_mach;	/* fraction */
	float		crz_lvl;	/* feet */
	float		des_ias;	/* knots */
	float		des_mach;	/* fraction */
	float		to_flap;	/* ratio */
	float		accel_hgt;	/* feet AGL */
#define	FLT_PERF_NUM_SPD_LIMS	2
	flt_spd_lim_t	clb_spd_lim[FLT_PERF_NUM_SPD_LIMS];
	flt_spd_lim_t	des_spd_lim[FLT_PERF_NUM_SPD_LIMS];

	float		thr_derate;
	float		bank_ratio;
	unsigned	num_eng;
} flt_perf_t;

typedef struct perf_table_set_s perf_table_set_t;

typedef struct {
	char		*acft_type;

	flt_perf_t	ref;	/* Reference performance values */

	char		*eng_type;
	unsigned	num_eng;

	/* Base max thrust in Newtons @ ISA conditions */
	double		eng_max_thr;
	/* Base min thrust in Newtons @ ISA conditions */
	double		eng_min_thr;
	/* Specific fuel consumption in kg/(N.s) @ ISA conditions */
	double		eng_sfc;
	/*
	 * eng_max_thr fraction as a function of air density (in kg/m^3).
	 */
	vect2_t	*thr_dens_curve;
	/*
	 * eng_max_thr fraction as a function of Mach number.
	 */
	vect2_t	*thr_mach_curve;
	/*
	 * Engine specific fuel consumption in kg/hr as a function of
	 * thrust in Kilonewtons.
	 */
	vect2_t	*sfc_thro_curve;
	/*
	 * Engine specific fuel consumption modifier (0 - 1) as a function
	 * of ISA temperature deviation in degrees C.
	 */
	vect2_t	*sfc_isa_curve;

	vect2_t	*cl_curve;
	vect2_t	*cl_flap_curve;
	double	cl_max_aoa;
	vect2_t	*cd_curve;
	vect2_t	*cd_flap_curve;
	double	cl_flap_max_aoa;
	double	wing_area;

	vect2_t	*half_bank_curve;
	vect2_t	*full_bank_curve;

	perf_table_set_t	*clb_tables;
	perf_table_set_t	*crz_tables;
	perf_table_set_t	*des_tables;
} acft_perf_t;

/* Type of acceleration-climb */
typedef enum {
	ACCEL_THEN_CLB,	/* First accelerate, then climb */
	ACCEL_AND_CLB,	/* Accel & climb simultaneously (50/50 energy split) */
	ACCEL_TAKEOFF	/* Accel to target speed first without needing lift */
} accelclb_t;

API_EXPORT void lacf_set_perf_step_debug(bool_t flag);
API_EXPORT bool_t lacf_get_perf_step_debug(void);

#define	acft_perf_parse		ACFSYM(acft_perf_parse)
API_EXPORT acft_perf_t *acft_perf_parse(const char *filename);
#define	acft_perf_destroy	ACFSYM(acft_perf_destroy)
API_EXPORT void acft_perf_destroy(acft_perf_t *perf);

#define	flt_perf_new		ACFSYM(flt_perf_new)
API_EXPORT flt_perf_t *flt_perf_new(const acft_perf_t *acft);
#define	flt_perf_destroy	ACFSYM(flt_perf_destroy)
API_EXPORT void flt_perf_destroy(flt_perf_t *flt);

#define	eng_max_thr_avg	ACFSYM(eng_max_thr_avg)
API_EXPORT double eng_max_thr_avg(const flt_perf_t *flt,
    const acft_perf_t *acft, double alt1, double alt2, double ktas,
    double qnh, double isadev, double tp_alt);

#define	accelclb2dist	ACFSYM(accelclb2dist)
API_EXPORT double accelclb2dist(const flt_perf_t *flt, const acft_perf_t *acft,
    double isadev, double qnh, double tp_alt, double accel_alt,
    double fuel, vect2_t dir,
    double alt1, double kcas1, vect2_t wind1,
    double alt2, double kcas2, vect2_t wind2,
    double flap_ratio, double mach_lim, accelclb_t type, double *burnp,
    double *kcas_out);
#define	dist2accelclb	ACFSYM(dist2accelclb)
API_EXPORT double dist2accelclb(const flt_perf_t *flt, const acft_perf_t *acft,
    double isadev, double qnh, double tp_alt, double accel_alt,
    double fuel, vect2_t dir,
    double flap_ratio, double *alt, double *kcas, vect2_t wind,
    double alt_tgt, double kcas_tgt, double mach_lim, double dist_tgt,
    accelclb_t type, double *burnp, double *ttg_out);
#define	decel2dist	ACFSYM(decel2dist)
API_EXPORT double decel2dist(const flt_perf_t *flt, const acft_perf_t *acft,
    double isadev, double qnh, double tp_alt, double fuel,
    double alt, double kcas1, double kcas2, double dist_tgt,
    double *kcas_out, double *burn_out);
#define	perf_crz2burn	ACFSYM(perf_crz2burn)
API_EXPORT double perf_crz2burn(double isadev, double tp_alt, double qnh,
    double alt_ft, double spd, bool_t is_mach, double hdg, vect2_t wind1,
    vect2_t wind2, double fuel, double dist_nm, const acft_perf_t *acft,
    const flt_perf_t *flt, double *ttg_out);
#define	perf_des2burn	ACFSYM(perf_des2burn)
API_EXPORT double perf_des2burn(const flt_perf_t *flt, const acft_perf_t *acft,
    double isadev, double qnh, double fuel, double hdgt, double dist_nm,
    double mach_lim,
    double alt1_ft, double kcas1, vect2_t wind1,
    double alt2_ft, double kcas2, vect2_t wind2,
    double *ttg_out);

#define	perf_TO_spd	ACFSYM(perf_TO_spd)
API_EXPORT double perf_TO_spd(const flt_perf_t *flt, const acft_perf_t *acft);

#define	acft_get_sfc	ACFSYM(acft_get_sfc)
API_EXPORT double acft_get_sfc(const flt_perf_t *flt, const acft_perf_t *acft,
    double thr, double alt, double ktas, double qnh, double isadev,
    double tp_alt);

#define	perf_get_turn_rate	ACFSYM(perf_get_turn_rate)
API_EXPORT double perf_get_turn_rate(double bank_ratio, double gs_kts,
    const flt_perf_t *flt, const acft_perf_t *acft);
/*
 * Aviation versions, assuming ISA conditions. Altitude in feet!
 */
#define	alt2press	ACFSYM(alt2press)
API_EXPORT double alt2press(double alt_ft, double qnh_Pa);
#define	press2alt	ACFSYM(press2alt)
API_EXPORT double press2alt(double press_Pa, double qnh_Pa);
/*
 * Generic barometric versions - altitudes in meters!
 */
#define	alt2press_baro	ACFSYM(alt2press_baro)
API_EXPORT double alt2press_baro(double alt_m, double p0_Pa, double T0_K,
    double g_mss);
#define	press2alt_baro	ACFSYM(press2alt_baro)
API_EXPORT double press2alt_baro(double p_Pa, double p0_Pa, double T0_K,
    double g_mss);

#define	alt2fl		ACFSYM(alt2fl)
API_EXPORT double alt2fl(double alt, double qnh);
#define	fl2alt		ACFSYM(fl2alt)
API_EXPORT double fl2alt(double alt, double qnh);

#define	ktas2mach		ACFSYM(ktas2mach)
API_EXPORT double ktas2mach(double ktas, double oat);
#define	mach2ktas		ACFSYM(mach2ktas)
API_EXPORT double mach2ktas(double mach, double oat);

#define	ktas2kcas		ACFSYM(ktas2kcas)
API_EXPORT double ktas2kcas(double ktas, double pressure, double oat);
#define	kcas2ktas		ACFSYM(kcas2ktas)
API_EXPORT double kcas2ktas(double kcas, double pressure, double oat);
#define	impact_press2kcas	ACFSYM(impact_press2kcas)
API_EXPORT double impact_press2kcas(double impact_pressure);

#define	kcas2mach		ACFSYM(kcas2mach)
API_EXPORT double kcas2mach(double kcas, double alt_ft, double qnh, double oat);
#define	mach2kcas		ACFSYM(mach2kcas)
API_EXPORT double mach2kcas(double mach, double alt_ft, double qnh, double oat);

#define	mach2keas	ACFSYM(mach2keas)
API_EXPORT double mach2keas(double mach, double press);
#define	keas2mach	ACFSYM(keas2mach)
API_EXPORT double keas2mach(double keas, double press);

#define	sat2tat		ACFSYM(sat2tat)
API_EXPORT double sat2tat(double sat, double mach);
#define	tat2sat		ACFSYM(tat2sat)
API_EXPORT double tat2sat(double tat, double mach);

#define	sat2isadev	ACFSYM(sat2isadev)
API_EXPORT double sat2isadev(double fl, double sat);
#define	isadev2sat	ACFSYM(isadev2sat)
API_EXPORT double isadev2sat(double fl, double isadev);

#define	speed_sound	ACFSYM(speed_sound)
#define	speed_sound_gas	ACFSYM(speed_sound_gas)
API_EXPORT double speed_sound(double oat);
API_EXPORT double speed_sound_gas(double T, double gamma, double R);

#define	air_density	ACFSYM(air_density)
API_EXPORT double air_density(double pressure, double oat);

#define	gas_density	ACFSYM(gas_density)
API_EXPORT double gas_density(double pressure, double oat, double gas_const);

#define	impact_press	ACFSYM(impact_press)
API_EXPORT double impact_press(double mach, double pressure);

#define	dyn_press	ACFSYM(dyn_press)
API_EXPORT double dyn_press(double ktas, double press, double oat);

#define	dyn_gas_press	ACFSYM(dyn_gas_press)
API_EXPORT double dyn_gas_press(double ktas, double press, double oat,
    double gas_const);

#define	static_press	ACFSYM(static_press)
API_EXPORT double static_press(double rho, double oat);

#define	static_gas_press	ACFSYM(static_gas_press)
API_EXPORT double static_gas_press(double rho, double oat, double gas_const);

#define	adiabatic_heating_gas	ACFSYM(adiabatic_heating_gas)
API_EXPORT double adiabatic_heating_gas(double press_ratio, double start_temp,
    double gamma);

#define	adiabatic_heating	ACFSYM(adiabatic_heating)
API_EXPORT double adiabatic_heating(double press_ratio, double start_temp);

/*
 * Returns the kinematic viscosity of dry air in m^2/s.
 * @param temp_K Air temperature in Kelvin.
 */
#define	air_kin_visc	ACFSYM(air_kin_visc)
API_EXPORT double air_kin_visc(double temp_K);

/*
 * Returns the Reynolds number of an airfoil.
 * @param vel Air velocity in m/s.
 * @param chord Chord length in meters.
 * @param temp_K Air temperature in Kelvin.
 */
#define	air_reynolds	ACFSYM(air_reynolds)
API_EXPORT double air_reynolds(double vel, double chord, double temp_K);

/*
 * Returns the ratio of specific heats for dry air.
 * @param T Absolute temperature of the air in Kelvin.
 */
API_EXPORT double lacf_gamma_air(double T);

/*
 * Returns the thermal conductivity of dry air (in W/(m.K)).
 * @param T Absolute temperature of the air in Kelvin.
 */
API_EXPORT double lacf_therm_cond_air(double T);

/*
 * Returns the thermal conductivity of aluminum (in W/(m.K)).
 * @param T Absolute temperature of the air in Kelvin.
 */
API_EXPORT double lacf_therm_cond_aluminum(double T);

/*
 * Returns the thermal conductivity of Pyrex 7740 glass (in W/(m.K)).
 * @param T Absolute temperature of the air in Kelvin.
 */
API_EXPORT double lacf_therm_cond_glass(double T);

/*
 * Returns very accurate Earth gravitational acceleration at a specific
 * point around the Earth. For simplicity, this assumes that the gravitational
 * field is uniform along longitude.
 * @param lat Latitude in degrees.
 * @param alt Altitude in meters.
 * @return Gravitational acceleration at the chosen point, in m/s^2.
 */
#define	earth_gravity_accurate	ACFSYM(earth_gravity_accurate)
API_EXPORT double earth_gravity_accurate(double lat, double alt);

#ifdef	__cplusplus
}
#endif

#endif	/* _ACF_UTILS_PERF_H_ */
