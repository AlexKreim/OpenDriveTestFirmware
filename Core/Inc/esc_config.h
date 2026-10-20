#ifndef ESC_CONFIG_H
#define ESC_CONFIG_H

/* =====================================================================
 *  ESC test firmware - configuration
 *  Everything marked PLACEHOLDER must be checked against your schematic /
 *  datasheets before you trust the numbers it produces.
 * ===================================================================== */

/* ---------- Clock / PWM ---------- */
#define SYSCLK_HZ          170000000UL
#define PWM_FREQ_HZ        20000UL
#define PWM_ARR            (SYSCLK_HZ / (2UL * PWM_FREQ_HZ))   /* center-aligned -> 4250 */
#define DEADTIME_NS        500UL                                /* BSC030N08NS5 + L6387E: conservative */
#define DEADTIME_DTG       ((DEADTIME_NS * 170UL) / 1000UL)     /* 5.88 ns/step, must be <= 127 */
#define SAMPLE_MARGIN_CNT  150UL   /* software trigger: counts the counter must still be above the highest CCR when the ADC starts (12.5-cycle sampling = 50 counts, rest settling) */
#define ADC_TRIG_OFFSET    100UL   /* CC4 compare point, counts before the PWM peak (low sides on) */

/* ---------- Analog front end ---------- */
#define ADC_VREF_V         3.3f    /* assumed VDDA */
#define VBUS_DIV           23.0f   /* (22k + 1k) / 1k */
#define PHASE_DIV          16.0f   /* (33k + 2k2) / 2k2 */
#define PHASE_VD_V         0.30f   /* Schottky drop (BAT30KF) in series with the 2k2; calibrate with the high side on: Vd = (16 * Vpin - Vbus) / 15 */
#define BEMF_EN_ACTIVE_LOW 1       /* PB10 low = divider enabled (from schematic, please confirm) */

/* Op-amps: 1 = OPAMP1..3 are configured in CubeMX (PGA mode, output left on the pin) and started
 * from esc_app_init() with HAL_OPAMP_Start(). 0 = old firmware setup (external feedback network). */
#define OPAMP_FROM_CUBEMX  1

/* Phase current chain (from the schematic): shunt 1 mOhm -> R46..48 (1k5) -> op-amp VINP, which also has the
 * 62k/10k bias to 3V3; the inverting pin sits on the ground side of the shunt, the op-amp is in PGA mode and
 * the output is read directly from the pad.
 *   VINP = 0.068 V + 0.852 * Vshunt      (0.852 = 8.62k / (8.62k + 1k5))
 *   idle output = 0.068 V * gain          (about 1.09 V for gain 16, check it at the test point with a multimeter)
 *   sensitivity = 0.852 * gain * 1 mOhm   (13.6 mV/A for gain 16)
 * CS_PGA_GAIN must equal the gain chosen in CubeMX. The zero offset is calibrated at boot. */
#define CS_PGA_GAIN        16.0f
#define CS_SHUNT_OHM       0.001f
#define CS_ATTEN           0.852f
#define CS_V_PER_A         (CS_ATTEN * CS_PGA_GAIN * CS_SHUNT_OHM)
#define CS_SIGN            (-1.0f)  /* low-side shunt: current out of the phase gives a positive shunt voltage; -1 = positive into the motor */

/* PLACEHOLDER: CC6905 sensitivity in V/A for YOUR variant (see datasheet) */
#define CC6905_V_PER_A     0.040f

/* NTC parameters - change these when you calibrate (beta model) */
#define NTC_DRV_R25        10000.0f   /* driver NTC, 10k pulldown to GND, NTC on the 3V3 side */
#define NTC_DRV_BETA       3380.0f
#define NTC_DRV_RFIX       10000.0f
#define NTC_MOT_R25        10000.0f   /* motor NTC, 10k pullup to 3V3, NTC on the GND side */
#define NTC_MOT_BETA       3435.0f   /* ODrive M8325s: NTC 10k, beta 3435 */
#define NTC_MOT_RFIX       10000.0f

/* ---------- Encoder (AS5047P) ---------- */
#define ENC_CPR            4000UL     /* PLACEHOLDER: ABI counts per revolution in x4 mode (check AS5047P ABI setting) */
#define ENC_USE_INDEX      0          /* 1: TIM3 counter reset by index pulse on PD2 (ARR = CPR-1) */

/* ---------- Protection limits (software only, there is no hardware OCP) ---------- */
#define VBUS_MIN_V         9.0f
#define VBUS_MAX_V         62.0f

/* ---------- Firmware version reported to the GUI ---------- */
#define ESC_FW_VERSION     "2.0"
#define DEF_PRESET_NAME    "ODrive M8325s 100KV"
#define OC_SAMPLES         4          /* consecutive 50 us samples above the trip level before an overcurrent fault */
#define VBUS_UV_SAMPLES    5          /* consecutive 10 ms samples below VBUS_MIN_V before an undervoltage fault */
#define ZERO_MIN_COUNTS    200.0f     /* idle level must lie inside this window (expected ~1350), else the op-amp is saturated */
#define ZERO_MAX_COUNTS    3900.0f
#define ZERO_MAX_NOISE     150.0f     /* ADC counts peak-to-peak: noisier than this and "zero" refuses to save */

/* ---------- Default parameters (used until the GUI writes its own to flash) ----------
 * ODrive M8325s-100kV, auto mode. Datasheet: 20 pole pairs, 100 rpm/V, 24 mOhm phase-neutral
 * (48 mOhm line-line), 9.9 uH phase-neutral, 0.083 Nm/A.
 * Auto mode (DEF_AUTO = 1) derives boost, align voltage and the modulation limit from the
 * motor constants and the live bus voltage; the manual values below are used when auto = 0.
 * 'start' still has to be sent, nothing starts on its own. */
#define DEF_AUTO           1
#define DEF_POLE_PAIRS     20
#define DEF_KV             100.0f     /* rpm per volt */
#define DEF_RPM            60.0f      /* mechanical rpm */
#define DEF_ACCEL          60.0f      /* rpm per second */
#define DEF_RLL_MOHM       48.0f      /* line-line resistance */
#define DEF_LLL_UH         19.8f      /* line-line inductance */
#define DEF_KT             0.083f     /* Nm per A */
#define DEF_IDRIVE         3.0f       /* auto mode: peak phase current used for holding / boost */
#define DEF_RPM_MAX        1500.0f
#define DEF_ILIM_A         8.0f       /* software overcurrent trip, per phase (needs a correct CS_V_PER_A) */
#define DEF_ALIGN_MS       2000
/* Stage 2 defaults */
#define DEF_CUR_BW         1000.0f    /* Hz, current loop bandwidth (auto gains) */
#define DEF_CUR_KP         0.06f      /* manual gains, V/A */
#define DEF_CUR_KI         150.0f     /* V/(A s) */
#define DEF_SPD_KP         0.04f     /* A per rpm */
#define DEF_SPD_KI         0.20f      /* A per (rpm s) */
#define DEF_POS_KP         1.5f       /* rpm per degree */
#define DEF_POS_KI         0.0f
#define DEF_POS_KD         0.0f
#define DEF_POS_VMAX       100.0f     /* rpm */
#define DEF_POS_ACC        500.0f     /* rpm/s */
#define DEF_IMAX           2.0f       /* A, torque / FOC current limit */
#define DEF_IALIGN         1.0f       /* A, d-axis current for the alignment at every FOC start */
#define DEF_ICAL           2.0f       /* A, motor calibration current */
#define DEF_OT_DRV_C       80.0f      /* driver NTC trip */
#define DEF_OT_MOT_C       100.0f     /* motor NTC trip */
#define DEF_OT_DRV_EN      1
#define DEF_OT_MOT_EN      0          /* the motor NTC is not always fitted */
#define FOC_DMAX           0.90f      /* duty limit so every low side is on at the sampling instant */
#define DEF_SIX_STEP       0          /* 0 = sinusoidal, 1 = 6-step */
/* manual mode values */
#define DEF_BOOST_V        0.30f      /* extra line-line volts, also covers the dead-time loss */
#define DEF_MAX_MOD        0.10f
#define DEF_ALIGN_V        0.30f

#endif
