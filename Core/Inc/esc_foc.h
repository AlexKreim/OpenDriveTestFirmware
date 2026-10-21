#ifndef ESC_FOC_H
#define ESC_FOC_H

/* Field oriented control, closed loops and motor calibration.
 * Pure C without hardware access: the caller feeds the measurements in and writes the duties out
 * (esc_motor.c does that from the 20 kHz ADC interrupt). This file is also compiled on the PC for the simulation tests.
 *
 * Conventions
 *   alpha axis = phase A axis, amplitude invariant Clarke transform, d axis = rotor magnet axis.
 *   Position and speed are in the SENSOR frame (what the AS5047P reports, 0..360 deg, positive = sensor angle rising).
 *   enc_dir (+1 / -1) tells how the sensor angle relates to the electrical angle: theta_e = enc_dir * pp * theta_sensor + const.
 *   A positive torque command always turns the shaft towards rising sensor angle.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esc_params.h"

typedef enum { FM_TORQUE = 1, FM_SPEED = 2, FM_POSITION = 3 } foc_mode_t;
typedef enum { FP_OFF = 0, FP_ALIGN, FP_RUN } foc_phase_t;
typedef enum { FF_NONE = 0, FF_ENCODER, FF_OVERSPEED } foc_fault_t;

typedef struct {
    float    ia, ib, ic;      /* phase currents, A */
    uint16_t enc;             /* AS5047P ANGLECOM, 14 bit */
    bool     enc_ok;          /* false: this reading is invalid (parity / error flag / no answer) */
    float    vbus;            /* V */
    float    dt;              /* s since the previous call (0 = nominal period) */
} foc_in_t;

typedef struct {
    float tq_a;               /* torque mode: q-axis current, A (sensor frame, signed) */
    float rpm;                /* speed mode: target, rpm (sensor frame, signed) */
    float pos_deg;            /* position mode: absolute target in the FOC frame, degrees (multi-turn, no wrapping) */
    float iq_ff;              /* speed / position modes: torque-frame current added to the loop output, A (anti-cogging) */
} foc_cmd_t;

#define FOC_SPD_WIN 8

typedef struct {
    /* outputs / telemetry */
    float duty[3];            /* 0..1, quantised to the timer resolution */
    float id, iq, vd, vq;     /* measured currents and applied voltage in the rotor frame */
    float theta_e;            /* electrical angle used for the Park transform, rad */
    float rpm;                /* measured speed, sensor frame */
    float pos_deg;            /* measured position, 0..360 */
    float pos_sp_deg;         /* position set point after the motion profile, 0..360 */
    float iq_ref, id_ref;     /* current references (rotor frame), A */
    float iq_ref_t;           /* q reference in the torque frame (positive = towards rising sensor angle) */
    float rpm_sp;             /* speed set point after the ramp / position loop, rpm */
    foc_phase_t phase;
    foc_fault_t fault;
    /* internal state */
    foc_mode_t mode;
    float id_int, iq_int, spd_int, pos_int;
    float enc_off;
    uint16_t enc_prev, enc_cand;
    uint8_t enc_cand_n;
    int32_t pos_cnt;
    int32_t turns_base;       /* whole turns removed from pos_cnt to keep the floats small: absolute = turns_base * 16384 + pos_cnt */
    int32_t ring[FOC_SPD_WIN + 1];
    uint8_t ring_idx, outer_cnt, enc_fail, ovs;
    uint32_t tick, align_ticks;
    float al_first, al_sum;
    uint32_t al_n;
    float p_sp, v_sp;         /* position profile: continuous degrees, deg/s */
    float p_hat, v_hat;       /* tracking observer for position / speed, degrees, deg/s */
    float    dt_acc;          /* time accumulated since the last outer tick */
} foc_t;

void foc_clarke_park(float ia, float ib, float ic, float theta, float *id, float *iq);

void foc_begin(foc_t *f, foc_mode_t mode, const esc_params_t *p, const foc_in_t *in);
/* One 20 kHz step. false = fault (see f->fault), outputs must be switched off. */
bool foc_step(foc_t *f, const foc_in_t *in, const foc_cmd_t *cmd, const esc_params_t *p, const esc_eff_t *e);

/* ---------- motor calibration: R, L, encoder direction and pole pairs ---------- */
typedef struct {
    float duty[3];
    int   step;
    uint32_t tick, acc_n;
    float v_a;                         /* alpha voltage command */
    float acc_v, acc_i;
    float v1, i1;
    float l_num, l_den, vh_p, vh_m;
    uint32_t n_p, n_m;
    float i_prev, i_fl;
    float th_cmd, th_cmd_end;
    float id_int, iq_int;
    int32_t pos_cnt, pos0;
    uint16_t enc_prev, enc_cand;
    uint8_t enc_fail, enc_cand_n;
    /* results */
    float r_phase_ohm, l_phase_h, pp_meas;
    int8_t enc_dir;
    const char *err;
} foc_cal_t;

void foc_cal_begin(foc_cal_t *c, const foc_in_t *in);
/* 0 = running, 1 = finished (results valid), -1 = failed (c->err) */
int  foc_cal_step(foc_cal_t *c, const foc_in_t *in, const esc_params_t *p);

/* ---------- speed / position gain auto-tune ----------
 * Runs on top of an ordinary torque-mode FOC run (foc_step is not changed): it only produces the torque command.
 * After the alignment: +I until rpm_tgt, then -I down to 10 % of rpm_tgt, then 0. The speed is fitted with a straight line
 * between 20 % and 80 % of rpm_tgt in both phases: J = 2 Kt I / (a1 - a2), friction torque Tf = -J (a1 + a2) / 2. */
typedef struct {
    int   step;
    float i_tq, rpm_tgt, sgn;          /* test current (A), target speed (rpm), direction (+1 / -1, from 'dir') */
    float t;                           /* time in the current step, s */
    float sx, sy, sxx, sxy;            /* least-squares sums: x = t, y = rpm */
    uint32_t n;
    uint8_t  last_outer;
    float a1, a2;                      /* fitted acceleration / deceleration, rpm/s (in the test direction) */
    float tq;                          /* torque command for the next step, A */
    /* results */
    float j_kgm2, tf_nm, kt;
    const char *err;
} foc_tune_t;

void  foc_tune_begin(foc_tune_t *t, const esc_params_t *p, float vbus);
/* call after every foc_step of the torque-mode run; dt = s since the previous call. 0 = running, 1 = done, -1 = failed (t->err).
 * t->tq is the torque command (A, positive = towards rising sensor angle) for the next foc_step. */
int   foc_tune_step(foc_tune_t *t, const foc_t *f, float dt);
/* gains from the identified inertia for a speed-loop bandwidth bw_hz: spdkp = J w / Kt (A/rpm), spdki = spdkp w / 4, poskp = w / 24 */
void  foc_tune_gains(const foc_tune_t *t, float bw_hz, float *spd_kp, float *spd_ki, float *pos_kp);

/* ---------- anti-cogging map measurement ----------
 * Runs on top of a position-mode FOC run and produces its position command (FOC frame): the set point creeps one mechanical turn
 * forward and back, the loop's torque-frame current reference (iq_ref_t) is averaged per sensor-angle bin (COG_N bins = sensor
 * counts >> COG_SHIFT). Forward and backward means are averaged (friction cancels), the mean over the turn is removed.
 * foc_cog_step (control interrupt) only accumulates; foc_cog_finish does the heavy part in the main loop. */
#define COG_SHIFT 2u
#define COG_N     (16384u >> COG_SHIFT)
#define COG_EMPTY ((int16_t)-32768)

typedef struct {
    int      step;
    float    t;                        /* time in the current step, s */
    float    v_dps;                    /* sweep speed, deg/s */
    float    p0, p_cmd;                /* start position and position command, degrees, FOC frame */
    float   *sum;                      /* COG_N each, the caller's buffers */
    uint16_t *cnt;
    int16_t  *map;                     /* mA: forward means after the first sweep, the result after foc_cog_finish */
    uint32_t conv_i;                   /* bins converted so far (in chunks during the pause) */
    float    sum_f, sum_r;             /* sum of the bin means per direction, A (friction estimate) */
    uint32_t nb_f, nb_r;
    /* results */
    float    pp_ma, rms_ma, fric_a;
    uint32_t empty;
    const char *err;
} foc_cog_t;

void  foc_cog_begin(foc_cog_t *c, float s_per_turn, float *sum, uint16_t *cnt, int16_t *map);
/* call after every foc_step of the position-mode run: 0 = running, 1 = sweeps done (then foc_cog_finish), -1 = failed (c->err).
 * c->p_cmd is the position command for the next foc_step. */
int   foc_cog_step(foc_cog_t *c, const foc_t *f, float dt);
/* main loop, after foc_cog_step returned 1: builds the final map in c->map. false = failed (c->err). */
bool  foc_cog_finish(foc_cog_t *c);
/* feed-forward current (A, torque frame) for a 14-bit sensor angle, linear interpolation between bins */
float foc_cog_lookup(const int16_t *map, uint16_t enc);

#endif
