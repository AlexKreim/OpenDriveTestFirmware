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
    float pos_deg;            /* position mode: target, degrees 0..360 */
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

#endif
