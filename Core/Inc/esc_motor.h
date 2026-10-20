#ifndef ESC_MOTOR_H
#define ESC_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_params.h"

typedef enum { ST_IDLE = 0, ST_PRECHARGE, ST_ALIGN, ST_RUN, ST_FAULT, ST_CALIB } esc_state_t;

/* control mode: 0 = open loop (stage 1), 1 = torque, 2 = speed, 3 = position (FOC) */
typedef enum { MODE_OPEN = 0, MODE_TORQUE = 1, MODE_SPEED = 2, MODE_POSITION = 3 } esc_mode_t;
extern volatile float g_oc_last_a;
extern volatile int g_oc_last_ph;
extern volatile uint16_t g_oc_last_raw;

typedef enum { FLT_NONE = 0, FLT_OVERCURRENT, FLT_OVERVOLT, FLT_UNDERVOLT, FLT_OVERTEMP, FLT_ENCODER, FLT_OVERSPEED } esc_fault_t;


extern volatile float        g_vbus;        /* updated by the main loop */
extern volatile esc_state_t  g_state;
extern volatile esc_fault_t  g_fault;
extern volatile float        g_i[3];        /* phase currents, A */
extern volatile float        g_rpm_meas;    /* from the TIM3 encoder, mechanical rpm */
extern volatile float        g_rpm_cmd;
extern volatile float        g_v_ll;        /* line-line voltage the controller is applying, V */
extern volatile bool         g_cal_done;
extern volatile esc_mode_t   g_mode;        /* selected control mode, change only while stopped */
extern volatile float        g_tq_cmd;      /* torque mode: q current, A (towards rising sensor angle when positive) */
extern volatile float        g_pos_cmd;     /* position mode: degrees 0..360 */
/* FOC telemetry (zero in open loop except imag / id / iq which are projected on the commanded angle) */
extern volatile float g_id, g_iq, g_imag, g_iq_ref, g_pos_sp, g_vd, g_vq;
extern volatile uint16_t     g_raw[3];      /* last raw injected ADC counts (phase A/B/C), diagnostics */

void  motor_init(void);
bool  motor_calibrate_offsets(void);        /* needs state IDLE, outputs off */
float motor_offset_counts(int ph);
float motor_cal_noise(int ph);             /* peak-to-peak ADC counts seen during the last calibration */
void  motor_set_offsets(const float off[3]);
uint32_t motor_cal_count(void);            /* number of injected samples collected so far (diagnostics) */
bool  motor_start(void);
void  motor_stop(void);
void  motor_fault(esc_fault_t f);
void  motor_clear_fault(void);
/* Gate-driver test: per phase low side on, then high side on (others off). Needs bus power, motor may be connected. */
bool  motor_trigger_timing_ok(void);       /* boot check of the hardware ADC trigger (rate and instant) */
bool  motor_cal_start(void);                /* motor R/L + encoder calibration, runs in the control interrupt */
int   motor_cal_status(void);               /* 0 = running / not started, 1 = finished ok, -1 = failed */
bool  motor_cal_result(float *r_phase_ohm, float *l_phase_h, int *enc_dir, float *pp, const char **err);
bool  motor_angle(uint16_t *a);             /* last angle read by the control interrupt (valid while FOC / calibration runs) */
bool  motor_foc_active(void);
int   motor_foc_log_count(void);            /* FOC samples kept for the last fault (diagnostics) */
bool  motor_foc_log(int i, float out[12]);
extern volatile uint32_t g_isr_cycles_max;
extern volatile uint32_t g_cs_rejected;
extern volatile uint32_t g_cs_late;   /* longest control interrupt since the last start, CPU cycles */
void  motor_gate_test(float ls_v[3], float hs_v[3]);

#endif
