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

typedef enum { FLT_NONE = 0, FLT_OVERCURRENT, FLT_OVERVOLT, FLT_UNDERVOLT, FLT_OVERTEMP, FLT_ENCODER, FLT_OVERSPEED,
               FLT_ESTOP, FLT_CAN_TIMEOUT } esc_fault_t;


extern volatile float        g_vbus;        /* updated by the main loop */
extern volatile esc_state_t  g_state;
extern volatile esc_fault_t  g_fault;
extern volatile float        g_i[3];        /* phase currents, A */
extern volatile float        g_rpm_meas;    /* from the TIM3 encoder, mechanical rpm */
extern volatile float        g_rpm_cmd;
extern volatile float        g_rpm_abi;     /* from the TIM3 ABI counter in every mode (diagnostics), 50 ms window */
extern volatile float        g_v_ll;        /* line-line voltage the controller is applying, V */
extern volatile bool         g_cal_done;
extern volatile esc_mode_t   g_mode;        /* selected control mode, change only while stopped */
extern volatile float        g_tq_cmd;      /* torque mode: q current, A (towards rising sensor angle when positive) */
extern volatile float        g_pos_cmd;     /* position mode: absolute target in degrees, multi-turn (360 = one turn), user frame */
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
/* speed / position gain auto-tune (torque pulse test, see foc_tune_t). Same preconditions as a FOC start. */
bool  motor_tune_start(void);
int   motor_tune_status(void);              /* 0 = running / not started, 1 = finished ok, -1 = failed */
bool  motor_tune_result(float bw_hz, float *j_kgm2, float *tf_nm, float *spd_kp, float *spd_ki, float *pos_kp, const char **err);
/* scope: SCOPE_N samples of the FOC outer loop (2 kHz / div), starting at the next FOC tick after the alignment.
 * Channels: rpm, rpm set point, iq, iq reference (torque frame), id, vq (torque frame), position deg. Diagnostics only. */
#define SCOPE_N  1000
#define SCOPE_CH 7
void  motor_scope_arm(int div);
void  motor_scope_stop(void);              /* end a capture early (the samples so far stay readable) */
int   motor_scope_state(int *n);           /* 0 idle, 1 armed, 2 capturing, 3 done */
bool  motor_scope_get(int i, float out[SCOPE_CH]);
int   motor_scope_div(void);
/* ADC code histogram (diagnostics): counts the lowest 5 bits of every phase-current conversion while the motor is driven
 * (ALIGN / RUN / CALIB). Even ADC codes give about 3.1 % per bin; a large bin 31 means codes stick just below 32-count steps. */
void     motor_adc_hist_start(void);       /* clears and starts counting */
void     motor_adc_hist_stop(void);
bool     motor_adc_hist_on(void);
uint32_t motor_adc_hist_n(void);           /* samples counted */
uint32_t motor_adc_hist_get(int ph, int bin);
/* Multi-turn position (like ODrive pos_estimate): followed by the control interrupt while FOC runs and by the main loop
 * (motor_mt_track, every 2 ms) otherwise. User frame = encoder turns - offset (Set_Absolute_Position / 'setpos'). */
void  motor_mt_track(uint16_t enc);         /* main loop: fresh 14-bit angle while FOC is not running */
float motor_pos_turns(void);                /* user frame, turns */
void  motor_set_pos_turns(float turns);     /* the present position becomes 'turns' (position target moves with it) */
float motor_pos_sp_turns(bool *valid);      /* position-mode profile set point, user frame */
/* Control source: whoever starts the motor owns the set points until it stops; stop / estop are always accepted.
 * The USB GUI can take over a motor that CAN started (motor_takeover_usb). */
typedef enum { SRC_NONE = 0, SRC_USB = 1, SRC_CAN = 2 } ctl_src_t;
void      motor_set_owner(ctl_src_t s);      /* after a successful start / calibration */
ctl_src_t motor_owner(void);                 /* SRC_NONE while stopped */
void      motor_takeover_usb(void);          /* CAN speed set point -> 'rpm' parameter, then USB owns the motor */
extern volatile bool  g_vel_ext;             /* speed / open-loop set point from CAN instead of the 'rpm' parameter */
extern volatile float g_vel_ext_rpm;         /* signed rpm, sensor frame */
bool  motor_angle(uint16_t *a);             /* last angle read by the control interrupt (valid while FOC / calibration runs) */
bool  motor_foc_active(void);
int   motor_foc_log_count(void);            /* FOC samples kept for the last fault (diagnostics) */
bool  motor_foc_log(int i, float out[12]);
extern volatile uint32_t g_isr_cycles_max;
extern volatile uint32_t g_cs_rejected;
extern volatile uint32_t g_cs_late;   /* longest control interrupt since the last start, CPU cycles */
void  motor_gate_test(float ls_v[3], float hs_v[3]);

#endif
