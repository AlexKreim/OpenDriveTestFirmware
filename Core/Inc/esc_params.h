#ifndef ESC_PARAMS_H
#define ESC_PARAMS_H

/* Parameters, derived ("effective") values and the flash image format.
 * This file and esc_params.c have no hardware dependencies. */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PRESET_NAME_LEN 24

typedef struct {
    uint8_t  pole_pairs;
    uint8_t  six_step;     /* 0 = sinusoidal, 1 = 6-step */
    int8_t   dir;          /* +1 / -1 */
    uint8_t  auto_mode;    /* 1: boost / align voltage / modulation limit are derived from the motor constants */
    float    kv;           /* rpm per volt (peak line-line back-EMF) */
    float    rpm;          /* target, mechanical */
    float    accel;        /* rpm/s */
    float    boost_v;      /* manual mode: extra line-line volts */
    float    max_mod;      /* manual mode: modulation limit */
    float    ilim;         /* A, per phase, software overcurrent trip */
    float    align_v;      /* manual mode: align voltage */
    uint32_t align_ms;
    float    r_ll_mohm;    /* line-line resistance, mOhm */
    float    l_ll_uh;      /* line-line inductance, uH (used by stage 2) */
    float    kt_nm_a;      /* torque constant, Nm/A (used by stage 2) */
    float    i_drive;      /* auto mode: target peak phase current for holding / boost, A */
    float    rpm_max;      /* speed limit, rpm */
    /* ---- Stage 2: FOC / closed loops / protection ---- */
    float    cur_bw;       /* current loop bandwidth, Hz (auto gains) */
    float    cur_kp;       /* manual current loop gains: V/A */
    float    cur_ki;       /* V/(A s) */
    float    spd_kp;       /* speed loop: A per rpm */
    float    spd_ki;       /* A per (rpm s) */
    float    pos_kp;       /* position loop: rpm per degree */
    float    pos_ki;       /* rpm per (degree s) */
    float    pos_kd;       /* rpm per (degree/s) */
    float    pos_vmax;     /* position move speed limit, rpm */
    float    pos_acc;      /* position move acceleration, rpm/s */
    float    i_max;        /* torque/FOC current limit (q-axis), A */
    float    i_align;      /* d-axis current for the rotor alignment at every FOC start, A */
    float    i_cal;        /* current used by the motor calibration, A */
    float    ot_drv_c;     /* driver temperature trip, C */
    float    ot_mot_c;     /* motor temperature trip, C */
    uint8_t  cur_auto;     /* 1: current loop gains from R, L and bandwidth */
    uint8_t  ot_drv_en;    /* driver overtemperature trip enabled */
    uint8_t  ot_mot_en;    /* motor overtemperature trip enabled (needs a fitted NTC) */
    int8_t   enc_dir;      /* 0 = not calibrated, +1 / -1 = encoder counts with / against the electrical angle */
    char     preset_name[PRESET_NAME_LEN];   /* name of the motor preset these values came from (GUI bookkeeping) */
} esc_params_t;            /* 144 bytes */

/* Values the control loop really uses (equal to the manual values when auto_mode = 0) */
typedef struct {
    volatile float boost_v;
    volatile float align_v;
    volatile float max_mod;
    volatile float rpm_limit;
    volatile float cur_kp;      /* gains the current loop really uses */
    volatile float cur_ki;
} esc_eff_t;

extern esc_params_t g_par;
extern esc_eff_t    g_eff;

void        params_defaults(esc_params_t *p);
int         params_count(void);
const char *params_name(int idx);
bool        params_set(esc_params_t *p, const char *name, float v);     /* false: unknown name or out of range */
bool        params_get(const esc_params_t *p, const char *name, float *v);
bool        params_set_name(esc_params_t *p, const char *name);          /* printable ASCII, no commas, 1..23 chars */
int         params_format_name(const esc_params_t *p, char *buf, size_t n);   /* "N,<name>\r\n" */
int         params_format(const esc_params_t *p, int idx, char *buf, size_t n);   /* "P,<name>,<value>\r\n" */
void        params_derive(const esc_params_t *p, float vbus, esc_eff_t *e);
float       params_max_rpm(const esc_params_t *p, float vbus);          /* what the bus voltage allows */

/* Flash image */
#define PARAMS_MAGIC    0x50435345u     /* "ESCP" */
#define PARAMS_VERSION  5u
#define SAVED_FLAG_PAR  0x1u            /* the image holds valid motor/drive parameters */
#define SAVED_FLAG_CAL  0x2u            /* the image holds valid current-sensor zero offsets */

typedef struct {
    uint32_t     magic;
    uint32_t     version;
    uint32_t     flags;                 /* SAVED_FLAG_* */
    esc_params_t par;
    float        cs_offset[3];          /* current sensor zero offsets, ADC counts (phase A, B, C) */
    uint32_t     reserved;
    uint32_t     crc;                   /* CRC-32 (zlib) over everything before this field */
} esc_saved_t;                          /* 176 bytes, multiple of 8 for double-word programming */

uint32_t params_crc32(const void *data, size_t n);
void     params_pack(esc_saved_t *s, uint32_t flags, const esc_params_t *p, const float off[3]);
/* false if magic / version / crc are wrong or a flagged part is out of range; outputs are only written for flagged parts */
bool     params_unpack(const esc_saved_t *s, uint32_t *flags, esc_params_t *p, float off[3]);

#endif
