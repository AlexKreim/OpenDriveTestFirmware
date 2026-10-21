#include "esc_motor.h"
#include "esc_hw.h"
#include "stm32g4xx_hal.h"
#include "esc_foc.h"
#include "esc_faultlog.h"
#include <math.h>
#include <string.h>

#define TWO_PI          6.28318531f
#define CAL_N           1024u
#define PRECHARGE_TICKS (PWM_FREQ_HZ * 5UL / 1000UL)      /* 5 ms: all low sides on, charges the bootstrap caps */
#define TS              (1.0f / (float)PWM_FREQ_HZ)
#define CCER_ALL        (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                         TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC4E)

volatile float       g_vbus      = 0.0f;
volatile esc_state_t g_state     = ST_IDLE;
volatile esc_fault_t g_fault     = FLT_NONE;
volatile float       g_i[3]      = {0, 0, 0};
volatile float       g_rpm_meas  = 0.0f;
volatile float       g_rpm_cmd   = 0.0f;
volatile float       g_rpm_abi   = 0.0f;
volatile float       g_v_ll      = 0.0f;
volatile bool        g_cal_done  = false;
volatile uint16_t    g_raw[3];
volatile esc_mode_t  g_mode      = MODE_OPEN;
volatile float       g_tq_cmd    = 0.0f;
volatile float       g_pos_cmd   = 0.0f;
volatile bool        g_vel_ext   = false;
volatile float       g_vel_ext_rpm = 0.0f;
static volatile ctl_src_t owner = SRC_NONE;
volatile float       g_id, g_iq, g_imag, g_iq_ref, g_pos_sp, g_vd, g_vq;

#define FOC_LOG_N 100
static float         foc_log[FOC_LOG_N][12];      /* ia ib ic id iq vd vq theta_e(deg) id_ref iq_ref enc_raw enc_ok */
static uint32_t      foc_log_head;
static volatile bool foc_log_frozen;
static foc_t         foc;
static foc_cal_t     cal;
static esc_mode_t    run_mode;                   /* mode latched at start */
static bool          cal_request;                /* the coming start is a calibration */
static volatile int  cal_rc;
/* gain auto-tune: a torque-mode FOC run whose torque command comes from foc_tune_step. tune_arm is set by motor_tune_start just
 * before it calls motor_start; motor_start moves it to tune_request (one-shot). Stop, fault and the end of the test clear it. */
static foc_tune_t    tune;
static bool          tune_arm;
static volatile bool tune_request;
static volatile int  tune_rc;
/* anti-cogging: a position-mode FOC run whose position command comes from foc_cog_step (cog_arm -> cog_request, one-shot like
 * the tune). The map (mA per sensor-angle bin) is added as Iq feed-forward in speed and position mode while cog_on. */
static foc_cog_t     cog;
static float         cog_sum[COG_N];
static uint16_t      cog_cnt[COG_N];
static int16_t       cog_map[COG_N];
static float         cog_s_turn = 60.0f;
static bool          cog_arm;
static volatile bool cog_request;
static volatile int  cog_rc;
static volatile bool cog_valid, cog_on;
static volatile uint16_t enc_cached;
static volatile bool enc_cached_ok;

static float offs[3] = {2048.0f, 2048.0f, 2048.0f};
static volatile uint32_t cal_n = CAL_N;          /* CAL_N = not calibrating */
volatile uint32_t g_trig_win_n;     /* boot check: interrupts that came right around the PWM peak (correct trigger source) */
static uint32_t   cal_ms;
static volatile uint32_t cal_acc[3];
static volatile uint16_t cal_min[3], cal_max[3];

static float    scope_buf[SCOPE_N][SCOPE_CH];
static volatile uint16_t scope_n;
static volatile uint8_t  scope_st;               /* 0 idle, 1 armed, 2 capturing, 3 done */
static uint8_t  scope_div = 1, scope_cnt;

static volatile bool     adc_hist_on;
static volatile uint32_t adc_hist[3][32];
static volatile uint32_t adc_hist_n;

static volatile int32_t mt_abs;                 /* encoder position, multi-turn, 16384 counts per turn */
static volatile bool    mt_valid;
static uint16_t         mt_prev;
static volatile bool    mt_resync = true;        /* main loop: take mt_prev from mt_abs (after FOC ran) */
static volatile float   pos_off_turns;           /* user frame offset */
static volatile float   pos_sp_user;             /* profile set point in the user frame, degrees */

static float    theta, rpm_cur;
static uint32_t tick;
static uint32_t sp_tick;
static uint16_t sp_last;
static int32_t  sp_acc;

/* ------------------------------------------------------------------ */
void motor_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;        /* cycle counter for the interrupt timing statistics */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    sp_last = hw_enc_count();
}

float motor_offset_counts(int ph) { return offs[ph]; }
float motor_cal_noise(int ph) { return (cal_max[ph] >= cal_min[ph]) ? (float)(cal_max[ph] - cal_min[ph]) : 0.0f; }
void  motor_set_offsets(const float off[3]) { for (int i = 0; i < 3; i++) offs[i] = off[i]; }
uint32_t motor_cal_count(void) { return cal_n; }

bool motor_calibrate_offsets(void)
{
    if (g_state != ST_IDLE) return false;
    cal_acc[0] = cal_acc[1] = cal_acc[2] = 0;
    for (int i = 0; i < 3; i++) { cal_min[i] = 0xFFFFu; cal_max[i] = 0u; }
    g_trig_win_n = 0;
    cal_n = 0;
    uint32_t t0 = HAL_GetTick();
    while (cal_n < CAL_N) {
        if (HAL_GetTick() - t0 > 200) return false;   /* no ADC triggers: something is wrong with TIM1/ADC */
    }
    cal_ms = HAL_GetTick() - t0;
    for (int i = 0; i < 3; i++) offs[i] = (float)cal_acc[i] / (float)CAL_N;
    g_cal_done = true;
    return true;
}

/* after motor_calibrate_offsets() on a hardware trigger: did the samples come once per PWM period, at the right instant? */
bool motor_trigger_timing_ok(void)
{
    uint32_t expect_ms = (CAL_N * 1000u) / PWM_FREQ_HZ;                 /* 51 ms */
    return cal_ms + 10u >= expect_ms && cal_ms <= expect_ms + 15u && g_trig_win_n >= (CAL_N * 95u) / 100u;
}

/* ------------------------------------------------------------------ */
static void apply_vector(float th, float v_ll)
{
    float vb = g_vbus;
    if (vb < 5.0f) vb = 5.0f;
    volatile uint32_t *ccr = &TIM1->CCR1;

    if (g_par.six_step) {
        static const uint8_t hi[6] = {0, 0, 1, 1, 2, 2};
        static const uint8_t lo[6] = {1, 2, 2, 0, 0, 1};
        float d = v_ll / vb;
        if (d > g_eff.max_mod) d = g_eff.max_mod;
        uint32_t duty = (uint32_t)(d * (float)PWM_ARR);
        uint32_t sec  = (uint32_t)(th * 0.95492966f);          /* 3/pi: 60 electrical degrees per sector */
        if (sec > 5u) sec = 5u;
        uint32_t ccer = TIM_CCER_CC4E;
        for (uint32_t ph = 0; ph < 3; ph++) {
            if (ph == hi[sec])      { ccr[ph] = duty; ccer |= (0x5u << (4u * ph)); }
            else if (ph == lo[sec]) { ccr[ph] = 0;    ccer |= (0x5u << (4u * ph)); }
            else                    { ccr[ph] = 0; }                                  /* floating phase */
        }
        if (TIM1->CCER != ccer) TIM1->CCER = ccer;
    } else {
        float m = (v_ll * 0.57735027f) / (0.5f * vb);          /* phase peak = line-line peak / sqrt(3) */
        if (m > g_eff.max_mod) m = g_eff.max_mod;
        for (uint32_t ph = 0; ph < 3; ph++) {
            float s = sinf(th - (float)ph * 2.0943951f);
            float d = 0.5f + 0.5f * m * s;
            ccr[ph] = (uint32_t)(d * (float)PWM_ARR);
        }
        if (TIM1->CCER != CCER_ALL) TIM1->CCER = CCER_ALL;
    }
}

static void write_duty(const float d[3])
{
    volatile uint32_t *ccr = &TIM1->CCR1;
    for (int i = 0; i < 3; i++) {
        float x = d[i];
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        ccr[i] = (uint32_t)(x * (float)PWM_ARR + 0.5f);
    }
    if (TIM1->CCER != CCER_ALL) TIM1->CCER = CCER_ALL;
}

volatile uint32_t g_cs_rejected;                 /* current samples replaced because the three phases did not add up */
volatile uint32_t g_cs_late;                     /* periods skipped: software ADC start came too late for a clean sample */

static void make_input(foc_in_t *in, uint16_t a, bool ok)
{
    /* The phase currents must add up to zero. A sample where they do not (a spike on one shunt channel) is replaced by the previous
     * one for up to 8 cycles so that one bad reading cannot kick the current controller. Real overcurrent still trips the debounced
     * protection, which looks at the raw samples. */
    static float last[3];
    static uint8_t bad_run;
    float lim = fmaxf(10.0f, 0.5f * g_par.ilim);
    if (fabsf(g_i[0] + g_i[1] + g_i[2]) > lim && bad_run < 8u) {
        bad_run++;
        g_cs_rejected++;
    } else {
        bad_run = 0;
        last[0] = g_i[0]; last[1] = g_i[1]; last[2] = g_i[2];
    }
    in->ia = last[0]; in->ib = last[1]; in->ic = last[2];
    static uint32_t t_last;
    uint32_t t_now = DWT->CYCCNT;
    float dt = (float)(t_now - t_last) * (1.0f / (float)SYSCLK_HZ);
    t_last = t_now;
    in->dt = (dt > 0.5f / (float)PWM_FREQ_HZ && dt < 20.0f / (float)PWM_FREQ_HZ) ? dt : 0.0f;    /* first call / long pause: nominal */
    in->enc = a; in->enc_ok = ok; in->vbus = g_vbus;
}

/* one 20 kHz FOC / calibration step; returns false when the outputs were switched off */
static bool foc_tick(const foc_in_t *in)
{
    if (g_state == ST_CALIB) {
        int rc = foc_cal_step(&cal, in, &g_par);
        if (rc == 0) { write_duty(cal.duty); return true; }
        cal_rc = rc;
        cal_request = false;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
        g_state = ST_IDLE;
        return false;
    }
    foc_cmd_t cmd;
    cmd.tq_a = tune_request ? tune.tq : g_tq_cmd;
    cmd.rpm = g_vel_ext ? g_vel_ext_rpm : g_par.rpm * (float)g_par.dir;
    cmd.pos_deg = g_pos_cmd + (pos_off_turns - (float)foc.turns_base) * 360.0f;     /* user frame -> FOC frame */
    cmd.iq_ff = 0.0f;
    if (cog_request) cmd.pos_deg = cog.p_cmd;
    else if (cog_on && cog_valid && !tune_request) cmd.iq_ff = foc_cog_lookup(cog_map, foc.enc_prev);
    bool step_ok = foc_step(&foc, in, &cmd, &g_par, &g_eff);
    if (!foc_log_frozen) {
        float *r = foc_log[foc_log_head % FOC_LOG_N];
        r[0] = in->ia; r[1] = in->ib; r[2] = in->ic; r[3] = foc.id; r[4] = foc.iq; r[5] = foc.vd; r[6] = foc.vq;
        r[7] = foc.theta_e * (180.0f / 3.14159265f); r[8] = foc.id_ref; r[9] = foc.iq_ref; r[10] = (float)in->enc; r[11] = in->enc_ok ? 1.0f : 0.0f;
        foc_log_head++;
    }
    if (!step_ok) {
        motor_fault(foc.fault == FF_ENCODER ? FLT_ENCODER : FLT_OVERSPEED);
        return false;
    }
    if (cog_request) {
        int rc = foc_cog_step(&cog, &foc, in->dt);
        if (rc != 0) {                                        /* sweeps finished or failed: outputs off, the main loop finishes */
            cog_rc = rc;
            cog_request = false;
            TIM1->BDTR &= ~TIM_BDTR_MOE;
            TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
            g_state = ST_IDLE;
            g_rpm_cmd = 0.0f; g_v_ll = 0.0f;
            return false;
        }
    }
    if (tune_request) {
        int rc = foc_tune_step(&tune, &foc, in->dt);
        if (rc != 0) {                                        /* test finished or failed: outputs off, back to idle */
            tune_rc = rc;
            tune_request = false;
            TIM1->BDTR &= ~TIM_BDTR_MOE;
            TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
            g_state = ST_IDLE;
            g_rpm_cmd = 0.0f; g_v_ll = 0.0f;
            return false;
        }
    }
    write_duty(foc.duty);
    if (foc.phase == FP_RUN && g_state == ST_ALIGN) g_state = ST_RUN;
    g_id = foc.id; g_iq = foc.iq * (float)g_par.enc_dir;      /* torque frame: positive = towards rising sensor angle */ g_vd = foc.vd; g_vq = foc.vq;
    mt_abs = foc.turns_base * 16384 + foc.pos_cnt;
    if (run_mode == MODE_POSITION) {
        float sp = foc.p_sp + ((float)foc.turns_base - pos_off_turns) * 360.0f;
        pos_sp_user = sp;
        sp -= 360.0f * floorf(sp * (1.0f / 360.0f));
        g_pos_sp = (sp >= 360.0f) ? 0.0f : sp;
    } else {
        g_pos_sp = -1.0f;
    }
    g_iq_ref = foc.iq_ref_t;
    g_rpm_cmd = foc.rpm_sp; g_rpm_meas = foc.rpm; g_v_ll = 1.7320508f * sqrtf(foc.vd * foc.vd + foc.vq * foc.vq);
    if ((scope_st == 1u || scope_st == 2u) && foc.phase == FP_RUN && foc.outer_cnt == 0u) {   /* 2 kHz, right after the outer loop */
        scope_st = 2u;
        if (++scope_cnt >= scope_div) {
            scope_cnt = 0;
            float *r = scope_buf[scope_n];
            r[0] = foc.rpm; r[1] = foc.rpm_sp; r[2] = g_iq; r[3] = foc.iq_ref_t; r[4] = foc.id; r[5] = foc.vq; r[6] = foc.pos_deg;
            if (++scope_n >= SCOPE_N) scope_st = 3u;
        }
    }
    return true;
}

static void motor_tick(void)
{
    switch (g_state) {
    case ST_PRECHARGE:
        if (++tick >= PRECHARGE_TICKS) {
            tick = 0; theta = 0.0f; rpm_cur = 0.0f;
            if (run_mode != MODE_OPEN || cal_request) {
                foc_in_t in;
                make_input(&in, enc_cached, enc_cached_ok);
                if (!enc_cached_ok) { motor_fault(FLT_ENCODER); break; }
                if (cal_request) { foc_cal_begin(&cal, &in); g_state = ST_CALIB; }
                else {
                    /* continue the multi-turn count: nearest whole turn that matches the fresh encoder angle */
                    int32_t e = (int32_t)(in.enc & 0x3FFFu);
                    int32_t d = mt_abs - e;
                    int32_t turns = mt_valid ? ((d >= 0) ? (d + 8192) / 16384 : -((-d + 8192) / 16384)) : 0;
                    foc_begin(&foc, (foc_mode_t)run_mode, &g_par, &in);
                    foc.turns_base = turns;
                    mt_abs = turns * 16384 + e; mt_valid = true; mt_resync = true;
                    g_pos_cmd = ((float)mt_abs * (1.0f / 16384.0f) - pos_off_turns) * 360.0f;   /* hold where the shaft is */
                    g_state = ST_ALIGN;
                }
                break;
            }
            apply_vector(0.0f, g_eff.align_v);
            g_v_ll = g_eff.align_v;
            g_state = ST_ALIGN;
        }
        break;

    case ST_CALIB:
        break;                       /* handled in control_isr */

    case ST_ALIGN:
        if (run_mode != MODE_OPEN) break;
        apply_vector(0.0f, g_eff.align_v);
        if (++tick >= (g_par.align_ms * PWM_FREQ_HZ) / 1000UL) { tick = 0; g_state = ST_RUN; }
        break;

    case ST_RUN: {
        if (run_mode != MODE_OPEN) break;
        /* signed set point: 'rpm' x 'dir' from USB, or the CAN speed. A sign change ramps to 0 first, then reverses. */
        static float ol_dir = 1.0f;
        float sp = g_vel_ext ? g_vel_ext_rpm : g_par.rpm * (float)g_par.dir;
        float sp_dir = (sp < 0.0f) ? -1.0f : 1.0f;
        if (rpm_cur == 0.0f) ol_dir = sp_dir;
        float tgt  = (sp_dir == ol_dir) ? fminf(fabsf(sp), g_eff.rpm_limit) : 0.0f;
        float step = g_par.accel * TS;
        if (rpm_cur < tgt)      { rpm_cur += step; if (rpm_cur > tgt) rpm_cur = tgt; }
        else if (rpm_cur > tgt) { rpm_cur -= step; if (rpm_cur < tgt) rpm_cur = tgt; }
        g_rpm_cmd = rpm_cur * ol_dir;

        float w = rpm_cur * (1.0f / 60.0f) * TWO_PI * (float)g_par.pole_pairs * ol_dir;
        theta += w * TS;
        if (theta >= TWO_PI) theta -= TWO_PI;
        if (theta < 0.0f)    theta += TWO_PI;

        float v_ll = rpm_cur / g_par.kv + g_eff.boost_v;
        g_v_ll = v_ll;
        apply_vector(theta, v_ll);
        break;
    }
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
static uint8_t oc_cnt;
volatile float    g_oc_last_a;      /* last sample above the trip level (value, phase, raw ADC counts) */
volatile int      g_oc_last_ph = -1;
volatile uint16_t g_oc_last_raw;

static void control_isr(void)
{
    uint16_t r[3];
    if (!hw_inj_collect(r)) return;
    g_raw[0] = r[0]; g_raw[1] = r[1]; g_raw[2] = r[2];
    if (adc_hist_on) {
        esc_state_t hs = g_state;
        if (hs == ST_ALIGN || hs == ST_RUN || hs == ST_CALIB) {
            adc_hist[0][r[0] & 31u]++; adc_hist[1][r[1] & 31u]++; adc_hist[2][r[2] & 31u]++;
            adc_hist_n++;
        }
    }

    if (cal_n < CAL_N) {
        for (int i = 0; i < 3; i++) {
            cal_acc[i] += r[i];
            if (r[i] < cal_min[i]) cal_min[i] = r[i];
            if (r[i] > cal_max[i]) cal_max[i] = r[i];
        }
        cal_n++;
    }
    for (int i = 0; i < 3; i++)
        g_i[i] = ((float)r[i] - offs[i]) * (ADC_VREF_V / 4095.0f) / CS_V_PER_A * CS_SIGN;

    /* phase current magnitude sqrt(2/3 sum i^2): constant for balanced sinusoidal currents, so it is readable at any speed */
    {
        float m2 = (2.0f / 3.0f) * (g_i[0] * g_i[0] + g_i[1] * g_i[1] + g_i[2] * g_i[2]);
        g_imag += (sqrtf(m2) - g_imag) * (1.0f / 100.0f);        /* ~5 ms low pass */
    }

    esc_state_t s = g_state;
    bool foc_on = (run_mode != MODE_OPEN) || cal_request;
    if (foc_on && (s == ST_PRECHARGE || s == ST_ALIGN || s == ST_RUN || s == ST_CALIB)) {
        /* pipelined AS5047P read: one SPI frame, the angle is one control period old (compensated in esc_foc.c) */
        bool ok;
        uint16_t a = hw_as5047_pipe(&ok);
        enc_cached = a; enc_cached_ok = ok;
    }

    /* speed from the encoder, 50 ms window */
    uint16_t c = hw_enc_count();
    sp_acc += (int16_t)(uint16_t)(c - sp_last);
    sp_last = c;
    if (++sp_tick >= PWM_FREQ_HZ / 20UL) {
        g_rpm_abi = (float)sp_acc * (60.0f * 20.0f / (float)g_enc_cpr);
        if (run_mode == MODE_OPEN) g_rpm_meas = g_rpm_abi;
        sp_acc = 0; sp_tick = 0;
    }

    if (s == ST_PRECHARGE || s == ST_ALIGN || s == ST_RUN || s == ST_CALIB) {
        float lim = g_par.ilim;
        int bad = -1;
        for (int i = 0; i < 3; i++) if (fabsf(g_i[i]) > lim) bad = i;
        if (bad >= 0) {
            /* a switching spike lasts one sample; a real overcurrent lasts several */
            g_oc_last_a = g_i[bad]; g_oc_last_ph = bad; g_oc_last_raw = g_raw[bad];
            if (++oc_cnt >= OC_SAMPLES) {
                motor_fault(FLT_OVERCURRENT);
                oc_cnt = 0;
                return;
            }
        } else {
            oc_cnt = 0;
        }
    }
    if (g_state == ST_CALIB || (foc_on && (g_state == ST_ALIGN || g_state == ST_RUN))) {
        foc_in_t in;
        make_input(&in, enc_cached, enc_cached_ok);
        foc_tick(&in);
        return;
    }
    motor_tick();
    if (!foc_on) {
        /* open loop: show the currents projected on the commanded voltage vector (angle theta - 90 deg for the sin() convention) */
        if (g_state == ST_RUN || g_state == ST_ALIGN) {
            float id, iq;
            foc_clarke_park(g_i[0], g_i[1], g_i[2], theta - 1.5707963f, &id, &iq);
            g_id += (id - g_id) * (1.0f / 100.0f);
            g_iq += (iq - g_iq) * (1.0f / 100.0f);
        }
    }
}


/* hardware-triggered path: the ADC end-of-injected-sequence interrupt */
volatile uint32_t g_isr_cycles_max;

void ADC1_2_IRQHandler(void)
{
    /* hardware trigger at CNT = ARR - 100 (counting up), conversion ~0.6 us, so the interrupt starts within a few hundred
     * counts around the peak. A trigger from a wrong source would come at random counter values. */
    if (cal_n < CAL_N && TIM1->CNT > PWM_ARR - 600u) g_trig_win_n++;
    uint32_t t0 = DWT->CYCCNT;
    control_isr();
    uint32_t dt = DWT->CYCCNT - t0;
    if (dt > g_isr_cycles_max) g_isr_cycles_max = dt;
}

/* software-trigger fallback: TIM1 CC4 compare (just after the PWM peak) starts the conversions */
void TIM1_CC_IRQHandler(void)
{
    if (!(TIM1->SR & TIM_SR_CC4IF)) return;
    TIM1->SR = ~TIM_SR_CC4IF;
    /* center-aligned mode 1: the compare flag is only set while counting down, so this runs once per period (do not filter on DIR).
     * Software start means the sampling instant depends on the interrupt latency. If the interrupt is served late (another
     * interrupt was running), a phase may already be switching or its high side may be on, and its shunt shows garbage (the
     * +-100 A single-phase spikes). Only sample while every low side is still on for the whole ADC sampling time. */
    uint32_t t0 = DWT->CYCCNT;
    {
        uint32_t cnt = TIM1->CNT;
        uint32_t cmax = TIM1->CCR1;
        if (TIM1->CCR2 > cmax) cmax = TIM1->CCR2;
        if (TIM1->CCR3 > cmax) cmax = TIM1->CCR3;
        if (!(TIM1->CR1 & TIM_CR1_DIR) || cnt < cmax + SAMPLE_MARGIN_CNT) {
            g_cs_late++;
            if (run_mode != MODE_OPEN || cal_request) return;   /* open loop keeps its timing (motor_tick runs from here) */
        }
    }
    hw_inj_sw_start();
    uint32_t t = 2000;
    while (!(ADC1->ISR & ADC_ISR_JEOS) && --t) { }
    control_isr();
    uint32_t dt = DWT->CYCCNT - t0;
    if (dt > g_isr_cycles_max) g_isr_cycles_max = dt;
}

/* ------------------------------------------------------------------ */
int motor_foc_log_count(void) { return foc_log_head < FOC_LOG_N ? (int)foc_log_head : FOC_LOG_N; }
bool motor_foc_log(int i, float out[12])
{
    int n = motor_foc_log_count();
    if (i < 0 || i >= n) return false;
    uint32_t first = foc_log_head - (uint32_t)n;
    const float *r = foc_log[(first + (uint32_t)i) % FOC_LOG_N];
    for (int k = 0; k < 12; k++) out[k] = r[k];
    return true;
}

void motor_fault(esc_fault_t f)
{
    flog_fault((int)f);               /* before the state changes: records state, currents and speed at the fault */
    if (g_state == ST_RUN || g_state == ST_ALIGN) foc_log_frozen = true;       /* keep the last samples for 'focdump' */
    TIM1->BDTR &= ~TIM_BDTR_MOE;      /* outputs forced low (OSSI = 1, idle levels = 0) */
    g_fault = f;
    g_state = ST_FAULT;
    g_rpm_cmd = 0.0f;
    g_v_ll = 0.0f;
    g_id = g_iq = g_vd = g_vq = g_iq_ref = 0.0f;
    g_tq_cmd = 0.0f;
    cal_request = false;
    tune_request = false; tune_arm = false;
    cog_request = false; cog_arm = false;
    g_vel_ext = false;
}

void motor_clear_fault(void)
{
    if (g_state == ST_FAULT) { g_state = ST_IDLE; g_fault = FLT_NONE; }
}

bool motor_start(void)
{
    tune_request = tune_arm;           /* one-shot: only the start made by motor_tune_start is a tune run */
    tune_arm = false;
    cog_request = cog_arm;             /* same for motor_cog_start */
    cog_arm = false;
    if (g_state != ST_IDLE || !g_cal_done) { tune_request = false; cog_request = false; return false; }
    float vb = g_vbus;
    params_derive(&g_par, vb, &g_eff);
    if (vb < VBUS_MIN_V || vb > VBUS_MAX_V) { tune_request = false; cog_request = false; return false; }
    if (!cal_request) {
        run_mode = tune_request ? MODE_TORQUE : (cog_request ? MODE_POSITION : g_mode);
        if ((tune_request || cog_request) && g_par.enc_dir == 0) { tune_request = false; cog_request = false; return false; }
        if (run_mode != MODE_OPEN && g_par.enc_dir == 0) return false;     /* FOC needs the encoder calibration */
        if (run_mode == MODE_OPEN) { g_id = g_iq = 0.0f; }
    }
    hw_as5047_reset();
    /* the control interrupt must not wait behind USB (CubeMX gives USB priority 0): control 0, USB 3 */
    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_SetPriority(TIM1_CC_IRQn, 0);
    NVIC_SetPriority(USB_LP_IRQn, 3);
    NVIC_SetPriority(USB_HP_IRQn, 3);
    g_isr_cycles_max = 0;
    enc_cached_ok = false;
    foc_log_frozen = false; foc_log_head = 0;
    g_vd = g_vq = g_iq_ref = 0.0f; g_pos_sp = -1.0f;
    if (tune_request) foc_tune_begin(&tune, &g_par, vb);
    if (cog_request) foc_cog_begin(&cog, cog_s_turn, cog_sum, cog_cnt, cog_map);

    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    TIM1->CCER = CCER_ALL;
    HAL_Delay(1);                      /* let the zero duty reach the outputs */
    tick = 0; theta = 0.0f; rpm_cur = 0.0f;
    g_rpm_cmd = 0.0f;
    g_state = ST_PRECHARGE;
    TIM1->BDTR |= TIM_BDTR_MOE;        /* low sides on (0 % duty), bootstrap charge starts */
    return true;
}

bool motor_cal_start(void)
{
    cal_request = true; cal_rc = 0;
    run_mode = MODE_OPEN;
    bool ok = motor_start();
    if (!ok) cal_request = false;
    return ok;
}

int motor_cal_status(void) { return cal_rc; }

bool motor_tune_start(void)
{
    if (g_state != ST_IDLE) return false;
    tune_rc = 0;
    tune_arm = true;
    return motor_start();                /* clears tune_arm, refuses (and clears the request) like a normal start */
}

int motor_tune_status(void) { return tune_rc; }

bool motor_cog_start(float s_per_turn)
{
    if (g_state != ST_IDLE) return false;
    cog_rc = 0;
    cog_valid = false;                   /* the map buffer is overwritten by the measurement */
    cog_s_turn = s_per_turn;
    cog_arm = true;
    bool ok = motor_start();
    if (!ok) motor_cog_load();
    return ok;
}

int motor_cog_status(void) { return cog_rc; }

bool motor_cog_finish(float *pp_ma, float *rms_ma, float *fric_a, const char **err)
{
    bool ok = (cog_rc > 0) && foc_cog_finish(&cog);
    if (err) *err = cog.err;
    cog_rc = 0;
    if (!ok) { motor_cog_load(); return false; }          /* the previous map (if any) back from flash */
    *pp_ma = cog.pp_ma; *rms_ma = cog.rms_ma; *fric_a = cog.fric_a;
    cog_valid = true; cog_on = true;
    return true;
}

bool motor_cog_load(void)
{
    bool en = false;
    cog_valid = flash_cog_load(cog_map, COG_N, &en);
    cog_on = cog_valid && en;
    return cog_valid;
}

flash_res_t motor_cog_save(void) { return flash_cog_save(cog_map, COG_N, cog_on); }
bool motor_cog_valid(void) { return cog_valid; }
bool motor_cog_enabled(void) { return cog_on && cog_valid; }
void motor_cog_enable(bool on) { cog_on = on && cog_valid; }
void motor_cog_clear(void) { cog_on = false; cog_valid = false; memset(cog_map, 0, sizeof(cog_map)); }
int  motor_cog_value(uint32_t i) { return (i < COG_N) ? cog_map[i] : 0; }

bool motor_tune_result(float bw_hz, float *j_kgm2, float *tf_nm, float *spd_kp, float *spd_ki, float *pos_kp, const char **err)
{
    if (tune_rc == 0) return false;
    if (err) *err = tune.err;
    if (tune_rc < 0) return false;
    *j_kgm2 = tune.j_kgm2; *tf_nm = tune.tf_nm;
    foc_tune_gains(&tune, bw_hz, spd_kp, spd_ki, pos_kp);
    return true;
}

bool motor_cal_result(float *r, float *l, int *dir, float *pp, const char **err)
{
    if (cal_rc == 0) return false;
    if (r) *r = cal.r_phase_ohm;
    if (l) *l = cal.l_phase_h;
    if (dir) *dir = cal.enc_dir;
    if (pp) *pp = cal.pp_meas;
    if (err) *err = cal.err;
    return cal_rc > 0;
}

void motor_adc_hist_start(void)
{
    adc_hist_on = false;
    for (int i = 0; i < 3; i++) for (int b = 0; b < 32; b++) adc_hist[i][b] = 0;
    adc_hist_n = 0;
    adc_hist_on = true;
}
void     motor_adc_hist_stop(void) { adc_hist_on = false; }
bool     motor_adc_hist_on(void) { return adc_hist_on; }
uint32_t motor_adc_hist_n(void) { return adc_hist_n; }
uint32_t motor_adc_hist_get(int ph, int bin) { return (ph >= 0 && ph < 3 && bin >= 0 && bin < 32) ? adc_hist[ph][bin] : 0u; }

void motor_scope_arm(int div)
{
    scope_st = 0u;
    scope_n = 0; scope_cnt = 0;
    scope_div = (uint8_t)(div < 1 ? 1 : (div > 50 ? 50 : div));
    scope_st = 1u;
}
void motor_scope_stop(void) { if (scope_st != 0u) scope_st = 3u; }
int  motor_scope_state(int *n) { if (n) *n = scope_n; return scope_st; }
int  motor_scope_div(void) { return scope_div; }
bool motor_scope_get(int i, float out[SCOPE_CH])
{
    if (i < 0 || i >= (int)scope_n || scope_st == 2u) return false;
    for (int k = 0; k < SCOPE_CH; k++) out[k] = scope_buf[i][k];
    return true;
}

void motor_mt_track(uint16_t enc)
{
    enc &= 0x3FFFu;
    if (motor_foc_active()) return;
    if (!mt_valid) { mt_abs = enc; mt_prev = enc; mt_valid = true; mt_resync = false; return; }
    if (mt_resync) { mt_prev = (uint16_t)(mt_abs & 0x3FFF); mt_resync = false; }
    int32_t d = (int32_t)enc - (int32_t)mt_prev;
    if (d > 8192) d -= 16384; else if (d < -8192) d += 16384;
    mt_abs += d;
    mt_prev = enc;
}

float motor_pos_turns(void) { return (float)mt_abs * (1.0f / 16384.0f) - pos_off_turns; }

void motor_set_pos_turns(float turns)
{
    float now = motor_pos_turns();
    float delta = turns - now;
    pos_off_turns -= delta;
    g_pos_cmd += delta * 360.0f;             /* a running position loop keeps holding the same physical position */
}

float motor_pos_sp_turns(bool *valid)
{
    if (valid) *valid = (g_pos_sp >= 0.0f);
    return pos_sp_user * (1.0f / 360.0f);
}

bool motor_angle(uint16_t *a) { *a = enc_cached; return enc_cached_ok; }
bool motor_foc_active(void)
{
    esc_state_t s = g_state;
    return (s == ST_PRECHARGE || s == ST_ALIGN || s == ST_RUN || s == ST_CALIB) && (run_mode != MODE_OPEN || cal_request);
}

void motor_set_owner(ctl_src_t s) { owner = s; }

ctl_src_t motor_owner(void)
{
    esc_state_t s = g_state;
    return (s == ST_PRECHARGE || s == ST_ALIGN || s == ST_RUN || s == ST_CALIB) ? owner : SRC_NONE;
}

void motor_takeover_usb(void)
{
    if (owner == SRC_CAN && g_vel_ext) {
        /* keep the present speed: the CAN set point becomes the 'rpm' parameter (only if the direction matches) */
        float v = g_vel_ext_rpm * (float)g_par.dir;
        g_par.rpm = (v > 0.0f) ? v : 0.0f;
    }
    g_vel_ext = false;
    owner = SRC_USB;
}

void motor_stop(void)
{
    if (g_state != ST_FAULT) g_state = ST_IDLE;
    g_vel_ext = false;
    cal_request = false;
    tune_request = false; tune_arm = false;
    cog_request = false; cog_arm = false;
    g_tq_cmd = 0.0f;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    g_rpm_cmd = 0.0f;
    g_v_ll = 0.0f;
}

/* ------------------------------------------------------------------ */
void motor_gate_test(float ls_v[3], float hs_v[3])
{
    volatile uint32_t *ccr = &TIM1->CCR1;
    for (int i = 0; i < 3; i++) { ls_v[i] = -1.0f; hs_v[i] = -1.0f; }
    if (g_state != ST_IDLE) return;

    hw_bemf_enable(true);
    for (uint32_t ph = 0; ph < 3; ph++) {
        TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
        TIM1->CCER = (0x5u << (4u * ph)) | TIM_CCER_CC4E;    /* only this phase driven, others forced low */
        HAL_Delay(1);
        TIM1->BDTR |= TIM_BDTR_MOE;
        HAL_Delay(5);                                         /* low side on: charges the bootstrap cap */
        ls_v[ph] = hw_phase_v((int)ph);
        ccr[ph] = PWM_ARR;                                    /* 100 % duty: high side on, low side off */
        HAL_Delay(2);
        hs_v[ph] = hw_phase_v((int)ph);
        ccr[ph] = 0;
        HAL_Delay(2);
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    TIM1->CCER = CCER_ALL;
    hw_bemf_enable(false);
}
