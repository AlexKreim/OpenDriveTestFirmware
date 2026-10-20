#include "esc_foc.h"
#include "esc_config.h"
#include <math.h>
#include <string.h>

#define PI_F          3.14159265f
#define TWO_PI_F      6.28318531f
#define TS            (1.0f / (float)PWM_FREQ_HZ)
#define OUTER_DIV     10                              /* speed / position loops run at 2 kHz */
#define DT_OUT_NOM    ((float)OUTER_DIV * TS)
/* real time between two outer-loop ticks (measured): control periods can be skipped (late ADC start, long interrupt), and a
 * fixed time step would then turn the position change into a too high and very jittery speed */
static float dt_out = DT_OUT_NOM;
#define DT_OUT        dt_out
#define ENC_JUMP_MAX  160                             /* counts per 50 us (3.5 deg: 11000 rpm) */
#define ENC_FAIL_MAX  20                              /* consecutive bad encoder frames (1 ms) before a fault */
#define DEG_PER_CNT   (360.0f / 16384.0f)
#define FOC_OBS_W     (6.2831853f * 80.0f)           /* observer bandwidth, rad/s */
#define MS(x)         ((uint32_t)((x) * (PWM_FREQ_HZ / 1000UL)))

/* ------------------------------------------------------------------ helpers */
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float wrap_pi(float x)  { while (x > PI_F) x -= TWO_PI_F; while (x < -PI_F) x += TWO_PI_F; return x; }
static float wrap180(float x)  { while (x > 180.0f) x -= 360.0f; while (x < -180.0f) x += 360.0f; return x; }
static float wrap360(float x)  { while (x >= 360.0f) x -= 360.0f; while (x < 0.0f) x += 360.0f; return x; }

void foc_clarke_park(float ia, float ib, float ic, float th, float *id, float *iq)
{
    float al = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
    float be = (ib - ic) * 0.57735027f;
    float s = sinf(th), c = cosf(th);
    *id =  al * c + be * s;
    *iq = -al * s + be * c;
}

/* largest voltage vector that keeps every duty <= FOC_DMAX (so each low side is on while the ADC samples) */
static float v_limit(float vbus) { return (FOC_DMAX - 0.5f) / 0.8660254f * vbus; }

static float quant(float d)
{
    if (d < 0.0f) d = 0.0f;
    if (d > FOC_DMAX) d = FOC_DMAX;
    return floorf(d * (float)PWM_ARR + 0.5f) / (float)PWM_ARR;
}

/* alpha/beta voltage -> three duties (min/max common mode injection = space vector PWM) */
static void out_ab(float va_, float vb_, float vbus, float duty[3])
{
    float a = va_;
    float b = -0.5f * va_ + 0.8660254f * vb_;
    float c = -0.5f * va_ - 0.8660254f * vb_;
    float mx = fmaxf(a, fmaxf(b, c)), mn = fminf(a, fminf(b, c));
    float cm = -0.5f * (mx + mn);
    float k = 1.0f / vbus;
    duty[0] = quant(0.5f + (a + cm) * k);
    duty[1] = quant(0.5f + (b + cm) * k);
    duty[2] = quant(0.5f + (c + cm) * k);
}

/* voltage the (quantised) duties really apply along alpha */
static float valpha_applied(const float duty[3], float vbus)
{
    return (2.0f / 3.0f) * vbus * (duty[0] - 0.5f * (duty[1] + duty[2]));
}

static bool track_enc(int32_t *pos, uint16_t *prev, uint8_t *fail, uint16_t *cand, uint8_t *cand_n, const foc_in_t *in)
{
    uint16_t enc = *prev;
    int32_t d = 0;
    bool good = in->enc_ok;
    if (good) {
        enc = (uint16_t)(in->enc & 0x3FFFu);
        d = (int32_t)enc - (int32_t)*prev;
        if (d > 8191) d -= 16384; else if (d < -8192) d += 16384;
        if (d > ENC_JUMP_MAX || d < -ENC_JUMP_MAX) {                      /* the shaft cannot move that far in 50 us: a corrupted frame ... */
            good = false;
            int32_t dc = (int32_t)enc - (int32_t)*cand;
            if (dc > 8191) dc -= 16384; else if (dc < -8192) dc += 16384;
            if (*cand_n > 0u && dc >= -ENC_JUMP_MAX && dc <= ENC_JUMP_MAX) {
                if (++(*cand_n) >= 4u) { good = true; *cand_n = 0u; }       /* ... unless it repeats: the position really changed, lock on again */
            } else {
                *cand_n = 1u;
            }
            *cand = enc;
        } else {
            *cand_n = 0u;
        }
    }
    if (good) { *fail = 0; }
    else {
        if (++(*fail) > ENC_FAIL_MAX) return false;
        enc = *prev; d = 0;                                                /* hold the last good angle */
    }
    *pos += d;
    *prev = enc;
    return true;
}

/* ================================================================== closed loops */
void foc_begin(foc_t *f, foc_mode_t mode, const esc_params_t *p, const foc_in_t *in)
{
    memset(f, 0, sizeof(*f));
    f->mode = mode;
    f->phase = FP_ALIGN;
    f->enc_prev = (uint16_t)(in->enc & 0x3FFFu);
    f->pos_cnt = f->enc_prev;
    f->p_hat = (float)f->pos_cnt * DEG_PER_CNT;
    f->align_ticks = (p->align_ms < 300u ? 300u : p->align_ms) * (PWM_FREQ_HZ / 1000UL);
    f->p_sp = (float)f->pos_cnt * DEG_PER_CNT;
    f->duty[0] = f->duty[1] = f->duty[2] = 0.5f;
}

static float speed_pi(foc_t *f, const esc_params_t *p, float sp, float meas, float imax)
{
    float e = sp - meas;
    float u = p->spd_kp * e + f->spd_int;
    float out = clampf(u, -imax, imax);
    if (u == out || (e * u) < 0.0f) f->spd_int = clampf(f->spd_int + p->spd_ki * e * DT_OUT, -imax, imax);   /* no winding up into the limit */
    return out;
}

bool foc_step(foc_t *f, const foc_in_t *in, const foc_cmd_t *cmd, const esc_params_t *p, const esc_eff_t *e)
{
    const float vbus = (in->vbus < 5.0f) ? 5.0f : in->vbus;
    const float vlim = v_limit(vbus);
    const float imax = fminf(p->i_max, 0.9f * p->ilim);
    const float ppf  = (float)p->pole_pairs;
    const float dir  = (p->enc_dir < 0) ? -1.0f : 1.0f;

    /* ---- encoder */
    if (!track_enc(&f->pos_cnt, &f->enc_prev, &f->enc_fail, &f->enc_cand, &f->enc_cand_n, in)) { f->fault = FF_ENCODER; return false; }
    float th_raw = (float)(((uint32_t)f->enc_prev * (uint32_t)p->pole_pairs) & 0x3FFFu) * (TWO_PI_F / 16384.0f);

    /* ---- speed estimate and outer loops at 2 kHz */
    f->dt_acc += (in->dt > 0.0f) ? in->dt : TS;
    bool outer = (++f->outer_cnt >= OUTER_DIV);
    if (outer) {
        f->outer_cnt = 0;
        dt_out = clampf(f->dt_acc, 0.5f * DT_OUT_NOM, 5.0f * DT_OUT_NOM);
        f->dt_acc = 0.0f;
        /* second order tracking observer on the (quantised) sensor position: smooth speed without the delay of a long difference */
        float pm = (float)f->pos_cnt * DEG_PER_CNT;
        float oe = pm - f->p_hat;
        f->p_hat += (f->v_hat + 2.0f * FOC_OBS_W * oe) * DT_OUT;
        f->v_hat += FOC_OBS_W * FOC_OBS_W * oe * DT_OUT;
        f->rpm = f->v_hat * (1.0f / 6.0f);
        if (f->pos_cnt > (1 << 22) || f->pos_cnt < -(1 << 22)) {            /* keep the float position small */
            int32_t sh = (f->pos_cnt / 16384) * 16384;
            f->pos_cnt -= sh;
            f->p_hat -= (float)sh * DEG_PER_CNT;
            f->p_sp -= (float)sh * DEG_PER_CNT;
        }
    }
    const float w_e = dir * f->rpm * (TWO_PI_F / 60.0f) * ppf;              /* electrical rad/s */

    float theta_c, theta_a;
    float we = 0.0f;
    if (f->phase == FP_ALIGN) {
        theta_c = theta_a = 0.0f;
        float rampt = 0.3f * (float)f->align_ticks;
        float k = (float)f->tick / rampt;
        f->id_ref = p->i_align * (k > 1.0f ? 1.0f : k);
        f->iq_ref = 0.0f;
        if (f->tick >= (f->align_ticks * 3u) / 4u) {                         /* average the raw electrical angle over the last quarter */
            if (f->al_n == 0u) f->al_first = th_raw;
            f->al_sum += wrap_pi(th_raw - f->al_first);
            f->al_n++;
        }
        f->theta_e = 0.0f;
    } else {
        float th = dir * wrap_pi(th_raw - f->enc_off);
        f->theta_e = th;
        we = w_e;
        theta_c = th + w_e * TS;                                             /* angle at the current sample (frame is one period old) */
        theta_a = th + w_e * 2.0f * TS;                                      /* angle in the middle of the PWM period we are about to apply */

        if (outer) {
            float icmd = 0.0f;
            const float rlim = p->rpm_max;
            if (f->mode == FM_TORQUE) {
                icmd = clampf(cmd->tq_a, -imax, imax);
                float over = fabsf(f->rpm) - rlim;                           /* no more torque into the speed limit */
                if (over > 0.0f && icmd * f->rpm > 0.0f) icmd *= clampf(1.0f - over / (0.1f * rlim), 0.0f, 1.0f);
                f->rpm_sp = 0.0f;
            } else if (f->mode == FM_SPEED) {
                float tgt = clampf(cmd->rpm, -rlim, rlim);
                float st = p->accel * DT_OUT;
                f->rpm_sp += clampf(tgt - f->rpm_sp, -st, st);
                icmd = speed_pi(f, p, f->rpm_sp, f->rpm, imax);
            } else {
                const float vmax = p->pos_vmax * 6.0f, acc = p->pos_acc * 6.0f;    /* deg/s, deg/s^2 */
                float dd = wrap180(cmd->pos_deg - f->p_sp);
                float vdes = copysignf(fminf(vmax, sqrtf(2.0f * acc * fabsf(dd))), dd);
                f->v_sp += clampf(vdes - f->v_sp, -acc * DT_OUT, acc * DT_OUT);
                f->p_sp += f->v_sp * DT_OUT;
                if (fabsf(dd) < 0.05f && fabsf(f->v_sp) < acc * DT_OUT * 1.5f) { f->p_sp += dd; f->v_sp = 0.0f; }
                float pe = f->p_sp - (float)f->pos_cnt * DEG_PER_CNT;
                if (fabsf(pe) < 30.0f) f->pos_int = clampf(f->pos_int + p->pos_ki * pe * DT_OUT, -0.3f * rlim, 0.3f * rlim);
                float ed = f->v_sp - f->rpm * 6.0f;                          /* error derivative from the profile and the speed estimate */
                f->rpm_sp = clampf(p->pos_kp * pe + f->pos_int + p->pos_kd * ed + f->v_sp / 6.0f, -rlim, rlim);
                icmd = speed_pi(f, p, f->rpm_sp, f->rpm, imax);
            }
            f->iq_ref = dir * icmd;
            f->id_ref = 0.0f;
            if (fabsf(f->rpm) > 1.5f * rlim + 50.0f) { if (++f->ovs >= 6u) { f->fault = FF_OVERSPEED; return false; } }   /* 3 ms in a row: ignores single bad samples */
            else f->ovs = 0u;
        }
    }

    /* ---- current loop (every cycle) */
    float id, iq;
    foc_clarke_park(in->ia, in->ib, in->ic, theta_c, &id, &iq);
    const float ls  = fmaxf(p->l_ll_uh * 0.5e-6f, 1.0e-6f);
    const float lam = 60.0f / (TWO_PI_F * 1.7320508f * p->kv * ppf);        /* flux linkage, V s/rad (electrical) */
    float ed = f->id_ref - id, eq = f->iq_ref - iq;
    float vd_u = e->cur_kp * ed + f->id_int - we * ls * iq;
    float vq_u = e->cur_kp * eq + f->iq_int + we * (ls * id + lam);
    float mag = sqrtf(vd_u * vd_u + vq_u * vq_u);
    float vd = vd_u, vq = vq_u;
    if (mag > vlim) {
        float s = vlim / mag;
        vd *= s; vq *= s;                                                    /* integrators hold while the voltage is limited */
    } else {
        f->id_int = clampf(f->id_int + e->cur_ki * ed * TS, -vlim, vlim);
        f->iq_int = clampf(f->iq_int + e->cur_ki * eq * TS, -vlim, vlim);
    }
    float s_a = sinf(theta_a), c_a = cosf(theta_a);
    out_ab(vd * c_a - vq * s_a, vd * s_a + vq * c_a, vbus, f->duty);

    f->id = id; f->iq = dir * iq; f->vd = vd; f->vq = dir * vq;
    f->iq_ref_t = dir * f->iq_ref;
    f->pos_deg = wrap360((float)f->pos_cnt * DEG_PER_CNT);
    f->pos_sp_deg = wrap360(f->p_sp);

    if (f->phase == FP_ALIGN) {
        if (++f->tick >= f->align_ticks) {
            f->enc_off = f->al_first + (f->al_n ? f->al_sum / (float)f->al_n : 0.0f);
            f->phase = FP_RUN;
            f->p_sp = (float)f->pos_cnt * DEG_PER_CNT;
            f->v_sp = 0.0f; f->spd_int = 0.0f; f->pos_int = 0.0f; f->rpm_sp = 0.0f;
            f->p_hat = f->p_sp; f->v_hat = 0.0f;
            f->outer_cnt = 0;
        }
    }
    return true;
}

/* ================================================================== calibration */
enum { CS_ALIGN = 0, CS_R1, CS_R2, CS_L, CS_ROT, CS_DONE };

void foc_cal_begin(foc_cal_t *c, const foc_in_t *in)
{
    memset(c, 0, sizeof(*c));
    c->enc_prev = (uint16_t)(in->enc & 0x3FFFu);
    c->pos_cnt = c->enc_prev;
    c->duty[0] = c->duty[1] = c->duty[2] = 0.5f;
}

static int cal_fail(foc_cal_t *c, const char *why)
{
    c->err = why;
    c->duty[0] = c->duty[1] = c->duty[2] = 0.5f;
    return -1;
}

int foc_cal_step(foc_cal_t *c, const foc_in_t *in, const esc_params_t *p)
{
    const float vbus = (in->vbus < 5.0f) ? 5.0f : in->vbus;
    const float vlim = 0.8f * v_limit(vbus);
    const float ical = p->i_cal;
    if (!track_enc(&c->pos_cnt, &c->enc_prev, &c->enc_fail, &c->enc_cand, &c->enc_cand_n, in)) return cal_fail(c, "encoder");

    float ial = (2.0f * in->ia - in->ib - in->ic) * (1.0f / 3.0f);
    float l_est = fmaxf(p->l_ll_uh * 0.5e-6f, 5.0e-6f);
    float g = 0.04f * l_est / TS;                                            /* integrator gain, V per A and cycle: loop gain ~0.04 */

    switch (c->step) {
    case CS_ALIGN:
    case CS_R1:
    case CS_R2:
    case CS_L: {
        float iref;
        if (c->step == CS_ALIGN)      iref = ical * clampf((float)c->tick / (float)MS(300), 0.0f, 1.0f);
        else if (c->step == CS_R2)    iref = ical;
        else                          iref = 0.5f * ical;
        float gg = (c->step == CS_L) ? 0.25f * g : g;
        c->v_a = clampf(c->v_a + gg * (iref - ial), -vlim, vlim);
        float vh = 0.0f, s = 0.0f;
        const uint32_t m = c->tick;                                          /* index of this cycle */
        if (c->step == CS_L) {
            vh = clampf(0.4f * l_est / TS, 0.03f, 0.15f * vbus);
            s = (((m >> 1) & 1u) ? -1.0f : 1.0f);
        }
        out_ab(c->v_a + s * vh, 0.0f, vbus, c->duty);
        float va_applied = valpha_applied(c->duty, vbus);
        c->tick++;
        c->i_fl += (ial - c->i_fl) * (1.0f / 32.0f);

        if (c->step == CS_ALIGN) {
            if (c->tick >= MS(600)) { c->step = CS_R1; c->tick = 0; c->acc_n = 0; c->acc_v = c->acc_i = 0.0f; c->n_p = c->n_m = 0; }
        } else if (c->step == CS_R1 || c->step == CS_R2) {
            float tol = 0.05f * iref;
            if (c->acc_n == 0u) {
                /* settle: current within 5 % of the target for 30 ms */
                if (fabsf(iref - c->i_fl) < tol) { if (++c->n_p >= MS(30)) { c->acc_n = 1; c->n_p = 0; } } else c->n_p = 0;
                if (c->tick > MS(2500)) return cal_fail(c, "no_current");
            } else {
                c->acc_v += va_applied; c->acc_i += ial;
                if (++c->acc_n > MS(100)) {
                    float n = (float)(c->acc_n - 1u);
                    float v = c->acc_v / n, i = c->acc_i / n;
                    if (c->step == CS_R1) {
                        c->v1 = v; c->i1 = i; c->step = CS_R2;
                    } else {
                        float di = i - c->i1;
                        if (di < 0.2f * ical) return cal_fail(c, "no_current");
                        c->r_phase_ohm = (v - c->v1) / di;
                        if (!(c->r_phase_ohm > 0.0002f && c->r_phase_ohm < 20.0f)) return cal_fail(c, "bad_resistance");
                        c->step = CS_L;
                    }
                    c->tick = 0; c->acc_n = 0; c->acc_v = c->acc_i = 0.0f; c->n_p = 0; c->n_m = 0;
                    c->l_num = c->l_den = c->vh_p = c->vh_m = 0.0f;
                    c->i_prev = ial;
                }
            }
        } else {   /* CS_L: dither +,+,-,- on top of a DC current, correlate the current steps with the pattern */
            if (m >= MS(100) && m < MS(100) + MS(200)) {
                /* the step between two samples is driven by the two PWM periods before this sample: patterns of cycles m-1 and m-2 */
                float s1 = (((m - 1u) >> 1) & 1u) ? -1.0f : 1.0f;
                float s2 = (((m - 2u) >> 1) & 1u) ? -1.0f : 1.0f;
                float w = 0.5f * (s1 + s2);
                float dI = ial - c->i_prev;
                c->l_num += w * dI;
                c->l_den += w * w;
                if (s > 0.0f) { c->vh_p += va_applied; c->n_p++; } else { c->vh_m += va_applied; c->n_m++; }
            }
            c->i_prev = ial;
            if (c->tick >= MS(100) + MS(200)) {
                if (c->l_den <= 0.0f || c->n_p == 0u || c->n_m == 0u) return cal_fail(c, "no_data");
                float vh_eff = 0.5f * (c->vh_p / (float)c->n_p - c->vh_m / (float)c->n_m);
                float slope = c->l_num / c->l_den;                        /* A per cycle = Ts * Vh / L */
                if (!(slope > 1.0e-5f)) return cal_fail(c, "bad_inductance");
                c->l_phase_h = TS * vh_eff / slope;
                if (!(c->l_phase_h > 1.0e-7f && c->l_phase_h < 0.05f)) return cal_fail(c, "bad_inductance");
                c->step = CS_ROT; c->tick = 0; c->th_cmd = 0.0f;
                c->id_int = c->iq_int = 0.0f;
            }
        }
        return 0;
    }

    case CS_ROT: {
        /* rotate the current vector 2 electrical turns slowly; the rotor follows, the sensor tells direction and pole pairs */
        const uint32_t t_hold0 = MS(500), t_ramp = MS(4000), t_hold1 = MS(700);
        float rate = 4.0f * PI_F / ((float)t_ramp * TS);               /* rad/s: 2 turns in 4 s */
        if (c->tick >= t_hold0 && c->tick < t_hold0 + t_ramp) c->th_cmd += rate * TS;
        if (c->tick == t_hold0) c->pos0 = c->pos_cnt;                    /* rotor was aligned at theta = 0 */
        float k = clampf((float)c->tick / (float)MS(300), 0.0f, 1.0f);
        float al = TWO_PI_F * 300.0f;
        float rs = c->r_phase_ohm, ls = c->l_phase_h;
        float kp = al * ls, ki = al * rs;
        float id, iq;
        foc_clarke_park(in->ia, in->ib, in->ic, c->th_cmd, &id, &iq);
        float ed = ical * k - id, eq = -iq;
        float vd = kp * ed + c->id_int, vq = kp * eq + c->iq_int;
        float mag = sqrtf(vd * vd + vq * vq);
        if (mag > vlim) { vd *= vlim / mag; vq *= vlim / mag; }
        else { c->id_int += ki * ed * TS; c->iq_int += ki * eq * TS; }
        float sa = sinf(c->th_cmd), ca = cosf(c->th_cmd);
        out_ab(vd * ca - vq * sa, vd * sa + vq * ca, vbus, c->duty);
        c->tick++;
        if (c->tick >= t_hold0 + t_ramp + t_hold1) {
            float dd = (float)(c->pos_cnt - c->pos0) * DEG_PER_CNT;
            if (fabsf(dd) < 3.0f) return cal_fail(c, "no_motion");
            c->enc_dir = (dd > 0.0f) ? 1 : -1;
            c->pp_meas = 2.0f / (fabsf(dd) / 360.0f);
            c->step = CS_DONE;
            c->duty[0] = c->duty[1] = c->duty[2] = 0.5f;
            return 1;
        }
        return 0;
    }
    default:
        return 1;
    }
}

/* ================================================================== speed / position gain auto-tune */
enum { TU_WAIT = 0, TU_ACC, TU_DEC, TU_DONE };

#define TU_TIMEOUT_S  4.0f
#define TU_RPM_CAP    1500.0f

void foc_tune_begin(foc_tune_t *t, const esc_params_t *p, float vbus)
{
    memset(t, 0, sizeof(*t));
    float imax = fminf(p->i_max, 0.9f * p->ilim);
    t->i_tq = 0.5f * imax;
    t->kt = 8.27f / fmaxf(p->kv, 1.0f);
    t->sgn = (p->dir < 0) ? -1.0f : 1.0f;
    /* half the speed limit, at most 1500 rpm, and the back-EMF at most 40 % of the bus voltage */
    t->rpm_tgt = fminf(fminf(TU_RPM_CAP, 0.5f * p->rpm_max), 0.4f * p->kv * vbus);
    if (t->rpm_tgt < 100.0f) t->err = "rpm_range";
    else if (t->i_tq < 0.05f) t->err = "current_range";
}

static void tune_fit_reset(foc_tune_t *t) { t->sx = t->sy = t->sxx = t->sxy = 0.0f; t->n = 0; t->t = 0.0f; }

static void tune_fit_add(foc_tune_t *t, float x, float y)
{
    t->sx += x; t->sy += y; t->sxx += x * x; t->sxy += x * y; t->n++;
}

static bool tune_fit_slope(const foc_tune_t *t, float *a)
{
    float n = (float)t->n;
    float den = n * t->sxx - t->sx * t->sx;
    if (t->n < 20u || den <= 0.0f) return false;
    *a = (n * t->sxy - t->sx * t->sy) / den;
    return true;
}

static int tune_fail(foc_tune_t *t, const char *why) { t->err = why; t->tq = 0.0f; t->step = TU_DONE; return -1; }

int foc_tune_step(foc_tune_t *t, const foc_t *f, float dt)
{
    if (t->err) return tune_fail(t, t->err);
    if (t->step == TU_DONE) { t->tq = 0.0f; return 1; }
    if (f->phase != FP_RUN) { t->tq = 0.0f; return 0; }               /* alignment of the normal FOC start */
    t->t += (dt > 0.0f) ? dt : TS;
    bool outer = (f->outer_cnt == 0u);                                /* the speed estimate was just updated (2 kHz) */
    float v = t->sgn * f->rpm;                                         /* speed in the test direction */
    float lo = 0.2f * t->rpm_tgt, hi = 0.8f * t->rpm_tgt;

    switch (t->step) {
    case TU_WAIT:
        tune_fit_reset(t);
        t->step = TU_ACC;
        /* fall through */
    case TU_ACC:
        t->tq = t->sgn * t->i_tq;
        if (outer && v > lo && v < hi) tune_fit_add(t, t->t, v);
        if (v < -0.2f * t->rpm_tgt) return tune_fail(t, "wrong_direction");
        if (v >= t->rpm_tgt) {
            if (!tune_fit_slope(t, &t->a1)) return tune_fail(t, "fit_accel");
            tune_fit_reset(t);
            t->step = TU_DEC;
        } else if (t->t > TU_TIMEOUT_S) return tune_fail(t, "no_accel");
        return 0;
    case TU_DEC:
        t->tq = -t->sgn * t->i_tq;
        if (outer && v > lo && v < hi) tune_fit_add(t, t->t, v);
        if (v <= 0.1f * t->rpm_tgt) {
            t->tq = 0.0f;
            t->step = TU_DONE;
            if (!tune_fit_slope(t, &t->a2)) return tune_fail(t, "fit_decel");
            float da = (t->a1 - t->a2) * (TWO_PI_F / 60.0f);            /* rad/s^2 */
            if (!(t->a1 > 0.0f && t->a2 < 0.0f && da > 0.0f)) return tune_fail(t, "bad_fit");
            t->j_kgm2 = 2.0f * t->kt * t->i_tq / da;
            t->tf_nm  = -t->j_kgm2 * 0.5f * (t->a1 + t->a2) * (TWO_PI_F / 60.0f);
            if (!(t->j_kgm2 > 1.0e-8f && t->j_kgm2 < 1.0f)) return tune_fail(t, "inertia_range");
            return 1;
        }
        if (t->t > TU_TIMEOUT_S) return tune_fail(t, "no_decel");
        return 0;
    default:
        t->tq = 0.0f;
        return 1;
    }
}

void foc_tune_gains(const foc_tune_t *t, float bw_hz, float *spd_kp, float *spd_ki, float *pos_kp)
{
    float w = TWO_PI_F * bw_hz;
    float kp = t->j_kgm2 * w / t->kt * (TWO_PI_F / 60.0f);             /* A per rpm */
    *spd_kp = kp;
    *spd_ki = kp * w * 0.25f;                                          /* PI zero at a quarter of the crossover */
    *pos_kp = w / 24.0f;                                               /* position bandwidth w/4; 1 rpm = 6 deg/s */
}
