/* Host simulation of the FOC code against a PMSM model. Build and run with tests/run_foc_sim.sh (needs only gcc).
 * Not part of the firmware. */
#include "esc_foc.h"
#include "esc_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TS (1.0 / PWM_FREQ_HZ)
#define NSUB 10

typedef struct {
    double R, L, lam, J, B, tload;   /* phase R, Ls, flux linkage, inertia, viscous, load torque */
    int pp;
    double id, iq, w_m, th_m;        /* rotor frame currents, mech speed rad/s, mech angle rad */
    double vbus;
    int sensor_dir;                  /* +1: sensor counts with electrical angle, -1: against it */
    double dead_v;                   /* dead-time voltage error magnitude */
    double enc0;                     /* sensor zero offset, rad (mech) */
    double noise_a;                  /* current noise, A rms */
} plant_t;

static double frand(void) { return (rand() / (double)RAND_MAX) * 2.0 - 1.0; }

/* integrate the plant for one PWM period with the voltage (alpha,beta) applied; pend = the duty applied in this period */
static void plant_step(plant_t *m, const float duty[3], double dt)
{
    double vb = m->vbus;
    double d[3] = { duty[0], duty[1], duty[2] };
    double v[3];
    double dtn = dt / NSUB;
    for (int k = 0; k < NSUB; k++) {
        double th_e = m->th_m * m->pp * m->sensor_dir;     /* electrical angle (sensor_dir folds the wiring sense in) */
        /* phase currents from dq to decide the sign for the dead-time error */
        double ce = cos(th_e), se = sin(th_e);
        double ial = m->id * ce - m->iq * se, ibe = m->id * se + m->iq * ce;
        double ia = ial, ib = -0.5 * ial + 0.8660254 * ibe, ic = -0.5 * ial - 0.8660254 * ibe;
        double cur[3] = { ia, ib, ic };
        for (int p = 0; p < 3; p++) {
            v[p] = (d[p] - 0.5) * vb;
            double s = (cur[p] > 0.02) ? 1.0 : ((cur[p] < -0.02) ? -1.0 : cur[p] / 0.02);
            v[p] -= m->dead_v * s;                         /* dead time pulls the phase voltage towards zero current */
        }
        double mean = (v[0] + v[1] + v[2]) / 3.0;
        double va = (2.0 / 3.0) * (v[0] - 0.5 * (v[1] + v[2]));
        double vbe = (v[1] - v[2]) / 1.7320508;
        (void)mean;
        double vd =  va * ce + vbe * se;
        double vq = -va * se + vbe * ce;
        double w_e = m->w_m * m->pp * m->sensor_dir;
        double did = (vd - m->R * m->id + w_e * m->L * m->iq) / m->L;
        double diq = (vq - m->R * m->iq - w_e * m->L * m->id - w_e * m->lam) / m->L;
        m->id += did * dtn;
        m->iq += diq * dtn;
        double torque = 1.5 * m->pp * m->lam * m->iq * m->sensor_dir;   /* torque about the sensor-positive direction */
        double acc = (torque - m->B * m->w_m - m->tload) / m->J;
        m->w_m += acc * dtn;
        m->th_m += m->w_m * dtn;
    }
}

static void phase_currents(const plant_t *m, double *ia, double *ib, double *ic)
{
    double th_e = m->th_m * m->pp * m->sensor_dir;
    double ce = cos(th_e), se = sin(th_e);
    double al = m->id * ce - m->iq * se, be = m->id * se + m->iq * ce;
    *ia = al; *ib = -0.5 * al + 0.8660254 * be; *ic = -0.5 * al - 0.8660254 * be;
}

static uint16_t encoder(const plant_t *m)
{
    double t = m->th_m + m->enc0;
    t = fmod(t, 2.0 * M_PI); if (t < 0) t += 2.0 * M_PI;
    return (uint16_t)((int)(t / (2.0 * M_PI) * 16384.0) & 0x3FFF);
}

static plant_t make_plant(void)
{
    plant_t m;
    memset(&m, 0, sizeof m);
    m.R = 0.024; m.L = 9.9e-6; m.pp = 20;
    m.lam = 60.0 / (2.0 * M_PI * sqrt(3.0) * 100.0 * m.pp);   /* Kv = 100 */
    m.J = 2.0e-4; m.B = 1.0e-4; m.tload = 0.0;
    m.vbus = 16.0; m.sensor_dir = 1; m.dead_v = 0.15; m.enc0 = 0.7; m.noise_a = 0.03;
    return m;
}

static esc_params_t make_params(void)
{
    esc_params_t p;
    params_defaults(&p);
    p.enc_dir = 1;
    p.align_ms = 600;
    return p;
}

/* pipeline model of the real firmware: encoder frame is one cycle old, duty applies half a cycle later (we apply it
 * for the whole next period, which is the same thing for the plant), current sampled at the start of the cycle */
static double g_glitch = 0.0;
typedef struct { uint16_t enc_pipe; float duty[3]; } link_t;

static void sample(const plant_t *m, foc_in_t *in, link_t *lk)
{
    double ia, ib, ic;
    phase_currents(m, &ia, &ib, &ic);
    in->ia = (float)(ia + m->noise_a * frand());
    in->ib = (float)(ib + m->noise_a * frand());
    in->ic = (float)(ic + m->noise_a * frand());
    in->enc = lk->enc_pipe;                  /* value latched one cycle ago */
    if (g_glitch > 0.0 && (rand() / (double)RAND_MAX) < g_glitch) in->enc = (uint16_t)(rand() & 0x3FFF);   /* corrupted frame that still passes parity */
    in->enc_ok = true;
    in->vbus = (float)m->vbus;
    lk->enc_pipe = encoder(m);
}

static double wrapdeg(double x) { while (x > 180.0) x -= 360.0; while (x < -180.0) x += 360.0; return x; }
static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } else { printf("  ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void params_gains(const esc_params_t *p, esc_eff_t *e)
{
    esc_eff_t t = { 0, 0, 0.03f, 0, 0, 0 };
    *e = t;
    params_derive(p, 16.0f, e);
}

/* run FOC for a time; returns the final state. cmd may be changed by the callback. */
typedef void (*cb_t)(double t, foc_cmd_t *cmd, void *u);

static int run(plant_t *m, esc_params_t *p, foc_mode_t mode, double t_end, cb_t cb, void *u, foc_t *f, FILE *log, double *rec_t, double *rec_a, double *rec_b, int nrec)
{
    esc_eff_t e;
    params_gains(p, &e);
    link_t lk; memset(&lk, 0, sizeof lk);
    lk.enc_pipe = encoder(m);
    foc_in_t in; foc_cmd_t cmd; memset(&cmd, 0, sizeof cmd);
    sample(m, &in, &lk);
    foc_begin(f, mode, p, &in);
    float pend[3] = {0.5f, 0.5f, 0.5f};
    int ok = 1;
    long n = (long)(t_end / TS);
    for (long i = 0; i < n; i++) {
        double t = i * TS;
        if (cb) cb(t, &cmd, u);
        plant_step(m, pend, TS);                       /* duty computed in the previous cycle acts during this one */
        sample(m, &in, &lk);
        if (!foc_step(f, &in, &cmd, p, &e)) { printf("  fault %d at t=%.3f\n", (int)f->fault, t); ok = 0; break; }
        memcpy(pend, f->duty, sizeof pend);
        if (log && (i % 200) == 0) fprintf(log, "%.4f %.3f %.3f %.3f %.3f %.3f %.3f\n", t, f->id, f->iq, f->rpm, m->w_m * 60.0 / (2 * M_PI), f->pos_deg, f->pos_sp_deg);
        if (rec_t && (i % 200) == 0 && (i / 200) < nrec) { rec_t[i / 200] = t; rec_a[i / 200] = f->iq; rec_b[i / 200] = m->w_m * 60.0 / (2 * M_PI); }
    }
    return ok;
}

/* ------------------------------------------------------------------ tests */
static void cb_tq(double t, foc_cmd_t *c, void *u) { (void)u; c->tq_a = (t > 1.0) ? 0.3f : 0.0f; }

static void test_align_and_torque(int sdir)
{
    printf("align + torque step (sensor_dir %+d)\n", sdir);
    plant_t m = make_plant(); m.sensor_dir = sdir;
    esc_params_t p = make_params(); p.enc_dir = (int8_t)sdir; p.rpm_max = 1500;
    foc_t f;
    FILE *lg = getenv("FOC_LOGT") ? fopen(getenv("FOC_LOGT"), "a") : NULL;
    double tt[140], ra[140], rb[140];
    int ok = run(&m, &p, FM_TORQUE, 1.4, cb_tq, NULL, &f, lg, tt, ra, rb, 140);
    CHECK(ok, "no fault");
    CHECK(f.phase == FP_RUN, "alignment finished");
    double w_rpm = m.w_m * 60.0 / (2 * M_PI);
    printf("     rpm %.1f, iq %.3f (ref %.3f), id %.3f\n", w_rpm, f.iq, f.iq_ref_t, f.id);
    double iqm = 0; for (int k = 105; k < 135; k++) iqm += ra[k] / 30.0;
    CHECK(fabs(iqm - 0.3) < 0.04, "mean iq tracks 0.3 A (%.3f)", iqm);
    CHECK(fabs(f.id) < 0.15, "id stays near 0 (%.3f)", f.id);
    CHECK((sdir > 0 ? w_rpm : w_rpm) > 100.0, "positive torque accelerates towards rising sensor angle (%.1f rpm)", w_rpm);
    /* expected acceleration: T = 1.5 pp lam iq */
    double T = 1.5 * m.pp * m.lam * 0.3;
    double a_exp = T / m.J * 60.0 / (2 * M_PI);
    printf("     torque %.4f Nm, expected accel %.0f rpm/s\n", T, a_exp);
}

static void cb_spd(double t, foc_cmd_t *c, void *u) { (void)u; c->rpm = (t < 3.0) ? 300.0f : -200.0f; }

static void test_speed(void)
{
    printf("speed loop: 0 -> 300 rpm -> -200 rpm\n");
    plant_t m = make_plant(); m.tload = 0.002;
    esc_params_t p = make_params(); p.accel = 500; p.rpm_max = 1500;
    foc_t f;
    double tt[100], a[100], b[100];
    int ok = run(&m, &p, FM_SPEED, 6.0, cb_spd, NULL, &f, NULL, tt, a, b, 100);
    CHECK(ok, "no fault");
    /* sample speed at t=2.9 s (index 2.9/0.01=290 -> > 100 entries?); each rec is 200 cycles = 10 ms, nrec 100 -> 1 s only */
    (void)tt; (void)a; (void)b;
    double w = m.w_m * 60.0 / (2 * M_PI);
    printf("     final rpm %.1f (target -200), est %.1f\n", w, f.rpm);
    CHECK(fabs(w + 200.0) < 5.0, "reaches -200 rpm");
    CHECK(fabs(f.rpm - w) < 3.0, "speed estimate matches (%.1f vs %.1f)", f.rpm, w);
}


static void test_calibration(int sdir)
{
    printf("calibration (sensor_dir %+d)\n", sdir);
    plant_t m = make_plant(); m.sensor_dir = sdir;
    esc_params_t p = make_params(); p.enc_dir = 0; p.pole_pairs = 20;
    link_t lk; memset(&lk, 0, sizeof lk);
    lk.enc_pipe = encoder(&m);
    foc_in_t in; sample(&m, &in, &lk);
    foc_cal_t c; memset(&c, 0, sizeof c);
    foc_cal_begin(&c, &in);
    float pend[3] = {0.5f, 0.5f, 0.5f};
    int rc = 0;
    long n = (long)(12.0 / TS);
    for (long i = 0; i < n && rc == 0; i++) {
        plant_step(&m, pend, TS);
        sample(&m, &in, &lk);
        rc = foc_cal_step(&c, &in, &p);
        memcpy(pend, c.duty, sizeof pend);
    }
    CHECK(rc == 1, "calibration finished (%s)", c.err ? c.err : "ok");
    if (rc != 1) return;
    printf("     R_phase %.4f ohm (true %.4f), L_phase %.2f uH (true %.2f), enc_dir %d, pp %.2f\n",
           c.r_phase_ohm, m.R, c.l_phase_h * 1e6, m.L * 1e6, c.enc_dir, c.pp_meas);
    CHECK(fabs(c.r_phase_ohm / m.R - 1.0) < 0.10, "resistance within 10%%");
    CHECK(fabs(c.l_phase_h / m.L - 1.0) < 0.25, "inductance within 25%%");
    CHECK(c.enc_dir == sdir, "encoder direction");
    CHECK(fabs(c.pp_meas - m.pp) < 0.5, "pole pairs");
}

static void cb_ext(double t, foc_cmd_t *c, void *u) { (void)c; if (t > 1.0) ((plant_t *)u)->w_m = 40.0; }   /* shaft forced by something else */

static void test_overspeed(void)
{
    printf("overspeed fault (shaft driven externally at ~380 rpm, limit 100 rpm)\n");
    plant_t m = make_plant();
    esc_params_t p = make_params(); p.enc_dir = 1; p.rpm_max = 100;
    foc_t f;
    int ok = run(&m, &p, FM_TORQUE, 1.5, cb_ext, &m, &f, NULL, NULL, NULL, NULL, 0);
    CHECK(!ok && f.fault == FF_OVERSPEED, "overspeed fault raised (ok=%d fault=%d)", ok, (int)f.fault);
}

static foc_t *g_f; static double g_peak, g_vh;
static void cb_hand(double t, foc_cmd_t *c, void *u)
{
    plant_t *m = u; c->tq_a = 0; c->pos_deg = 40.1f;
    if (t > 1.0 && t < 1.5) m->w_m = g_vh * 2 * M_PI / 60.0;      /* hand turns the shaft */
    if (t > 1.0 && g_f) { double a = sqrt(g_f->id * g_f->id + g_f->iq * g_f->iq); if (a > g_peak) g_peak = a; }
}
static double g_lscale = 1.0, g_dead = 0.15;
static void test_hand(int mode, int sdir, int wrongdir, double rpm)
{
    plant_t m = make_plant(); m.sensor_dir = sdir; m.dead_v = g_dead;
    esc_params_t p = make_params(); p.l_ll_uh *= (float)g_lscale; p.cur_kp = 0; p.enc_dir = (int8_t)(wrongdir ? -sdir : sdir); p.rpm_max = 1500;
    foc_t f; g_f = &f; g_peak = 0; g_vh = rpm;
    int ok = run(&m, &p, (foc_mode_t)mode, 2.0, cb_hand, &m, &f, NULL, NULL, NULL, NULL, 0);
    printf("hand test mode %d sdir %+d wrongdir %d rpm %.0f: ok=%d peak |I| %.2f A\n", mode, sdir, wrongdir, rpm, ok, g_peak);
    g_f = NULL;
}

typedef struct { double t_step; float tgt1, tgt2; } pcb_t;
static void cb_pos(double t, foc_cmd_t *c, void *u) { pcb_t *x = u; c->pos_deg = (t < x->t_step) ? x->tgt1 : x->tgt2; }

static void test_position(int sdir)
{
    printf("position loop (sensor_dir %+d): move to 90 deg, then to 350 deg (shortest way through 0)\n", sdir);
    plant_t m = make_plant(); m.sensor_dir = sdir; m.tload = 0.0;
    esc_params_t p = make_params(); p.enc_dir = (int8_t)sdir; p.pos_kp = 3; p.pos_vmax = 100; p.pos_acc = 500;
    foc_t f;
    pcb_t x = { 4.0, 90.0f, 350.0f };
    FILE *lg = getenv("FOC_LOG") ? fopen(getenv("FOC_LOG"), "w") : NULL;
    int ok = run(&m, &p, FM_POSITION, 3.9, cb_pos, &x, &f, lg, NULL, NULL, NULL, 0);
    if (lg) fclose(lg);
    double pos = fmod(m.th_m * 180.0 / M_PI + m.enc0 * 180.0 / M_PI + 720.0, 360.0);
    printf("     pos after first move %.2f deg (target 90)\n", pos);
    CHECK(ok, "no fault");
    CHECK(fabs(pos - 90.0) < 0.5, "settles at 90 deg (error %.3f)", pos - 90.0);
    esc_eff_t e; params_gains(&p, &e);
    /* continue from the current state with the second target: re-run is not possible, run both targets in one go instead */
    plant_t m2 = make_plant(); m2.sensor_dir = sdir;
    foc_t f2;
    pcb_t y = { 0.0, 350.0f, 350.0f };
    ok = run(&m2, &p, FM_POSITION, 3.5, cb_pos, &y, &f2, NULL, NULL, NULL, NULL, 0);
    double pos2 = fmod(m2.th_m * 180.0 / M_PI + m2.enc0 * 180.0 / M_PI + 720.0, 360.0);
    printf("     pos %.2f deg (target 350), start was %.1f deg\n", pos2, m2.enc0 * 180.0 / M_PI);
    CHECK(ok && fabs(wrapdeg(pos2 - 350.0)) < 0.5, "settles at 350 deg");
}


/* sweep for sensible default gains: position step of 60 deg, report overshoot and the wobble in the last second */
static void tune_case(double J, float pkp, float skp, float ski)
{
    plant_t m = make_plant(); m.J = J; m.noise_a = 0.03;
    esc_params_t p = make_params(); p.pos_kp = pkp; p.spd_kp = skp; p.spd_ki = ski; p.pos_vmax = 100; p.pos_acc = 500;
    esc_eff_t e; params_gains(&p, &e);
    link_t lk; memset(&lk, 0, sizeof lk); lk.enc_pipe = encoder(&m);
    foc_in_t in; foc_cmd_t cmd; memset(&cmd, 0, sizeof cmd);
    sample(&m, &in, &lk);
    foc_t f; foc_begin(&f, FM_POSITION, &p, &in);
    float pend[3] = {0.5f, 0.5f, 0.5f};
    double start = 0, maxos = 0, wob = 0; int nw = 0;
    long n = (long)(4.0 / TS);
    for (long i = 0; i < n; i++) {
        double t = i * TS;
        plant_step(&m, pend, TS); sample(&m, &in, &lk);
        if (t > 0.7 && start == 0) start = (m.th_m + m.enc0) * 180.0 / M_PI;
        cmd.pos_deg = (float)(fmod(start + 60.0 + 720.0, 360.0) * (start != 0));
        if (!foc_step(&f, &in, &cmd, &p, &e)) { printf("  J=%.0e pkp=%.1f skp=%.3f ski=%.2f  FAULT\n", J, pkp, skp, ski); return; }
        memcpy(pend, f.duty, sizeof pend);
        if (start != 0) {
            double pos = (m.th_m + m.enc0) * 180.0 / M_PI;
            double os = pos - (start + 60.0);
            if (os > maxos) maxos = os;
            if (t > 3.0) { wob += os * os; nw++; }
        }
    }
    printf("  J=%.0e pkp=%.1f skp=%.3f ski=%.2f  overshoot %.2f deg, wobble rms %.3f deg\n", J, pkp, skp, ski, maxos, sqrt(wob / (nw ? nw : 1)));
}

static void tune_sweep(void)
{
    double Js[] = { 5e-5, 2e-4, 1e-3 };
    float pk[] = { 0.5f, 1.0f, 2.0f };
    float sk[] = { 0.01f, 0.02f, 0.05f };
    float si[] = { 0.05f, 0.1f, 0.3f };
    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) for (int c = 0; c < 3; c++) for (int d = 0; d < 3; d++)
        if (si[d] / sk[c] < 12) tune_case(Js[a], pk[b], sk[c], si[d]);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "tune")) { srand(1); tune_sweep(); return 0; }
    srand(1);
    if (argc > 1 && !strcmp(argv[1], "hand")) { double ls[3] = {0.3, 1, 3}, dd[2] = {0.15, 0.4}; for (int a = 0; a < 3; a++) for (int b = 0; b < 2; b++) { g_lscale = ls[a]; g_dead = dd[b]; printf("Lscale %.1f dead %.2f: ", ls[a], dd[b]); test_hand(3, 1, 0, 30); } return 0; }
    if (getenv("GLITCH")) g_glitch = atof(getenv("GLITCH"));
    printf("FOC simulation\n");
    test_align_and_torque(1);
    test_align_and_torque(-1);
    test_speed();
    test_calibration(1);
    test_calibration(-1);
    test_overspeed();
    test_position(1);
    test_position(-1);
    printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "ALL OK", fails);
    return fails ? 1 : 0;
}
