#include "esc_can_odrive.h"
#include "esc_config.h"

#if ESC_CAN_ENABLE

#include "esc_app.h"
#include "esc_motor.h"
#include "esc_params.h"
#include "esc_hw.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <string.h>

enum {
    CMD_GET_VERSION = 0x00, CMD_HEARTBEAT = 0x01, CMD_ESTOP = 0x02, CMD_GET_ERROR = 0x03, CMD_SET_NODE_ID = 0x06,
    CMD_SET_AXIS_STATE = 0x07, CMD_ENCODER_EST = 0x09, CMD_SET_CTRL_MODE = 0x0B, CMD_SET_INPUT_POS = 0x0C,
    CMD_SET_INPUT_VEL = 0x0D, CMD_SET_INPUT_TORQUE = 0x0E, CMD_SET_LIMITS = 0x0F, CMD_SET_TRAJ_VEL = 0x11,
    CMD_SET_TRAJ_ACCEL = 0x12, CMD_SET_TRAJ_INERTIA = 0x13, CMD_GET_IQ = 0x14, CMD_GET_TEMP = 0x15, CMD_REBOOT = 0x16,
    CMD_GET_BUS = 0x17, CMD_CLEAR_ERRORS = 0x18, CMD_SET_ABS_POS = 0x19, CMD_SET_POS_GAIN = 0x1A, CMD_SET_VEL_GAINS = 0x1B,
    CMD_GET_TORQUES = 0x1C, CMD_GET_POWERS = 0x1D
};

/* ODrive control modes: 1 torque, 2 velocity, 3 position (0 voltage: not supported). Default like ODrive: position, passthrough. */
static uint32_t ctrl_mode = 3u, input_mode = 1u;
static float    vel_sp_turns;                     /* last Set_Input_Vel, turns/s */
static uint32_t err_latched;                      /* calibration error (cleared by Clear_Errors) */
static uint32_t ignored;                          /* set points ignored because USB owns the motor */
static const char *last_refusal = "";
static bool     braking;
static uint32_t brake_t0;
static bool     cal_requested;

static float rd_f32(const uint8_t *p) { float v; memcpy(&v, p, 4); return v; }
static void  wr_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }
static void  wr_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void  wr_i16(uint8_t *p, float v, float scale)
{
    float x = v * scale;
    int16_t i = (x > 32767.0f) ? 32767 : (x < -32768.0f ? -32768 : (int16_t)lrintf(x));
    memcpy(p, &i, 2);
}
static void  wr_u16(uint8_t *p, uint32_t v) { uint16_t u = (v > 65535u) ? 65535u : (uint16_t)v; memcpy(p, &u, 2); }
static uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

static float kt_nm_per_a(void)
{
    return (g_par.kt_nm_a > 0.0f) ? g_par.kt_nm_a : 8.27f / fmaxf(g_par.kv, 1.0f);
}

/* CAN may set points only while it owns the motor, or while the motor is stopped (stored for the next start) */
static bool can_owns(void) { return motor_owner() == SRC_CAN; }
static bool usb_owns(void) { return motor_owner() == SRC_USB; }
static bool motor_running_any(void) { return motor_owner() != SRC_NONE || g_state == ST_PRECHARGE || g_state == ST_ALIGN || g_state == ST_RUN || g_state == ST_CALIB; }

static void set_param(const char *n, float v)
{
    if (params_set(&g_par, n, v)) params_derive(&g_par, g_vbus, &g_eff);
}

static void start_closed_loop(bool open_loop)
{
    if (motor_owner() != SRC_NONE) { if (usb_owns()) ignored++; return; }       /* already running (or USB has it) */
    if (open_loop) g_mode = MODE_OPEN;
    else if (ctrl_mode >= 1u && ctrl_mode <= 3u) g_mode = (esc_mode_t)ctrl_mode;
    else { last_refusal = "control_mode"; return; }
    g_tq_cmd = 0.0f;
    const char *why = app_start(SRC_CAN);
    if (why) { last_refusal = why; return; }
    last_refusal = "";
    g_vel_ext_rpm = vel_sp_turns * 60.0f;
    g_vel_ext = true;
}

/* ------------------------------------------------------------------ telemetry (ODrive frames + extra 29-bit frames) */
static uint8_t proc_result(void)
{
    const char *err;
    int s = app_cal_state(&err);
    if (s == 1) return 1u;                                          /* BUSY */
    if (s != 3) return 0u;                                          /* SUCCESS */
    if (!strcmp(err, "bad_resistance")) return 6u;                 /* PHASE_RESISTANCE_OUT_OF_RANGE */
    if (!strcmp(err, "bad_inductance")) return 7u;                 /* PHASE_INDUCTANCE_OUT_OF_RANGE */
    if (!strcmp(err, "pole_pairs"))     return 5u;                 /* POLE_PAIR_CPR_MISMATCH */
    if (!strcmp(err, "aborted"))        return 2u;                 /* CANCELLED */
    if (!strcmp(err, "fault"))          return 3u;                 /* DISARMED */
    return 15u;                                                    /* NOT_CONVERGING (other calibration failures) */
}

static float elec_power(void) { return 1.5f * (g_vd * g_id + g_vq * g_iq); }

/* fills the data of ODrive message 'cmd'; false: not a telemetry message */
static bool fill(uint32_t cmd, can_frame_t *f)
{
    uint8_t *d = f->data;
    memset(d, 0, 8);
    f->len = 8;
    float kt = kt_nm_per_a();
    switch (cmd) {
    case CMD_GET_VERSION:
        d[0] = 2; d[1] = 1; d[2] = 0; d[3] = 0; d[4] = 2; d[5] = 0; d[6] = 0; d[7] = 0;     /* protocol 2, hw 1.0, fw 2.0 */
        return true;
    case CMD_HEARTBEAT: {
        bool spv;
        float sp = motor_pos_sp_turns(&spv) * 360.0f;
        uint8_t done = (!spv || fabsf(sp - g_pos_cmd) < 0.05f) ? 1u : 0u;
        wr_u32(d, odc_axis_error()); d[4] = odc_axis_state(); d[5] = proc_result(); d[6] = done;
        return true;
    }
    case CMD_GET_ERROR:     wr_u32(d, odc_axis_error()); wr_u32(d + 4, odc_axis_error()); return true;
    case CMD_ENCODER_EST:   wr_f32(d, motor_pos_turns()); wr_f32(d + 4, g_rpm_meas * (1.0f / 60.0f)); return true;
    case CMD_GET_IQ:        wr_f32(d, g_iq_ref); wr_f32(d + 4, g_iq); return true;
    case CMD_GET_TEMP:      wr_f32(d, app_temp_driver()); wr_f32(d + 4, app_temp_motor()); return true;
    case CMD_GET_BUS: {
        float vb = g_vbus;
        wr_f32(d, vb); wr_f32(d + 4, (vb > 1.0f) ? elec_power() / vb : 0.0f);           /* bus current estimated from Vdq.Idq */
        return true;
    }
    case CMD_GET_TORQUES:   wr_f32(d, kt * g_iq_ref); wr_f32(d + 4, kt * g_iq); return true;
    case CMD_GET_POWERS:    wr_f32(d, elec_power()); wr_f32(d + 4, kt * g_iq * g_rpm_meas * (6.2831853f / 60.0f)); return true;
    default: return false;
    }
}

static void reply(uint32_t cmd)
{
    can_frame_t f;
    if (!fill(cmd, &f)) return;
    f.id = ((uint32_t)can_cfg()->node_id << 5) | cmd; f.ext = false; f.rtr = false;
    (void)can_send(&f);
}

static bool fill_ext(uint32_t idx, can_frame_t *f)
{
    uint8_t *d = f->data;
    memset(d, 0, 8);
    f->len = 8;
    switch (idx) {
    case 0:  wr_i16(d, g_i[0], 100.0f); wr_i16(d + 2, g_i[1], 100.0f); wr_i16(d + 4, g_i[2], 100.0f);
             d[6] = (uint8_t)g_state; d[7] = (uint8_t)g_fault; return true;                       /* ESC_Phase */
    case 1:  wr_i16(d, g_id, 100.0f); wr_i16(d + 2, g_iq, 100.0f); wr_i16(d + 4, g_vd, 100.0f); wr_i16(d + 6, g_vq, 100.0f);
             return true;                                                                         /* ESC_DQ */
    case 2:  wr_f32(d, g_rpm_meas); wr_f32(d + 4, g_rpm_cmd); return true;                        /* ESC_Motion */
    case 3: {                                                                                     /* ESC_Supply */
        float vb = g_vbus;
        wr_u16(d, (uint32_t)(vb * 1000.0f)); wr_i16(d + 2, (vb > 1.0f) ? elec_power() / vb : 0.0f, 100.0f);
        wr_i16(d + 4, app_temp_driver(), 10.0f); wr_i16(d + 6, app_temp_motor(), 10.0f);
        return true;
    }
    case 4:  wr_u16(d, (uint32_t)((float)g_isr_cycles_max * 10.0f / (float)(SYSCLK_HZ / 1000000UL)));   /* ESC_Diag */
             wr_u16(d + 2, g_cs_rejected); wr_u16(d + 4, g_cs_late); d[6] = (uint8_t)motor_owner(); d[7] = (uint8_t)g_mode;
             return true;
    default: return false;
    }
}

#define N_CYC 13
static uint32_t cyc_last[N_CYC];

static void cyclic(uint32_t now)
{
    const can_cfg_t *c = can_cfg();
    static const uint8_t std_cmd[8] = { CMD_HEARTBEAT, CMD_ENCODER_EST, CMD_GET_IQ, CMD_GET_TEMP, CMD_GET_BUS,
                                        CMD_GET_TORQUES, CMD_GET_POWERS, CMD_GET_ERROR };
    const uint16_t rate[N_CYC] = { c->r_hb, c->r_enc, c->r_iq, c->r_temp, c->r_bus, c->r_torque, c->r_power, c->r_err,
                                   c->r_ext, c->r_ext, c->r_ext, c->r_ext, c->r_ext };
    for (int i = 0; i < N_CYC; i++) {
        if (rate[i] == 0u || (now - cyc_last[i]) < rate[i]) continue;
        if (!can_tx_free()) return;                                    /* TX FIFO full: the rest goes out next pass */
        can_frame_t f;
        f.rtr = false;
        if (i < 8) { fill(std_cmd[i], &f); f.id = ((uint32_t)c->node_id << 5) | std_cmd[i]; f.ext = false; }
        else       { fill_ext((uint32_t)(i - 8), &f); f.id = ODC_EXT_BASE | ((uint32_t)c->node_id << 8) | (uint32_t)(i - 8); f.ext = true; }
        if (!can_send(&f)) return;
        cyc_last[i] = now;
    }
}

void odc_rx(const can_frame_t *f)
{
    if (f->ext) return;
    uint32_t cmd = f->id & 0x1Fu;
    const uint8_t *d = f->data;
    if (f->rtr) { reply(cmd); return; }                  /* ODrive v0.5 style request: answer with the data frame */

    switch (cmd) {
    case CMD_ESTOP:
        braking = false;
        app_fault(FLT_ESTOP, "CAN estop");
        break;
    case CMD_SET_AXIS_STATE: {
        if (f->len < 4) break;
        uint32_t s = rd_u32(d);
        if (s == ODRV_STATE_IDLE) {
            braking = false;
            if (!usb_owns()) app_stop("CAN: axis state IDLE"); else ignored++;
        } else if (s == ODRV_STATE_CLOSED_LOOP) {
            start_closed_loop(false);
        } else if (s == ODRV_STATE_LOCKIN_SPIN) {
            start_closed_loop(true);                          /* open loop (V/f) at the Set_Input_Vel speed */
        } else if (s == ODRV_STATE_FULL_CAL || s == ODRV_STATE_MOTOR_CAL || s == ODRV_STATE_ENC_OFFSET_CAL) {
            const char *why = app_calibrate(SRC_CAN, true);   /* R, L, encoder direction, pole pairs; saved to flash */
            last_refusal = why ? why : "";
            cal_requested = (why == NULL);
        }
        break;
    }
    case CMD_SET_CTRL_MODE:
        if (f->len < 8) break;
        ctrl_mode = rd_u32(d); input_mode = rd_u32(d + 4);     /* takes effect at the next CLOSED_LOOP request */
        break;
    case CMD_SET_INPUT_POS:
        if (f->len < 4) break;
        if (usb_owns()) { ignored++; break; }
        g_pos_cmd = rd_f32(d) * 360.0f;                       /* vel_ff / torque_ff (bytes 4..7) are not used */
        break;
    case CMD_SET_INPUT_VEL:
        if (f->len < 4) break;
        if (usb_owns()) { ignored++; break; }
        vel_sp_turns = rd_f32(d);                             /* torque_ff (bytes 4..7) is not used */
        if (can_owns()) { g_vel_ext_rpm = vel_sp_turns * 60.0f; g_vel_ext = true; }
        break;
    case CMD_SET_INPUT_TORQUE: {
        if (f->len < 4) break;
        if (usb_owns()) { ignored++; break; }
        float a = rd_f32(d) / kt_nm_per_a();
        float lim = g_par.i_max;
        g_tq_cmd = (a > lim) ? lim : (a < -lim ? -lim : a);
        break;
    }
    case CMD_SET_LIMITS:
        if (f->len < 8) break;
        set_param("rpmmax", rd_f32(d) * 60.0f);
        set_param("imax", rd_f32(d + 4));
        break;
    case CMD_SET_TRAJ_VEL:
        if (f->len >= 4) set_param("posvmax", rd_f32(d) * 60.0f);
        break;
    case CMD_SET_TRAJ_ACCEL:
        if (f->len >= 4) set_param("posacc", rd_f32(d) * 60.0f);   /* deceleration (bytes 4..7): same as acceleration here */
        break;
    case CMD_SET_TRAJ_INERTIA:
        break;
    case CMD_SET_POS_GAIN:
        if (f->len >= 4) set_param("poskp", rd_f32(d) / 6.0f);     /* (turns/s)/turn -> rpm/deg */
        break;
    case CMD_SET_VEL_GAINS:
        if (f->len < 8) break;
        set_param("spdkp", rd_f32(d) / (60.0f * kt_nm_per_a()));     /* Nm/(turns/s) -> A/rpm */
        set_param("spdki", rd_f32(d + 4) / (60.0f * kt_nm_per_a()));
        break;
    case CMD_SET_ABS_POS:
        if (f->len >= 4) motor_set_pos_turns(rd_f32(d));
        break;
    case CMD_CLEAR_ERRORS:
        err_latched = 0;
        app_clear();
        break;
    case CMD_SET_NODE_ID:
        if (f->len < 1 || motor_running_any()) break;
        if (can_cfg_set("node", (float)d[0])) (void)can_cfg_save();
        break;
    case CMD_REBOOT: {
        if (motor_running_any()) break;
        uint8_t action = (f->len >= 1) ? d[0] : 0u;
        if (action == 0u) NVIC_SystemReset();
        else if (action == 1u) { (void)flash_save_params(&g_par); (void)can_cfg_save(); }   /* save_configuration */
        break;
    }
    default:
        break;
    }
}

/* CAN watchdog and the optional braking on loss of CAN */
static void watchdog(uint32_t now)
{
    const can_cfg_t *c = can_cfg();
    if (braking) {
        esc_mode_t m = g_mode;
        float rpm = g_rpm_meas;
        if (m == MODE_TORQUE) {
            float i = 0.5f * g_par.i_max;
            g_tq_cmd = (rpm > 0.0f) ? -i : i;
        }
        bool slow = (m == MODE_OPEN) ? (fabsf(g_rpm_cmd) < 1.0f) : (fabsf(rpm) < 30.0f);
        if (!can_owns()) { braking = false; return; }
        if (slow || (now - brake_t0) > 3000u) {
            braking = false;
            app_fault(FLT_CAN_TIMEOUT, "CAN timeout (braked, then coast)");
        }
        return;
    }
    if (!can_owns() || c->timeout_ms == 0u) return;
    if ((int32_t)(now - can_stat()->last_rx_ms) <= (int32_t)c->timeout_ms) return;
    if (!c->loss_brake || g_mode == MODE_POSITION) {                 /* position mode: always coast */
        app_fault(FLT_CAN_TIMEOUT, "CAN timeout (coast)");
        return;
    }
    braking = true;
    brake_t0 = now;
    g_vel_ext_rpm = 0.0f; g_vel_ext = true;                          /* speed and open loop: ramp to 0 with 'accel' */
}

uint32_t odc_axis_error(void)
{
    uint32_t e = err_latched;
    switch (g_fault) {
    case FLT_OVERCURRENT:  e |= ODRV_ERR_CURRENT_LIMIT; break;
    case FLT_OVERVOLT:     e |= ODRV_ERR_DC_OVER_VOLTAGE; break;
    case FLT_UNDERVOLT:    e |= ODRV_ERR_DC_UNDER_VOLTAGE; break;
    case FLT_OVERTEMP:     e |= ODRV_ERR_INVERTER_OVER_TEMP; break;
    case FLT_ENCODER:      e |= ODRV_ERR_MISSING_ESTIMATE; break;
    case FLT_OVERSPEED:    e |= ODRV_ERR_VELOCITY_LIMIT; break;
    case FLT_ESTOP:        e |= ODRV_ERR_ESTOP; break;
    case FLT_CAN_TIMEOUT:  e |= ODRV_ERR_WATCHDOG; break;
    default: break;
    }
    return e;
}

uint8_t odc_axis_state(void)
{
    switch (g_state) {
    case ST_CALIB:     return ODRV_STATE_FULL_CAL;
    case ST_PRECHARGE:
    case ST_ALIGN:
    case ST_RUN:       return (g_mode == MODE_OPEN) ? ODRV_STATE_LOCKIN_SPIN : ODRV_STATE_CLOSED_LOOP;
    default:           return ODRV_STATE_IDLE;
    }
}

void odc_task(uint32_t now)
{
    if (cal_requested) {
        const char *err;
        int s = app_cal_state(&err);
        if (s == 3) { err_latched |= ODRV_ERR_CALIBRATION; cal_requested = false; }
        else if (s == 2) cal_requested = false;
    }
    watchdog(now);
    cyclic(now);
}

void odc_print(void (*pr)(const char *fmt, ...))
{
    pr("ODrive CANSimple: control %s, ODrive control mode %lu input mode %lu, axis state %u error 0x%08lX, "
       "ignored set points (USB owns) %lu, last refusal %s%s\r\n",
       can_owns() ? "CAN" : (usb_owns() ? "USB" : "none"), (unsigned long)ctrl_mode, (unsigned long)input_mode,
       (unsigned)odc_axis_state(), (unsigned long)odc_axis_error(), (unsigned long)ignored,
       last_refusal[0] ? last_refusal : "-", braking ? ", BRAKING" : "");
}

#endif
