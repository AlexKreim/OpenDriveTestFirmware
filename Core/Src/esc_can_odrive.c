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

void odc_rx(const can_frame_t *f)
{
    if (f->ext) return;
    uint32_t cmd = f->id & 0x1Fu;
    const uint8_t *d = f->data;
    if (f->rtr) return;                                   /* requests are answered in the telemetry part */

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
