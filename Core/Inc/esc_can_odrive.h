#ifndef ESC_CAN_ODRIVE_H
#define ESC_CAN_ODRIVE_H

/* ODrive CANSimple (v0.6 message layout) on top of esc_can.c.
 * 11-bit ID = node_id << 5 | cmd_id, little-endian, SI units of ODrive: turns, turns/s, Nm, A, V.
 * See README section "CAN bus" for the supported messages and the differences to a real ODrive. */

#include "esc_can.h"

/* ODrive axis states / control modes / errors (subset used here) */
#define ODRV_STATE_IDLE           1u
#define ODRV_STATE_FULL_CAL       3u
#define ODRV_STATE_MOTOR_CAL      4u
#define ODRV_STATE_ENC_OFFSET_CAL 7u
#define ODRV_STATE_CLOSED_LOOP    8u
#define ODRV_STATE_LOCKIN_SPIN    9u

#define ODRV_ERR_MISSING_ESTIMATE   0x00000008u
#define ODRV_ERR_DC_OVER_VOLTAGE    0x00000100u
#define ODRV_ERR_DC_UNDER_VOLTAGE   0x00000200u
#define ODRV_ERR_CURRENT_LIMIT      0x00001000u
#define ODRV_ERR_MOTOR_OVER_TEMP    0x00002000u
#define ODRV_ERR_INVERTER_OVER_TEMP 0x00004000u
#define ODRV_ERR_VELOCITY_LIMIT     0x00008000u
#define ODRV_ERR_WATCHDOG           0x01000000u
#define ODRV_ERR_ESTOP              0x02000000u
#define ODRV_ERR_CALIBRATION        0x40000000u

void     odc_rx(const can_frame_t *f);     /* a frame for this node (from can_task) */
void     odc_task(uint32_t now_ms);        /* watchdog, braking, cyclic messages */
uint32_t odc_axis_error(void);
uint8_t  odc_axis_state(void);
void     odc_print(void (*pr)(const char *fmt, ...));

/* extra telemetry (not in ODrive): 29-bit IDs ODC_EXT_BASE | node_id << 8 | index, little endian, see can/esc_odrive.dbc */
#define ODC_EXT_BASE 0x10000000UL

#endif
