#ifndef ESC_CAN_H
#define ESC_CAN_H

/* CAN bus on FDCAN2 (PB5 = RX, PB6 = TX, AF9), classic CAN 2.0, TCAN332 transceiver (no control pins, termination by solder
 * jumper). Register-level driver like esc_hw.c (no HAL FDCAN, no CubeMX change). The protocol (ODrive CANSimple) is in
 * esc_can_odrive.c. Settings live in their own flash page (CAN_CFG_FLASH_ADDR), not in the parameter struct.
 * Build without CAN: ESC_CAN_ENABLE 0 in esc_config.h. */

#include <stdint.h>
#include <stdbool.h>
#include "esc_flash.h"

#define CAN_CFG_FLASH_ADDR 0x0807D000UL       /* dual bank: bank 2 page 122; single bank: page 125 (4 KB) */

typedef struct {
    uint32_t id;           /* 11 or 29 bit */
    bool     ext, rtr;
    uint8_t  len;
    uint8_t  data[8];
} can_frame_t;

/* stored settings; rates in ms, 0 = message off */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t  enable;       /* 1 = CAN on */
    uint8_t  node_id;      /* 0..63, ODrive default 0 */
    uint16_t kbps;         /* 125, 250, 500, 1000 */
    uint16_t timeout_ms;   /* CAN control watchdog: no frame for this node while CAN owns the motor -> loss action. 0 = off */
    uint8_t  loss_brake;   /* 0 = coast (outputs off), 1 = brake to standstill, then coast */
    uint8_t  res0;
    uint16_t r_hb, r_enc, r_iq, r_temp, r_bus, r_torque, r_power, r_err, r_ext;
    uint16_t res1[2];
    uint32_t crc;
} can_cfg_t;

typedef struct {
    uint32_t rx, tx, tx_drop, rx_drop, busoff;
    uint8_t  tec, rec;
    bool     running, err_passive, bus_off;
    uint32_t last_rx_ms;   /* last frame addressed to this node */
} can_stat_t;

void        can_init(void);                       /* esc_app_init: load the settings, start the bus if enabled */
void        can_task(void);                       /* main loop */
bool        can_send(const can_frame_t *f);       /* false: TX FIFO full (counted) */
bool        can_tx_free(void);                    /* a TX FIFO slot is free */
can_cfg_t  *can_cfg(void);
const can_stat_t *can_stat(void);
bool        can_cfg_set(const char *name, float v);   /* RAM; bus restarts when needed. false: unknown name / range */
flash_res_t can_cfg_save(void);                   /* motor stopped (checked by the caller) */
void        can_print(void (*pr)(const char *fmt, ...));
void        can_restart(void);                    /* apply enable / node / bitrate */

#endif
