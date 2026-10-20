#ifndef ESC_FAULTLOG_H
#define ESC_FAULTLOG_H

#include <stdint.h>
#include <stdbool.h>

/* Log of the last FLOG_N faults, host-watchdog stops and abnormal MCU resets. Kept in RAM and written to its own flash page
 * (0x0807E000, separate from the parameters) by the main loop once the motor is stopped, so it survives a reset or power cycle.
 * A brown-out cannot be logged (no time); it shows as 'last MCU reset' in 'status'. */
#define FLOG_N 8
#define FLOG_FLASH_ADDR 0x0807E000UL

typedef enum { FLOG_FAULT = 1, FLOG_WATCHDOG = 2, FLOG_RESET = 3 } flog_kind_t;
typedef enum { FLOG_RST_IWDG = 1, FLOG_RST_WWDG = 2, FLOG_RST_SOFT = 3, FLOG_RST_LOWPWR = 4 } flog_reset_t;

typedef struct {
    uint32_t uptime_ms;
    uint16_t boot;          /* boot number: entries with the same number come from the same power-on */
    uint8_t  kind;          /* flog_kind_t */
    uint8_t  code;          /* esc_fault_t for FLOG_FAULT, flog_reset_t for FLOG_RESET */
    uint8_t  state, mode;   /* state machine and control mode when it happened */
    uint8_t  pad[2];
    float    vbus, ia, ib, ic, imag, rpm, iq_ref, oc_a;   /* oc_a: last sample above the overcurrent level */
} flog_entry_t;

void     flog_init(int reset_code);          /* at boot: load the flash copy; reset_code 0 = normal (power-on / pin) */
void     flog_fault(int code);               /* from motor_fault (interrupt safe), call before the state changes */
void     flog_watchdog(int code);            /* a running motor was stopped: 0 = host watchdog, 1 = USB send stuck */
void     flog_task(bool motor_running);      /* main loop: writes pending entries to flash while stopped */
int      flog_count(void);
bool     flog_get(int i, flog_entry_t *e);   /* 0 = oldest */
uint16_t flog_boot(void);                    /* number of this boot */
bool     flog_clear(void);                   /* erase RAM and flash copy (motor stopped) */
const char *flog_flash_state(void);         /* "none" (nothing written this boot), "ok", "fail" (last write), "pending" */

#endif
