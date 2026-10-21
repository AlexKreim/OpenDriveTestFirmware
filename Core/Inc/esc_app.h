#ifndef ESC_APP_H
#define ESC_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_motor.h"

void esc_early_init(void);                          /* first line of main() */
void esc_app_init(void);                            /* after MX_USB_Device_Init() */
void esc_app_loop(void);                            /* inside while(1) */
void esc_usb_rx(const uint8_t *buf, uint32_t len);  /* call from CDC_Receive_FS */

/* shared by the USB console and CAN (esc_can_odrive.c): same checks for both */
const char *app_start(ctl_src_t src);               /* NULL = started (mode = g_mode), else the reason */
const char *app_calibrate(ctl_src_t src, bool save);/* motor calibration; save = write the parameters to flash when done */
void        app_stop(const char *reason);           /* stop; reason shown as 'last stop' if the motor was running */
void        app_fault(esc_fault_t f, const char *reason);
void        app_clear(void);                        /* clear a fault, re-init the encoder SPI */
float       app_temp_driver(void);
float       app_temp_motor(void);
int         app_cal_state(const char **err);        /* 0 never run, 1 running, 2 ok, 3 failed (err = reason) */

#endif
