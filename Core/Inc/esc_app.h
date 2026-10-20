#ifndef ESC_APP_H
#define ESC_APP_H

#include <stdint.h>

void esc_early_init(void);                          /* first line of main() */
void esc_app_init(void);                            /* after MX_USB_Device_Init() */
void esc_app_loop(void);                            /* inside while(1) */
void esc_usb_rx(const uint8_t *buf, uint32_t len);  /* call from CDC_Receive_FS */

#endif
