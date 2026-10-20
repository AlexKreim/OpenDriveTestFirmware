#ifndef ESC_HW_H
#define ESC_HW_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"
#include "esc_config.h"

/* Call as the very first thing in main(): puts all six gate-driver inputs low. */
void hw_early_init(void);
/* Call after clocks are up: TIM1, ADCs, OPAMPs, TIM3 encoder, SPI3, LED, CRS. */
void hw_init(void);

/* RGB LED, bit0 = red, bit1 = green, bit2 = blue (1 = lit; the LED is active low on the board) */
void    hw_led_set(uint8_t mask);
uint8_t hw_led_get(void);

/* Injected (phase current) conversions, called from the ADC interrupt. false = no new data. */
bool hw_inj_collect(uint16_t raw[3]);

/* Fallback when the hardware trigger does not arrive: TIM1 CC4 interrupt starts the injected conversions. */
void hw_adc_use_software_trigger(void);
void hw_inj_sw_start(void);
bool hw_adc_sw_mode(void);
#define ADC_TRIG_CC4    1u     /* JEXTSEL: TIM1_CC4 */
#define ADC_TRIG_TRGO2  8u     /* JEXTSEL: TIM1_TRGO2 */
void hw_adc_use_hw_trigger(uint32_t jextsel);
uint32_t hw_adc_trigger_sel(void);
const char *hw_adc_trigger_name(void);

/* Slow measurements (blocking, a few microseconds each) */
float hw_vbus_v(void);
float hw_cc_vout_v(void);
float hw_cc_vref_v(void);
float hw_ibus_a(void);
float hw_phase_pin_v(int ph);   /* voltage at the ADC pin */
float hw_phase_v(int ph);       /* estimated phase voltage (pin voltage x PHASE_DIV) */
float hw_ntc_pin_v(int which);  /* 0 = driver, 1 = motor */
float hw_temp_driver_c(void);   /* -999 if the sensor looks open/shorted */
float hw_temp_motor_c(void);
void  hw_bemf_enable(bool en);

/* Encoder */
uint16_t hw_enc_count(void);
uint16_t hw_as5047_read(uint16_t addr, bool *err);
uint16_t hw_as5047_pipe(bool *ok);
void     hw_as5047_reset(void);       /* re-init SPI3 and flush the encoder interface */   /* control interrupt only: pipelined angle read, one SPI frame */

#endif
