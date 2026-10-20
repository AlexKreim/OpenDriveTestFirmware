#include "esc_hw.h"
#include <math.h>

/* ---------------------------------------------------------------------
 *  Pin map (from the schematic description)
 *  PA8/9/10  TIM1_CH1/2/3 (AF6)   high-side inputs A/B/C
 *  PB13/14/15 TIM1_CH1N/2N/3N (AF6/AF6/AF4) low-side inputs A/B/C
 *  PA1/PA3 OPAMP1 +/-, PA2 OPAMP1 out (ADC1_IN3)
 *  PA7/PA5 OPAMP2 +/-, PA6 OPAMP2 out (ADC2_IN3)
 *  PB0/PB2 OPAMP3 +/-, PB1 OPAMP3 out (ADC3_IN1)
 *  PA0 Vbus (ADC1_IN1), PB12 CC6905 Vref (ADC1_IN11), PC2 CC6905 Vout (ADC1_IN8)
 *  PA4 / PC4 / PB11 phase A/B/C (ADC2_IN17 / IN5 / IN14), PB10 divider enable
 *  PC1 driver NTC (ADC2_IN7), PC0 motor NTC (ADC2_IN6)
 *  PC6/PC7 TIM3 CH1/CH2 (AF2), PD2 TIM3_ETR (AF2)
 *  PC10/PC11/PC12 SPI3 SCK/MISO/MOSI (AF6), PB9 nCS
 *  PC13/14/15 LED R/G/B, active low
 * --------------------------------------------------------------------- */

#define MODE_IN   0u
#define MODE_OUT  1u
#define MODE_AF   2u
#define MODE_AN   3u

#if !OPAMP_FROM_CUBEMX
#define OPA_EN    (1u << 0)
#define OPA_HSM   (1u << 7)
#endif

static uint8_t led_state;
static bool sw_trig;

static void gpio_cfg(GPIO_TypeDef *g, uint32_t p, uint32_t mode, uint32_t af)
{
    g->OSPEEDR |=  (3u << (p * 2u));
    g->PUPDR   &= ~(3u << (p * 2u));
    g->OTYPER  &= ~(1u << p);
    if (p < 8u) g->AFR[0] = (g->AFR[0] & ~(0xFu << (p * 4u)))        | (af << (p * 4u));
    else        g->AFR[1] = (g->AFR[1] & ~(0xFu << ((p - 8u) * 4u))) | (af << ((p - 8u) * 4u));
    g->MODER = (g->MODER & ~(3u << (p * 2u))) | (mode << (p * 2u));
}

static void spi3_setup(void)
{
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | (4u << SPI_CR1_BR_Pos);
    SPI3->CR2 = (15u << SPI_CR2_DS_Pos);
    SPI3->CR1 |= SPI_CR1_SPE;
}

static void short_delay(void)
{
    for (volatile int i = 0; i < 30; i++) { __NOP(); }   /* roughly 0.5 - 1 us */
}

/* ------------------------------------------------------------------ */
void hw_early_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
                    RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;
    (void)RCC->AHB2ENR;

    /* gate driver inputs: drive low immediately */
    GPIOA->BSRR = (7u << (8u + 16u));
    GPIOB->BSRR = (7u << (13u + 16u));
    for (uint32_t p = 8; p <= 10; p++) gpio_cfg(GPIOA, p, MODE_OUT, 0);
    for (uint32_t p = 13; p <= 15; p++) gpio_cfg(GPIOB, p, MODE_OUT, 0);

    /* LEDs off (high), BEMF divider disabled, encoder nCS high */
    GPIOC->BSRR = (7u << 13u);
    for (uint32_t p = 13; p <= 15; p++) gpio_cfg(GPIOC, p, MODE_OUT, 0);
#if BEMF_EN_ACTIVE_LOW
    GPIOB->BSRR = (1u << 10);
#else
    GPIOB->BSRR = (1u << (10 + 16));
#endif
    gpio_cfg(GPIOB, 10, MODE_OUT, 0);
    GPIOB->BSRR = (1u << 9);
    gpio_cfg(GPIOB, 9, MODE_OUT, 0);
}

/* ------------------------------------------------------------------ */
void hw_led_set(uint8_t mask)
{
    led_state = mask & 7u;
    for (uint32_t i = 0; i < 3; i++) {
        uint32_t p = 13u + i;
        if (led_state & (1u << i)) GPIOC->BSRR = (1u << (p + 16u));   /* low = on */
        else                       GPIOC->BSRR = (1u << p);
    }
}
uint8_t hw_led_get(void) { return led_state; }

/* ------------------------------------------------------------------ */
static void adc_smp(ADC_TypeDef *a, uint32_t ch, uint32_t smp)
{
    if (ch < 10u) a->SMPR1 = (a->SMPR1 & ~(7u << (3u * ch)))        | (smp << (3u * ch));
    else          a->SMPR2 = (a->SMPR2 & ~(7u << (3u * (ch - 10u)))) | (smp << (3u * (ch - 10u)));
}

static void adc_power_up(ADC_TypeDef *a)
{
    a->CR = (a->CR & ~ADC_CR_DEEPPWD) | ADC_CR_ADVREGEN;
    for (volatile uint32_t i = 0; i < 8000; i++) { __NOP(); }   /* > 20 us regulator startup */
}

static void adc_calibrate_enable(ADC_TypeDef *a)
{
    a->CR &= ~ADC_CR_ADCALDIF;
    a->CR |= ADC_CR_ADCAL;
    uint32_t t = 1000000;
    while ((a->CR & ADC_CR_ADCAL) && --t) { }
    for (volatile uint32_t i = 0; i < 200; i++) { __NOP(); }
    a->ISR = ADC_ISR_ADRDY;
    a->CR |= ADC_CR_ADEN;
    t = 1000000;
    while (!(a->ISR & ADC_ISR_ADRDY) && --t) { }
}

static void adc_inj_arm(ADC_TypeDef *a, uint32_t ch)
{
    adc_smp(a, ch, 2u);                       /* 12.5 cycles, op-amp output is low impedance */
    /* JEXTSEL = 1: tim1_cc4, JEXTEN = 1: rising edge (please verify in RM0440 table for your device) */
    a->JSQR = (ch << ADC_JSQR_JSQ1_Pos) | (1u << ADC_JSQR_JEXTEN_Pos) | (1u << ADC_JSQR_JEXTSEL_Pos);
    a->CR |= ADC_CR_JADSTART;
}

static uint16_t adc_read(ADC_TypeDef *a, uint8_t ch)
{
    a->SQR1 = ((uint32_t)ch << ADC_SQR1_SQ1_Pos);
    a->ISR  = ADC_ISR_EOC | ADC_ISR_EOS;
    a->CR  |= ADC_CR_ADSTART;
    uint32_t t = 200000;
    while (!(a->ISR & ADC_ISR_EOC) && --t) { }
    return (uint16_t)a->DR;
}

static float adc_v(ADC_TypeDef *a, uint8_t ch)
{
    uint32_t s = 0;
    for (int i = 0; i < 8; i++) s += adc_read(a, ch);
    return ((float)s / 8.0f) * (ADC_VREF_V / 4095.0f);
}

/* ------------------------------------------------------------------ */
void hw_init(void)
{
    /* ---- USB clock recovery (HSI48 trimmed by USB SOF) ---- */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) { }
    RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    /* ---- analog pins ---- */
    const struct { GPIO_TypeDef *g; uint8_t p; } an[] = {
        {GPIOA,0},{GPIOA,1},{GPIOA,2},{GPIOA,3},{GPIOA,4},{GPIOA,5},{GPIOA,6},{GPIOA,7},
        {GPIOB,0},{GPIOB,1},{GPIOB,2},{GPIOB,11},{GPIOB,12},
        {GPIOC,0},{GPIOC,1},{GPIOC,2},{GPIOC,4}
    };
    for (unsigned i = 0; i < sizeof(an) / sizeof(an[0]); i++) gpio_cfg(an[i].g, an[i].p, MODE_AN, 0);

    /* ---- OPAMPs: external feedback network (VINP0 / VINM0 pads, output on pad) ---- */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
#if !OPAMP_FROM_CUBEMX
    OPAMP1->CSR = OPA_EN | OPA_HSM;
    OPAMP2->CSR = OPA_EN | OPA_HSM;
    OPAMP3->CSR = OPA_EN | OPA_HSM;
#endif

    /* ---- ADCs ---- */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_ADC345EN;
    (void)RCC->AHB2ENR;
    ADC12_COMMON->CCR  = (ADC12_COMMON->CCR  & ~ADC_CCR_CKMODE) | (3u << ADC_CCR_CKMODE_Pos);   /* HCLK/4 = 42.5 MHz */
    ADC345_COMMON->CCR = (ADC345_COMMON->CCR & ~ADC_CCR_CKMODE) | (3u << ADC_CCR_CKMODE_Pos);

    adc_power_up(ADC1); adc_power_up(ADC2); adc_power_up(ADC3);
    ADC1->CFGR |= ADC_CFGR_JQDIS;
    ADC2->CFGR |= ADC_CFGR_JQDIS;
    ADC3->CFGR |= ADC_CFGR_JQDIS;
    adc_calibrate_enable(ADC1); adc_calibrate_enable(ADC2); adc_calibrate_enable(ADC3);

    /* slow channels: 247.5 cycles */
    const uint8_t s1[] = {1, 11, 8};
    const uint8_t s2[] = {17, 5, 14, 7, 6};
    for (unsigned i = 0; i < sizeof(s1); i++) adc_smp(ADC1, s1[i], 6u);
    for (unsigned i = 0; i < sizeof(s2); i++) adc_smp(ADC2, s2[i], 6u);

    /* ---- TIM1: 3 complementary PWM channels, center aligned, CH4 = ADC trigger ---- */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC  = 0;
    TIM1->ARR  = PWM_ARR;
    TIM1->RCR  = 0;
    TIM1->CR1  = TIM_CR1_CMS_0 | TIM_CR1_ARPE;                 /* center-aligned mode 1 */
    TIM1->CR2  = 0;                                            /* all idle states low */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;   /* PWM mode 1 */
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;   /* CH4: PWM mode 2 */
    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    TIM1->CCR4 = PWM_ARR - ADC_TRIG_OFFSET;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                 TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC4E;    /* active high everywhere */
    TIM1->BDTR = DEADTIME_DTG | TIM_BDTR_OSSR | TIM_BDTR_OSSI;      /* MOE = 0: outputs forced low */
    TIM1->EGR  = TIM_EGR_UG;

    /* arm the injected ADCs before the timer runs */
    adc_inj_arm(ADC1, 3);    /* PA2  OPAMP1 out, phase A */
    adc_inj_arm(ADC2, 3);    /* PA6  OPAMP2 out, phase B */
    adc_inj_arm(ADC3, 1);    /* PB1  OPAMP3 out, phase C */
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC1->IER |= ADC_IER_JEOSIE;
    NVIC_SetPriority(ADC1_2_IRQn, 1);
    NVIC_EnableIRQ(ADC1_2_IRQn);

    /* hand the pins to TIM1 (MOE still 0, so they stay low) */
    for (uint32_t p = 8; p <= 10; p++) gpio_cfg(GPIOA, p, MODE_AF, 6);
    gpio_cfg(GPIOB, 13, MODE_AF, 6);                       /* TIM1_CH1N */
    gpio_cfg(GPIOB, 14, MODE_AF, 6);                       /* TIM1_CH2N */
    gpio_cfg(GPIOB, 15, MODE_AF, 4);                       /* TIM1_CH3N is AF4 on PB15 (not AF6) */
    TIM1->CR1 |= TIM_CR1_CEN;

    /* ---- TIM3 quadrature decoder ---- */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    gpio_cfg(GPIOC, 6, MODE_AF, 2);
    gpio_cfg(GPIOC, 7, MODE_AF, 2);
    gpio_cfg(GPIOD, 2, MODE_AF, 2);
    TIM3->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (1u << TIM_CCMR1_CC2S_Pos) |
                  (3u << TIM_CCMR1_IC1F_Pos) | (3u << TIM_CCMR1_IC2F_Pos);
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->SMCR  = 3u;                                          /* encoder mode 3: x4 on TI1 and TI2 */
#if ENC_USE_INDEX
    TIM3->ARR = ENC_CPR - 1u;
    TIM3->ECR = 1u;                                            /* IE: index enable (ETR = PD2) */
#else
    TIM3->ARR = 0xFFFFu;
#endif
    TIM3->CR1 = TIM_CR1_CEN;

    /* ---- SPI3 for the AS5047P (mode 1, 16 bit, 5.3 MHz) ---- */
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
    gpio_cfg(GPIOC, 10, MODE_AF, 6);
    gpio_cfg(GPIOC, 11, MODE_AF, 6);
    gpio_cfg(GPIOC, 12, MODE_AF, 6);
    spi3_setup();
}


/* ------------------------------------------------------------------ */
bool hw_adc_sw_mode(void) { return sw_trig; }

void hw_adc_use_software_trigger(void)
{
    ADC_TypeDef *adcs[3] = {ADC1, ADC2, ADC3};
    const uint8_t ch[3]  = {3, 3, 1};

    ADC1->IER &= ~ADC_IER_JEOSIE;
    NVIC_DisableIRQ(ADC1_2_IRQn);
    for (int i = 0; i < 3; i++) {
        ADC_TypeDef *a = adcs[i];
        if (a->CR & ADC_CR_JADSTART) {
            a->CR |= ADC_CR_JADSTP;
            uint32_t t = 100000;
            while ((a->CR & ADC_CR_JADSTP) && --t) { }
        }
        a->ISR  = ADC_ISR_JEOS | ADC_ISR_JEOC;
        a->JSQR = ((uint32_t)ch[i] << ADC_JSQR_JSQ1_Pos);      /* JEXTEN = 0: software trigger */
    }
    TIM1->SR = ~TIM_SR_CC4IF;
    TIM1->DIER |= TIM_DIER_CC4IE;
    NVIC_SetPriority(TIM1_CC_IRQn, 1);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    sw_trig = true;
}

void hw_inj_sw_start(void)
{
    ADC1->CR |= ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_JADSTART;
    ADC3->CR |= ADC_CR_JADSTART;
}

/* ------------------------------------------------------------------ */
bool hw_inj_collect(uint16_t raw[3])
{
    if (!(ADC1->ISR & ADC_ISR_JEOS)) return false;
    uint32_t t = 2000;
    while ((!(ADC2->ISR & ADC_ISR_JEOS) || !(ADC3->ISR & ADC_ISR_JEOS)) && --t) { }
    raw[0] = (uint16_t)ADC1->JDR1;
    raw[1] = (uint16_t)ADC2->JDR1;
    raw[2] = (uint16_t)ADC3->JDR1;
    ADC1->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC2->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    ADC3->ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
    return true;
}

/* ------------------------------------------------------------------ */
float hw_vbus_v(void)    { return adc_v(ADC1, 1) * VBUS_DIV; }
float hw_cc_vout_v(void) { return adc_v(ADC1, 8); }
float hw_cc_vref_v(void) { return adc_v(ADC1, 11); }
float hw_ibus_a(void)    { return (hw_cc_vout_v() - hw_cc_vref_v()) / CC6905_V_PER_A; }

float hw_phase_pin_v(int ph)
{
    static const uint8_t ch[3] = {17, 5, 14};
    return adc_v(ADC2, ch[ph]);
}
float hw_phase_v(int ph)
{
    /* Divider with a Schottky in series with the 2k2: below one diode drop it does not conduct and the node
     * simply follows the phase; above that, node = Vd + (Vph - Vd) / PHASE_DIV. */
    float n = hw_phase_pin_v(ph);
    if (n <= PHASE_VD_V) return n;
    return PHASE_DIV * n - (PHASE_DIV - 1.0f) * PHASE_VD_V;
}

void hw_bemf_enable(bool en)
{
#if BEMF_EN_ACTIVE_LOW
    if (en) GPIOB->BSRR = (1u << (10 + 16)); else GPIOB->BSRR = (1u << 10);
#else
    if (en) GPIOB->BSRR = (1u << 10); else GPIOB->BSRR = (1u << (10 + 16));
#endif
}

float hw_ntc_pin_v(int which) { return which == 0 ? adc_v(ADC2, 7) : adc_v(ADC2, 6); }

static float ntc_c(float r, float r25, float beta)
{
    return 1.0f / (1.0f / 298.15f + logf(r / r25) / beta) - 273.15f;
}

float hw_temp_driver_c(void)
{
    float v = hw_ntc_pin_v(0);
    if (v < 0.02f || v > ADC_VREF_V - 0.02f) return -999.0f;
    float r = NTC_DRV_RFIX * (ADC_VREF_V / v - 1.0f);        /* NTC on top, fixed resistor to GND */
    return ntc_c(r, NTC_DRV_R25, NTC_DRV_BETA);
}

float hw_temp_motor_c(void)
{
    float v = hw_ntc_pin_v(1);
    if (v < 0.02f || v > ADC_VREF_V - 0.02f) return -999.0f;
    float r = NTC_MOT_RFIX * v / (ADC_VREF_V - v);           /* fixed resistor to 3V3, NTC to GND */
    return ntc_c(r, NTC_MOT_R25, NTC_MOT_BETA);
}

/* ------------------------------------------------------------------ */
uint16_t hw_enc_count(void) { return (uint16_t)TIM3->CNT; }

static uint16_t as_parity(uint16_t v)
{
    v &= 0x7FFFu;
    v ^= (uint16_t)(v >> 8); v ^= (uint16_t)(v >> 4); v ^= (uint16_t)(v >> 2); v ^= (uint16_t)(v >> 1);
    return v & 1u;
}

static uint16_t as_xfer(uint16_t w)
{
    GPIOB->BSRR = (1u << (9 + 16));
    short_delay();
    *(volatile uint16_t *)&SPI3->DR = w;
    uint32_t t = 100000;
    while (!(SPI3->SR & SPI_SR_RXNE) && --t) { }
    uint16_t r = *(volatile uint16_t *)&SPI3->DR;
    t = 100000;
    while ((SPI3->SR & SPI_SR_BSY) && --t) { }
    short_delay();
    GPIOB->BSRR = (1u << 9);
    short_delay();
    return r;
}

static volatile bool as_pipe_valid;     /* the previous frame was an ANGLECOM read, so this frame's answer is an angle */

/* Re-initialises the SPI peripheral and clocks the encoder interface clean (used after an encoder fault). */
void hw_as5047_reset(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    SPI3->CR1 &= ~SPI_CR1_SPE;
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI3RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI3RST;
    spi3_setup();
    GPIOB->BSRR = (1u << 9);
    as_pipe_valid = false;
    for (int i = 0; i < 4; i++) (void)as_xfer(0xC000u);          /* NOP frames */
    if (pm == 0u) __enable_irq();
}

uint16_t hw_as5047_read(uint16_t addr, bool *err)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();                    /* the control interrupt shares the SPI: a transaction must not be split */
    as_pipe_valid = false;
    uint16_t cmd = (uint16_t)(0x4000u | (addr & 0x3FFFu));
    cmd |= (uint16_t)(as_parity(cmd) << 15);
    (void)as_xfer(cmd);                 /* the answer arrives in the next frame */
    uint16_t r = as_xfer(0xC000u);      /* NOP (read of 0x0000 with parity) */
    if (pm == 0u) __enable_irq();
    if (err) *err = ((r & 0x4000u) != 0) || (as_parity(r) != (uint16_t)(r >> 15)) || (r == 0xFFFFu);   /* error flag, parity, or no chip answering */
    return r & 0x3FFFu;
}

/* One frame per call from the control interrupt: sends READ ANGLECOM and returns the angle requested by the previous call.
 * The angle is therefore one control period old; the FOC code compensates for that. */
uint16_t hw_as5047_pipe(bool *ok)
{
    uint16_t r = as_xfer(0xFFFFu);                    /* 0x3FFF | read bit | even parity */
    bool good = as_pipe_valid && ((r & 0x4000u) == 0) && (as_parity(r) == (uint16_t)(r >> 15)) && (r != 0xFFFFu);
    as_pipe_valid = true;
    if (ok) *ok = good;
    return r & 0x3FFFu;
}
