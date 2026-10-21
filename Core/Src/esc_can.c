#include "esc_can.h"
#include "esc_can_odrive.h"
#include "esc_config.h"
#include "esc_params.h"
#include "stm32g4xx_hal.h"
#include <stddef.h>
#include <string.h>

#if ESC_CAN_ENABLE

/* ------------------------------------------------------------------ message RAM (fixed layout on the STM32G4, RM0440)
 * per FDCAN instance 0x350 bytes: 28 standard filters, 8 extended filters, RX FIFO 0 / 1 (3 x 18 words each),
 * TX event FIFO (3 x 2 words), TX FIFO (3 x 18 words). FDCAN2's block follows FDCAN1's. */
#define MRAM_BASE   (SRAMCAN_BASE + 0x350UL)
#define MRAM_FLS    (MRAM_BASE + 0x000UL)
#define MRAM_RF0    (MRAM_BASE + 0x0B0UL)
#define MRAM_TXB    (MRAM_BASE + 0x278UL)
#define ELEM_BYTES  72UL

#define CFG_MAGIC   0x43414E31UL             /* "CAN1" */
#define CFG_VERSION 1u

_Static_assert((sizeof(can_cfg_t) % 8u) == 0u, "CAN settings must be a multiple of 8 bytes");

static can_cfg_t  cfg;
static can_stat_t st;

/* software RX ring filled by the interrupt, emptied by the main loop */
#define RXQ_N 32u
static can_frame_t rxq[RXQ_N];
static volatile uint32_t rxq_h, rxq_t;

static void cfg_defaults(can_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->magic = CFG_MAGIC; c->version = CFG_VERSION;
    c->enable = 1; c->node_id = 0; c->kbps = 250;
    c->timeout_ms = 500; c->loss_brake = 0;
    c->r_hb = 100; c->r_enc = 10; c->r_iq = 100; c->r_temp = 100; c->r_bus = 100; c->r_torque = 100;
    c->r_power = 0; c->r_err = 0; c->r_ext = 100;
}

static bool cfg_load(can_cfg_t *c)
{
    const can_cfg_t *f = (const can_cfg_t *)CAN_CFG_FLASH_ADDR;
    if (f->magic != CFG_MAGIC || f->version != CFG_VERSION) return false;
    if (params_crc32(f, offsetof(can_cfg_t, crc)) != f->crc) return false;
    *c = *f;
    return true;
}

/* ------------------------------------------------------------------ peripheral */
static void fdcan_stop(void)
{
    FDCAN2->ILE = 0;
    NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    uint32_t t0 = HAL_GetTick();
    while (!(FDCAN2->CCCR & FDCAN_CCCR_INIT) && (HAL_GetTick() - t0) < 10u) { }
    st.running = false;
}

static void gpio_af9(uint32_t pin)
{
    GPIOB->MODER   = (GPIOB->MODER & ~(3u << (pin * 2u))) | (2u << (pin * 2u));
    GPIOB->OSPEEDR |= (2u << (pin * 2u));
    GPIOB->OTYPER &= ~(1u << pin);
    GPIOB->PUPDR  &= ~(3u << (pin * 2u));
    GPIOB->AFR[0]  = (GPIOB->AFR[0] & ~(0xFu << (pin * 4u))) | (9u << (pin * 4u));
}

static void fdcan_start(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->CCIPR     = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL_Msk) | RCC_CCIPR_FDCANSEL_1;     /* kernel clock = PCLK1 */
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;
    gpio_af9(5u);                                                                     /* PB5 FDCAN2_RX */
    gpio_af9(6u);                                                                     /* PB6 FDCAN2_TX */

    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    uint32_t t0 = HAL_GetTick();
    while (!(FDCAN2->CCCR & FDCAN_CCCR_INIT) && (HAL_GetTick() - t0) < 10u) { }
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    FDCAN2->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR | FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM);

    /* nominal bit timing: 17 time quanta, sample point 82 % (1 + 13 | 3), SJW 3 */
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    uint32_t brp = pclk / ((uint32_t)cfg.kbps * 1000u * 17u);
    if (brp < 1u) brp = 1u;
    if (brp > 512u) brp = 512u;
    FDCAN2->NBTP = ((3u - 1u) << FDCAN_NBTP_NSJW_Pos) | ((brp - 1u) << FDCAN_NBTP_NBRP_Pos) |
                   ((13u - 1u) << FDCAN_NBTP_NTSEG1_Pos) | ((3u - 1u) << FDCAN_NBTP_NTSEG2_Pos);

    /* filter 0: standard IDs node_id << 5 | cmd (cmd 0..31), classic id + mask, store in RX FIFO 0; reject everything else */
    volatile uint32_t *fls = (volatile uint32_t *)MRAM_FLS;
    for (uint32_t i = 0; i < 0x350u / 4u; i++) ((volatile uint32_t *)MRAM_BASE)[i] = 0u;
    uint32_t id = ((uint32_t)cfg.node_id & 0x3Fu) << 5;
    fls[0] = (2u << 30) | (1u << 27) | (id << 16) | 0x7E0u;
    FDCAN2->RXGFC = (1u << FDCAN_RXGFC_LSS_Pos) | (2u << FDCAN_RXGFC_ANFS_Pos) | (2u << FDCAN_RXGFC_ANFE_Pos);
    FDCAN2->TXBC = 0;                                                                 /* TX FIFO mode */

    FDCAN2->IR  = 0xFFFFFFFFu;
    FDCAN2->IE  = FDCAN_IE_RF0NE;
    FDCAN2->ILS = 0;                                                                  /* all on line 0 */
    FDCAN2->ILE = FDCAN_ILE_EINT0;
    NVIC_SetPriority(FDCAN2_IT0_IRQn, 4);                                             /* below control (0) and USB (3) */
    NVIC_ClearPendingIRQ(FDCAN2_IT0_IRQn);
    NVIC_EnableIRQ(FDCAN2_IT0_IRQn);

    FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;                                                 /* join the bus */
    st.running = true;
}

void FDCAN2_IT0_IRQHandler(void)
{
    FDCAN2->IR = FDCAN_IR_RF0N;
    uint32_t s;
    while (((s = FDCAN2->RXF0S) & FDCAN_RXF0S_F0FL_Msk) != 0u) {
        uint32_t gi = (s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
        const volatile uint32_t *e = (const volatile uint32_t *)(MRAM_RF0 + gi * ELEM_BYTES);
        uint32_t w0 = e[0], w1 = e[1], d0 = e[2], d1 = e[3];
        FDCAN2->RXF0A = gi;
        uint32_t nx = (rxq_h + 1u) % RXQ_N;
        if (nx == rxq_t) { st.rx_drop++; continue; }
        can_frame_t *f = &rxq[rxq_h];
        f->ext = (w0 >> 30) & 1u;
        f->rtr = (w0 >> 29) & 1u;
        f->id  = f->ext ? (w0 & 0x1FFFFFFFu) : ((w0 >> 18) & 0x7FFu);
        f->len = (uint8_t)((w1 >> 16) & 0xFu);
        if (f->len > 8u) f->len = 8u;
        memcpy(&f->data[0], &d0, 4); memcpy(&f->data[4], &d1, 4);
        rxq_h = nx;
        st.rx++;
    }
}

static bool can_recv(can_frame_t *f)
{
    if (rxq_t == rxq_h) return false;
    *f = rxq[rxq_t];
    rxq_t = (rxq_t + 1u) % RXQ_N;
    return true;
}

bool can_tx_free(void) { return st.running && !st.bus_off && !(FDCAN2->TXFQS & FDCAN_TXFQS_TFQF); }

bool can_send(const can_frame_t *f)
{
    if (!st.running || st.bus_off) return false;
    if (FDCAN2->TXFQS & FDCAN_TXFQS_TFQF) { st.tx_drop++; return false; }
    uint32_t pi = (FDCAN2->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;
    volatile uint32_t *e = (volatile uint32_t *)(MRAM_TXB + pi * ELEM_BYTES);
    uint32_t d0 = 0, d1 = 0;
    uint8_t n = f->len > 8u ? 8u : f->len;
    uint8_t b[8] = {0};
    memcpy(b, f->data, n);
    memcpy(&d0, &b[0], 4); memcpy(&d1, &b[4], 4);
    e[0] = (f->ext ? ((1u << 30) | (f->id & 0x1FFFFFFFu)) : ((f->id & 0x7FFu) << 18)) | (f->rtr ? (1u << 29) : 0u);
    e[1] = (uint32_t)n << 16;
    e[2] = d0; e[3] = d1;
    FDCAN2->TXBAR = 1u << pi;
    st.tx++;
    return true;
}

/* ------------------------------------------------------------------ settings */
can_cfg_t *can_cfg(void) { return &cfg; }
const can_stat_t *can_stat(void) { return &st; }

void can_restart(void)
{
    fdcan_stop();
    rxq_t = rxq_h;
    if (cfg.enable) fdcan_start();
}

void can_init(void)
{
    if (!cfg_load(&cfg)) cfg_defaults(&cfg);
    if (cfg.enable) fdcan_start();
}

bool can_cfg_set(const char *n, float v)
{
    uint32_t u = (v < 0.0f) ? 0u : (uint32_t)(v + 0.5f);
    bool restart = false;
    if (!strcmp(n, "enable"))       { if (u > 1u) return false; cfg.enable = (uint8_t)u; restart = true; }
    else if (!strcmp(n, "node"))    { if (u > 63u) return false; cfg.node_id = (uint8_t)u; restart = true; }
    else if (!strcmp(n, "kbps"))    { if (u != 125u && u != 250u && u != 500u && u != 1000u) return false; cfg.kbps = (uint16_t)u; restart = true; }
    else if (!strcmp(n, "timeout")) { if (u > 60000u) return false; cfg.timeout_ms = (uint16_t)u; }
    else if (!strcmp(n, "loss"))    { if (u > 1u) return false; cfg.loss_brake = (uint8_t)u; }
    else {
        uint16_t *r = !strcmp(n, "hb") ? &cfg.r_hb : !strcmp(n, "enc") ? &cfg.r_enc : !strcmp(n, "iq") ? &cfg.r_iq :
                      !strcmp(n, "temp") ? &cfg.r_temp : !strcmp(n, "bus") ? &cfg.r_bus : !strcmp(n, "torque") ? &cfg.r_torque :
                      !strcmp(n, "power") ? &cfg.r_power : !strcmp(n, "err") ? &cfg.r_err : !strcmp(n, "ext") ? &cfg.r_ext : NULL;
        if (!r || u > 60000u || (u != 0u && u < 2u)) return false;
        *r = (uint16_t)u;
    }
    if (restart) can_restart();
    return true;
}

/* Dual-bank mode (default on the G474): 2 KB pages, 0x0807D000 is page 122 of bank 2. Single-bank: 4 KB page 125. */
flash_res_t can_cfg_save(void)
{
    can_cfg_t s = cfg;
    s.magic = CFG_MAGIC; s.version = CFG_VERSION;
    s.crc = params_crc32(&s, offsetof(can_cfg_t, crc));
    if (!flash_guard(FLASH_PG_CANCFG)) return FLASH_RES_LIMIT;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    FLASH_EraseInitTypeDef er;
    uint32_t page_error = 0;
    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    bool dual = (FLASH->OPTR & FLASH_OPTR_DBANK) != 0u;
    er.Banks = dual ? FLASH_BANK_2 : FLASH_BANK_1;
    er.Page = dual ? 122u : 125u;
    er.NbPages = 1;
    if (HAL_FLASHEx_Erase(&er, &page_error) != HAL_OK) { HAL_FLASH_Lock(); return FLASH_RES_ERASE; }
    const uint8_t *src = (const uint8_t *)&s;
    for (uint32_t i = 0; i < sizeof(s); i += 8) {
        uint64_t dw;
        memcpy(&dw, src + i, 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAN_CFG_FLASH_ADDR + i, dw) != HAL_OK) { HAL_FLASH_Lock(); return FLASH_RES_PROGRAM; }
    }
    HAL_FLASH_Lock();
    return memcmp((const void *)CAN_CFG_FLASH_ADDR, &s, sizeof(s)) == 0 ? FLASH_RES_OK : FLASH_RES_VERIFY;
}

void can_print(void (*pr)(const char *fmt, ...))
{
    pr("CAN %s, node %u, %u kbit/s, timeout %u ms, on loss: %s, state %s%s, TEC %u REC %u\r\n",
       cfg.enable ? "on" : "off", (unsigned)cfg.node_id, (unsigned)cfg.kbps, (unsigned)cfg.timeout_ms,
       cfg.loss_brake ? "brake" : "coast", st.running ? "running" : "stopped",
       st.bus_off ? " BUS-OFF" : (st.err_passive ? " error-passive (no other node / no ACK?)" : ""), (unsigned)st.tec, (unsigned)st.rec);
    pr("rates ms (0 = off): hb %u enc %u iq %u temp %u bus %u torque %u power %u err %u ext %u\r\n",
       (unsigned)cfg.r_hb, (unsigned)cfg.r_enc, (unsigned)cfg.r_iq, (unsigned)cfg.r_temp, (unsigned)cfg.r_bus,
       (unsigned)cfg.r_torque, (unsigned)cfg.r_power, (unsigned)cfg.r_err, (unsigned)cfg.r_ext);
    pr("frames rx %lu tx %lu, dropped tx %lu rx %lu, bus-off %lu\r\n", (unsigned long)st.rx, (unsigned long)st.tx,
       (unsigned long)st.tx_drop, (unsigned long)st.rx_drop, (unsigned long)st.busoff);
    odc_print(pr);
}

/* ------------------------------------------------------------------ main loop */
void can_task(void)
{
    if (!st.running) return;
    uint32_t psr = FDCAN2->PSR, ecr = FDCAN2->ECR;
    st.tec = (uint8_t)(ecr & 0xFFu);
    st.rec = (uint8_t)((ecr >> 8) & 0x7Fu);
    st.err_passive = (psr & FDCAN_PSR_EP) != 0u;
    bool bo = (psr & FDCAN_PSR_BO) != 0u;
    if (bo && !st.bus_off) st.busoff++;
    st.bus_off = bo;
    if (bo && (FDCAN2->CCCR & FDCAN_CCCR_INIT)) FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;      /* bus-off recovery (128 x 11 recessive bits) */

    can_frame_t f;
    while (can_recv(&f)) {
        st.last_rx_ms = HAL_GetTick();
        odc_rx(&f);
    }
    odc_task(HAL_GetTick());
}

#else   /* ESC_CAN_ENABLE == 0: empty build, behaviour exactly as before CAN */
void can_init(void) { }
void can_task(void) { }
#endif
