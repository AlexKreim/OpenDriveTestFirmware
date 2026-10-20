#include "esc_faultlog.h"
#include "esc_motor.h"
#include "esc_params.h"
#include "stm32g4xx_hal.h"
#include <stddef.h>
#include <string.h>

#define FLOG_MAGIC 0x464C4F47UL              /* "FLOG" */

typedef struct {
    uint32_t magic;
    uint16_t boots;                          /* boot number of the last write */
    uint16_t n;
    flog_entry_t e[FLOG_N];
    uint32_t crc;                            /* over everything before it */
    uint32_t pad;                            /* size multiple of 8 (double-word programming) */
} flog_page_t;

_Static_assert((sizeof(flog_page_t) % 8u) == 0u, "flash image must be a multiple of 8 bytes");
_Static_assert(sizeof(flog_page_t) <= 2048u, "flash image must fit in one 2 KB page");

static flog_page_t log_ram;
static uint16_t    boot_no;
static volatile bool dirty;
static uint32_t    dirty_ms;
static int8_t      wr_res;                 /* 0 none this boot, 1 ok, -1 failed */

static void add(const flog_entry_t *x)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (log_ram.n >= FLOG_N) {
        memmove(&log_ram.e[0], &log_ram.e[1], sizeof(flog_entry_t) * (FLOG_N - 1u));
        log_ram.n = FLOG_N - 1u;
    }
    log_ram.e[log_ram.n++] = *x;
    dirty = true;
    dirty_ms = HAL_GetTick();
    if (pm == 0u) __enable_irq();
}

static void snapshot(flog_entry_t *x, uint8_t kind, uint8_t code)
{
    memset(x, 0, sizeof(*x));
    x->uptime_ms = HAL_GetTick();
    x->boot = boot_no;
    x->kind = kind; x->code = code;
    x->state = (uint8_t)g_state; x->mode = (uint8_t)g_mode;
    x->vbus = g_vbus; x->ia = g_i[0]; x->ib = g_i[1]; x->ic = g_i[2];
    x->imag = g_imag; x->rpm = g_rpm_meas; x->iq_ref = g_iq_ref; x->oc_a = g_oc_last_a;
}

void flog_init(int reset_code)
{
    const flog_page_t *f = (const flog_page_t *)FLOG_FLASH_ADDR;
    memset(&log_ram, 0, sizeof(log_ram));
    if (f->magic == FLOG_MAGIC && f->n <= FLOG_N && params_crc32(f, offsetof(flog_page_t, crc)) == f->crc) {
        log_ram = *f;
    }
    boot_no = (uint16_t)(log_ram.boots + 1u);
    if (reset_code != 0) {
        flog_entry_t x;
        snapshot(&x, FLOG_RESET, (uint8_t)reset_code);
        x.uptime_ms = 0;
        add(&x);
    }
}

void flog_fault(int code)
{
    if (g_state == ST_FAULT) return;         /* already latched: keep the first cause */
    flog_entry_t x;
    snapshot(&x, FLOG_FAULT, (uint8_t)code);
    add(&x);
}

void flog_watchdog(void)
{
    flog_entry_t x;
    snapshot(&x, FLOG_WATCHDOG, 0);
    add(&x);
}

int      flog_count(void) { return log_ram.n; }
uint16_t flog_boot(void)  { return boot_no; }

bool flog_get(int i, flog_entry_t *e)
{
    if (i < 0 || i >= (int)log_ram.n) return false;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    *e = log_ram.e[i];
    if (pm == 0u) __enable_irq();
    return true;
}

/* Dual-bank mode (default on the G474): 2 KB pages, 0x0807E000 is page 124 of bank 2.
 * Single-bank mode: 4 KB pages, it is page 126 of bank 1. The parameter page (0x0807F800) is not touched. */
static bool write_page(void)
{
    flog_page_t s;
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s = log_ram;
    dirty = false;
    if (pm == 0u) __enable_irq();
    s.magic = FLOG_MAGIC;
    s.boots = boot_no;
    s.pad = 0;
    s.crc = params_crc32(&s, offsetof(flog_page_t, crc));

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    FLASH_EraseInitTypeDef er;
    uint32_t page_error = 0;
    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    bool dual = (FLASH->OPTR & FLASH_OPTR_DBANK) != 0u;
    er.Banks   = dual ? FLASH_BANK_2 : FLASH_BANK_1;
    er.Page    = dual ? 124u : 126u;
    er.NbPages = 1;
    bool ok = HAL_FLASHEx_Erase(&er, &page_error) == HAL_OK;
    const uint8_t *src = (const uint8_t *)&s;
    for (uint32_t i = 0; ok && i < sizeof(s); i += 8) {
        uint64_t dw;
        memcpy(&dw, src + i, 8);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLOG_FLASH_ADDR + i, dw) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok && memcmp((const void *)FLOG_FLASH_ADDR, &s, sizeof(s)) == 0;
}

void flog_task(bool motor_running)
{
    /* only while stopped (like 'save'), and a moment after the event so the state has settled */
    if (!dirty || motor_running || (HAL_GetTick() - dirty_ms) < 200u) return;
    wr_res = write_page() ? 1 : -1;
}

const char *flog_flash_state(void)
{
    if (dirty) return "pending";
    return wr_res > 0 ? "ok" : (wr_res < 0 ? "fail" : "none");
}

bool flog_clear(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    log_ram.n = 0;
    memset(log_ram.e, 0, sizeof(log_ram.e));
    if (pm == 0u) __enable_irq();
    bool ok = write_page();
    wr_res = ok ? 1 : -1;
    return ok;
}
