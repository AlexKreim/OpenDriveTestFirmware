#include "esc_flash.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* minimum interval (ms) and maximum writes per boot for each page */
static const uint32_t guard_min_ms[FLASH_PG_N]  = { 500u, 10000u, 500u };
static const uint32_t guard_max_n[FLASH_PG_N]   = { 100u, 50u, 50u };
static uint32_t guard_last[FLASH_PG_N];
static uint32_t guard_n[FLASH_PG_N];

bool flash_guard(flash_page_t pg)
{
    uint32_t now = HAL_GetTick();
    if (guard_n[pg] >= guard_max_n[pg]) return false;
    if (guard_n[pg] > 0u && (now - guard_last[pg]) < guard_min_ms[pg]) return false;
    guard_n[pg]++;
    guard_last[pg] = now;
    return true;
}

uint32_t flash_writes(flash_page_t pg) { return guard_n[pg]; }

const char *flash_res_name(flash_res_t r)
{
    switch (r) {
    case FLASH_RES_OK:      return "ok";
    case FLASH_RES_ERASE:   return "flash_erase";
    case FLASH_RES_PROGRAM: return "flash_program";
    case FLASH_RES_VERIFY:  return "flash_verify";
    default:                return "flash_limit";
    }
}

bool flash_image_load(flash_image_t *img)
{
    esc_saved_t s;
    uint32_t flags = 0;
    esc_params_t p;
    float off[3];
    memset(img, 0, sizeof(*img));
    memcpy(&s, (const void *)PARAM_FLASH_ADDR, sizeof(s));       /* erased flash reads 0xFF and fails the magic check */
    if (!params_unpack(&s, &flags, &p, off)) return false;
    img->valid = true;
    if (flags & SAVED_FLAG_PAR) { img->par_valid = true; img->par = p; }
    if (flags & SAVED_FLAG_CAL) { img->cal_valid = true; memcpy(img->cs_offset, off, sizeof(off)); }
    return true;
}

static flash_res_t write_image(const flash_image_t *img)
{
    esc_saved_t s;
    uint32_t flags = (img->par_valid ? SAVED_FLAG_PAR : 0u) | (img->cal_valid ? SAVED_FLAG_CAL : 0u);
    params_pack(&s, flags, &img->par, img->cs_offset);
    if (!flash_guard(FLASH_PG_PARAMS)) return FLASH_RES_LIMIT;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    /* Dual-bank mode (default on the G474): 2 KB pages, the address is page 127 of bank 2.
     * Single-bank mode: 4 KB pages, the address lies in page 127 of bank 1 (0x0807F000..0x0807FFFF, kept free). */
    FLASH_EraseInitTypeDef e;
    uint32_t page_error = 0;
    memset(&e, 0, sizeof(e));
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks     = (FLASH->OPTR & FLASH_OPTR_DBANK) ? FLASH_BANK_2 : FLASH_BANK_1;
    e.Page      = 127;
    e.NbPages   = 1;
    if (HAL_FLASHEx_Erase(&e, &page_error) != HAL_OK) { HAL_FLASH_Lock(); return FLASH_RES_ERASE; }

    const uint8_t *src = (const uint8_t *)&s;
    for (uint32_t i = 0; i < sizeof(s); i += 8) {
        uint64_t dw;
        memcpy(&dw, src + i, 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PARAM_FLASH_ADDR + i, dw) != HAL_OK) {
            HAL_FLASH_Lock();
            return FLASH_RES_PROGRAM;
        }
    }
    HAL_FLASH_Lock();

    return (memcmp((const void *)PARAM_FLASH_ADDR, &s, sizeof(s)) == 0) ? FLASH_RES_OK : FLASH_RES_VERIFY;
}

flash_res_t flash_save_params(const esc_params_t *p)
{
    flash_image_t img;
    flash_image_load(&img);              /* keeps the offsets, an unreadable image just starts empty */
    img.valid = true;
    img.par_valid = true;
    img.par = *p;
    return write_image(&img);
}

flash_res_t flash_save_offsets(const float off[3])
{
    flash_image_t img;
    flash_image_load(&img);              /* keeps the parameters */
    img.valid = true;
    img.cal_valid = true;
    memcpy(img.cs_offset, off, sizeof(img.cs_offset));
    return write_image(&img);
}
