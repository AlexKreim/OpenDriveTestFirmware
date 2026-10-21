#ifndef ESC_FLASH_H
#define ESC_FLASH_H

#include <stdbool.h>
#include "esc_params.h"

/* The parameters and the current-sensor zero offsets live in the last 2 KB of the 512 KB flash (0x0807F800).
 * Reserve that area in the linker script: FLASH LENGTH = 504K (also keeps the fault-log page 0x0807E000 free). */
#define PARAM_FLASH_ADDR  0x0807F800UL

typedef enum { FLASH_RES_OK = 0, FLASH_RES_ERASE, FLASH_RES_PROGRAM, FLASH_RES_VERIFY, FLASH_RES_LIMIT } flash_res_t;

/* Flash wear guard. A page survives about 10 000 erase cycles; a command sent in a loop (USB or CAN) must not wear it out.
 * Every page erase asks first: a minimum time since the last write of that page and a maximum number of writes per boot.
 * Refused writes return FLASH_RES_LIMIT (the data stays in RAM). */
typedef enum { FLASH_PG_PARAMS = 0, FLASH_PG_FAULTLOG, FLASH_PG_CANCFG, FLASH_PG_COG, FLASH_PG_N } flash_page_t;
bool     flash_guard(flash_page_t pg);          /* true: the write may go ahead (counted) */
uint32_t flash_writes(flash_page_t pg);         /* writes of this page since boot */
const char *flash_res_name(flash_res_t r);

typedef struct {
    bool         valid;            /* an intact image is stored */
    bool         par_valid;
    esc_params_t par;
    bool         cal_valid;
    float        cs_offset[3];
} flash_image_t;

bool        flash_image_load(flash_image_t *img);              /* false: nothing valid stored */
flash_res_t flash_save_params(const esc_params_t *p);          /* keeps the stored offsets. Motor must be stopped. */
flash_res_t flash_save_offsets(const float off[3]);            /* keeps the stored parameters. Motor must be stopped. */

/* Anti-cogging map: 0x0807A000..0x0807CFFF (12 KB, header + n int16 values in mA). Motor must be stopped. */
#define COG_FLASH_ADDR  0x0807A000UL
bool        flash_cog_load(int16_t *map, uint32_t n, bool *enabled);   /* false: nothing valid stored (map untouched) */
flash_res_t flash_cog_save(const int16_t *map, uint32_t n, bool enabled);

#endif
