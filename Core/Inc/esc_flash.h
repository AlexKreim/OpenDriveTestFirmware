#ifndef ESC_FLASH_H
#define ESC_FLASH_H

#include <stdbool.h>
#include "esc_params.h"

/* The parameters and the current-sensor zero offsets live in the last 2 KB of the 512 KB flash (0x0807F800).
 * Reserve that area in the linker script: FLASH LENGTH = 508K (see README). */
#define PARAM_FLASH_ADDR  0x0807F800UL

typedef enum { FLASH_RES_OK = 0, FLASH_RES_ERASE, FLASH_RES_PROGRAM, FLASH_RES_VERIFY } flash_res_t;

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

#endif
