#ifndef SWD_FLASH_H
#define SWD_FLASH_H

#include "stm32f10x_conf.h"
#include "stm32f10x.h"
//#include "swd_bsp.h"
#include "swd_host.h"

typedef enum {
    /* Shared errors */
    ERROR_SUCCESS = 0,
    ERROR_FAILURE,
    ERROR_INTERNAL,
    /* Target flash errors */
    ERROR_RESET,
    ERROR_ALGO_DL,
    ERROR_ALGO_MISSING,
    ERROR_ALGO_DATA_SEQ,
    ERROR_INIT,
    ERROR_UNINIT,
    ERROR_SECURITY_BITS,
    ERROR_UNLOCK,
    ERROR_ERASE_SECTOR,
    ERROR_ERASE_ALL,
    ERROR_WRITE,
    ERROR_WRITE_VERIFY,
    /* Flash decoder error */
    ERROR_FD_BL_UPDT_ADDR_WRONG,
    ERROR_FD_INTF_UPDT_ADDR_WRONG,
    ERROR_FD_UNSUPPORTED_UPDATE,
    // Add new values here
    ERROR_COUNT
} error_t;

typedef enum {
    FLASH_FUNC_NOP,
    FLASH_FUNC_ERASE,
    FLASH_FUNC_PROGRAM,
    FLASH_FUNC_VERIFY
} flash_func_t;


typedef error_t (*flash_intf_init_cb_t)(void);
typedef error_t (*flash_intf_uninit_cb_t)(void);
typedef error_t (*flash_intf_program_page_cb_t)(uint32_t addr, const uint8_t *buf, uint32_t size);
typedef error_t (*flash_intf_erase_sector_cb_t)(uint32_t sector);
typedef error_t (*flash_intf_erase_chip_cb_t)(void);
typedef uint32_t (*flash_program_page_min_size_cb_t)(uint32_t addr);
typedef uint32_t (*flash_erase_sector_size_cb_t)(uint32_t addr);
typedef uint8_t (*flash_busy_cb_t)(void);
typedef error_t (*flash_algo_set_cb_t)(uint32_t addr);

typedef struct {
    flash_intf_init_cb_t init;
    flash_intf_uninit_cb_t uninit;
    flash_intf_program_page_cb_t program_page;
    flash_intf_erase_sector_cb_t erase_sector;
    flash_intf_erase_chip_cb_t erase_chip;
    flash_program_page_min_size_cb_t program_page_min_size;
    flash_erase_sector_size_cb_t erase_sector_size;
    flash_busy_cb_t flash_busy;
    flash_algo_set_cb_t flash_algo_set;
} flash_intf_t;





extern const flash_intf_t *const g_swd_flash_intf;















#ifdef __cplusplus
}
#endif

#endif
