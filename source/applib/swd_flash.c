#include "swd_flash.h"
#include "main_config.h"

typedef enum {
    STATE_CLOSED,
    STATE_OPEN,
    STATE_ERROR
} state_t;

//! Additional flash and ram regions
#define MAX_REGIONS (10)

//! @brief Option flags for memory regions.
enum _region_flags {
    kRegionIsDefault = (1 << 0), /*!< Out of bounds regions will use the same flash algo if this is set */
    kRegionIsSecure  = (1 << 1), /*!< The region can only be accessed from the secure world. Only applies for TrustZone-enabled targets. */
};

/*!
 * @brief Details of a target flash or RAM memory region.
 */
typedef struct __attribute__((__packed__)) region_info {
    uint32_t start;                 /*!< Region start address. */
    uint32_t end;                   /*!< Region end address. */
    uint32_t flags;                 /*!< Flags for this region from the #_region_flags enumeration. */
    uint32_t alias_index;           /*!< Use with flags; will point to a different index if there is an alias region */
    program_target_t *flash_algo;   /*!< A pointer to the flash algorithm structure */
} region_info_t;

/*!
 * @brief Information required to program target flash.
 */
typedef struct __attribute__((__packed__)) _target_cfg {
    uint32_t version;                           /*!< Target configuration version */
    const sector_info_t* sectors_info;          /*!< Sector start and length list */
    uint32_t sector_info_length;                /*!< Number of entries in the sectors_info array */
    region_info_t flash_regions[MAX_REGIONS];   /*!< Flash regions */
    region_info_t ram_regions[MAX_REGIONS];     /*!< RAM regions  */
    const char *rt_board_id;                    /*!< If assigned, this is a flexible board ID */
    uint16_t rt_family_id;                      /*!< If assigned, this is a flexible family ID */
    uint8_t erase_reset;                        /*!< Reset after performing an erase */
    uint8_t pad;
} target_cfg_t;


static const sector_info_t sectors_info[] = {
    {0x08000000, FLASH_PAGE_SIZE},
};

static const uint32_t STM32F103RB_flash_prog_blob[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x4603b510, 0x4c442000, 0x48446020, 0x48446060, 0x46206060, 0xf01069c0, 0xd1080f04, 0x5055f245,
    0x60204c40, 0x60602006, 0x70fff640, 0x200060a0, 0x4601bd10, 0x69004838, 0x0080f040, 0x61104a36,
    0x47702000, 0x69004834, 0x0004f040, 0x61084932, 0x69004608, 0x0040f040, 0xe0036108, 0x20aaf64a,
    0x60084930, 0x68c0482c, 0x0f01f010, 0x482ad1f6, 0xf0206900, 0x49280004, 0x20006108, 0x46014770,
    0x69004825, 0x0002f040, 0x61104a23, 0x61414610, 0xf0406900, 0x61100040, 0xf64ae003, 0x4a2120aa,
    0x481d6010, 0xf01068c0, 0xd1f60f01, 0x6900481a, 0x0002f020, 0x61104a18, 0x47702000, 0x4603b510,
    0xf0201c48, 0xe0220101, 0x69004813, 0x0001f040, 0x61204c11, 0x80188810, 0x480fbf00, 0xf01068c0,
    0xd1fa0f01, 0x6900480c, 0x0001f020, 0x61204c0a, 0x68c04620, 0x0f14f010, 0x4620d006, 0xf04068c0,
    0x60e00014, 0xbd102001, 0x1c921c9b, 0x29001e89, 0x2000d1da, 0x0000e7f7, 0x40022000, 0x45670123,
    0xcdef89ab, 0x40003000, 0x00000000
};

static const program_target_t flash = {
    0x20000021, // Init
    0x20000053, // UnInit
    0x20000065, // EraseChip
    0x2000009f, // EraseSector
    0x200000dd, // ProgramPage
    0x0,        // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000148,
        0x20000800
    },

    0x20000000 + 0x00000A00,  // mem buffer location
    0x20000000,               // location to write prog_blob in target RAM
    sizeof(STM32F103RB_flash_prog_blob),   // prog_blob size
    STM32F103RB_flash_prog_blob,           // address of prog_blob
    0x00000400       // ram_to_flash_bytes_to_be_written
};

target_cfg_t target_device = {
    .sectors_info                   = sectors_info,
    .sector_info_length             = (sizeof(sectors_info))/(sizeof(sector_info_t)),
    .flash_regions[0].start         = 0x08000000,
    .flash_regions[0].end           = 0x08020000,
    .flash_regions[0].flags         = kRegionIsDefault,
    .flash_regions[0].flash_algo    = (program_target_t *) &flash,    
		.flash_regions[1].start         = 0, // 
    .flash_regions[1].end           = 0, // 
    .ram_regions[0].start           = 0x20000000,
    .ram_regions[0].end             = 0x20005000,
};

target_cfg_t * target_cfg = &target_device;

int target_flash_auto_rst = 1;
int target_flash_automation_allowed = 1; // Verify data flashed if in automation mode



/*
const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .board_id = "0700",
    .family_id = kStub_HWReset_FamilyID,
    .target_cfg = &target_device,
};
*/



static error_t target_flash_init(void);
static error_t target_flash_uninit(void);
static error_t target_flash_program_page(uint32_t adr, const uint8_t *buf, uint32_t size);
static error_t target_flash_erase_sector(uint32_t addr);
static error_t target_flash_erase_chip(void);
static uint32_t target_flash_program_page_min_size(uint32_t addr);
static uint32_t target_flash_erase_sector_size(uint32_t addr);
static uint8_t target_flash_busy(void);
static error_t target_flash_set(uint32_t addr);

static const flash_intf_t flash_intf = {
    target_flash_init,
    target_flash_uninit,
    target_flash_program_page,
    target_flash_erase_sector,
    target_flash_erase_chip,
    target_flash_program_page_min_size,
    target_flash_erase_sector_size,
    target_flash_busy,
    target_flash_set,
};

const flash_intf_t *const g_swd_flash_intf = &flash_intf;

static state_t state = STATE_CLOSED;

static flash_func_t last_flash_func = FLASH_FUNC_NOP;

//saved flash algo
static program_target_t * current_flash_algo = NULL;

//saved default region for default flash algo
static region_info_t * default_region = NULL;

//saved flash start from flash algo
static uint32_t flash_start = 0;


static program_target_t * get_flash_algo(uint32_t addr)
{
    region_info_t * flash_region = target_cfg->flash_regions;

    for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
        if (addr >= flash_region->start && addr <= flash_region->end) {
            flash_start = flash_region->start; //save the flash start
            if (flash_region->flash_algo) {
                return flash_region->flash_algo;
            }else{
                return NULL;
            }
        }
    }

    //could not find a flash algo for the region; use default
    if (default_region) {
        flash_start = default_region->start;
        return default_region->flash_algo;
    } else {
        return NULL;
    }
}

static error_t flash_func_start(flash_func_t func)
{
    program_target_t * flash = current_flash_algo;

    if (last_flash_func != func)
    {
        // Finish the currently active function.
        if (FLASH_FUNC_NOP != last_flash_func &&
            0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->uninit, last_flash_func, 0, 0, 0)) {
            return ERROR_UNINIT;
        }

        // Start a new function.
        if (FLASH_FUNC_NOP != func &&
            0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->init, flash_start, 0, func, 0)) {
            return ERROR_INIT;
        }

        last_flash_func = func;
    }

    return ERROR_SUCCESS;
}

static error_t target_flash_set(uint32_t addr)
{
    program_target_t * new_flash_algo = get_flash_algo(addr);
    if (new_flash_algo == NULL) {
        return ERROR_ALGO_MISSING;
    }
    if(current_flash_algo != new_flash_algo){
        //run uninit to last func
        error_t status = flash_func_start(FLASH_FUNC_NOP);
        if (status != ERROR_SUCCESS) {
            return status;
        }
        // Download flash programming algorithm to target
        if (0 == swd_write_memory(new_flash_algo->algo_start, (uint8_t *)new_flash_algo->algo_blob, new_flash_algo->algo_size)) {
            return ERROR_ALGO_DL;
        }

        current_flash_algo = new_flash_algo;

    }
    return ERROR_SUCCESS;
}

static error_t target_flash_init()
{
    if (target_cfg) {
        last_flash_func = FLASH_FUNC_NOP;

        current_flash_algo = NULL;

			//JK: using my target_set_state
        if (0 == swd_set_target_state(RESET_PROGRAM)) {
            return ERROR_RESET;
        }

        //get default region
        region_info_t * flash_region = target_cfg->flash_regions;
        for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
            if (flash_region->flags & kRegionIsDefault) {
                default_region = flash_region;
                break;
            }
        }

        state = STATE_OPEN;
        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }

}

static error_t target_flash_uninit(void)
{
    if (target_cfg) {
        error_t status = flash_func_start(FLASH_FUNC_NOP);
        if (status != ERROR_SUCCESS) {
            return status;
        }
        if ( target_flash_auto_rst == 1) {
            // Resume the target if configured to do so
					//JK: replace target_set_state with my swd_set_target_state
            swd_set_target_state(RESET_RUN);
        } else {
            // Leave the target halted until a reset occurs
					//JK: replace target_set_state with my swd_set_target_state
            swd_set_target_state(RESET_PROGRAM);
        }
        // Check to see if anything needs to be done after programming.
        // This is usually a no-op for most targets.
				//JK: replace target_set_state with my swd_set_target_state
        swd_set_target_state(POST_FLASH_RESET);

        state = STATE_CLOSED;
        swd_off();
        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_program_page(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    if (target_cfg) {
        error_t status = ERROR_SUCCESS;
        program_target_t * flash = current_flash_algo;

        if (!flash) {
            return ERROR_INTERNAL;
        }

        // check if security bits were set
				/* //JK: no security bits setting now
        if (g_target_family && g_target_family->security_bits_set){
            if (1 == g_target_family->security_bits_set(addr, (uint8_t *)buf, size)) {
                return ERROR_SECURITY_BITS;
            }
        }
				*/

        status = flash_func_start(FLASH_FUNC_PROGRAM);

        if (status != ERROR_SUCCESS) {
            return status;
        }

        while (size > 0) {
						uint32_t write_size = (size < flash->program_buffer_size)? size:flash->program_buffer_size ;

            // Write page to buffer
            if (!swd_write_memory(flash->program_buffer, (uint8_t *)buf, write_size)) {
                return ERROR_ALGO_DATA_SEQ;
            }

            // Run flash programming
            if (!swd_flash_syscall_exec(&flash->sys_call_s,
                                        flash->program_page,
                                        addr,
                                        write_size,
                                        flash->program_buffer,
                                        0)) {
                return ERROR_WRITE;
            }

            if ( target_flash_automation_allowed == 1 ) {
                // Verify data flashed if in automation mode
                if (flash->verify != 0) {
                    status = flash_func_start(FLASH_FUNC_VERIFY);
                    if (status != ERROR_SUCCESS) {
                        return status;
                    }
                    if (!swd_flash_syscall_exec(&flash->sys_call_s,
                                        flash->verify,
                                        addr,
                                        write_size,
                                        flash->program_buffer,
                                        0)) {
                        return ERROR_WRITE_VERIFY;
                    }
                } else {
                    while (write_size > 0) {
                        uint8_t rb_buf[16];
												uint32_t verify_size = (write_size < sizeof(rb_buf)) ? write_size : sizeof(rb_buf);
                        if (!swd_read_memory(addr, rb_buf, verify_size)) {
                            return ERROR_ALGO_DATA_SEQ;
                        }
                        if (memcmp(buf, rb_buf, verify_size) != 0) {
                            return ERROR_WRITE_VERIFY;
                        }
                        addr += verify_size;
                        buf += verify_size;
                        size -= verify_size;
                        write_size -= verify_size;
                    }
                    continue;
                }
            }
            addr += write_size;
            buf += write_size;
            size -= write_size;

        }

        return ERROR_SUCCESS;

    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_erase_sector(uint32_t addr)
{
    if (target_cfg) {
        error_t status = ERROR_SUCCESS;
        program_target_t * flash = current_flash_algo;

        if (!flash) {
            return ERROR_INTERNAL;
        }

        // Check to make sure the address is on a sector boundary
        if ((addr % target_flash_erase_sector_size(addr)) != 0) {
            return ERROR_ERASE_SECTOR;
        }

        status = flash_func_start(FLASH_FUNC_ERASE);

        if (status != ERROR_SUCCESS) {
            return status;
        }

        if (0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->erase_sector, addr, 0, 0, 0)) {
            return ERROR_ERASE_SECTOR;
        }

        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_erase_chip(void)
{
    if (target_cfg){
        error_t status = ERROR_SUCCESS;
        region_info_t * flash_region = target_cfg->flash_regions;

        for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
            status = target_flash_set(flash_region->start);
            if (status != ERROR_SUCCESS) {
                return status;
            }
            status = flash_func_start(FLASH_FUNC_ERASE);
            if (status != ERROR_SUCCESS) {
                return status;
            }
            if (0 == swd_flash_syscall_exec(&current_flash_algo->sys_call_s, current_flash_algo->erase_chip, 0, 0, 0, 0)) {
                return ERROR_ERASE_ALL;
            }
        }

        // Reset and re-initialize the target after the erase if required
        if (target_cfg->erase_reset) {
            status = target_flash_init();
        }

        return status;
    } else {
        return ERROR_FAILURE;
    }
}

static uint32_t target_flash_program_page_min_size(uint32_t addr)
{
    if (target_cfg){
        uint32_t size = 256;
        if (size > target_flash_erase_sector_size(addr)) {
            size = target_flash_erase_sector_size(addr);
        }
        return size;
    } else {
        return 0;
    }
}

static uint32_t target_flash_erase_sector_size(uint32_t addr)
{
    if (target_cfg){
        if(target_cfg->sector_info_length > 0) {
            int sector_index = target_cfg->sector_info_length - 1;
            for (; sector_index >= 0; sector_index--) {
                if (addr >= target_cfg->sectors_info[sector_index].start) {
                    return target_cfg->sectors_info[sector_index].size;
                }
            }
        }
        //sector information should be in sector_info
				//JK: no BL and vfs, so no util_assert
        //util_assert(0);
        return 0;
    } else {
        return 0;
    }
}

static uint8_t target_flash_busy(void){
    return (state == STATE_OPEN);
}

