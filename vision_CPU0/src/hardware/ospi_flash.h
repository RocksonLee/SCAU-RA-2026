#ifndef OSPI_FLASH_H
#define OSPI_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum e_ospi_flash_status
{
    OSPI_FLASH_STATUS_NOT_INITIALIZED = 0,
    OSPI_FLASH_STATUS_READY,
    OSPI_FLASH_STATUS_OPEN_FAILED,
    OSPI_FLASH_STATUS_ID_READ_FAILED,
    OSPI_FLASH_STATUS_ID_MISMATCH,
} ospi_flash_status_t;

#define OSPI_FLASH_MEMORY_BASE  (0x80000000UL)
#define OSPI_FLASH_CAPACITY     (32UL * 1024UL * 1024UL)
#define OSPI_FLASH_SECTOR_SIZE  (4096UL)

bool ospi_flash_init(void);
bool ospi_flash_is_ready(void);
bool ospi_flash_read(uint32_t offset, void * data, size_t size);
bool ospi_flash_write(uint32_t offset, void const * data, size_t size);
bool ospi_flash_erase(uint32_t offset, size_t size);
uint8_t const * ospi_flash_mapped_address(uint32_t offset);
uint32_t ospi_flash_jedec_id(void);
uint32_t ospi_flash_jedec_raw(void);
ospi_flash_status_t ospi_flash_status(void);

#ifdef __cplusplus
}
#endif

#endif /* OSPI_FLASH_H */
