#include "ospi_flash.h"

#include <string.h>

#include "hal_data.h"

#define OSPI_FLASH_READ_JEDEC_ID_COMMAND (0x9FU)
#define OSPI_FLASH_EXPECTED_JEDEC_ID      (0xEF4019U)
#define OSPI_FLASH_CPU_WRITE_ALIGNMENT    (8U)
#define OSPI_FLASH_BUSY_TIMEOUT_MS        (10000U)

static ospi_flash_status_t g_ospi_flash_status = OSPI_FLASH_STATUS_NOT_INITIALIZED;
static uint32_t g_ospi_flash_jedec_id;
static uint32_t g_ospi_flash_jedec_raw;

static bool ospi_flash_range_is_valid(uint32_t offset, size_t size)
{
    return (size <= OSPI_FLASH_CAPACITY) &&
           (offset <= (OSPI_FLASH_CAPACITY - size));
}

static void ospi_flash_invalidate_cache(uint32_t offset, size_t size)
{
    uintptr_t const start = (OSPI_FLASH_MEMORY_BASE + offset) & ~(uintptr_t) 31U;
    uintptr_t const end = (OSPI_FLASH_MEMORY_BASE + offset + size + 31U) & ~(uintptr_t) 31U;

    if (end > start)
    {
        SCB_InvalidateDCache_by_Addr((void *) start, (int32_t) (end - start));
    }
}

static bool ospi_flash_wait_ready(void)
{
    spi_flash_status_t status = {0};

    for (uint32_t elapsed_ms = 0U; elapsed_ms < OSPI_FLASH_BUSY_TIMEOUT_MS; elapsed_ms++)
    {
        if (FSP_SUCCESS != g_ospi0.p_api->statusGet(g_ospi0.p_ctrl, &status))
        {
            return false;
        }

        if (!status.write_in_progress)
        {
            return true;
        }

        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }

    return false;
}

static uint32_t ospi_flash_normalize_jedec_id(uint32_t raw)
{
    uint32_t const low_24 = raw & 0x00FFFFFFU;
    uint32_t const high_24 = (raw >> 8U) & 0x00FFFFFFU;

    if ((OSPI_FLASH_EXPECTED_JEDEC_ID == low_24) ||
        (OSPI_FLASH_EXPECTED_JEDEC_ID == high_24))
    {
        return OSPI_FLASH_EXPECTED_JEDEC_ID;
    }

    if ((0x1940EFU == low_24) || (0x1940EFU == high_24))
    {
        return OSPI_FLASH_EXPECTED_JEDEC_ID;
    }

    return low_24;
}

bool ospi_flash_init(void)
{
    spi_flash_direct_transfer_t transfer =
    {
        .data_u64 = 0U,
        .address = 0U,
        .command = OSPI_FLASH_READ_JEDEC_ID_COMMAND,
        .dummy_cycles = 0U,
        .command_length = 1U,
        .address_length = 0U,
        .data_length = 3U,
    };
    fsp_err_t err;

    if (OSPI_FLASH_STATUS_NOT_INITIALIZED != g_ospi_flash_status)
    {
        return OSPI_FLASH_STATUS_READY == g_ospi_flash_status;
    }

    err = g_ospi0.p_api->open(g_ospi0.p_ctrl, g_ospi0.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_ospi_flash_status = OSPI_FLASH_STATUS_OPEN_FAILED;
        return false;
    }

    err = g_ospi0.p_api->directTransfer(g_ospi0.p_ctrl,
                                        &transfer,
                                        SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
    if (FSP_SUCCESS != err)
    {
        g_ospi_flash_status = OSPI_FLASH_STATUS_ID_READ_FAILED;
        return false;
    }

    g_ospi_flash_jedec_raw = transfer.data;
    g_ospi_flash_jedec_id = ospi_flash_normalize_jedec_id(g_ospi_flash_jedec_raw);
    if (OSPI_FLASH_EXPECTED_JEDEC_ID != g_ospi_flash_jedec_id)
    {
        g_ospi_flash_status = OSPI_FLASH_STATUS_ID_MISMATCH;
        return false;
    }

    g_ospi_flash_status = OSPI_FLASH_STATUS_READY;
    return true;
}

bool ospi_flash_is_ready(void)
{
    return OSPI_FLASH_STATUS_READY == g_ospi_flash_status;
}

uint8_t const * ospi_flash_mapped_address(uint32_t offset)
{
    if (!ospi_flash_is_ready() || !ospi_flash_range_is_valid(offset, 1U))
    {
        return NULL;
    }

    return (uint8_t const *) (uintptr_t) (OSPI_FLASH_MEMORY_BASE + offset);
}

bool ospi_flash_read(uint32_t offset, void * data, size_t size)
{
    if (!ospi_flash_is_ready() || (NULL == data) ||
        !ospi_flash_range_is_valid(offset, size))
    {
        return false;
    }

    if (0U == size)
    {
        return true;
    }

    ospi_flash_invalidate_cache(offset, size);
    memcpy(data, (void const *) (uintptr_t) (OSPI_FLASH_MEMORY_BASE + offset), size);
    return true;
}

bool ospi_flash_write(uint32_t offset, void const * data, size_t size)
{
    uint64_t page_words[8];
    uint8_t * const page = (uint8_t *) page_words;
    uint8_t const * source = (uint8_t const *) data;
    uint32_t current_offset = offset;
    size_t remaining = size;
    uint32_t const page_size = g_ospi0_cfg.page_size_bytes;

    if (!ospi_flash_is_ready() || (NULL == data) ||
        !ospi_flash_range_is_valid(offset, size) ||
        (0U == page_size) || (page_size > sizeof(page_words)) ||
        (0U != (offset % OSPI_FLASH_CPU_WRITE_ALIGNMENT)) ||
        (0U != (size % OSPI_FLASH_CPU_WRITE_ALIGNMENT)))
    {
        return false;
    }

    while (remaining > 0U)
    {
        uint32_t const page_offset = current_offset % page_size;
        size_t chunk = page_size - page_offset;
        if (chunk > remaining)
        {
            chunk = remaining;
        }

        memcpy(page, source, chunk);
        if (!ospi_flash_wait_ready() ||
            (FSP_SUCCESS != g_ospi0.p_api->write(g_ospi0.p_ctrl,
                                                  page,
                                                  (uint8_t *) (uintptr_t) (OSPI_FLASH_MEMORY_BASE + current_offset),
                                                  (uint32_t) chunk)))
        {
            return false;
        }

        current_offset += (uint32_t) chunk;
        source += chunk;
        remaining -= chunk;
    }

    if (!ospi_flash_wait_ready())
    {
        return false;
    }

    ospi_flash_invalidate_cache(offset, size);
    return true;
}

bool ospi_flash_erase(uint32_t offset, size_t size)
{
    uint32_t current_offset = offset;
    size_t remaining = size;

    if (!ospi_flash_is_ready() || !ospi_flash_range_is_valid(offset, size) ||
        (0U != (offset % OSPI_FLASH_SECTOR_SIZE)) ||
        (0U != (size % OSPI_FLASH_SECTOR_SIZE)))
    {
        return false;
    }

    while (remaining > 0U)
    {
        if (!ospi_flash_wait_ready() ||
            (FSP_SUCCESS != g_ospi0.p_api->erase(g_ospi0.p_ctrl,
                                                  (uint8_t *) (uintptr_t) (OSPI_FLASH_MEMORY_BASE + current_offset),
                                                  OSPI_FLASH_SECTOR_SIZE)))
        {
            return false;
        }

        if (!ospi_flash_wait_ready())
        {
            return false;
        }

        current_offset += OSPI_FLASH_SECTOR_SIZE;
        remaining -= OSPI_FLASH_SECTOR_SIZE;
    }

    ospi_flash_invalidate_cache(offset, size);
    return true;
}

uint32_t ospi_flash_jedec_id(void)
{
    return g_ospi_flash_jedec_id;
}

uint32_t ospi_flash_jedec_raw(void)
{
    return g_ospi_flash_jedec_raw;
}

ospi_flash_status_t ospi_flash_status(void)
{
    return g_ospi_flash_status;
}
