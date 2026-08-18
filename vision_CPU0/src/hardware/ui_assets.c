#include "ui_assets.h"

#include <stddef.h>
#include <stdint.h>

#include "ospi_flash.h"

#include "ui_assets_rle.inc"

#define UI_ASSET_HEADER_MAGIC       (0x55494131UL) /* "UIA1" */
#define UI_ASSET_HEADER_VERSION     (1UL)
#define UI_ASSET_HEADER_OFFSET      (0x00000000UL)
#define UI_ASSET_LOGO_OFFSET        (0x00001000UL)
#define UI_ASSET_ROBOT_ARM_OFFSET   (0x00004000UL)
#define UI_ASSET_STORAGE_SIZE       (0x00012000UL)
#define UI_ASSET_WRITE_PAGE_SIZE    (64U)

typedef struct st_ui_asset_header
{
    uint32_t magic;
    uint32_t version;
    uint32_t logo_size;
    uint32_t logo_crc32;
    uint32_t robot_arm_size;
    uint32_t robot_arm_crc32;
    uint32_t storage_size;
    uint32_t reserved;
} ui_asset_header_t;

static bool g_ui_assets_ready;

static const lv_image_dsc_t g_competition_logo_external =
{
    .header.magic = LV_IMAGE_HEADER_MAGIC,
    .header.cf = LV_COLOR_FORMAT_RGB565,
    .header.w = 88,
    .header.h = 57,
    .header.stride = 88 * sizeof(uint16_t),
    .data_size = UI_ASSET_COMPETITION_LOGO_RAW_SIZE,
    .data = (uint8_t const *) (uintptr_t) (OSPI_FLASH_MEMORY_BASE + UI_ASSET_LOGO_OFFSET),
};

static const lv_image_dsc_t g_robot_arm_external =
{
    .header.magic = LV_IMAGE_HEADER_MAGIC,
    .header.cf = LV_COLOR_FORMAT_RGB565,
    .header.w = 168,
    .header.h = 168,
    .header.stride = 168 * sizeof(uint16_t),
    .data_size = UI_ASSET_ROBOT_ARM_RAW_SIZE,
    .data = (uint8_t const *) (uintptr_t) (OSPI_FLASH_MEMORY_BASE + UI_ASSET_ROBOT_ARM_OFFSET),
};

static uint32_t ui_assets_crc32(uint8_t const * data, size_t size)
{
    uint32_t crc = UINT32_MAX;

    for (size_t i = 0U; i < size; i++)
    {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            uint32_t const mask = (uint32_t) (-(int32_t) (crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

static bool ui_assets_header_is_valid(ui_asset_header_t const * header)
{
    return (UI_ASSET_HEADER_MAGIC == header->magic) &&
           (UI_ASSET_HEADER_VERSION == header->version) &&
           (UI_ASSET_COMPETITION_LOGO_RAW_SIZE == header->logo_size) &&
           (UI_ASSET_COMPETITION_LOGO_CRC32 == header->logo_crc32) &&
           (UI_ASSET_ROBOT_ARM_RAW_SIZE == header->robot_arm_size) &&
           (UI_ASSET_ROBOT_ARM_CRC32 == header->robot_arm_crc32) &&
           (UI_ASSET_STORAGE_SIZE == header->storage_size);
}

static bool ui_assets_payload_is_valid(void)
{
    uint8_t const * logo = ospi_flash_mapped_address(UI_ASSET_LOGO_OFFSET);
    uint8_t const * robot_arm = ospi_flash_mapped_address(UI_ASSET_ROBOT_ARM_OFFSET);

    return (NULL != logo) && (NULL != robot_arm) &&
           (UI_ASSET_COMPETITION_LOGO_CRC32 ==
            ui_assets_crc32(logo, UI_ASSET_COMPETITION_LOGO_RAW_SIZE)) &&
           (UI_ASSET_ROBOT_ARM_CRC32 ==
            ui_assets_crc32(robot_arm, UI_ASSET_ROBOT_ARM_RAW_SIZE));
}

static bool ui_assets_write_rle(uint32_t offset,
                                uint8_t const * rle,
                                size_t rle_size,
                                size_t raw_size)
{
    uint64_t page_words[UI_ASSET_WRITE_PAGE_SIZE / sizeof(uint64_t)];
    uint8_t * const page = (uint8_t *) page_words;
    size_t page_used = 0U;
    size_t output_size = 0U;
    uint32_t write_offset = offset;

    if ((0U != (rle_size % 3U)) || (0U != (raw_size % sizeof(uint64_t))))
    {
        return false;
    }

    for (size_t source = 0U; source < rle_size; source += 3U)
    {
        uint32_t const run = rle[source];
        uint8_t const low = rle[source + 1U];
        uint8_t const high = rle[source + 2U];

        if ((0U == run) || ((output_size + (run * 2U)) > raw_size))
        {
            return false;
        }

        for (uint32_t pixel = 0U; pixel < run; pixel++)
        {
            page[page_used++] = low;
            page[page_used++] = high;
            output_size += 2U;

            if (UI_ASSET_WRITE_PAGE_SIZE == page_used)
            {
                if (!ospi_flash_write(write_offset, page, page_used))
                {
                    return false;
                }

                write_offset += UI_ASSET_WRITE_PAGE_SIZE;
                page_used = 0U;
            }
        }
    }

    if ((output_size != raw_size) ||
        ((page_used > 0U) && !ospi_flash_write(write_offset, page, page_used)))
    {
        return false;
    }

    return true;
}

static bool ui_assets_provision(void)
{
    ui_asset_header_t const header =
    {
        .magic = UI_ASSET_HEADER_MAGIC,
        .version = UI_ASSET_HEADER_VERSION,
        .logo_size = UI_ASSET_COMPETITION_LOGO_RAW_SIZE,
        .logo_crc32 = UI_ASSET_COMPETITION_LOGO_CRC32,
        .robot_arm_size = UI_ASSET_ROBOT_ARM_RAW_SIZE,
        .robot_arm_crc32 = UI_ASSET_ROBOT_ARM_CRC32,
        .storage_size = UI_ASSET_STORAGE_SIZE,
        .reserved = UINT32_MAX,
    };

    if (!ospi_flash_erase(UI_ASSET_HEADER_OFFSET, UI_ASSET_STORAGE_SIZE) ||
        !ui_assets_write_rle(UI_ASSET_LOGO_OFFSET,
                             g_competition_logo_rgb565_rle,
                             sizeof(g_competition_logo_rgb565_rle),
                             UI_ASSET_COMPETITION_LOGO_RAW_SIZE) ||
        !ui_assets_write_rle(UI_ASSET_ROBOT_ARM_OFFSET,
                             g_robot_arm_rgb565_rle,
                             sizeof(g_robot_arm_rgb565_rle),
                             UI_ASSET_ROBOT_ARM_RAW_SIZE) ||
        !ui_assets_payload_is_valid())
    {
        return false;
    }

    /* Write the header last so an interrupted update is retried next boot. */
    return ospi_flash_write(UI_ASSET_HEADER_OFFSET, &header, sizeof(header));
}

bool ui_assets_init(void)
{
    ui_asset_header_t const * header;

    if (g_ui_assets_ready)
    {
        return true;
    }

    if (!ospi_flash_is_ready())
    {
        return false;
    }

    header = (ui_asset_header_t const *) ospi_flash_mapped_address(UI_ASSET_HEADER_OFFSET);
    if ((NULL != header) && ui_assets_header_is_valid(header) && ui_assets_payload_is_valid())
    {
        g_ui_assets_ready = true;
        return true;
    }

    g_ui_assets_ready = ui_assets_provision();
    return g_ui_assets_ready;
}

bool ui_assets_is_ready(void)
{
    return g_ui_assets_ready;
}

lv_image_dsc_t const * ui_assets_competition_logo(void)
{
    return g_ui_assets_ready ? &g_competition_logo_external : NULL;
}

lv_image_dsc_t const * ui_assets_robot_arm(void)
{
    return g_ui_assets_ready ? &g_robot_arm_external : NULL;
}
