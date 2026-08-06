#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "common_data.h"

#include "sub_0003_tensors.h"
#include "sub_0003_command_stream.h"
#include "sub_0003_model_data.h"

#include "sub_0003_invoke.h"

// Include Ethos-U driver headers (Assumed to be available)
#include "ethosu_driver.h"

// Define arenas with allocation and cache-line alignment in external SDRAM
uint8_t sub_0003_arena[kArenaSize_sub_0003]
    BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);
// Fast scratch arena is not separate on Ethos-U55; alias the main Arena.
uint8_t* sub_0003_fast_scratch = sub_0003_arena;

static void sub_0003_clean_dcache(void)
{
  SCB_CleanDCache_by_Addr((uint32_t *) sub_0003_arena,
                          (int32_t) sizeof(sub_0003_arena));
}

static void sub_0003_invalidate_dcache(void)
{
  SCB_InvalidateDCache_by_Addr((void *) sub_0003_arena,
                               (int32_t) sizeof(sub_0003_arena));
}

int sub_0003_invoke(bool clean_outputs) {
  // Initialize base addresses and sizes
  uint64_t base_addrs[6] = {0};
  size_t base_addrs_size[6] = {0};
  int num_base_addrs = 6;

  // Variables for command stream
  uint8_t* cms_data = NULL;
  int cms_size = 0;

  // Prepare base_addrs and base_addrs_size arrays
  // Buffer sub_0003_model with size 46624 and address: 4294967295
  base_addrs[0] = (uint64_t)(uintptr_t)sub_0003_model_data;
  base_addrs_size[0] = sub_0003_model_data_size;
  // Buffer sub_0003_arena with size 63488 and address: 0
  base_addrs[1] = (uint64_t)(uintptr_t) (sub_0003_arena+0);
  base_addrs_size[1] = 63488;

  // Buffer sub_0003_fast_scratch with size 63488 and address: 0
  base_addrs[2] = (uint64_t)(uintptr_t) (sub_0003_arena+0);
  base_addrs_size[2] = 63488;

  // Buffer input_tensor_0 with size 24576 and address: 38912
  base_addrs[3] = (uint64_t)(uintptr_t) (sub_0003_arena+38912);
  base_addrs_size[3] = 24576;

  // Buffer input_tensor_1 with size 6144 and address: 0
  base_addrs[4] = (uint64_t)(uintptr_t) (sub_0003_arena+0);
  base_addrs_size[4] = 6144;

  // Buffer output_tensor_0 with size 6144 and address: 6144
  if (clean_outputs) {
    memset(sub_0003_arena + 6144, 0, 6144);
  }
  base_addrs[5] = (uint64_t)(uintptr_t) (sub_0003_arena+6144);
  base_addrs_size[5] = 6144;

  // Command stream data
  cms_data = (uint8_t*)sub_0003_command_stream;
  cms_size = (int) sub_0003_command_stream_size;

  // Invoke the Ethos-U driver
  if (num_base_addrs > 8) {
    num_base_addrs = 8;
  }
  sub_0003_clean_dcache();
  int result = ethosu_invoke_v3(&g_ethosu0, cms_data, cms_size, base_addrs, base_addrs_size, num_base_addrs, NULL);
  sub_0003_invalidate_dcache();

  if (result == -1) {
    // Ethos-U invocation failed
    return -1;
  }

  return 0;
}
