#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "common_data.h"

#include "sub_0001_tensors.h"
#include "sub_0001_command_stream.h"
#include "sub_0001_model_data.h"

#include "sub_0001_invoke.h"

// Include Ethos-U driver headers (Assumed to be available)
#include "ethosu_driver.h"

// Define arenas with allocation and cache-line alignment in external SDRAM
uint8_t sub_0001_arena[kArenaSize_sub_0001]
    BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);
// Fast scratch arena is not separate on Ethos-U55; alias the main Arena.
uint8_t* sub_0001_fast_scratch = sub_0001_arena;

static void sub_0001_clean_dcache(void)
{
  SCB_CleanDCache_by_Addr((uint32_t *) sub_0001_arena,
                          (int32_t) sizeof(sub_0001_arena));
}

static void sub_0001_invalidate_dcache(void)
{
  SCB_InvalidateDCache_by_Addr((void *) sub_0001_arena,
                               (int32_t) sizeof(sub_0001_arena));
}

int sub_0001_invoke(bool clean_outputs) {
  // Initialize base addresses and sizes
  uint64_t base_addrs[7] = {0};
  size_t base_addrs_size[7] = {0};
  int num_base_addrs = 7;

  // Variables for command stream
  uint8_t* cms_data = NULL;
  int cms_size = 0;

  // Prepare base_addrs and base_addrs_size arrays
  // Buffer sub_0001_model with size 387888 and address: 4294967295
  base_addrs[0] = (uint64_t)(uintptr_t)sub_0001_model_data;
  base_addrs_size[0] = sub_0001_model_data_size;
  // Buffer sub_0001_arena with size 786432 and address: 0
  base_addrs[1] = (uint64_t)(uintptr_t) (sub_0001_arena+0);
  base_addrs_size[1] = 786432;

  // Buffer sub_0001_fast_scratch with size 786432 and address: 0
  base_addrs[2] = (uint64_t)(uintptr_t) (sub_0001_arena+0);
  base_addrs_size[2] = 786432;

  // Buffer input_tensor_0 with size 196608 and address: 262144
  base_addrs[3] = (uint64_t)(uintptr_t) (sub_0001_arena+262144);
  base_addrs_size[3] = 196608;

  // Buffer output_tensor_0 with size 6144 and address: 16384
  if (clean_outputs) {
    memset(sub_0001_arena + 16384, 0, 6144);
  }
  base_addrs[4] = (uint64_t)(uintptr_t) (sub_0001_arena+16384);
  base_addrs_size[4] = 6144;

  // Buffer output_tensor_1 with size 6144 and address: 0
  if (clean_outputs) {
    memset(sub_0001_arena + 0, 0, 6144);
  }
  base_addrs[5] = (uint64_t)(uintptr_t) (sub_0001_arena+0);
  base_addrs_size[5] = 6144;

  // Buffer output_tensor_2 with size 1536 and address: 6144
  if (clean_outputs) {
    memset(sub_0001_arena + 6144, 0, 1536);
  }
  base_addrs[6] = (uint64_t)(uintptr_t) (sub_0001_arena+6144);
  base_addrs_size[6] = 1536;

  // Command stream data
  cms_data = (uint8_t*)sub_0001_command_stream;
  cms_size = (int) sub_0001_command_stream_size;

  // Invoke the Ethos-U driver
  if (num_base_addrs > 8) {
    num_base_addrs = 8;
  }
  sub_0001_clean_dcache();
  int result = ethosu_invoke_v3(&g_ethosu0, cms_data, cms_size, base_addrs, base_addrs_size, num_base_addrs, NULL);
  sub_0001_invalidate_dcache();

  if (result == -1) {
    // Ethos-U invocation failed
    return -1;
  }

  return 0;
}
