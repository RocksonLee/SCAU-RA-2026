#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "common_data.h"

#include "sub_0029_tensors.h"
#include "sub_0029_command_stream.h"
#include "sub_0029_model_data.h"

#include "sub_0029_invoke.h"

// Include Ethos-U driver headers (Assumed to be available)
#include "ethosu_driver.h"

// NPU/DMA-shared Arena: external linkage for model.c tensor routing.
uint8_t sub_0029_arena[kArenaSize_sub_0029]
    BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);
// Fast scratch arena not used for Ethos-U55
//  We will not create it for now and reuse the address of the other arena
// __attribute__((aligned(16))) static uint8_t sub_0029_fast_scratch[30720];
uint8_t* sub_0029_fast_scratch = sub_0029_arena;

static void sub_0029_clean_dcache(void) {
  SCB_CleanDCache_by_Addr((uint32_t *) sub_0029_arena,
                          (int32_t) sizeof(sub_0029_arena));
}

static void sub_0029_invalidate_dcache(void) {
  SCB_InvalidateDCache_by_Addr((void *) sub_0029_arena,
                               (int32_t) sizeof(sub_0029_arena));
}

int sub_0029_invoke(bool clean_outputs) {
  // Initialize base addresses and sizes
  uint64_t base_addrs[5] = {0};
  size_t base_addrs_size[5] = {0};
  int num_base_addrs = 5;

  // Variables for command stream
  uint8_t* cms_data = NULL;
  int cms_size = 0;

  // Prepare base_addrs and base_addrs_size arrays
  // Buffer sub_0029_model with size 24560 and address: 4294967295
  base_addrs[0] = (uint64_t)(uintptr_t)sub_0029_model_data;
  base_addrs_size[0] = sub_0029_model_data_size;
  // Buffer sub_0029_arena with size 30720 and address: 0
  base_addrs[1] = (uint64_t)(uintptr_t) (sub_0029_arena+0);
  base_addrs_size[1] = 30720;

  // Buffer sub_0029_fast_scratch with size 30720 and address: 0
  base_addrs[2] = (uint64_t)(uintptr_t) (sub_0029_arena+0);
  base_addrs_size[2] = 30720;

  // Buffer input_tensor_0 with size 12288 and address: 12288
  base_addrs[3] = (uint64_t)(uintptr_t) (sub_0029_arena+12288);
  base_addrs_size[3] = 12288;

  // Buffer output_tensor_0 with size 12288 and address: 12288
  if (clean_outputs) {
    memset(sub_0029_arena + 12288, 0, 12288);
  }
  base_addrs[4] = (uint64_t)(uintptr_t) (sub_0029_arena+12288);
  base_addrs_size[4] = 12288;

  // Command stream data
  cms_data = (uint8_t*)sub_0029_command_stream;
  cms_size = (int) sub_0029_command_stream_size;

  // Invoke the Ethos-U driver
  if (num_base_addrs > 8) {
    num_base_addrs = 8;
  }
  sub_0029_clean_dcache();
  int result = ethosu_invoke_v3(&g_ethosu0, cms_data, cms_size, base_addrs, base_addrs_size, num_base_addrs, NULL);
  sub_0029_invalidate_dcache();

  if (result == -1) {
    // Ethos-U invocation failed
    return -1;
  }

  return 0;
}
