#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "common_data.h"

#include "sub_0033_tensors.h"
#include "sub_0033_command_stream.h"
#include "sub_0033_model_data.h"

#include "sub_0033_invoke.h"

// Include Ethos-U driver headers (Assumed to be available)
#include "ethosu_driver.h"

// NPU/DMA-shared Arena: external linkage for model.c tensor routing.
uint8_t sub_0033_arena[kArenaSize_sub_0033]
    BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);
// Fast scratch arena not used for Ethos-U55
//  We will not create it for now and reuse the address of the other arena
// __attribute__((aligned(16))) static uint8_t sub_0033_fast_scratch[348160];
uint8_t* sub_0033_fast_scratch = sub_0033_arena;

static void sub_0033_clean_dcache(void) {
  SCB_CleanDCache_by_Addr((uint32_t *) sub_0033_arena,
                          (int32_t) sizeof(sub_0033_arena));
}

static void sub_0033_invalidate_dcache(void) {
  SCB_InvalidateDCache_by_Addr((void *) sub_0033_arena,
                               (int32_t) sizeof(sub_0033_arena));
}

int sub_0033_invoke(bool clean_outputs) {
  // Initialize base addresses and sizes
  uint64_t base_addrs[7] = {0};
  size_t base_addrs_size[7] = {0};
  int num_base_addrs = 7;

  // Variables for command stream
  uint8_t* cms_data = NULL;
  int cms_size = 0;

  // Prepare base_addrs and base_addrs_size arrays
  // Buffer sub_0033_model with size 218288 and address: 4294967295
  base_addrs[0] = (uint64_t)(uintptr_t)sub_0033_model_data;
  base_addrs_size[0] = sub_0033_model_data_size;
  // Buffer sub_0033_arena with size 348160 and address: 0
  base_addrs[1] = (uint64_t)(uintptr_t) (sub_0033_arena+0);
  base_addrs_size[1] = 348160;

  // Buffer sub_0033_fast_scratch with size 348160 and address: 0
  base_addrs[2] = (uint64_t)(uintptr_t) (sub_0033_arena+0);
  base_addrs_size[2] = 348160;

  // Buffer input_tensor_0 with size 12288 and address: 0
  base_addrs[3] = (uint64_t)(uintptr_t) (sub_0033_arena+0);
  base_addrs_size[3] = 12288;

  // Buffer input_tensor_1 with size 24576 and address: 69632
  base_addrs[4] = (uint64_t)(uintptr_t) (sub_0033_arena+69632);
  base_addrs_size[4] = 24576;

  // Buffer input_tensor_2 with size 49152 and address: 20480
  base_addrs[5] = (uint64_t)(uintptr_t) (sub_0033_arena+20480);
  base_addrs_size[5] = 49152;

  // Buffer output_tensor_0 with size 47040 and address: 67520
  if (clean_outputs) {
    memset(sub_0033_arena + 67520, 0, 47040);
  }
  base_addrs[6] = (uint64_t)(uintptr_t) (sub_0033_arena+67520);
  base_addrs_size[6] = 47040;

  // Command stream data
  cms_data = (uint8_t*)sub_0033_command_stream;
  cms_size = (int) sub_0033_command_stream_size;

  // Invoke the Ethos-U driver
  if (num_base_addrs > 8) {
    num_base_addrs = 8;
  }
  sub_0033_clean_dcache();
  int result = ethosu_invoke_v3(&g_ethosu0, cms_data, cms_size, base_addrs, base_addrs_size, num_base_addrs, NULL);
  sub_0033_invalidate_dcache();

  if (result == -1) {
    // Ethos-U invocation failed
    return -1;
  }

  return 0;
}
