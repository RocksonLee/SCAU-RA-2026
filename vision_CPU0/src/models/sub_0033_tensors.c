#include "sub_0033_tensors.h"

const TensorInfo sub_0033_tensors[] = {
  { "_split_1_command_stream", 3, 11440, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 4, 218288, "MODEL", 0xffffffff },
  { "_split_1_scratch", 5, 348160, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 6, 348160, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage4_stage4_3_Reshape_1_output_0_70500_71065_11984_70659", 2, 12288, "INPUT_TENSOR", 0x0 },
  { "_backbone_stage3_stage3_7_Reshape_1_output_0_70449_71064_11924_70593", 1, 24576, "INPUT_TENSOR", 0x11000 },
  { "_backbone_stage2_stage2_3_Reshape_1_output_0_70346_71063_11792_70443", 0, 49152, "INPUT_TENSOR", 0x5000 },
  { "output_70634_12028", 7, 47040, "OUTPUT_TENSOR", 0x107c0 },
};

const size_t sub_0033_tensors_count = sizeof(sub_0033_tensors) / sizeof(sub_0033_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0033_address__backbone_stage4_stage4_3_Reshape_1_output_0_70500_71065_11984_70659 = 0x0;
const uint32_t sub_0033_address__backbone_stage3_stage3_7_Reshape_1_output_0_70449_71064_11924_70593 = 0x11000;
const uint32_t sub_0033_address__backbone_stage2_stage2_3_Reshape_1_output_0_70346_71063_11792_70443 = 0x5000;
const uint32_t sub_0033_address_output_70634_12028 = 0x107c0;

