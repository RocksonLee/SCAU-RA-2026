#include "sub_0003_tensors.h"

const TensorInfo sub_0003_tensors[] = {
  { "_split_1_command_stream", 2, 1272, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 3552, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 122880, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 122880, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage2_stage2_0_Reshape_1_output_0_70307_11416", 0, 49152, "INPUT_TENSOR", 0x12000 },
  { "_backbone_stage2_stage2_1_Concat_output_0_70317_71067_11748", 1, 49152, "OUTPUT_TENSOR", 0xc000 },
};

const size_t sub_0003_tensors_count = sizeof(sub_0003_tensors) / sizeof(sub_0003_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0003_address__backbone_stage2_stage2_0_Reshape_1_output_0_70307_11416 = 0x12000;
const uint32_t sub_0003_address__backbone_stage2_stage2_1_Concat_output_0_70317_71067_11748 = 0xc000;

