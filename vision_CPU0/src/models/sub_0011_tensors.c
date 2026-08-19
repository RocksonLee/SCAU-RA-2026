#include "sub_0011_tensors.h"

const TensorInfo sub_0011_tensors[] = {
  { "_split_1_command_stream", 2, 1272, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 8800, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 61440, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 61440, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage3_stage3_0_Reshape_1_output_0_70358_11448", 0, 24576, "INPUT_TENSOR", 0x6000 },
  { "_backbone_stage3_stage3_1_Concat_output_0_70368_71071_11816", 1, 24576, "OUTPUT_TENSOR", 0x6000 },
};

const size_t sub_0011_tensors_count = sizeof(sub_0011_tensors) / sizeof(sub_0011_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0011_address__backbone_stage3_stage3_0_Reshape_1_output_0_70358_11448 = 0x6000;
const uint32_t sub_0011_address__backbone_stage3_stage3_1_Concat_output_0_70368_71071_11816 = 0x6000;

