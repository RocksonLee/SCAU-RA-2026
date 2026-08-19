#include "sub_0019_tensors.h"

const TensorInfo sub_0019_tensors[] = {
  { "_split_1_command_stream", 2, 1300, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 9056, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 61440, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 61440, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage3_stage3_4_Reshape_1_output_0_70410_11480", 0, 24576, "INPUT_TENSOR", 0x6000 },
  { "_backbone_stage3_stage3_5_Concat_output_0_70420_71075_11880", 1, 24576, "OUTPUT_TENSOR", 0x6000 },
};

const size_t sub_0019_tensors_count = sizeof(sub_0019_tensors) / sizeof(sub_0019_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0019_address__backbone_stage3_stage3_4_Reshape_1_output_0_70410_11480 = 0x6000;
const uint32_t sub_0019_address__backbone_stage3_stage3_5_Concat_output_0_70420_71075_11880 = 0x6000;

