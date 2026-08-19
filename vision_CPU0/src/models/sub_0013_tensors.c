#include "sub_0013_tensors.h"

const TensorInfo sub_0013_tensors[] = {
  { "_split_1_command_stream", 2, 1420, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 8944, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 61440, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 61440, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage3_stage3_1_Reshape_1_output_0_70371_11456", 0, 24576, "INPUT_TENSOR", 0x6000 },
  { "_backbone_stage3_stage3_2_Concat_output_0_70381_71072_11832", 1, 24576, "OUTPUT_TENSOR", 0x6000 },
};

const size_t sub_0013_tensors_count = sizeof(sub_0013_tensors) / sizeof(sub_0013_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0013_address__backbone_stage3_stage3_1_Reshape_1_output_0_70371_11456 = 0x6000;
const uint32_t sub_0013_address__backbone_stage3_stage3_2_Concat_output_0_70381_71072_11832 = 0x6000;

