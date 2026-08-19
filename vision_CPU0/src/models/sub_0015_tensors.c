#include "sub_0015_tensors.h"

const TensorInfo sub_0015_tensors[] = {
  { "_split_1_command_stream", 2, 1300, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 9040, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 61440, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 61440, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage3_stage3_2_Reshape_1_output_0_70384_11464", 0, 24576, "INPUT_TENSOR", 0x6000 },
  { "_backbone_stage3_stage3_3_Concat_output_0_70394_71073_11848", 1, 24576, "OUTPUT_TENSOR", 0x6000 },
};

const size_t sub_0015_tensors_count = sizeof(sub_0015_tensors) / sizeof(sub_0015_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0015_address__backbone_stage3_stage3_2_Reshape_1_output_0_70384_11464 = 0x6000;
const uint32_t sub_0015_address__backbone_stage3_stage3_3_Concat_output_0_70394_71073_11848 = 0x6000;

