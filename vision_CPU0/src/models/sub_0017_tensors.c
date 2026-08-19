#include "sub_0017_tensors.h"

const TensorInfo sub_0017_tensors[] = {
  { "_split_1_command_stream", 2, 1284, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 8976, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 61440, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 61440, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage3_stage3_3_Reshape_1_output_0_70397_11472", 0, 24576, "INPUT_TENSOR", 0x6000 },
  { "_backbone_stage3_stage3_4_Concat_output_0_70407_71074_11864", 1, 24576, "OUTPUT_TENSOR", 0x6000 },
};

const size_t sub_0017_tensors_count = sizeof(sub_0017_tensors) / sizeof(sub_0017_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0017_address__backbone_stage3_stage3_3_Reshape_1_output_0_70397_11472 = 0x6000;
const uint32_t sub_0017_address__backbone_stage3_stage3_4_Concat_output_0_70407_71074_11864 = 0x6000;

