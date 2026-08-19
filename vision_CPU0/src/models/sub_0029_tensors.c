#include "sub_0029_tensors.h"

const TensorInfo sub_0029_tensors[] = {
  { "_split_1_command_stream", 2, 1428, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 24560, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 30720, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 30720, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage4_stage4_1_Reshape_1_output_0_70474_11520", 0, 12288, "INPUT_TENSOR", 0x3000 },
  { "_backbone_stage4_stage4_2_Concat_output_0_70484_71080_11964", 1, 12288, "OUTPUT_TENSOR", 0x3000 },
};

const size_t sub_0029_tensors_count = sizeof(sub_0029_tensors) / sizeof(sub_0029_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0029_address__backbone_stage4_stage4_1_Reshape_1_output_0_70474_11520 = 0x3000;
const uint32_t sub_0029_address__backbone_stage4_stage4_2_Concat_output_0_70484_71080_11964 = 0x3000;

