#include "sub_0031_tensors.h"

const TensorInfo sub_0031_tensors[] = {
  { "_split_1_command_stream", 2, 1256, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 3, 24400, "MODEL", 0xffffffff },
  { "_split_1_scratch", 4, 30720, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 5, 30720, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage4_stage4_2_Reshape_1_output_0_70487_11528", 0, 12288, "INPUT_TENSOR", 0x3000 },
  { "_backbone_stage4_stage4_3_Concat_output_0_70497_71081_11980", 1, 12288, "OUTPUT_TENSOR", 0x3000 },
};

const size_t sub_0031_tensors_count = sizeof(sub_0031_tensors) / sizeof(sub_0031_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0031_address__backbone_stage4_stage4_2_Reshape_1_output_0_70487_11528 = 0x3000;
const uint32_t sub_0031_address__backbone_stage4_stage4_3_Concat_output_0_70497_71081_11980 = 0x3000;

