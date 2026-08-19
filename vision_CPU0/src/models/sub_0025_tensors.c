#include "sub_0025_tensors.h"

const TensorInfo sub_0025_tensors[] = {
  { "_split_1_command_stream", 3, 1584, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 4, 38048, "MODEL", 0xffffffff },
  { "_split_1_scratch", 5, 73728, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 6, 73728, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage3_stage3_7_Reshape_1_output_0_70449_71055_11916_70581", 0, 24576, "INPUT_TENSOR", 0x6000 },
  { "_backbone_stage3_stage3_7_Reshape_1_output_0_70449_71056_11920_70587", 1, 24576, "INPUT_TENSOR", 0x0 },
  { "_backbone_stage4_stage4_0_Concat_output_0_70458_71078_11940", 2, 12288, "OUTPUT_TENSOR", 0x0 },
};

const size_t sub_0025_tensors_count = sizeof(sub_0025_tensors) / sizeof(sub_0025_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0025_address__backbone_stage3_stage3_7_Reshape_1_output_0_70449_71055_11916_70581 = 0x6000;
const uint32_t sub_0025_address__backbone_stage3_stage3_7_Reshape_1_output_0_70449_71056_11920_70587 = 0x0;
const uint32_t sub_0025_address__backbone_stage4_stage4_0_Concat_output_0_70458_71078_11940 = 0x0;

