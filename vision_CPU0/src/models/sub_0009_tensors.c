#include "sub_0009_tensors.h"

const TensorInfo sub_0009_tensors[] = {
  { "_split_1_command_stream", 3, 1568, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 4, 14224, "MODEL", 0xffffffff },
  { "_split_1_scratch", 5, 147456, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 6, 147456, "FAST_SCRATCH", 0x0 },
  { "_backbone_stage2_stage2_3_Reshape_1_output_0_70346_71039_11784_70431", 0, 49152, "INPUT_TENSOR", 0xc000 },
  { "_backbone_stage2_stage2_3_Reshape_1_output_0_70346_71040_11788_70437", 1, 49152, "INPUT_TENSOR", 0x0 },
  { "_backbone_stage3_stage3_0_Concat_output_0_70355_71070_11808", 2, 24576, "OUTPUT_TENSOR", 0x0 },
};

const size_t sub_0009_tensors_count = sizeof(sub_0009_tensors) / sizeof(sub_0009_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0009_address__backbone_stage2_stage2_3_Reshape_1_output_0_70346_71039_11784_70431 = 0xc000;
const uint32_t sub_0009_address__backbone_stage2_stage2_3_Reshape_1_output_0_70346_71040_11788_70437 = 0x0;
const uint32_t sub_0009_address__backbone_stage3_stage3_0_Concat_output_0_70355_71070_11808 = 0x0;

