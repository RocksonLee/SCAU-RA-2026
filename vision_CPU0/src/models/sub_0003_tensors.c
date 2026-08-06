#include "sub_0003_tensors.h"

const TensorInfo sub_0003_tensors[] = {
  { "_split_1_command_stream", 3, 1060, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 4, 46624, "MODEL", 0xffffffff },
  { "_split_1_scratch", 5, 63488, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 6, 63488, "FAST_SCRATCH", 0x0 },
  { "_843_70441_11067", 1, 24576, "INPUT_TENSOR", 0x9800 },
  { "_737_70388_11119", 0, 6144, "INPUT_TENSOR", 0x0 },
  { "_864_70452_70604_11147", 2, 6144, "OUTPUT_TENSOR", 0x1800 },
};

const size_t sub_0003_tensors_count = sizeof(sub_0003_tensors) / sizeof(sub_0003_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0003_address__843_70441_11067 = 0x9800;
const uint32_t sub_0003_address__737_70388_11119 = 0x0;
const uint32_t sub_0003_address__864_70452_70604_11147 = 0x1800;

