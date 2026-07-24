#include "sub_0003_tensors.h"

const TensorInfo sub_0003_tensors[] = {
  { "_split_1_command_stream", 3, 1060, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 4, 46400, "MODEL", 0xffffffff },
  { "_split_1_scratch", 5, 35712, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 6, 35712, "FAST_SCRATCH", 0x0 },
  { "_843_70441_11067", 1, 13824, "INPUT_TENSOR", 0x5580 },
  { "_737_70388_11119", 0, 3456, "INPUT_TENSOR", 0x0 },
  { "_864_70452_70604_11147", 2, 3456, "OUTPUT_TENSOR", 0xd80 },
};

const size_t sub_0003_tensors_count = sizeof(sub_0003_tensors) / sizeof(sub_0003_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0003_address__843_70441_11067 = 0x5580;
const uint32_t sub_0003_address__737_70388_11119 = 0x0;
const uint32_t sub_0003_address__864_70452_70604_11147 = 0xd80;

