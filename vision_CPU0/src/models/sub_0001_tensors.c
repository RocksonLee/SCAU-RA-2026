#include "sub_0001_tensors.h"

const TensorInfo sub_0001_tensors[] = {
  { "_split_1_command_stream", 3, 10164, "COMMAND_STREAM", 0xffffffff },
  { "_split_1_flash", 4, 388448, "MODEL", 0xffffffff },
  { "_split_1_scratch", 5, 442368, "ARENA", 0x0 },
  { "_split_1_scratch_fast", 6, 442368, "FAST_SCRATCH", 0x0 },
  { "images_70602_11151_70256", 7, 110592, "INPUT_TENSOR", 0x0 },
  { "_737_70388_11119", 0, 3456, "OUTPUT_TENSOR", 0x2400 },
  { "_838_70438_10737", 2, 3456, "OUTPUT_TENSOR", 0x0 },
  { "_837_70434_70603_11143", 1, 864, "OUTPUT_TENSOR", 0xd80 },
};

const size_t sub_0001_tensors_count = sizeof(sub_0001_tensors) / sizeof(sub_0001_tensors[0]);

// Addresses for each input and output buffer inside of the arena
const uint32_t sub_0001_address_images_70602_11151_70256 = 0x0;
const uint32_t sub_0001_address__737_70388_11119 = 0x2400;
const uint32_t sub_0001_address__838_70438_10737 = 0x0;
const uint32_t sub_0001_address__837_70434_70603_11143 = 0xd80;

