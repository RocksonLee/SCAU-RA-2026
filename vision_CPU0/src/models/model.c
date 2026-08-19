/*
 * This file is developed by EdgeCortix Inc. to be used with certain Renesas Electronics Hardware only.
 *
 * Copyright © 2025 EdgeCortix Inc. Licensed to Renesas Electronics Corporation with the
 * right to sublicense under the Apache License, Version 2.0.
 *
 * This file also includes source code originally developed by the Renesas Electronics Corporation.
 * The Renesas disclaimer below applies to any Renesas-originated portions for usage of the code.
 *
 * The Renesas Electronics Corporation
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED 'AS IS' AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Changed from original python code to C source code.
 * Copyright (C) 2017 Renesas Electronics Corporation. All rights reserved.
 *
 * This file also includes source codes originally developed by the TensorFlow Authors which were distributed under the following conditions.
 *
 * The TensorFlow Authors
 * Copyright 2023 The Apache Software Foundation
 *
 * This product includes software developed at
 * The Apache Software Foundation (http://www.apache.org/).
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "model.h"
#include "common_data.h"

// CPU compute declarations
#include "compute_sub_0000.h"
#include "sub_0001_invoke.h"
#include "compute_sub_0002.h"
#include "sub_0003_invoke.h"
#include "compute_sub_0004.h"
#include "sub_0005_invoke.h"
#include "compute_sub_0006.h"
#include "sub_0007_invoke.h"
#include "compute_sub_0008.h"
#include "sub_0009_invoke.h"
#include "compute_sub_0010.h"
#include "sub_0011_invoke.h"
#include "compute_sub_0012.h"
#include "sub_0013_invoke.h"
#include "compute_sub_0014.h"
#include "sub_0015_invoke.h"
#include "compute_sub_0016.h"
#include "sub_0017_invoke.h"
#include "compute_sub_0018.h"
#include "sub_0019_invoke.h"
#include "compute_sub_0020.h"
#include "sub_0021_invoke.h"
#include "compute_sub_0022.h"
#include "sub_0023_invoke.h"
#include "compute_sub_0024.h"
#include "sub_0025_invoke.h"
#include "compute_sub_0026.h"
#include "sub_0027_invoke.h"
#include "compute_sub_0028.h"
#include "sub_0029_invoke.h"
#include "compute_sub_0030.h"
#include "sub_0031_invoke.h"
#include "compute_sub_0032.h"
#include "sub_0033_invoke.h"
#include "compute_sub_0034.h"

// Application-facing float buffers and the shared CPU scratch live in cached SDRAM.
float buf_data[196608]
    BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(32);
float buf_output_70634[47040]
    BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(32);
static uint8_t g_compute_arena[kBufferSize_sub_0000]
    BSP_PLACE_IN_SECTION(".sdram") BSP_ALIGN_VARIABLE(32);

#define MODEL_ARENA_I8(arena, address) ((int8_t *) ((arena) + (address)))

float* GetModelInputPtr_data(void) {
  return buf_data;
}

float* GetModelOutputPtr_output_70634(void) {
  return buf_output_70634;
}

bool RunModel(void) {
  compute_sub_0000(
      g_compute_arena,
      buf_data,
      MODEL_ARENA_I8(sub_0001_arena, sub_0001_address_data_71032_12024));
  if (sub_0001_invoke(false) != 0) return false;

  compute_sub_0002(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0001_arena, sub_0001_address__backbone_stage2_stage2_0_Concat_output_0_70304_71066_11740),
      MODEL_ARENA_I8(sub_0003_arena, sub_0003_address__backbone_stage2_stage2_0_Reshape_1_output_0_70307_11416));
  if (sub_0003_invoke(false) != 0) return false;

  compute_sub_0004(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0003_arena, sub_0003_address__backbone_stage2_stage2_1_Concat_output_0_70317_71067_11748),
      MODEL_ARENA_I8(sub_0005_arena, sub_0005_address__backbone_stage2_stage2_1_Reshape_1_output_0_70320_11424));
  if (sub_0005_invoke(false) != 0) return false;

  compute_sub_0006(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0005_arena, sub_0005_address__backbone_stage2_stage2_2_Concat_output_0_70330_71068_11764),
      MODEL_ARENA_I8(sub_0007_arena, sub_0007_address__backbone_stage2_stage2_2_Reshape_1_output_0_70333_11432));
  if (sub_0007_invoke(false) != 0) return false;

  // The third branch is retained directly in sub_0033 until the final head invocation.
  compute_sub_0008(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0007_arena, sub_0007_address__backbone_stage2_stage2_3_Concat_output_0_70343_71069_11780),
      MODEL_ARENA_I8(sub_0009_arena, sub_0009_address__backbone_stage2_stage2_3_Reshape_1_output_0_70346_71039_11784_70431),
      MODEL_ARENA_I8(sub_0009_arena, sub_0009_address__backbone_stage2_stage2_3_Reshape_1_output_0_70346_71040_11788_70437),
      MODEL_ARENA_I8(sub_0033_arena, sub_0033_address__backbone_stage2_stage2_3_Reshape_1_output_0_70346_71063_11792_70443));
  if (sub_0009_invoke(false) != 0) return false;

  compute_sub_0010(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0009_arena, sub_0009_address__backbone_stage3_stage3_0_Concat_output_0_70355_71070_11808),
      MODEL_ARENA_I8(sub_0011_arena, sub_0011_address__backbone_stage3_stage3_0_Reshape_1_output_0_70358_11448));
  if (sub_0011_invoke(false) != 0) return false;

  compute_sub_0012(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0011_arena, sub_0011_address__backbone_stage3_stage3_1_Concat_output_0_70368_71071_11816),
      MODEL_ARENA_I8(sub_0013_arena, sub_0013_address__backbone_stage3_stage3_1_Reshape_1_output_0_70371_11456));
  if (sub_0013_invoke(false) != 0) return false;

  compute_sub_0014(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0013_arena, sub_0013_address__backbone_stage3_stage3_2_Concat_output_0_70381_71072_11832),
      MODEL_ARENA_I8(sub_0015_arena, sub_0015_address__backbone_stage3_stage3_2_Reshape_1_output_0_70384_11464));
  if (sub_0015_invoke(false) != 0) return false;

  compute_sub_0016(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0015_arena, sub_0015_address__backbone_stage3_stage3_3_Concat_output_0_70394_71073_11848),
      MODEL_ARENA_I8(sub_0017_arena, sub_0017_address__backbone_stage3_stage3_3_Reshape_1_output_0_70397_11472));
  if (sub_0017_invoke(false) != 0) return false;

  compute_sub_0018(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0017_arena, sub_0017_address__backbone_stage3_stage3_4_Concat_output_0_70407_71074_11864),
      MODEL_ARENA_I8(sub_0019_arena, sub_0019_address__backbone_stage3_stage3_4_Reshape_1_output_0_70410_11480));
  if (sub_0019_invoke(false) != 0) return false;

  compute_sub_0020(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0019_arena, sub_0019_address__backbone_stage3_stage3_5_Concat_output_0_70420_71075_11880),
      MODEL_ARENA_I8(sub_0021_arena, sub_0021_address__backbone_stage3_stage3_5_Reshape_1_output_0_70423_11488));
  if (sub_0021_invoke(false) != 0) return false;

  compute_sub_0022(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0021_arena, sub_0021_address__backbone_stage3_stage3_6_Concat_output_0_70433_71076_11896),
      MODEL_ARENA_I8(sub_0023_arena, sub_0023_address__backbone_stage3_stage3_6_Reshape_1_output_0_70436_11496));
  if (sub_0023_invoke(false) != 0) return false;

  // The third branch is retained directly in sub_0033 until the final head invocation.
  compute_sub_0024(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0023_arena, sub_0023_address__backbone_stage3_stage3_7_Concat_output_0_70446_71077_11912),
      MODEL_ARENA_I8(sub_0025_arena, sub_0025_address__backbone_stage3_stage3_7_Reshape_1_output_0_70449_71055_11916_70581),
      MODEL_ARENA_I8(sub_0025_arena, sub_0025_address__backbone_stage3_stage3_7_Reshape_1_output_0_70449_71056_11920_70587),
      MODEL_ARENA_I8(sub_0033_arena, sub_0033_address__backbone_stage3_stage3_7_Reshape_1_output_0_70449_71064_11924_70593));
  if (sub_0025_invoke(false) != 0) return false;

  compute_sub_0026(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0025_arena, sub_0025_address__backbone_stage4_stage4_0_Concat_output_0_70458_71078_11940),
      MODEL_ARENA_I8(sub_0027_arena, sub_0027_address__backbone_stage4_stage4_0_Reshape_1_output_0_70461_11512));
  if (sub_0027_invoke(false) != 0) return false;

  compute_sub_0028(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0027_arena, sub_0027_address__backbone_stage4_stage4_1_Concat_output_0_70471_71079_11948),
      MODEL_ARENA_I8(sub_0029_arena, sub_0029_address__backbone_stage4_stage4_1_Reshape_1_output_0_70474_11520));
  if (sub_0029_invoke(false) != 0) return false;

  compute_sub_0030(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0029_arena, sub_0029_address__backbone_stage4_stage4_2_Concat_output_0_70484_71080_11964),
      MODEL_ARENA_I8(sub_0031_arena, sub_0031_address__backbone_stage4_stage4_2_Reshape_1_output_0_70487_11528));
  if (sub_0031_invoke(false) != 0) return false;

  compute_sub_0032(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0031_arena, sub_0031_address__backbone_stage4_stage4_3_Concat_output_0_70497_71081_11980),
      MODEL_ARENA_I8(sub_0033_arena, sub_0033_address__backbone_stage4_stage4_3_Reshape_1_output_0_70500_71065_11984_70659));
  if (sub_0033_invoke(false) != 0) return false;

  compute_sub_0034(
      g_compute_arena,
      MODEL_ARENA_I8(sub_0033_arena, sub_0033_address_output_70634_12028),
      buf_output_70634);

  return true;
}

#undef MODEL_ARENA_I8
