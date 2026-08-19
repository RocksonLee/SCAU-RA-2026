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
 */

#include <stdint.h>

#include "compute_sub_0004.h"

#include "arm_nn_types.h"
#include "arm_nnfunctions.h"
#include "kernel_library_utils.h"

#include "kernel_library_int.h" 

 

void compute_sub_0004(
  // buffer for intermediate results
  uint8_t* main_storage, // should provide at least 49157 bytes of storage

  // inputs
  
  const int8_t _backbone_stage2_stage2_1_Concat_output_0_70317_71067_11748[49152], // 1,48,32,32
  

  // outputs
  
  int8_t _backbone_stage2_stage2_1_Reshape_1_output_0_70320_11424[49152]  // 1,48,32,32
  
) {
  // Buffers allocated on the main storage (note: depends on the execution order)
    
  
  int8_t* _backbone_stage2_stage2_1_Transpose_output_0_70319_11760 = (int8_t *) &main_storage[0]; // 1,24,2,32,32 == 49152
  
  

  // Parameters
  







//
// Identity - bypassing _backbone_stage2_stage2_1_Reshape_output_0_70318_11428 operation
//
// Input _backbone_stage2_stage2_1_Concat_output_0_70317_71067_11748: int8_t - 1,48,32,32
// Output _backbone_stage2_stage2_1_Reshape_output_0_70318_11428: int8_t - 1,2,24,32,32


const int8_t* _backbone_stage2_stage2_1_Reshape_output_0_70318_11428 = _backbone_stage2_stage2_1_Concat_output_0_70317_71067_11748;





//
// Transpose
//
// Input _backbone_stage2_stage2_1_Reshape_output_0_70318_11428: int8_t - 1,2,24,32,32
// Output _backbone_stage2_stage2_1_Transpose_output_0_70319_11760: int8_t - 1,24,2,32,32
// Perm: ( 0,  2,  1,  3,  4, )

int32_t strides__backbone_stage2_stage2_1_Transpose_output_0_70319_11760[5] = { 49152, 1024, 24576, 32, 1,  };

int32_t next_dim_sizes__backbone_stage2_stage2_1_Transpose_output_0_70319_11760[5] = { 49152, 49152, 2048, 1024, 32,  };

int32_t dim_sizes__backbone_stage2_stage2_1_Transpose_output_0_70319_11760[5] = { 49152, 2048, 1024, 32, 1,  };


Transpose(
      _backbone_stage2_stage2_1_Reshape_output_0_70318_11428
    , _backbone_stage2_stage2_1_Transpose_output_0_70319_11760
    , 49152
    , 5
    , strides__backbone_stage2_stage2_1_Transpose_output_0_70319_11760
    , next_dim_sizes__backbone_stage2_stage2_1_Transpose_output_0_70319_11760
    , dim_sizes__backbone_stage2_stage2_1_Transpose_output_0_70319_11760
);

//
// Identity - bypassing _backbone_stage2_stage2_1_Reshape_1_output_0_70320_11424 operation
//
// Input _backbone_stage2_stage2_1_Transpose_output_0_70319_11760: int8_t - 1,24,2,32,32
// Output _backbone_stage2_stage2_1_Reshape_1_output_0_70320_11424: int8_t - 1,48,32,32


memcpy(_backbone_stage2_stage2_1_Reshape_1_output_0_70320_11424, _backbone_stage2_stage2_1_Transpose_output_0_70319_11760, 49152 * sizeof(int8_t));





}
