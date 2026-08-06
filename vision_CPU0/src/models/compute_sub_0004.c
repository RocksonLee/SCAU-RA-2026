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
  uint8_t* main_storage, // should provide at least 6149 bytes of storage

  // inputs
  
  const int8_t _864_70452_70604_11147[6144], // 1,24,16,16
  

  // outputs
  
  float p4_16x16_70454[6144]  // 1,3,16,16,8
  
) {
  // Buffers allocated on the main storage (note: depends on the execution order)
    
  
  int8_t* p4_16x16_70454_11155 = (int8_t *) &main_storage[0]; // 1,3,16,16,8 == 6144
  
  

  // Parameters
  







//
// Identity - bypassing _887_70453_10891 operation
//
// Input _864_70452_70604_11147: int8_t - 1,24,16,16
// Output _887_70453_10891: int8_t - 1,3,8,16,16


const int8_t* _887_70453_10891 = _864_70452_70604_11147;





//
// Transpose
//
// Input _887_70453_10891: int8_t - 1,3,8,16,16
// Output p4_16x16_70454_11155: int8_t - 1,3,16,16,8
// Perm: ( 0,  1,  3,  4,  2, )

int32_t strides_p4_16x16_70454_11155[5] = { 6144, 2048, 16, 1, 256,  };

int32_t next_dim_sizes_p4_16x16_70454_11155[5] = { 6144, 6144, 2048, 128, 8,  };

int32_t dim_sizes_p4_16x16_70454_11155[5] = { 6144, 2048, 128, 8, 1,  };


Transpose(
      _887_70453_10891
    , p4_16x16_70454_11155
    , 6144
    , 5
    , strides_p4_16x16_70454_11155
    , next_dim_sizes_p4_16x16_70454_11155
    , dim_sizes_p4_16x16_70454_11155
);

//
// Dequantize
//
// Input  p4_16x16_70454_11155: int8_t - 1,3,16,16,8
// Output p4_16x16_70454: float - 1,3,16,16,8
AffineDequantizeInt8ToFloat(p4_16x16_70454_11155, p4_16x16_70454, 6144, 0, 0.299101859331131);


}
