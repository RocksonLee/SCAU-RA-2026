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
#include <stdbool.h>
#include <stdint.h>

// NPU unit addresses
#include "sub_0001_tensors.h"
#include "sub_0003_tensors.h"
#include "sub_0005_tensors.h"
#include "sub_0007_tensors.h"
#include "sub_0009_tensors.h"
#include "sub_0011_tensors.h"
#include "sub_0013_tensors.h"
#include "sub_0015_tensors.h"
#include "sub_0017_tensors.h"
#include "sub_0019_tensors.h"
#include "sub_0021_tensors.h"
#include "sub_0023_tensors.h"
#include "sub_0025_tensors.h"
#include "sub_0027_tensors.h"
#include "sub_0029_tensors.h"
#include "sub_0031_tensors.h"
#include "sub_0033_tensors.h"

// Arenas for NPU units
extern uint8_t sub_0001_arena[kArenaSize_sub_0001];
extern uint8_t sub_0003_arena[kArenaSize_sub_0003];
extern uint8_t sub_0005_arena[kArenaSize_sub_0005];
extern uint8_t sub_0007_arena[kArenaSize_sub_0007];
extern uint8_t sub_0009_arena[kArenaSize_sub_0009];
extern uint8_t sub_0011_arena[kArenaSize_sub_0011];
extern uint8_t sub_0013_arena[kArenaSize_sub_0013];
extern uint8_t sub_0015_arena[kArenaSize_sub_0015];
extern uint8_t sub_0017_arena[kArenaSize_sub_0017];
extern uint8_t sub_0019_arena[kArenaSize_sub_0019];
extern uint8_t sub_0021_arena[kArenaSize_sub_0021];
extern uint8_t sub_0023_arena[kArenaSize_sub_0023];
extern uint8_t sub_0025_arena[kArenaSize_sub_0025];
extern uint8_t sub_0027_arena[kArenaSize_sub_0027];
extern uint8_t sub_0029_arena[kArenaSize_sub_0029];
extern uint8_t sub_0031_arena[kArenaSize_sub_0031];
extern uint8_t sub_0033_arena[kArenaSize_sub_0033];

// Buffers
extern float buf_data[196608];
extern float buf_output_70634[47040];


bool RunModel(void);

  // Model input pointers
float* GetModelInputPtr_data(void);

  // Model output pointers
float* GetModelOutputPtr_output_70634(void);

