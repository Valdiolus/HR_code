/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-03-06T20:53:10+0200
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0x9a9589d7b8aeff47e39e5cb05985df90"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-03-06T20:53:10+0200"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  onnxGemm_0_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 40, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_1_Elu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_3_Elu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_5_Elu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 12, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 20480, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 512, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 131072, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 256, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32768, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 128, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1536, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 12, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 40, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 512, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 256, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 128, AI_STATIC)

/**  Array metadata declarations section  *************************************/
/* Int quant #0 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_0_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.02103327214717865f),
    AI_PACK_INTQ_ZP(2)))

/* Int quant #1 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_0_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0012448746711015701f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #2 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_1_Elu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.013970606029033661f),
    AI_PACK_INTQ_ZP(-61)))

/* Int quant #3 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_2_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.010827859863638878f),
    AI_PACK_INTQ_ZP(-2)))

/* Int quant #4 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_2_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00034798492561094463f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #5 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_3_Elu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.008398804813623428f),
    AI_PACK_INTQ_ZP(-39)))

/* Int quant #6 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_4_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.004658861085772514f),
    AI_PACK_INTQ_ZP(1)))

/* Int quant #7 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_4_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0004921106155961752f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #8 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_net_5_Elu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.004074521828442812f),
    AI_PACK_INTQ_ZP(-17)))

/* Int quant #9 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(node_15_QuantizeLinear_Input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00263336393982172f),
    AI_PACK_INTQ_ZP(4)))

/* Int quant #10 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(node_15_QuantizeLinear_Input_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0006954072159714997f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #11 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(onnxGemm_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.030627528205513954f),
    AI_PACK_INTQ_ZP(-8)))

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_bias, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &_net_net_0_Gemm_output_0_bias_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_output, AI_STATIC,
  1, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_net_net_0_Gemm_output_0_output_array, &_net_net_0_Gemm_output_0_output_array_intq)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_scratch0, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 40, 1, 1), AI_STRIDE_INIT(4, 2, 2, 80, 80),
  1, &_net_net_0_Gemm_output_0_scratch0_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_weights, AI_STATIC,
  3, 0x1,
  AI_SHAPE_INIT(4, 40, 512, 1, 1), AI_STRIDE_INIT(4, 1, 40, 20480, 20480),
  1, &_net_net_0_Gemm_output_0_weights_array, &_net_net_0_Gemm_output_0_weights_array_intq)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_1_Elu_output_0_output, AI_STATIC,
  4, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_net_net_1_Elu_output_0_output_array, &_net_net_1_Elu_output_0_output_array_intq)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_bias, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_net_net_2_Gemm_output_0_bias_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_output, AI_STATIC,
  6, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_net_net_2_Gemm_output_0_output_array, &_net_net_2_Gemm_output_0_output_array_intq)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_scratch0, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 2, 2, 1024, 1024),
  1, &_net_net_2_Gemm_output_0_scratch0_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_weights, AI_STATIC,
  8, 0x1,
  AI_SHAPE_INIT(4, 512, 256, 1, 1), AI_STRIDE_INIT(4, 1, 512, 131072, 131072),
  1, &_net_net_2_Gemm_output_0_weights_array, &_net_net_2_Gemm_output_0_weights_array_intq)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_3_Elu_output_0_output, AI_STATIC,
  9, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_net_net_3_Elu_output_0_output_array, &_net_net_3_Elu_output_0_output_array_intq)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_bias, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_net_net_4_Gemm_output_0_bias_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_output, AI_STATIC,
  11, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_net_net_4_Gemm_output_0_output_array, &_net_net_4_Gemm_output_0_output_array_intq)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_scratch0, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 2, 2, 512, 512),
  1, &_net_net_4_Gemm_output_0_scratch0_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_weights, AI_STATIC,
  13, 0x1,
  AI_SHAPE_INIT(4, 256, 128, 1, 1), AI_STRIDE_INIT(4, 1, 256, 32768, 32768),
  1, &_net_net_4_Gemm_output_0_weights_array, &_net_net_4_Gemm_output_0_weights_array_intq)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_5_Elu_output_0_output, AI_STATIC,
  14, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_net_net_5_Elu_output_0_output_array, &_net_net_5_Elu_output_0_output_array_intq)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_bias, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 1), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &node_15_QuantizeLinear_Input_bias_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_output, AI_STATIC,
  16, 0x1,
  AI_SHAPE_INIT(4, 1, 12, 1, 1), AI_STRIDE_INIT(4, 1, 1, 12, 12),
  1, &node_15_QuantizeLinear_Input_output_array, &node_15_QuantizeLinear_Input_output_array_intq)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_scratch0, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 2, 2, 256, 256),
  1, &node_15_QuantizeLinear_Input_scratch0_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_weights, AI_STATIC,
  18, 0x1,
  AI_SHAPE_INIT(4, 128, 12, 1, 1), AI_STRIDE_INIT(4, 1, 128, 1536, 1536),
  1, &node_15_QuantizeLinear_Input_weights_array, &node_15_QuantizeLinear_Input_weights_array_intq)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  onnxGemm_0_output, AI_STATIC,
  19, 0x1,
  AI_SHAPE_INIT(4, 1, 40, 1, 1), AI_STRIDE_INIT(4, 1, 1, 40, 40),
  1, &onnxGemm_0_output_array, &onnxGemm_0_output_array_intq)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_5_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &node_15_QuantizeLinear_Input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &node_15_QuantizeLinear_Input_weights, &node_15_QuantizeLinear_Input_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &node_15_QuantizeLinear_Input_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  node_15_QuantizeLinear_Input_layer, 29,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA,
  &node_15_QuantizeLinear_Input_chain,
  NULL, &node_15_QuantizeLinear_Input_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_i8 _net_net_5_Elu_output_0_nl_params_data[] = { -128, -127, -127, -126, -125, -125, -124, -123, -123, -122, -121, -121, -120, -119, -119, -118, -117, -117, -116, -115, -115, -114, -113, -113, -112, -111, -111, -110, -109, -108, -108, -107, -106, -106, -105, -104, -103, -103, -102, -101, -100, -100, -99, -98, -97, -96, -96, -95, -94, -93, -93, -92, -91, -90, -89, -89, -88, -87, -86, -85, -84, -84, -83, -82, -81, -80, -79, -79, -78, -77, -76, -75, -74, -73, -72, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -20, -19, -18, -17, -16, -15, -14, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50, 52, 53, 54, 55, 56, 57, 58, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90, 92, 93, 94, 95, 96, 97, 98, 100, 101, 102, 103, 104, 105, 106, 108, 109, 110, 111, 112, 113, 114, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _net_net_5_Elu_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _net_net_5_Elu_output_0_nl_params_data, _net_net_5_Elu_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_5_Elu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_5_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_5_Elu_output_0_layer, 26,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_net_net_5_Elu_output_0_chain,
  NULL, &node_15_QuantizeLinear_Input_layer, AI_STATIC, 
  .nl_params = &_net_net_5_Elu_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_3_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_4_Gemm_output_0_weights, &_net_net_4_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_4_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_net_4_Gemm_output_0_layer, 23,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA,
  &_net_net_4_Gemm_output_0_chain,
  NULL, &_net_net_5_Elu_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_i8 _net_net_3_Elu_output_0_nl_params_data[] = { -128, -127, -127, -127, -126, -126, -126, -125, -125, -125, -124, -124, -123, -123, -123, -122, -122, -121, -121, -121, -120, -120, -119, -119, -119, -118, -118, -117, -117, -116, -116, -116, -115, -115, -114, -114, -113, -113, -112, -112, -111, -111, -110, -110, -109, -109, -108, -107, -107, -106, -106, -105, -105, -104, -103, -103, -102, -102, -101, -100, -100, -99, -99, -98, -97, -97, -96, -95, -95, -94, -93, -92, -92, -91, -90, -90, -89, -88, -87, -86, -86, -85, -84, -83, -83, -82, -81, -80, -79, -78, -77, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -52, -51, -50, -49, -48, -46, -45, -44, -43, -42, -40, -39, -38, -36, -35, -34, -33, -31, -30, -29, -27, -26, -25, -24, -22, -21, -20, -18, -17, -16, -15, -13, -12, -11, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29, 31, 32, 33, 34, 36, 37, 38, 40, 41, 42, 44, 45, 46, 47, 49, 50, 51, 53, 54, 55, 56, 58, 59, 60, 62, 63, 64, 65, 67, 68, 69, 71, 72, 73, 74, 76, 77, 78, 80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 92, 94, 95, 96, 98, 99, 100, 102, 103, 104, 105, 107, 108, 109, 111, 112, 113, 114, 116, 117, 118, 120, 121, 122, 123, 125, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _net_net_3_Elu_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _net_net_3_Elu_output_0_nl_params_data, _net_net_3_Elu_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_3_Elu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_3_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_3_Elu_output_0_layer, 20,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_net_net_3_Elu_output_0_chain,
  NULL, &_net_net_4_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = &_net_net_3_Elu_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_1_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_2_Gemm_output_0_weights, &_net_net_2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_2_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_net_2_Gemm_output_0_layer, 17,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA,
  &_net_net_2_Gemm_output_0_chain,
  NULL, &_net_net_3_Elu_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_i8 _net_net_1_Elu_output_0_nl_params_data[] = { -128, -128, -128, -128, -128, -127, -127, -127, -127, -127, -127, -127, -127, -126, -126, -126, -126, -126, -126, -126, -125, -125, -125, -125, -125, -125, -125, -124, -124, -124, -124, -124, -123, -123, -123, -123, -123, -122, -122, -122, -122, -122, -121, -121, -121, -121, -120, -120, -120, -120, -119, -119, -119, -118, -118, -118, -117, -117, -117, -117, -116, -116, -115, -115, -115, -114, -114, -114, -113, -113, -112, -112, -111, -111, -111, -110, -110, -109, -109, -108, -108, -107, -106, -106, -105, -105, -104, -104, -103, -102, -102, -101, -100, -100, -99, -98, -98, -97, -96, -95, -94, -94, -93, -92, -91, -90, -89, -88, -88, -87, -86, -85, -84, -83, -81, -80, -79, -78, -77, -76, -75, -73, -72, -71, -69, -68, -67, -65, -64, -62, -61, -59, -58, -56, -55, -53, -52, -50, -49, -47, -46, -44, -43, -41, -40, -38, -37, -35, -34, -32, -31, -29, -28, -26, -25, -23, -22, -20, -19, -17, -16, -14, -13, -11, -10, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26, 28, 29, 31, 32, 34, 35, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 70, 71, 73, 74, 76, 78, 79, 81, 82, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 99, 100, 102, 103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 118, 120, 121, 123, 124, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _net_net_1_Elu_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _net_net_1_Elu_output_0_nl_params_data, _net_net_1_Elu_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_1_Elu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_1_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_1_Elu_output_0_layer, 14,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_net_net_1_Elu_output_0_chain,
  NULL, &_net_net_2_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = &_net_net_1_Elu_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &onnxGemm_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_0_Gemm_output_0_weights, &_net_net_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_0_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_net_0_Gemm_output_0_layer, 11,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA,
  &_net_net_0_Gemm_output_0_chain,
  NULL, &_net_net_1_Elu_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 189488, 1, 1),
    189488, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1792, 1, 1),
    1792, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &onnxGemm_0_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &node_15_QuantizeLinear_Input_output),
  &_net_net_0_Gemm_output_0_layer, 0x82ef8450, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 189488, 1, 1),
      189488, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1792, 1, 1),
      1792, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &onnxGemm_0_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &node_15_QuantizeLinear_Input_output),
  &_net_net_0_Gemm_output_0_layer, 0x82ef8450, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    onnxGemm_0_output_array.data = AI_PTR(g_network_activations_map[0] + 904);
    onnxGemm_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 904);
    _net_net_0_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 944);
    _net_net_0_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 944);
    _net_net_0_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _net_net_0_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _net_net_1_Elu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1024);
    _net_net_1_Elu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1024);
    _net_net_2_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _net_net_2_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _net_net_2_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1536);
    _net_net_2_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1536);
    _net_net_3_Elu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _net_net_3_Elu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _net_net_4_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 256);
    _net_net_4_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 256);
    _net_net_4_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 768);
    _net_net_4_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 768);
    _net_net_5_Elu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _net_net_5_Elu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    node_15_QuantizeLinear_Input_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 128);
    node_15_QuantizeLinear_Input_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 128);
    node_15_QuantizeLinear_Input_output_array.data = AI_PTR(g_network_activations_map[0] + 384);
    node_15_QuantizeLinear_Input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 384);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    _net_net_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_0_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    _net_net_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    _net_net_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_0_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 20480);
    _net_net_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 20480);
    _net_net_2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_2_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 22528);
    _net_net_2_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 22528);
    _net_net_2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_2_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 153600);
    _net_net_2_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 153600);
    _net_net_4_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_4_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 154624);
    _net_net_4_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 154624);
    _net_net_4_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_4_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 187392);
    _net_net_4_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 187392);
    node_15_QuantizeLinear_Input_weights_array.format |= AI_FMT_FLAG_CONST;
    node_15_QuantizeLinear_Input_weights_array.data = AI_PTR(g_network_weights_map[0] + 187904);
    node_15_QuantizeLinear_Input_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 187904);
    node_15_QuantizeLinear_Input_bias_array.format |= AI_FMT_FLAG_CONST;
    node_15_QuantizeLinear_Input_bias_array.data = AI_PTR(g_network_weights_map[0] + 189440);
    node_15_QuantizeLinear_Input_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 189440);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 187660,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x82ef8450,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 187660,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x82ef8450,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

