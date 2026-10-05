#include "nn.h"

// AI Handle
ai_handle network;

// Buffers allocated by X-CUBE-AI
//AI_ALIGNED(4) ai_i8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
//AI_ALIGNED(4) ai_float in_data[AI_NETWORK_IN_1_SIZE];
//AI_ALIGNED(4) ai_float out_data[AI_NETWORK_OUT_1_SIZE];

// Force the intermediate activations into DTCM
__attribute__((section(".dtcmram"))) AI_ALIGNED(4) ai_i8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

// Force your input (sensor states) and output (motor commands) into DTCM
__attribute__((section(".dtcmram"))) AI_ALIGNED(4) ai_i8 in_data[AI_NETWORK_IN_1_SIZE];
__attribute__((section(".dtcmram"))) AI_ALIGNED(4) ai_i8 out_data[AI_NETWORK_OUT_1_SIZE];

// Inference timing (DWT cycle counts)
static uint32_t inf_min_cycles = UINT32_MAX;
static uint32_t inf_max_cycles = 0;
static uint32_t inf_count = 0;


void AI_init(void) 
{
  ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  /*ai_network_params params = 
  {
      AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
      AI_NETWORK_DATA_ACTIVATIONS(activations)
  };*/
  //ai_network_init(network, &params);
}

// with DWT
void AI_RunInference(void) 
{
    // 1. Fill input with test data (INT8: range -128..127)
    for (int i = 0; i < AI_NETWORK_IN_1_SIZE; i++) {
      in_data[i] = 1;
    }

    // 2. Get the buffer descriptors from the runtime
    ai_buffer *ai_input = ai_network_inputs_get(network, NULL);
    ai_buffer *ai_output = ai_network_outputs_get(network, NULL);

    // 3. Point the buffers to our data arrays
    ai_input[0].data = AI_HANDLE_PTR(in_data);
    ai_output[0].data = AI_HANDLE_PTR(out_data);

    // 4. Run the MLP policy and measure inference time
    uint32_t cyc_start = DWT->CYCCNT;
    ai_network_run(network, &ai_input[0], &ai_output[0]);
    uint32_t cyc_elapsed = DWT->CYCCNT - cyc_start;

    if (cyc_elapsed < inf_min_cycles) inf_min_cycles = cyc_elapsed;
    if (cyc_elapsed > inf_max_cycles) inf_max_cycles = cyc_elapsed;
    inf_count++;
    //Results: 
    // FP32 ONNX 40_256_256_12 - 1ms
    // INT8 ONNX 40_256_256_12 - 437us
}
