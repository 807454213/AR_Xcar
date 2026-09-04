#ifndef _RKNN_DEMO_PPOCRREC_H_
#define _RKNN_DEMO_PPOCRREC_H_

#include "rknn_api.h"
#include "common.h"

// 字符字典大小（根据你的模型实际情况修改）
#define MODEL_OUT_CHANNEL 6625

/*-------------------------------------------
                应用上下文结构
-------------------------------------------*/
typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs;
    rknn_tensor_attr* output_attrs;
    
    // 零拷贝核心：存储 NPU 映射的内存句柄
    rknn_tensor_mem* in_mem;        // 输入内存
    rknn_tensor_mem* out_mem;       // 输出内存
    
    int model_channel;
    int model_width;
    int model_height;
} rknn_app_context_t;

/*-------------------------------------------
                识别结果结构
-------------------------------------------*/
typedef struct ppocr_rec_result {
    char str[512];      // 识别出的文本内容
    int str_size;       // 文本长度
    float score;        // 置信度得分
} ppocr_rec_result;

/*-------------------------------------------
                函数声明
-------------------------------------------*/

// 初始化模型
int init_ppocr_rec_model(const char* model_path, rknn_app_context_t* app_ctx);

// 释放资源
int release_ppocr_rec_model(rknn_app_context_t* app_ctx);

// 执行识别推理
int inference_ppocr_rec_model(rknn_app_context_t* app_ctx, image_buffer_t* img, ppocr_rec_result* out_result);

// 识别后处理（CTC 解码）
int rec_postprocess(float* out_data, int out_channel, int out_seq_len, ppocr_rec_result* text);

#endif //_RKNN_DEMO_PPOCRREC_H_