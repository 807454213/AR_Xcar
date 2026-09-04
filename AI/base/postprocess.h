#ifndef _RKNN_PPYOLOE_DEMO_POSTPROCESS_H_
#define _RKNN_PPYOLOE_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 5
#define PROP_BOX_SIZE (5 + OBJ_CLASS_NUM)
#define NMS_THRESH 0.5
#define BOX_THRESH 0.4
// #define BOX_THRESH 0.25
#define REG_MAX 16
#define MIN_BOX_SIZE 2.0
#define PRE_NMS_TOPK 100
// #define PRE_NMS_TOPK 300
#define KEEP_TOPK 20
// #define KEEP_TOPK 100

// class rknn_app_context_t;

typedef struct
{
    float scale_x;
    float scale_y;
} resize_param_t;

typedef struct
{
    image_rect_t box;
    float prop;
    int cls_id;
} object_detect_result;

typedef struct
{
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

// initialize postprocessing with a path to the label text file
int init_post_process(const char *label_path);
void deinit_post_process();
char *coco_cls_to_name(int cls_id);
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, resize_param_t *resize_param, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);

void deinitPostProcess();
#endif //_RKNN_PPYOLOE_DEMO_POSTPROCESS_H_
