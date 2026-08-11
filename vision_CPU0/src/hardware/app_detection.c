#include "app_detection.h"

#include <stddef.h>
#include <stdio.h>

#include "camera_ov5640.h"
#include "hal_data.h"
#include "model.h"

#define DET_INPUT_SIZE              (256U)
#define DET_NUM_ANCHORS             (3U)
#define DET_NUM_VALUES              (8U)
#define DET_P4_GRID                 (16U)
#define DET_P5_GRID                 (8U)
#define DET_P4_STRIDE               (16.0f)
#define DET_P5_STRIDE               (32.0f)
#define DET_ANCHOR_SCALE            ((float) DET_INPUT_SIZE / 256.0f)
#define DET_CONF_THRESHOLD          (0.8f)
#define DET_NMS_IOU_THRESHOLD       (0.20f)
#define DET_MAX_CANDIDATES          (64U)
#define DET_MAX_OUTPUTS             APP_DETECTION_MAX_RESULTS
#define DET_UART_LINE_BYTES         (96U)
#define DET_GREEN_R_MARGIN          (12U)
#define DET_GREEN_B_MARGIN          (20U)
#define DET_GREEN_EXCESS_MIN        (45)
#define DET_GREEN_PIXEL_PERCENT_MIN (8U)

typedef struct st_detection_box
{
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    uint32_t class_id;
} detection_box_t;

static detection_box_t g_candidates[DET_MAX_CANDIDATES];
static detection_box_t g_outputs[DET_MAX_OUTPUTS];
static char g_det_line[DET_UART_LINE_BYTES];
static bool g_detection_initialized;

static const float g_anchors_p4[DET_NUM_ANCHORS][2] =
{
    {12.0f * DET_ANCHOR_SCALE, 18.0f * DET_ANCHOR_SCALE},
    {37.0f * DET_ANCHOR_SCALE, 49.0f * DET_ANCHOR_SCALE},
    {52.0f * DET_ANCHOR_SCALE, 132.0f * DET_ANCHOR_SCALE},
};

static const float g_anchors_p5[DET_NUM_ANCHORS][2] =
{
    {115.0f * DET_ANCHOR_SCALE, 73.0f * DET_ANCHOR_SCALE},
    {119.0f * DET_ANCHOR_SCALE, 199.0f * DET_ANCHOR_SCALE},
    {242.0f * DET_ANCHOR_SCALE, 238.0f * DET_ANCHOR_SCALE},
};

static float det_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float det_sigmoid(float value)
{
    value = (value > 16.0f) ? 16.0f : value;
    value = (value < -16.0f) ? -16.0f : value;

    float const x = det_absf(value);
    float y = 1.0f + (x / 16.0f);

    y *= y;
    y *= y;
    y *= y;
    y *= y;

    float const exp_neg_abs = 1.0f / y;
    float const sigmoid_abs = 1.0f / (1.0f + exp_neg_abs);

    return (value >= 0.0f) ? sigmoid_abs : (1.0f - sigmoid_abs);
}

static float det_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float det_minf(float a, float b)
{
    return (a < b) ? a : b;
}

static float det_clampf(float value, float low, float high)
{
    return det_minf(det_maxf(value, low), high);
}

static uint8_t det_rgb565_r(uint16_t pixel)
{
    uint8_t const r5 = (uint8_t) ((pixel >> 11) & 0x1FU);
    return (uint8_t) ((r5 << 3) | (r5 >> 2));
}

static uint8_t det_rgb565_g(uint16_t pixel)
{
    uint8_t const g6 = (uint8_t) ((pixel >> 5) & 0x3FU);
    return (uint8_t) ((g6 << 2) | (g6 >> 4));
}

static uint8_t det_rgb565_b(uint16_t pixel)
{
    uint8_t const b5 = (uint8_t) (pixel & 0x1FU);
    return (uint8_t) ((b5 << 3) | (b5 >> 2));
}

static void det_preprocess_rgb565(uint8_t const * p_frame)
{
    float * const p_input = GetModelInputPtr_images();
    uint32_t const crop_size = CAMERA_OV5640_HEIGHT;
    uint32_t const crop_x = (CAMERA_OV5640_WIDTH - crop_size) / 2U;
    uint32_t const crop_y = 0U;
    uint32_t const plane_size = DET_INPUT_SIZE * DET_INPUT_SIZE;

    for (uint32_t y = 0U; y < DET_INPUT_SIZE; y++)
    {
        uint32_t const src_y = crop_y + ((y * crop_size) / DET_INPUT_SIZE);

        for (uint32_t x = 0U; x < DET_INPUT_SIZE; x++)
        {
            uint32_t const src_x = crop_x + ((x * crop_size) / DET_INPUT_SIZE);
            uint32_t const src_index = ((src_y * CAMERA_OV5640_WIDTH) + src_x) * CAMERA_OV5640_BYTES_PER_PIXEL;
            uint16_t const pixel = (uint16_t) (p_frame[src_index] | ((uint16_t) p_frame[src_index + 1U] << 8));
            uint32_t const dst_index = (y * DET_INPUT_SIZE) + x;

            p_input[dst_index] = (float) det_rgb565_r(pixel) / 255.0f;
            p_input[plane_size + dst_index] = (float) det_rgb565_g(pixel) / 255.0f;
            p_input[(2U * plane_size) + dst_index] = (float) det_rgb565_b(pixel) / 255.0f;
        }
    }
}

static void det_insert_candidate(detection_box_t const * p_box, uint32_t * p_count)
{
    if (p_box->score < DET_CONF_THRESHOLD)
    {
        return;
    }

    uint32_t insert = *p_count;

    while ((insert > 0U) && (g_candidates[insert - 1U].score < p_box->score))
    {
        if (insert < DET_MAX_CANDIDATES)
        {
            g_candidates[insert] = g_candidates[insert - 1U];
        }

        insert--;
    }

    if (insert < DET_MAX_CANDIDATES)
    {
        g_candidates[insert] = *p_box;
    }

    if (*p_count < DET_MAX_CANDIDATES)
    {
        (*p_count)++;
    }
}

static uint32_t det_head_index(uint32_t anchor, uint32_t gy, uint32_t gx, uint32_t grid, bool anchor_first)
{
    if (anchor_first)
    {
        return ((((anchor * grid) + gy) * grid) + gx) * DET_NUM_VALUES;
    }

    return ((((gy * grid) + gx) * DET_NUM_ANCHORS) + anchor) * DET_NUM_VALUES;
}

static void det_decode_head(float const * p_head,
                            uint32_t grid,
                            float stride,
                            float const anchors[DET_NUM_ANCHORS][2],
                            uint32_t * p_candidate_count)
{
    for (uint32_t anchor = 0U; anchor < DET_NUM_ANCHORS; anchor++)
    {
        for (uint32_t gy = 0U; gy < grid; gy++)
        {
            for (uint32_t gx = 0U; gx < grid; gx++)
            {
                uint32_t const base = det_head_index(anchor, gy, gx, grid, true);
                float const sx = det_sigmoid(p_head[base]);
                float const sy = det_sigmoid(p_head[base + 1U]);
                float const sw = det_sigmoid(p_head[base + 2U]);
                float const sh = det_sigmoid(p_head[base + 3U]);
                float const objectness = det_sigmoid(p_head[base + 4U]);
                float cls = det_sigmoid(p_head[base + 5U]);
                uint32_t class_id = 0U;

                for (uint32_t c = 1U; c < (DET_NUM_VALUES - 5U); c++)
                {
                    float const class_score = det_sigmoid(p_head[base + 5U + c]);

                    if (class_score > cls)
                    {
                        cls = class_score;
                        class_id = c;
                    }
                }

                float const score = objectness * cls;

                float const cx = (((sx * 2.0f) - 0.5f) + (float) gx) * stride;
                float const cy = (((sy * 2.0f) - 0.5f) + (float) gy) * stride;
                float const aw = anchors[anchor][0];
                float const ah = anchors[anchor][1];
                float const ww = (sw * 2.0f) * (sw * 2.0f) * aw;
                float const hh = (sh * 2.0f) * (sh * 2.0f) * ah;

                detection_box_t box;
                box.x1 = det_clampf(cx - (ww * 0.5f), 0.0f, (float) DET_INPUT_SIZE);
                box.y1 = det_clampf(cy - (hh * 0.5f), 0.0f, (float) DET_INPUT_SIZE);
                box.x2 = det_clampf(cx + (ww * 0.5f), 0.0f, (float) DET_INPUT_SIZE);
                box.y2 = det_clampf(cy + (hh * 0.5f), 0.0f, (float) DET_INPUT_SIZE);
                box.score = score;
                box.class_id = class_id;

                det_insert_candidate(&box, p_candidate_count);
            }
        }
    }
}

static float det_iou(detection_box_t const * a, detection_box_t const * b)
{
    float const ix1 = det_maxf(a->x1, b->x1);
    float const iy1 = det_maxf(a->y1, b->y1);
    float const ix2 = det_minf(a->x2, b->x2);
    float const iy2 = det_minf(a->y2, b->y2);
    float const iw = det_maxf(0.0f, ix2 - ix1);
    float const ih = det_maxf(0.0f, iy2 - iy1);
    float const intersection = iw * ih;
    float const area_a = det_maxf(0.0f, a->x2 - a->x1) * det_maxf(0.0f, a->y2 - a->y1);
    float const area_b = det_maxf(0.0f, b->x2 - b->x1) * det_maxf(0.0f, b->y2 - b->y1);
    float const denom = area_a + area_b - intersection;

    return (denom > 0.0f) ? (intersection / denom) : 0.0f;
}

static uint32_t det_nms(uint32_t candidate_count)
{
    uint32_t output_count = 0U;

    for (uint32_t i = 0U; (i < candidate_count) && (output_count < DET_MAX_OUTPUTS); i++)
    {
        bool keep = true;

        for (uint32_t j = 0U; j < output_count; j++)
        {
            if ((g_candidates[i].class_id == g_outputs[j].class_id) &&
                (det_iou(&g_candidates[i], &g_outputs[j]) > DET_NMS_IOU_THRESHOLD))
            {
                keep = false;
                break;
            }
        }

        if (keep)
        {
            g_outputs[output_count] = g_candidates[i];
            output_count++;
        }
    }

    return output_count;
}

static void det_to_camera_coords(detection_box_t const * p_model_box,
                                 int32_t * p_x1,
                                 int32_t * p_y1,
                                 int32_t * p_x2,
                                 int32_t * p_y2)
{
    float const crop_size = (float) CAMERA_OV5640_HEIGHT;
    float const crop_x = ((float) CAMERA_OV5640_WIDTH - crop_size) * 0.5f;
    float const scale = crop_size / (float) DET_INPUT_SIZE;

    *p_x1 = (int32_t) ((p_model_box->x1 * scale) + crop_x);
    *p_y1 = (int32_t) (p_model_box->y1 * scale);
    *p_x2 = (int32_t) ((p_model_box->x2 * scale) + crop_x);
    *p_y2 = (int32_t) (p_model_box->y2 * scale);
}

static uint32_t det_refine_grape_class(uint8_t const * p_rgb565_frame,
                                       detection_box_t const * p_model_box)
{
    if (APP_DETECTION_CLASS_GREEN_GRAPE != p_model_box->class_id)
    {
        return p_model_box->class_id;
    }

    int32_t box_x1;
    int32_t box_y1;
    int32_t box_x2;
    int32_t box_y2;
    det_to_camera_coords(p_model_box, &box_x1, &box_y1, &box_x2, &box_y2);

    int32_t const margin_x = (box_x2 - box_x1) / 4;
    int32_t const margin_y = (box_y2 - box_y1) / 4;
    int32_t const roi_x1 = box_x1 + margin_x;
    int32_t const roi_y1 = box_y1 + margin_y;
    int32_t const roi_x2 = box_x2 - margin_x;
    int32_t const roi_y2 = box_y2 - margin_y;

    if ((roi_x2 <= roi_x1) || (roi_y2 <= roi_y1))
    {
        return p_model_box->class_id;
    }

    uint32_t green_pixels = 0U;
    uint32_t total_pixels = 0U;

    for (int32_t y = roi_y1; y < roi_y2; y++)
    {
        for (int32_t x = roi_x1; x < roi_x2; x++)
        {
            uint32_t const pixel_index = (((uint32_t) y * CAMERA_OV5640_WIDTH) + (uint32_t) x) *
                                         CAMERA_OV5640_BYTES_PER_PIXEL;
            uint16_t const pixel = (uint16_t) (p_rgb565_frame[pixel_index] |
                                               ((uint16_t) p_rgb565_frame[pixel_index + 1U] << 8));
            uint32_t const r = det_rgb565_r(pixel);
            uint32_t const g = det_rgb565_g(pixel);
            uint32_t const b = det_rgb565_b(pixel);
            int32_t const green_excess = (int32_t) (2U * g) - (int32_t) r - (int32_t) b;

            if ((g >= (r + DET_GREEN_R_MARGIN)) &&
                (g >= (b + DET_GREEN_B_MARGIN)) &&
                (green_excess >= DET_GREEN_EXCESS_MIN))
            {
                green_pixels++;
            }

            total_pixels++;
        }
    }

    if ((green_pixels * 100U) < (total_pixels * DET_GREEN_PIXEL_PERCENT_MIN))
    {
        return APP_DETECTION_CLASS_PURPLE_GRAPE;
    }

    return APP_DETECTION_CLASS_GREEN_GRAPE;
}

bool app_detection_init(void)
{
    fsp_err_t const err = g_rm_ethosu0.p_api->open(g_rm_ethosu0.p_ctrl, g_rm_ethosu0.p_cfg);
    g_detection_initialized = (FSP_SUCCESS == err) || (FSP_ERR_ALREADY_OPEN == err);

    return g_detection_initialized;
}

bool app_detection_run_frame(uint8_t const                 * p_rgb565_frame,
                             app_detection_result_t        * p_results,
                             uint32_t                        result_capacity,
                             uint32_t                      * p_result_count,
                             app_detection_text_writer_t     write_text)
{
    if ((NULL == p_rgb565_frame) || (NULL == p_results) || (NULL == p_result_count))
    {
        return false;
    }

    *p_result_count = 0U;

    if (!g_detection_initialized && !app_detection_init())
    {
        if (NULL != write_text)
        {
            write_text("DET_ERR npu_open\r\n");
        }

        return false;
    }

    det_preprocess_rgb565(p_rgb565_frame);
    RunModel(true);

    uint32_t candidate_count = 0U;
    det_decode_head(GetModelOutputPtr_p4_16x16_70454(), DET_P4_GRID, DET_P4_STRIDE, g_anchors_p4, &candidate_count);
    det_decode_head(GetModelOutputPtr_p5_8x8_70436(), DET_P5_GRID, DET_P5_STRIDE, g_anchors_p5, &candidate_count);

    uint32_t const output_count = det_nms(candidate_count);
    uint32_t const result_count = (output_count < result_capacity) ? output_count : result_capacity;

    for (uint32_t i = 0U; i < result_count; i++)
    {
        int32_t x1;
        int32_t y1;
        int32_t x2;
        int32_t y2;
        det_to_camera_coords(&g_outputs[i], &x1, &y1, &x2, &y2);
        int32_t const center_x = (x1 + x2) / 2;
        int32_t const center_y = (y1 + y2) / 2;

        p_results[i].class_id = det_refine_grape_class(p_rgb565_frame, &g_outputs[i]);
        p_results[i].x        = center_x;
        p_results[i].y        = center_y;

        if (NULL != write_text)
        {
            int const line_count = snprintf(g_det_line,
                                            sizeof(g_det_line),
                                            "DET class=%lu x=%ld y=%ld\r\n",
                                            (unsigned long) p_results[i].class_id,
                                            (long) p_results[i].x,
                                            (long) p_results[i].y);

            if ((line_count > 0) && ((size_t) line_count < sizeof(g_det_line)))
            {
                write_text(g_det_line);
            }
        }
    }

    *p_result_count = result_count;

    return true;
}
