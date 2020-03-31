#pragma once
#include "mls_query.h"
#include "location_decode.h"
#include "visual_odometry.h"

typedef struct {
    ImageMatrix derotated_image;
    ImageMatrix sharpened_image;
    BitMatrix64 binary_image;
    BitMatrix64 binary_mask;
    AxisCode row_code;
    AxisCode col_code;
    ScaleMatch scale_match;
    OutlierFilter outlier_filter;
    VisualOdometry odom;
    float rotation_scale;
    float scale_decay_rate;
    uint32_t histogram[256];
    uint32_t frame_count;
    uint8_t otsu_threshold;
} LocalizationContext;

bool localization_loop_run(LocalizationContext* ctx, const ImageMatrix image);

Vector2f img_derotate(ImageMatrix dst, const ImageMatrix src, float scale, uint8_t bg_fill);
