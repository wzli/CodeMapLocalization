#pragma once
#include "location_decode.h"
#include "visual_odometry.h"

typedef struct {
    ImageMatrix derotated_image;
    ImageMatrix sharpened_image;
    ImageMatrix denoised_image;
    BitMatrix64 binary_image;
    BitMatrix64 binary_mask;
    ScaleQuery scale_query;
    ScaleMatch scale_match;
    OutlierFilter outlier_filter;
    VisualOdometry odom;
    float rotation_scale;
    uint32_t histogram[256];
    uint8_t threshold[2];
    uint32_t frame_count;
} LocalizationContext;

bool localization_loop_run(LocalizationContext* ctx, const ImageMatrix image);

Vector2f img_derotate(ImageMatrix dst, const ImageMatrix src, float scale, uint8_t bg_fill);
