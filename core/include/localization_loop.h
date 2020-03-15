#pragma once
#include "location_decode.h"
#include "visual_odometry.h"

typedef struct {
    ImageMatrix unrotated_image;
    ImageMatrix sharpened_image;
    BitMatrix64 binary_image;
    BitMatrix64 binary_mask;
    ScaleQuery scale_query;
    ScaleMatch scale_match;
    OutlierFilter outlier_filter;
    OdometryContext odom_ctx;
    float rotation_scale;
    uint32_t histogram[256];
    uint8_t threshold[2];
} LocalizationContext;

bool localization_loop_run(LocalizationContext* ctx, const ImageMatrix image);

Vector2f img_derotate(ImageMatrix dst, const ImageMatrix src, float scale, uint8_t bg_fill);
