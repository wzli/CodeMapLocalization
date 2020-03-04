#pragma once
#include "location_decode.h"
#include "visual_odometry.h"

typedef struct {
    ImageMatrix unrotated_image;
    ImageMatrix sharpened_image;
    BitMatrix64 binary_image;
    BitMatrix64 binary_mask;
    Correlation correlation;
    ScaleQuery scale_query;
    ScaleMatch scale_match;
    Vector2f rotation_estimate;
    float rotation_scale;
    uint32_t histogram[256];
    uint8_t threshold[2];
} LocalizationContext;

void localization_loop_run(LocalizationContext* ctx, const ImageMatrix image);

Vector2f img_derotation_filter(
        ImageMatrix dst, const ImageMatrix src, float rotation_scale, uint8_t bg_fill);
