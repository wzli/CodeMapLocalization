#pragma once
#include "location_decode.h"
#include "optical_flow.h"

typedef struct {
    ImageMatrix original_image;
    ImageMatrix unrotated_image;
    ImageMatrix sharpened_image;
    BitMatrix64 binary_image;
    BitMatrix64 binary_mask;
    ScaleQuery scale_query;
    ScaleMatch scale_match;
    Vector2f rotation_estimate;
    float rotation_scale;
    uint32_t histogram[256];
    uint8_t threshold[2];
    OpticalFlowContext flow_ctx;
} LocalizationContext;

void localization_loop_run(LocalizationContext* ctx);
