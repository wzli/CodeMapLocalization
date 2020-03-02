#pragma once

#include "image_utils.h"

typedef struct {
    ImageMatrixComplex correlation_image;
    ImageMatrixComplex correlation_buffer;
} OpticalFlowContext;

void optical_flow_run(OpticalFlowContext* ctx, ImageMatrix frame);

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame);
