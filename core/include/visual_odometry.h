#pragma once
#include "image_utils.h"

typedef struct {
    ImageMatrixComplex image;
    ImageMatrixComplex buffer;
    Vector2f translation;
} Correlation;

void img_estimate_translation(Correlation* correlation, const ImageMatrix frame);

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame);
