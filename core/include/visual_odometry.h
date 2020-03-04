#pragma once
#include "image_utils.h"

typedef struct {
    ImageMatrixComplex image;
    ImageMatrixComplex buffer;
    Vector2f translation;
    float max_squared_magnitude;
} Correlation;

Vector2f img_estimate_rotation(const ImageMatrix mat);

void img_estimate_translation(Correlation* correlation, const ImageMatrix frame);

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame);
