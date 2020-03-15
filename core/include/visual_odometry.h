#pragma once
#include "image_utils.h"

typedef struct {
    ImageMatrixComplex image;
    ImageMatrixComplex buffer;
    Vector2f translation;
    float max_squared_magnitude;
} Correlation;

typedef struct {
    Vector2f quadrant_rotation;
    int32_t quadrant_count;
} OdometryContext;

Vector2f img_estimate_rotation(const ImageMatrix mat);

void track_rotation(OdometryContext* ctx, Vector2f quadrant_rotation);

void img_estimate_translation(Correlation* correlation, const ImageMatrix frame);

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame);

Vector2f subpixel_registration(const Correlation* correlation);
