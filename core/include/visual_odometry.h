#pragma once
#include "image_utils.h"

typedef struct {
    ImageMatrixComplex image;
    ImageMatrixComplex buffer;
    Vector2f translation;
    float max_squared_magnitude;
} Correlation;

typedef struct {
    Correlation correlation;
    Vector2f position;
    Vector2f quadrant_rotation;
    int32_t quadrant_count;
} OdometryContext;

static const Vector2f QUADRANT_LOOKUP[4] = {{{1, 0}}, {{0, 1}}, {{-1, 0}}, {{0, -1}}};

void odom_update(OdometryContext* ctx, ImageMatrix image, Vector2f rotation, float scale);

int8_t odom_track_rotation(OdometryContext* ctx, Vector2f quadrant_rotation);

Vector2f img_estimate_rotation(const ImageMatrix mat);

void img_estimate_translation(Correlation* correlation, const ImageMatrix frame);

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame);

Vector2f img_subpixel_registration(const Correlation* correlation);
