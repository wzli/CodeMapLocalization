#pragma once
#include <stdint.h>

typedef struct {
    int16_t* data;
    int16_t n_cols;
    int16_t n_rows;
} ImageMatrix;

typedef struct {
    float x;
    float y;
} Vector2f;

void convolution(
        ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);
