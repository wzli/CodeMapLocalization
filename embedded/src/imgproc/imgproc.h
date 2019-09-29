#pragma once
#include <stdint.h>

typedef struct {
    int16_t* data;
    uint16_t width;
    uint16_t height;
} ImageMatrix;

void convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);
