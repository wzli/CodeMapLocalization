#pragma once
#include <stdint.h>

typedef struct {
    uint8_t* data;
    uint16_t width;
    uint16_t height;
} Image;

void convolution(Image* dst, const Image src, const Image kernel);
