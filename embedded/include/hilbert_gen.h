#pragma once

#include "stdint.h"

typedef enum {
    HILBERT_CURVE_RIGHT = 0,  // 0b00
    HILBERT_CURVE_UP = 1,     // 0b01
    HILBERT_CURVE_LEFT = 2,   // 0b10
    HILBERT_CURVE_DOWN = 3,   // 0b11
} HilbertCurveDirection;

#define HILBERT_CURVE_FOR_EACH_XY(curve, order)                                               \
    for (int32_t x = 0, y = 0, i = 0, curve_len = hilbert_curve_length(order); i < curve_len; \
            hilbert_curve_increment_xy(hilbert_curve_query(curve, i++), &x, &y))

static inline HilbertCurveDirection hilbert_curve_query(const uint8_t* curve, uint32_t index) {
    return (curve[index / 4] >> (2 * (index & 3))) & 3;
};

static inline uint32_t hilbert_curve_length(uint8_t order) {
    return 1 << (2 * order);
}

static inline void hilbert_curve_increment_xy(HilbertCurveDirection dir, int32_t* x, int32_t* y) {
    switch (dir) {
        case HILBERT_CURVE_RIGHT:
            ++*x;
            break;
        case HILBERT_CURVE_UP:
            ++*y;
            break;
        case HILBERT_CURVE_LEFT:
            --*x;
            break;
        case HILBERT_CURVE_DOWN:
            --*y;
            break;
    }
}

void hilbert_curve_generate(uint8_t* curve, uint8_t order);
