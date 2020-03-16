#include "code_extraction.h"
#include <assert.h>

#define WIDTH 32
#include "bm_extraction_impl.h"

#define WIDTH 64
#include "bm_extraction_impl.h"

void img_hyper_sharpen(ImageMatrix* dst, const ImageMatrix src) {
    IMG_SET_SIZE(*dst, src.size.x - 2, src.size.y - 2);
    FOR_EACH_PIXEL(*dst) {
        int16_t value = 17 * PIXEL(src, row + 1, col + 1) -
                        2 * (PIXEL(src, row, col) + PIXEL(src, row, col + 1) +
                                    PIXEL(src, row, col + 2) + PIXEL(src, row + 1, col) +
                                    PIXEL(src, row + 1, col + 2) + PIXEL(src, row + 2, col) +
                                    PIXEL(src, row + 2, col + 1) + PIXEL(src, row + 2, col + 2));
        PIXEL(*dst, row, col) = CLAMP(value, 0, UINT8_MAX);
    }
}

AxisCode32 downsample_axiscode(AxisCode64 axiscode64) {
    uint64_t edges = axiscode64.bits ^ (axiscode64.bits >> 1);
    uint8_t offset_index = 0;
    uint8_t lowest_bit_errors = UINT8_MAX;
    for (uint64_t repeating001s = 0x1249249249249249ull; repeating001s & 7; repeating001s <<= 1) {
        uint8_t bit_errors = count_bits((edges ^ repeating001s) & axiscode64.mask);
        if (bit_errors < lowest_bit_errors) {
            lowest_bit_errors = bit_errors;
            offset_index = count_trailing_zeros((uint32_t) repeating001s);
        }
    }
    if (offset_index < 2) {
        axiscode64.bits >>= offset_index + 1;
        axiscode64.mask >>= offset_index + 1;
    }
    AxisCode32 axiscode32 = {0, 0, axiscode64.n_errors, axiscode64.n_samples};
    static const uint8_t count_bits_3[8] = {0, 1, 1, 2, 1, 2, 2, 3};
    uint32_t current_bit = 1;
    while (axiscode64.mask) {
        if (count_bits_3[axiscode64.mask & 7] >= 3) {
            axiscode32.mask |= current_bit;
            if (2 * count_bits_3[axiscode64.bits & 7] > 3) {
                axiscode32.bits |= current_bit;
            }
        }
        current_bit <<= 1;
        axiscode64.bits >>= 3;
        axiscode64.mask >>= 3;
    }
    return axiscode32;
};
