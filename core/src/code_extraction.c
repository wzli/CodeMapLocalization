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

uint8_t estimate_bit_triplet_offset(
        uint32_t* bit_errors, const uint32_t* bits, const uint32_t* mask, uint16_t len) {
    assert(bit_errors && bits && mask && len > 0 && len <= 30);
    static const uint64_t repeating001s = 0x4924924924924924ull;
    uint8_t offset = 0;
    *bit_errors = UINT8_MAX;
    for (uint8_t i = 0; i < 3; ++i) {
        uint32_t offset_bit_errors = 0;
        for (uint32_t j = 0; j < len; ++j) {
            uint32_t edges = bits[j] ^ (bits[j] << 1);
            uint32_t match_pattern = repeating001s >> (32 - (j + i));
            offset_bit_errors += count_bits((edges ^ match_pattern) & mask[j]);
        }
        if (offset_bit_errors < *bit_errors) {
            *bit_errors = offset_bit_errors;
            offset = i;
        }
    }
    return offset;
}
