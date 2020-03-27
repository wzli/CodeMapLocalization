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
        uint32_t* bit_errors, const uint32_t* bits, const uint32_t* mask, uint8_t len) {
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

uint32_t downsample_axiscode_64(
        uint32_t* bits, uint32_t* mask, float scale, const AxisCode64* axiscode) {
    assert(scale > 0 && scale <= 1);
    bits[0] = (uint32_t) axiscode->bits;
    bits[1] = (uint32_t)(axiscode->bits >> 32);
    mask[0] = (uint32_t) axiscode->mask;
    mask[1] = (uint32_t)(axiscode->mask >> 32);
    uint32_t scaled_bits[6] = {0};
    uint32_t scaled_mask[6] = {0};
    uint8_t scaled_len = bv32_scale(scaled_bits, bits, 6 * 32, 2 * 32, scale * 3);
    bv32_scale(scaled_mask, mask, 6 * 32, 2 * 32, scale * 3);
    uint32_t bit_errors;
    uint8_t offset = estimate_bit_triplet_offset(&bit_errors, scaled_bits, scaled_mask, 6);

    bv32_clear_all(bits, 2 * 32);
    bv32_clear_all(mask, 2 * 32);

    static const uint8_t count_bits_3[8] = {0, 1, 1, 2, 1, 2, 2, 3};
    for (uint8_t i = 0; offset < scaled_len - 3; offset += 3, ++i) {
        uint8_t mask_triplet = bv32_get_slice(scaled_mask, offset, 3);
        if (mask_triplet == 7) {
            bv32_set_bit(mask, i);
            uint8_t bit_triplet = bv32_get_slice(scaled_bits, offset, 3);
            if (2 * count_bits_3[bit_triplet] > 3) {
                bv32_set_bit(bits, i);
            }
            if (bit_triplet != 0 && bit_triplet != 7) {
                bit_errors += 1;
            }
        }
    }
    return bit_errors;
}
