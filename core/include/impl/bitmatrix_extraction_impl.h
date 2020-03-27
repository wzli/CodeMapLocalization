#ifndef WIDTH
#error "define WIDTH macro before including this header"
#endif

#include "code_extraction.h"

#define TEMPLATE_CAT3(A, B, C) A##B##C
#define TEMPLATE(A, B, C) TEMPLATE_CAT3(A, B, C)

void TEMPLATE(img_to_bm, WIDTH, )(TEMPLATE(BitMatrix, WIDTH, ) dst,
        TEMPLATE(BitMatrix, WIDTH, ) mask, const ImageMatrix src, uint8_t low_thresh,
        uint8_t high_thresh) {
    assert(high_thresh >= low_thresh);
    for (uint8_t row = 0; row < (WIDTH); ++row) {
        dst[row] = 0;
        mask[row] = 0;
    }
    FOR_EACH_PIXEL(src) {
        if (row >= (WIDTH) || col >= (WIDTH)) {
            break;
        } else if (PIXEL(src, row, col) > high_thresh) {
            TEMPLATE(bm, WIDTH, _set_bit)(dst, row, col);
            TEMPLATE(bm, WIDTH, _set_bit)(mask, row, col);
        } else if (PIXEL(src, row, col) < low_thresh) {
            TEMPLATE(bm, WIDTH, _set_bit)(mask, row, col);
        }
    }
}

void TEMPLATE(bm, WIDTH, _to_img)(ImageMatrix* dst, const TEMPLATE(BitMatrix, WIDTH, ) src,
        const TEMPLATE(BitMatrix, WIDTH, ) mask) {
    IMG_SET_SIZE(*dst, (WIDTH), (WIDTH));
    FOR_EACH_PIXEL(*dst) {
        PIXEL(*dst, row, col) = TEMPLATE(bm, WIDTH, _get_bit)(mask, row, col)
                                        ? TEMPLATE(bm, WIDTH, _get_bit)(src, row, col) * UINT8_MAX
                                        : UINT8_MAX / 2;
    }
}

void TEMPLATE(bm, WIDTH, _from_axiscodes)(TEMPLATE(BitMatrix, WIDTH, ) dst,
        TEMPLATE(BitMatrix, WIDTH, ) mask, const TEMPLATE(AxisCode, WIDTH, ) row_code,
        const TEMPLATE(AxisCode, WIDTH, ) col_code) {
    for (uint8_t i = 0; i < (WIDTH); ++i) {
        dst[i] = ((col_code.bits >> i) & 1) ? ~row_code.bits : row_code.bits;
        mask[i] = ((col_code.mask >> i) & 1) ? row_code.mask : 0;
    }
}

TEMPLATE(AxisCode, WIDTH, )
TEMPLATE(bm, WIDTH, _extract_column_code)
(TEMPLATE(uint, WIDTH, _t) row_estimate, const TEMPLATE(BitMatrix, WIDTH, ) matrix,
        const TEMPLATE(BitMatrix, WIDTH, ) mask, uint8_t min_row_samples) {
    assert(matrix && mask);
    TEMPLATE(AxisCode, WIDTH, ) column_code = {0};
    int8_t bit_votes[(WIDTH)] = {0};
    for (uint8_t i = 0; i < (WIDTH); ++i) {
        uint8_t mask_sum = count_bits(mask[i]);
        if (mask_sum < min_row_samples) {
            continue;
        }
        uint8_t row_diff = count_bits((row_estimate ^ matrix[i]) & mask[i]);
        row_estimate &= ~mask[i];
        if (2 * row_diff > mask_sum) {
            row_estimate |= ~matrix[i] & mask[i];
            column_code.bits |= (TEMPLATE(uint, WIDTH, _t)) 1 << i;
            column_code.n_errors += mask_sum - row_diff;
        } else {
            row_estimate |= matrix[i] & mask[i];
            column_code.n_errors += row_diff;
        }
        column_code.n_samples += mask_sum;
        column_code.mask |= (TEMPLATE(uint, WIDTH, _t)) 1 << i;
        for (uint8_t j = count_trailing_zeros(mask[i]); j < (WIDTH); ++j) {
            if (!TEMPLATE(bm, WIDTH, _get_bit)(mask, i, j)) {
                continue;
            }
            bit_votes[j] += (row_estimate >> j) & 1 ? 1 : -1;
            row_estimate &= ~((TEMPLATE(uint, WIDTH, _t)) 1 << j);
            row_estimate |= (TEMPLATE(uint, WIDTH, _t))((bit_votes[j]) > 0) << j;
        }
    }
    return column_code;
}

void TEMPLATE(bm, WIDTH, _extract_axiscodes)(TEMPLATE(AxisCode, WIDTH, ) * row_code,
        TEMPLATE(AxisCode, WIDTH, ) * col_code, TEMPLATE(BitMatrix, WIDTH, ) matrix,
        TEMPLATE(BitMatrix, WIDTH, ) mask, uint8_t min_samples) {
    assert(row_code && col_code);
    *col_code = TEMPLATE(bm, WIDTH, _extract_column_code)(
            matrix[(WIDTH) / 2], matrix, mask, min_samples);
    TEMPLATE(bm, WIDTH, _transpose)(matrix);
    TEMPLATE(bm, WIDTH, _transpose)(mask);
    *row_code =
            TEMPLATE(bm, WIDTH, _extract_column_code)(col_code->bits, matrix, mask, min_samples);
    uint8_t offset = count_trailing_zeros(col_code->mask);
    col_code->bits >>= offset;
    col_code->mask >>= offset;
    offset = count_trailing_zeros(row_code->mask);
    row_code->bits >>= offset;
    row_code->mask >>= offset;
}

uint8_t TEMPLATE(ac, WIDTH, _downsample)(TEMPLATE(AxisCode, WIDTH, ) * axiscode, float scale) {
    assert(scale > 0 && scale <= 1);
    uint32_t bits[(WIDTH) / 32] = {axiscode->bits};
    uint32_t mask[(WIDTH) / 32] = {axiscode->mask};
    if ((WIDTH) > 32) {
        bits[1] = (uint32_t)((uint64_t) axiscode->bits >> 32);
        mask[1] = (uint32_t)((uint64_t) axiscode->mask >> 32);
    }
    uint32_t scaled_bits[3 * (WIDTH) / 32] = {0};
    uint32_t scaled_mask[3 * (WIDTH) / 32] = {0};
    uint8_t scaled_len = bv32_scale(scaled_bits, bits, 3 * (WIDTH), (WIDTH), scale * 3);
    bv32_scale(scaled_mask, mask, 3 * (WIDTH), (WIDTH), scale * 3);
    uint8_t offset;
    estimate_bit_triplet_offset(&offset, scaled_bits, scaled_mask, 3 * (WIDTH));
    uint8_t bit_errors = downsample_triplet_code(
            bits, mask, (WIDTH), scaled_bits, scaled_mask, scaled_len, offset);
    axiscode->bits = bits[0];
    axiscode->mask = mask[0];
    if ((WIDTH) > 32) {
        axiscode->bits |= (uint64_t) bits[1] << 32;
        axiscode->mask |= (uint64_t) mask[1] << 32;
    }
    return bit_errors;
}

#undef WIDTH
#undef TEMPLATE_CAT3
#undef TEMPLATE
