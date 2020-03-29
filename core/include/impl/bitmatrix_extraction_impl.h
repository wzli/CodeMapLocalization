#ifndef WIDTH
#error "define WIDTH macro before including this header"
#endif

#include "code_extraction.h"

#define TEMPLATE_CAT3(A, B, C) A##B##C
#define T(A, B, C) TEMPLATE_CAT3(A, B, C)

void T(img_to_bm, WIDTH, )(T(BitMatrix, WIDTH, ) dst, T(BitMatrix, WIDTH, ) mask,
        const ImageMatrix src, uint8_t low_thresh, uint8_t high_thresh) {
    assert(high_thresh >= low_thresh);
    for (uint8_t row = 0; row < (WIDTH); ++row) {
        dst[row] = 0;
        mask[row] = 0;
    }
    FOR_EACH_PIXEL(src) {
        if (row >= (WIDTH) || col >= (WIDTH)) {
            break;
        } else if (PIXEL(src, row, col) > high_thresh) {
            T(bm, WIDTH, _set_bit)(dst, row, col);
            T(bm, WIDTH, _set_bit)(mask, row, col);
        } else if (PIXEL(src, row, col) < low_thresh) {
            T(bm, WIDTH, _set_bit)(mask, row, col);
        }
    }
}

void T(bm, WIDTH, _to_img)(
        ImageMatrix* dst, const T(BitMatrix, WIDTH, ) src, const T(BitMatrix, WIDTH, ) mask) {
    IMG_SET_SIZE(*dst, (WIDTH), (WIDTH));
    FOR_EACH_PIXEL(*dst) {
        PIXEL(*dst, row, col) = T(bm, WIDTH, _get_bit)(mask, row, col)
                                        ? T(bm, WIDTH, _get_bit)(src, row, col) * UINT8_MAX
                                        : UINT8_MAX / 2;
    }
}

void T(bm, WIDTH, _from_axiscodes)(T(BitMatrix, WIDTH, ) matrix, T(BitMatrix, WIDTH, ) mask,
        const AxisCode* row_code, const AxisCode* col_code) {
    assert(matrix && mask && row_code && col_code);
    for (uint8_t i = 0; i < (WIDTH); ++i) {
        matrix[i] = ((T(col_code->bits.x, WIDTH, ) >> i) & 1) ? ~T(row_code->bits.x, WIDTH, )
                                                              : T(row_code->bits.x, WIDTH, );
        mask[i] = ((T(col_code->mask.x, WIDTH, ) >> i) & 1) ? T(row_code->mask.x, WIDTH, ) : 0;
    }
}

AxisCode T(bm, WIDTH, _extract_column_code)(T(uint, WIDTH, _t) row_estimate,
        const T(BitMatrix, WIDTH, ) matrix, const T(BitMatrix, WIDTH, ) mask,
        uint8_t min_row_samples) {
    assert(matrix && mask);
    AxisCode column_code = {0};
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
            T(column_code.bits.x, WIDTH, ) |= (T(uint, WIDTH, _t)) 1 << i;
            column_code.n_errors += mask_sum - row_diff;
        } else {
            row_estimate |= matrix[i] & mask[i];
            column_code.n_errors += row_diff;
        }
        column_code.n_samples += mask_sum;
        T(column_code.mask.x, WIDTH, ) |= (T(uint, WIDTH, _t)) 1 << i;
        for (uint8_t j = count_trailing_zeros(mask[i]); j < (WIDTH); ++j) {
            if (!T(bm, WIDTH, _get_bit)(mask, i, j)) {
                continue;
            }
            bit_votes[j] += (row_estimate >> j) & 1 ? 1 : -1;
            row_estimate &= ~((T(uint, WIDTH, _t)) 1 << j);
            row_estimate |= (T(uint, WIDTH, _t))((bit_votes[j]) > 0) << j;
        }
    }
    return column_code;
}

void T(bm, WIDTH, _extract_axiscodes)(AxisCode* row_code, AxisCode* col_code,
        T(BitMatrix, WIDTH, ) matrix, T(BitMatrix, WIDTH, ) mask, uint8_t min_samples) {
    assert(row_code && col_code);
    *col_code = T(bm, WIDTH, _extract_column_code)(matrix[(WIDTH) / 2], matrix, mask, min_samples);
    T(bm, WIDTH, _transpose)(matrix);
    T(bm, WIDTH, _transpose)(mask);
    *row_code = T(bm, WIDTH, _extract_column_code)(
            T(col_code->bits.x, WIDTH, ), matrix, mask, min_samples);
    uint8_t offset = count_trailing_zeros(T(col_code->mask.x, WIDTH, ));
    T(col_code->bits.x, WIDTH, ) >>= offset;
    T(col_code->mask.x, WIDTH, ) >>= offset;
    offset = count_trailing_zeros(T(row_code->mask.x, WIDTH, ));
    T(row_code->bits.x, WIDTH, ) >>= offset;
    T(row_code->mask.x, WIDTH, ) >>= offset;
}

uint8_t T(ac, WIDTH, _downsample)(AxisCode* axiscode, float scale) {
    assert(scale > 0 && scale <= 1);
    uint32_t bits[(WIDTH) / 32] = {T(axiscode->bits.x, WIDTH, )};
    uint32_t mask[(WIDTH) / 32] = {T(axiscode->mask.x, WIDTH, )};
    if ((WIDTH) > 32) {
        bits[1] = (uint32_t)(axiscode->bits.x64 >> 32);
        mask[1] = (uint32_t)(axiscode->mask.x64 >> 32);
    }
    uint32_t scaled_bits[3 * (WIDTH) / 32];
    uint32_t scaled_mask[3 * (WIDTH) / 32];
    uint8_t offset;
    uint8_t scaled_len = bv32_scale(scaled_bits, bits, 3 * (WIDTH), (WIDTH), scale * 3);
    bv32_scale(scaled_mask, mask, 3 * (WIDTH), (WIDTH), scale * 3);
    estimate_bit_triplet_offset(&offset, scaled_bits, scaled_mask, 3 * (WIDTH));
    uint8_t bit_errors = downsample_triplet_code(
            bits, mask, (WIDTH), scaled_bits, scaled_mask, scaled_len, offset);
    T(axiscode->bits.x, WIDTH, ) = bits[0];
    T(axiscode->mask.x, WIDTH, ) = mask[0];
    if ((WIDTH) > 32) {
        axiscode->bits.x64 |= (uint64_t) bits[1] << 32;
        axiscode->mask.x64 |= (uint64_t) mask[1] << 32;
    }
    return bit_errors;
}

#undef WIDTH
#undef TEMPLATE_CAT3
#undef T
