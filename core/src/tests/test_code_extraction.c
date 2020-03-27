#include "test_utils.h"
#include "code_extraction.h"
#include "mls_query.h"
#include <stdlib.h>

static int test_hyper_sharpen() {
    ImageMatrix img = {malloc(32 * 32), {32, 32}};
    ImageMatrix ref_img = {malloc(32 * 32), {32, 32}};
    ImageMatrixInt8 kernel = {(int8_t[9]){-3, -3, -3, -3, 25, -3, -3, -3, -3}, {3, 3}};
    IMG_FILL(img, 0);
    FOR_EACH_PIXEL(img) { PIXEL(img, row, col) = 255 * (row & 1); }
    img_convolution_filter(&ref_img, img, kernel);
    img_hyper_sharpen(&img, img);
    FOR_EACH_PIXEL(img) { test_assert(PIXEL(img, row, col) == PIXEL(ref_img, row, col)); }
    free(img.data);
    free(ref_img.data);
    return 0;
}

static int test_code_extract_64() {
    uint64_t* matrix = malloc(64 * sizeof(uint64_t));
    uint64_t* matrix_mask = malloc(64 * sizeof(uint64_t));
    uint32_t src_row_pos = 1000;
    uint32_t src_col_pos = 1001;
    uint64_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
    src_row_code |= (uint64_t) mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos + 32)
                    << 32;
    uint64_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
    src_col_code |= (uint64_t) mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos + 32)
                    << 32;
    for (uint8_t i = 0; i < 64; ++i) {
        matrix_mask[i] = ~0ull;
        matrix[i] = src_row_code ^ -((uint64_t)(src_col_code >> i) & 1ull);
    }
    AxisCode64 col_code = bm64_extract_column_code(0, matrix, matrix_mask, 5);
    bm64_transpose(matrix);
    bm64_transpose(matrix_mask);
    AxisCode64 row_code = bm64_extract_column_code(col_code.bits, matrix, matrix_mask, 5);
    test_assert((row_code.bits == src_row_code && col_code.bits == src_col_code) ||
                (row_code.bits == ~src_row_code && col_code.bits == ~src_col_code));
    bm64_transpose(matrix);
    bm64_transpose(matrix_mask);
    bm64_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 5);
    test_assert((row_code.bits == src_row_code && col_code.bits == src_col_code) ||
                (row_code.bits == ~src_row_code && col_code.bits == ~src_col_code));
    bm64_from_axiscodes(matrix, matrix_mask, row_code, col_code);
    for (uint8_t i = 0; i < 64; ++i) {
        test_assert(matrix_mask[i] == ~0ull);
        test_assert(matrix[i] == src_row_code || matrix[i] == ~src_row_code);
    }
    free(matrix);
    free(matrix_mask);
    return 0;
}

static int test_bit_matrix_from_axiscodes() {
    uint32_t* matrix = malloc(32 * sizeof(uint32_t));
    uint32_t* matrix_mask = malloc(32 * sizeof(uint32_t));
    uint32_t* generated_matrix = malloc(32 * sizeof(uint32_t));
    uint32_t* generated_matrix_mask = malloc(32 * sizeof(uint32_t));
    uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, 1000);
    uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, 1100);
    for (uint8_t i = 0; i < 32; ++i) {
        matrix_mask[i] = ~0;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
    }
    AxisCode32 row_axiscode = {src_row_code, ~0, 0, 0};
    AxisCode32 col_axiscode = {src_col_code, ~0, 0, 0};
    bm32_from_axiscodes(generated_matrix, generated_matrix_mask, row_axiscode, col_axiscode);
    for (uint8_t i = 0; i < 32; ++i) {
        test_assert(matrix[i] == generated_matrix[i]);
        test_assert(matrix_mask[i] == generated_matrix_mask[i]);
    }
    free(matrix);
    free(matrix_mask);
    free(generated_matrix);
    free(generated_matrix_mask);
    return 0;
}

static int test_estimate_bit_triplet_offset() {
    uint32_t bits[3] = {0};
    uint32_t mask[3] = {~0, ~0, ~0};
    uint32_t bit_errors;
    uint32_t comb_pattern = 0xAAAAAAAA;
    bv32_scale(bits, &comb_pattern, 3 * 32, 32, 2.999);
    uint8_t offset = estimate_bit_triplet_offset(&bit_errors, bits, mask, 3 * 32);
    test_assert(offset == 0);
    test_assert(bit_errors <= 2);

    bits[0] <<= 1;
    bits[1] <<= 1;
    bits[2] <<= 1;
    offset = estimate_bit_triplet_offset(&bit_errors, bits, mask, 3 * 32);
    test_assert(offset == 1);
    test_assert(offset <= 3);

    bits[0] <<= 1;
    bits[1] <<= 1;
    bits[2] <<= 1;
    offset = estimate_bit_triplet_offset(&bit_errors, bits, mask, 3 * 32);
    test_assert(offset == 2);
    test_assert(offset <= 4);

    bits[0] <<= 1;
    bits[1] <<= 1;
    bits[2] <<= 1;
    offset = estimate_bit_triplet_offset(&bit_errors, bits, mask, 3 * 32);
    test_assert(offset == 0);
    test_assert(offset <= 5);
    return 0;
}

static int test_downsample_triplet_code() {
    uint32_t dst_bits[2] = {0};
    uint32_t dst_mask[2] = {0};
    uint32_t src_bits[6] = {0, ~0, ~0, ~0, ~0, ~0};
    uint32_t src_mask[6] = {0, 0, 0, ~0, ~0, ~0};
    uint32_t bit_errors =
            downsample_triplet_code(dst_bits, dst_mask, 2 * 32, src_bits, src_mask, 6 * 32, 0);
    test_assert(bit_errors == 0);
    test_assert(dst_bits[0] == 0);
    test_assert(dst_bits[1] == ~0u);
    test_assert(dst_mask[0] == 0);
    test_assert(dst_mask[1] == ~0u);

    bv32_clear_all(src_bits, 6 * 32);
    src_bits[3] = 0xFAC688;
    bit_errors = downsample_triplet_code(dst_bits, dst_mask, 64, src_bits, src_mask, 6 * 32, 0);
    test_assert(bit_errors == 6);
    test_assert(bv32_get_bit(dst_bits, 32 + 0) == 0);  // 000
    test_assert(bv32_get_bit(dst_bits, 32 + 1) == 0);  // 001
    test_assert(bv32_get_bit(dst_bits, 32 + 2) == 0);  // 010
    test_assert(bv32_get_bit(dst_bits, 32 + 3) == 1);  // 011
    test_assert(bv32_get_bit(dst_bits, 32 + 4) == 0);  // 100
    test_assert(bv32_get_bit(dst_bits, 32 + 5) == 1);  // 101
    test_assert(bv32_get_bit(dst_bits, 32 + 6) == 1);  // 110
    test_assert(bv32_get_bit(dst_bits, 32 + 7) == 1);  // 111

    bv32_clear_all(src_bits, 6 * 32);
    bv32_clear_all(src_mask, 6 * 32);
    src_bits[0] = 0x3FE00;
    src_mask[0] = 0x22311;
    bit_errors = downsample_triplet_code(dst_bits, dst_mask, 64, src_bits, src_mask, 6 * 32, 0);
    test_assert(bit_errors == 0);
    test_assert(bv32_get_bit(dst_bits, 0) == 0);
    test_assert(bv32_get_bit(dst_bits, 1) == 0);
    test_assert(bv32_get_bit(dst_bits, 2) == 0);
    test_assert(bv32_get_bit(dst_bits, 3) == 1);
    test_assert(bv32_get_bit(dst_bits, 4) == 1);
    test_assert(bv32_get_bit(dst_bits, 5) == 1);

    src_bits[0] = 0x1511;
    src_mask[0] = 0x5D9B;
    bit_errors = downsample_triplet_code(dst_bits, dst_mask, 64, src_bits, src_mask, 6 * 32, 0);

    test_assert(bit_errors == 4);
    test_assert(bv32_get_bit(dst_bits, 0) == 0);
    test_assert(bv32_get_bit(dst_bits, 1) == 1);
    test_assert(bv32_get_bit(dst_bits, 2) == 0);
    test_assert(bv32_get_bit(dst_bits, 3) == 1);
    test_assert(bv32_get_bit(dst_bits, 4) == 0);
    test_assert(bv32_get_bit(dst_mask, 0) == 1);
    test_assert(bv32_get_bit(dst_mask, 1) == 1);
    test_assert(bv32_get_bit(dst_mask, 2) == 1);
    test_assert(bv32_get_bit(dst_mask, 3) == 1);
    test_assert(bv32_get_bit(dst_mask, 4) == 0);

    return 0;
}

int test_code_extraction() {
    test_run(test_hyper_sharpen);
    test_run(test_code_extract_64);
    test_run(test_bit_matrix_from_axiscodes);
    test_run(test_estimate_bit_triplet_offset);
    test_run(test_downsample_triplet_code);
    return 0;
}
