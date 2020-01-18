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
    bm64_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 5);
    test_assert((row_code.bits == src_row_code && col_code.bits == src_col_code) ||
                (row_code.bits == ~src_row_code && col_code.bits == ~src_col_code));
    bm64_from_axis_codes(matrix, matrix_mask, row_code, col_code);
    for (uint8_t i = 0; i < 64; ++i) {
        test_assert(matrix_mask[i] == ~0ull);
        test_assert(matrix[i] == src_row_code || matrix[i] == ~src_row_code);
    }
    free(matrix);
    free(matrix_mask);
    return 0;
}

static int test_bit_matrix_from_axis_codes() {
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
    AxisCode32 row_axis_code = {src_row_code, ~0, 0, 0};
    AxisCode32 col_axis_code = {src_col_code, ~0, 0, 0};
    bm32_from_axis_codes(generated_matrix, generated_matrix_mask, row_axis_code, col_axis_code);
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

static int test_downsample_axis_code() {
    AxisCode64 axis_code_64 = {0x71C71C71C71C71C7ull, ~0ull, 0, 0};
    AxisCode32 axis_code_32 = downsample_axis_code(axis_code_64);
    test_assert(axis_code_32.bits == 0x155555);
    test_assert(axis_code_32.mask == (1 << 21) - 1);

    axis_code_64.bits = 0xF0F0;
    axis_code_64.mask = 0xFFFF;
    AxisCode64 scaled_axis_code = scale_axis_code(axis_code_64, 1.0f);
    test_assert(axis_code_64.bits == scaled_axis_code.bits);
    test_assert(axis_code_64.mask == scaled_axis_code.mask);

    scaled_axis_code = scale_axis_code(axis_code_64, 2.0f);
    test_assert(scaled_axis_code.bits == 0xFF00FF00);
    test_assert(scaled_axis_code.mask == 0xFFFFFFFF);

    scaled_axis_code = scale_axis_code(axis_code_64, 100.0f);
    test_assert(scaled_axis_code.bits == 0ull);
    test_assert(scaled_axis_code.mask == ~0ull);

    scaled_axis_code = scale_axis_code(axis_code_64, 0.5f);
    test_assert(scaled_axis_code.bits == 0xCC);
    test_assert(scaled_axis_code.mask == 0xFF);
    return 0;
}

int test_code_extraction() {
    test_run(test_hyper_sharpen);
    test_run(test_code_extract_64);
    test_run(test_bit_matrix_from_axis_codes);
    test_run(test_downsample_axis_code);
    return 0;
}
