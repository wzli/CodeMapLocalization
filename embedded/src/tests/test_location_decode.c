#include "test_utils.h"
#include "location_decode.h"
#include "mls_query.h"

#define TEST_VECTOR_SIZE 20

static int test_next_valid_code_segment() {
    AxisCode32 test_code = {0xFFFF00FF, 0xFFFF00FF, 0, 1};
    uint8_t valid_segment_length;
    valid_segment_length = next_valid_code_segment(&test_code, 1);
    test_assert(test_code.bits == 0xFFFF00FF);
    test_assert(test_code.mask == 0xFFFF00FF);
    test_assert(valid_segment_length == 8);
    valid_segment_length = next_valid_code_segment(&test_code, 9);
    test_assert(valid_segment_length == 16);
    test_assert(test_code.bits == 0x0000FFFF);
    test_assert(test_code.mask == 0x0000FFFF);
    test_code = (AxisCode32){~0, ~0, 0, 1};
    test_assert(next_valid_code_segment(&test_code, 15) == 32);
    test_assert(test_code.bits == ~0u);
    test_assert(test_code.mask == ~0u);
    test_code = (AxisCode32){0, 0, 0, 1};
    test_assert(next_valid_code_segment(&test_code, 15) == 0);
    test_assert(test_code.bits == 0);
    test_assert(test_code.mask == 0);
    return 0;
}

static int test_location_decode_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            AxisCode32 row_code = {src_row_code, ~0, 0, 0};
            AxisCode32 col_code = {src_col_code, ~0, 0, 0};
            bm32_from_axis_codes(matrix, matrix_mask, row_code, col_code);
            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);
            test_assert(row_code.bits == src_row_code ||
                        invert_bits(row_code.bits, 32) == src_row_code);
            test_assert(col_code.bits == src_col_code ||
                        invert_bits(col_code.bits, 32) == src_col_code);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + row_pos.span / 2 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_reversed_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            src_row_code = reverse_bits(src_row_code, 32);
            src_col_code = reverse_bits(src_col_code, 32);
            AxisCode32 row_code = {src_row_code, ~0, 0, 0};
            AxisCode32 col_code = {src_col_code, ~0, 0, 0};
            bm32_from_axis_codes(matrix, matrix_mask, row_code, col_code);
            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 1);
            test_assert(col_pos.reversed == 1);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + (row_pos.span + 1) / 2 == row_pos.center);
            test_assert(src_col_pos + (col_pos.span + 1) / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_punctured_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0xA;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }
            AxisCode32 row_code, col_code;
            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 0);
            test_assert(col_pos.reversed == 0);
            test_assert(row_pos.span + MLS_INDEX.code_length == 33 - 4);
            test_assert(col_pos.span + MLS_INDEX.code_length == 33);
            test_assert(src_row_pos + row_pos.span / 2 + 4 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_truncated_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0x400000F;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }
            AxisCode32 row_code, col_code;
            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 0);
            test_assert(col_pos.reversed == 0);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length - 10);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + row_pos.span / 2 + 4 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_full_chain() {
    for (int i = 0; i < TEST_VECTOR_SIZE; ++i) {
        // src image setup
        static uint8_t src_img_buf[30 * 30];
        static ImageMatrix src_img = {src_img_buf, 30, 30};
        FOR_EACH_PIXEL(src_img) { PIXEL(src_img, row, col) = row + col; }

        // unrotate image
        static uint8_t img_buf[32 * 32];
        static ImageMatrix img = {img_buf, 32, 32};
        Vector2f rot = img_estimate_rotation(src_img);
        img_rotate(img, src_img, rot, 127, img_bilinear_interpolation);

        // convert to bit matrix
        static BitMatrix32 bit_matrix, bit_mask;
        img_to_bm32(bit_matrix, bit_mask, img, 125, 130);

        // extract codes
        static AxisCode32 row_code, col_code;
        bm32_extract_axis_codes(&row_code, &col_code, bit_matrix, bit_mask, 3);

        // overwrite with simulate extracted code
        row_code.bits = mlsq_code_from_position(MLS_INDEX.sequence, 32, i);
        row_code.mask = ~0;
        col_code = row_code;

        // decode positions
        AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
        AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

        // deduce location
        Location loc = deduce_location(row_pos, col_pos);

        // expect a valid location
        test_assert(loc.x == i + row_pos.span / 2);
        test_assert(loc.y == i + col_pos.span / 2);
    }
    return 0;
}

int test_location_decode() {
    test_run(test_next_valid_code_segment);
    test_run(test_location_decode_blocks);
    test_run(test_location_decode_reversed_blocks);
    test_run(test_location_decode_punctured_blocks);
    test_run(test_location_decode_truncated_blocks);
    test_run(test_location_decode_full_chain);
    return 0;
}
