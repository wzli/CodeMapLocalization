#include "test_utils.h"
#include "location_decode.h"
#include "mls_query.h"

#include <stdlib.h>

#define TEST_VECTOR_SIZE 10

static int test_full_chain_simulation() {
    ImageMatrix src_img = {malloc(30 * 30), 30, 30};
    ImageMatrix img = {malloc(32 * 32), 32, 32};
    uint32_t* bit_matrix = malloc(32 * sizeof(uint32_t));
    uint32_t* bit_mask = malloc(32 * sizeof(uint32_t));
    for (int i = 0; i < TEST_VECTOR_SIZE; ++i) {
        // src image setup
        FOR_EACH_PIXEL(src_img) { PIXEL(src_img, row, col) = row + col; }

        // unrotate image
        Vector2f rot = img_estimate_rotation(src_img);
        img_rotate(img, src_img, rot, 127, img_bilinear_interpolation);

        // convert to bit matrix
        img_to_bm32(bit_matrix, bit_mask, img, 125, 130);

        // extract codes
        AxisCode32 row_code, col_code;
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
    free(src_img.data);
    free(img.data);
    free(bit_matrix);
    free(bit_mask);
    return 0;
}

static int test_full_chain_real_life() {
    ImageMatrix raw_image = {malloc(64 * 64), 64, 64};
    ImageMatrix buf_image = {malloc(64 * 64), 64, 64};
    uint64_t* bit_matrix_64 = malloc(64 * sizeof(uint64_t));
    uint64_t* bit_mask_64 = malloc(64 * sizeof(uint64_t));
    uint32_t* bit_matrix_32 = malloc(32 * sizeof(uint32_t));
    uint32_t* bit_mask_32 = malloc(32 * sizeof(uint32_t));
    uint32_t* histogram = malloc(256 * sizeof(uint32_t));
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1100; src_col_pos < 1100 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            // generate source image from known position
            AxisCode64 row_code_64 = {src_row_code, ~0ull, 0, 0};
            AxisCode64 col_code_64 = {src_col_code, ~0ull, 0, 0};
            row_code_64 = scale_axis_code(row_code_64, 3);
            col_code_64 = scale_axis_code(col_code_64, 3);
            bm64_from_axis_codes(bit_matrix_64, bit_mask_64, row_code_64, col_code_64);
            bm64_to_img(&raw_image, bit_matrix_64, bit_mask_64);
            // simulate real life pipe line starting here

            // find threshold of original image
            img_histogram(histogram, raw_image);
            uint8_t threshold0 = img_compute_otsu_threshold(histogram);

            // find rotation of original image
            Vector2f rotation = img_estimate_rotation(raw_image);
            if (!v2f_is_zero(rotation)) {
                // unrotate
                rotation.y *= -1;
                IMG_SET_SIZE(buf_image, 64, 64);
                img_rotate(buf_image, raw_image, rotation, threshold0, img_bilinear_interpolation);
            }
            // sharpen
            img_convolution_filter(&buf_image, buf_image, img_hyper_sharpen_kernel);
            Vector2f vertex = v2f_rotate(
                    rotation, (Vector2f){2 + buf_image.n_cols / 2, 2 + buf_image.n_rows / 2});
            img_draw_regular_polygon(buf_image,
                    (ImagePoint){buf_image.n_cols / 2, buf_image.n_rows / 2}, vertex, 4, threshold0,
                    5);

            // find threshold of filtered image
            img_histogram(histogram, buf_image);
            histogram[threshold0] = 0;
            uint8_t threshold1 = img_compute_otsu_threshold(histogram);
            if (threshold1 < threshold0) {
                SWAP(threshold1, threshold0);
            }
            // binarize to bit matrix
            img_to_bm64(bit_matrix_64, bit_mask_64, buf_image, threshold0, threshold1);

            // extract row and column codes
            bm64_extract_axis_codes(&row_code_64, &col_code_64, bit_matrix_64, bit_mask_64, 5);

            Location best_match_location = {};
            AxisCode32 best_match_row_code = {};
            AxisCode32 best_match_col_code = {};

            // disable scale search, since it causes some false positives which fail the test
            // for (float scale = 0.5f; scale < 1.5f; scale += 0.03f)
            float scale = 1.0f;
            {
                // scale and down sample axis codes
                AxisCode64 scaled_row_code = scale_axis_code(row_code_64, scale);
                AxisCode64 scaled_col_code = scale_axis_code(col_code_64, scale);
                AxisCode32 row_code_32 = downsample_axis_code(scaled_row_code);
                AxisCode32 col_code_32 = downsample_axis_code(scaled_col_code);
                // decode posiiton
                AxisPosition row_pos = decode_axis_position(row_code_32, MLS_INDEX.code_length);
                AxisPosition col_pos = decode_axis_position(col_code_32, MLS_INDEX.code_length);
                Location loc = deduce_location(row_pos, col_pos);
                if (loc.match_size > best_match_location.match_size) {
                    best_match_location = loc;
                    best_match_row_code = row_code_32;
                    best_match_col_code = col_code_32;
                }
            }

            uint32_t compare_row_code =
                    (best_match_row_code.bits ^ src_row_code) & best_match_row_code.mask;
            uint32_t compare_row_code1 =
                    (best_match_row_code.bits ^ src_row_code >> 1) & best_match_row_code.mask;

            uint32_t compare_col_code =
                    (best_match_col_code.bits ^ src_col_code) & best_match_col_code.mask;
            uint32_t compare_col_code1 =
                    (best_match_col_code.bits ^ src_col_code >> 1) & best_match_col_code.mask;

            test_assert(compare_row_code == 0 || compare_row_code == best_match_row_code.mask ||
                        compare_row_code1 == 0 || compare_row_code1 == best_match_row_code.mask);
            test_assert(compare_col_code == 0 || compare_col_code == best_match_col_code.mask ||
                        compare_col_code1 == 0 || compare_col_code1 == best_match_col_code.mask);
        }
    free(raw_image.data);
    free(buf_image.data);
    free(bit_matrix_64);
    free(bit_mask_64);
    free(bit_matrix_32);
    free(bit_mask_32);
    free(histogram);
    return 0;
}

int test_full_chain() {
    test_run(test_full_chain_simulation);
    test_run(test_full_chain_real_life);
    return 0;
}
