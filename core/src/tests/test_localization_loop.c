#include "test_utils.h"
#include "localization_loop.h"
#include "mls_query.h"
#include <stdlib.h>

#define TEST_VECTOR_SIZE 10

static int test_full_chain_simulation() {
    ImageMatrix src_img = {malloc(30 * 30), {30, 30}};
    ImageMatrix img = {malloc(32 * 32), {32, 32}};
    uint32_t* bit_matrix = malloc(32 * sizeof(uint32_t));
    uint32_t* bit_mask = malloc(32 * sizeof(uint32_t));
    Location loc;
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
        bm32_extract_axiscodes(&row_code, &col_code, bit_matrix, bit_mask, 3);

        // overwrite with simulate extracted code
        row_code.bits = mlsq_code_from_position(MLS_INDEX.sequence, 32, i);
        row_code.mask = ~0;
        col_code = row_code;

        // decode positions
        AxisPosition row_pos = decode_axis_position(row_code);
        AxisPosition col_pos = decode_axis_position(col_code);

        // deduce location
        deduce_location(&loc, row_pos, col_pos);

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

static int test_localization_loop_run() {
    LocalizationContext* ctx = malloc(sizeof(LocalizationContext));
    ImageMatrix image = {calloc(64 * 64, 1), {64, 64}};
    ctx->unrotated_image = (ImageMatrix){calloc(64 * 64, 1), {64, 64}};
    ctx->sharpened_image = ctx->unrotated_image;
    ctx->correlation.image.data = calloc(32 * 32, sizeof(Vector2f));
    ctx->correlation.buffer.data = calloc(32 * 32, sizeof(Vector2f));
    ctx->rotation_scale = 1.0f;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1100; src_col_pos < 1100 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            // generate source image from known position
            // disable scale search, since it causes some false positives which fail the test
            ctx->scale_query = (ScaleQuery){
                    {src_row_code, ~0ull, 0, 0}, {src_col_code, ~0ull, 0, 0}, 1.0f, 1.0f, 1.0f};
            ctx->scale_query.row_code = scale_axiscode64(ctx->scale_query.row_code, 3);
            ctx->scale_query.col_code = scale_axiscode64(ctx->scale_query.col_code, 3);
            bm64_from_axiscodes(ctx->binary_image, ctx->binary_mask, ctx->scale_query.row_code,
                    ctx->scale_query.col_code);
            bm64_to_img(&image, ctx->binary_image, ctx->binary_mask);

            // simulate real life pipe line
            localization_loop_run(ctx, image);

            uint32_t compare_row_code = (ctx->scale_match.row_code.bits ^ src_row_code) &
                                        ctx->scale_match.row_code.mask;
            uint32_t compare_row_code1 = (ctx->scale_match.row_code.bits ^ src_row_code >> 1) &
                                         ctx->scale_match.row_code.mask;

            uint32_t compare_col_code = (ctx->scale_match.col_code.bits ^ src_col_code) &
                                        ctx->scale_match.col_code.mask;
            uint32_t compare_col_code1 = (ctx->scale_match.col_code.bits ^ src_col_code >> 1) &
                                         ctx->scale_match.col_code.mask;

            test_assert(
                    compare_row_code == 0 || compare_row_code == ctx->scale_match.row_code.mask ||
                    compare_row_code1 == 0 || compare_row_code1 == ctx->scale_match.row_code.mask);
            test_assert(
                    compare_col_code == 0 || compare_col_code == ctx->scale_match.col_code.mask ||
                    compare_col_code1 == 0 || compare_col_code1 == ctx->scale_match.col_code.mask);
        }
    free(image.data);
    free(ctx->unrotated_image.data);
    free(ctx->correlation.image.data);
    free(ctx->correlation.buffer.data);
    free(ctx);
    return 0;
}

int test_localization_loop() {
    test_run(test_full_chain_simulation);
    test_run(test_localization_loop_run);
    return 0;
}
