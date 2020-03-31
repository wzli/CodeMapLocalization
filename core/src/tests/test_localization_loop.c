#include "test_utils.h"
#include "localization_loop.h"
#include "mls_query.h"
#include <stdlib.h>

#define TEST_VECTOR_SIZE 10

static int test_localization_loop_run() {
    LocalizationContext* ctx = calloc(1, sizeof(LocalizationContext));
    ImageMatrix image = {calloc(64 * 64, 1), {64, 64}};
    ctx->derotated_image = (ImageMatrix){calloc(64 * 64, 1), {64, 64}};
    ctx->sharpened_image = ctx->derotated_image;
    ctx->odom.correlation.image.data = calloc(32 * 32, sizeof(Vector2f));
    ctx->odom.correlation.buffer.data = calloc(32 * 32, sizeof(Vector2f));
    ctx->scale_step = 0.015f;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1100; src_col_pos < 1100 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = bv32_get_slice(MLS_INDEX.sequence, src_row_pos, 32);
            uint32_t src_col_code = bv32_get_slice(MLS_INDEX.sequence, src_col_pos, 32);
            // generate source image from known position
            // disable scale search, since it causes some false positives which fail the test
            uint32_t scaled_bits[2];
            bv32_scale(scaled_bits, &src_row_code, 64, 32, 2);
            ctx->row_code = (AxisCode){
                    {(uint64_t) scaled_bits[0] | (uint64_t) scaled_bits[1] << 32}, {~0ull}, 0, 0};
            bv32_scale(scaled_bits, &src_col_code, 64, 32, 2);
            ctx->col_code = (AxisCode){
                    {(uint64_t) scaled_bits[0] | (uint64_t) scaled_bits[1] << 32}, {~0ull}, 0, 0};
            bm64_from_axiscodes(
                    ctx->binary_image, ctx->binary_mask, &ctx->row_code, &ctx->col_code);
            bm64_to_img(&image, ctx->binary_image, ctx->binary_mask);

            // simulate real life pipe line
            localization_loop_run(ctx, image);

            test_assert(ctx->scale_match.row_code.mask.x64 != 0);
            test_assert(ctx->scale_match.col_code.mask.x64 != 0);
            test_assert(ABS(ctx->scale_match.location.x - (int) src_row_pos) < 10);
            test_assert(ABS(ctx->scale_match.location.y - (int) src_col_pos) < 10);
            test_assert(ABS(ctx->scale_match.scale - 0.5f) < 0.01f);
            test_assert(ctx->scale_match.location.match_size > SQR(12));
        }
    free(image.data);
    free(ctx->derotated_image.data);
    free(ctx->odom.correlation.image.data);
    free(ctx->odom.correlation.buffer.data);
    free(ctx);
    return 0;
}

int test_localization_loop() {
    test_run(test_localization_loop_run);
    return 0;
}
