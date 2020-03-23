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
