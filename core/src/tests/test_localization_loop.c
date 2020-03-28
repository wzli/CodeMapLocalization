#include "test_utils.h"
#include "localization_loop.h"
#include "mls_query.h"
#include <stdlib.h>

#define TEST_VECTOR_SIZE 20

static int test_localization_loop_run() {
    LocalizationContext* ctx = calloc(1, sizeof(LocalizationContext));
    ImageMatrix image = {calloc(64 * 64, 1), {64, 64}};
    ctx->derotated_image = (ImageMatrix){calloc(64 * 64, 1), {64, 64}};
    ctx->sharpened_image = ctx->derotated_image;
    ctx->odom.correlation.image.data = calloc(32 * 32, sizeof(Vector2f));
    ctx->odom.correlation.buffer.data = calloc(32 * 32, sizeof(Vector2f));
    ctx->rotation_scale = 1.0f;
    ctx->scale_decay_rate = 1.0f;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1100; src_col_pos < 1100 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = bv32_get_slice(MLS_INDEX.sequence, src_row_pos, 32);
            uint32_t src_col_code = bv32_get_slice(MLS_INDEX.sequence, src_col_pos, 32);
            // generate source image from known position
            // disable scale search, since it causes some false positives which fail the test
            ctx->row_code = (AxisCode){{(uint64_t)src_row_code << 16}, {mask_bits_64(32) << 16}, 0, 0};
            ctx->col_code = (AxisCode){{(uint64_t)src_col_code << 16}, {mask_bits_64(32) << 16}, 0, 0};
            puts("");
            print_bits(ctx->col_code.bits.x64, 64);
            print_bits(ctx->col_code.mask.x64, 64);
            bm64_from_axiscodes(
                    ctx->binary_image, ctx->binary_mask, &ctx->row_code, &ctx->col_code);
            bm64_to_img(&image, ctx->binary_image, ctx->binary_mask);

            img_save_to_pgm(image, "original.pgm");


            // simulate real life pipe line
            localization_loop_run(ctx, image);
            img_save_to_pgm(ctx->sharpened_image, "sharpened.pgm");
            bm64_to_img(&image, ctx->binary_image, ctx->binary_mask);
            img_save_to_pgm(ctx->sharpened_image, "thresholded.pgm");
            printf("thresholds %d %d \n", ctx->threshold[0], ctx->threshold[1]);

            print_bits(ctx->col_code.bits.x64, 64);
            print_bits(ctx->col_code.mask.x64, 64);

            print_bits(ctx->scale_match.col_code.bits.x64, 64);
            print_bits(ctx->scale_match.col_code.mask.x64, 64);
            printf("scale %f\n", ctx->scale_match.scale);

            uint64_t compare_row_code = (ctx->scale_match.row_code.bits.x64 ^ src_row_code) &
                                        ctx->scale_match.row_code.mask.x64;
            uint64_t compare_row_code1 = (ctx->scale_match.row_code.bits.x64 ^ src_row_code >> 1) &
                                         ctx->scale_match.row_code.mask.x64;

            uint64_t compare_col_code = (ctx->scale_match.col_code.bits.x64 ^ src_col_code) &
                                        ctx->scale_match.col_code.mask.x64;
            uint64_t compare_col_code1 = (ctx->scale_match.col_code.bits.x64 ^ src_col_code >> 1) &
                                         ctx->scale_match.col_code.mask.x64;

            test_assert(compare_row_code == 0 ||
                        compare_row_code == ctx->scale_match.row_code.mask.x64 ||
                        compare_row_code1 == 0 ||
                        compare_row_code1 == ctx->scale_match.row_code.mask.x64);
            test_assert(compare_col_code == 0 ||
                        compare_col_code == ctx->scale_match.col_code.mask.x64 ||
                        compare_col_code1 == 0 ||
                        compare_col_code1 == ctx->scale_match.col_code.mask.x64);
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
