#include "test_utils.h"
#include "localization_loop.h"
#include "mls_query.h"
#include <stdlib.h>

#define TEST_VECTOR_SIZE 10
#define CONV_SIZE (8)

static inline bool latch_comp(float complex cur_val, float complex max_val) {
    return cabsf(cur_val) > cabsf(max_val);
}

static int test_phase_correlation() {
    ImageMatrixComplex frame = {(float complex[SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};
    ImageMatrixComplex next_frame = {(float complex[SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};

    IMG_FILL(frame, 0);
    IMG_FILL(next_frame, 0);

    PIXEL(frame, 3, 3) = 1;
    PIXEL(next_frame, 2, 2) = 1;

    img_phase_correlation(frame, next_frame, false);

#if 0
    FOR_EACH_PIXEL(frame) {
        if (!col) { puts(""); }
        printf("%3d ", (int16_t)(PIXEL(frame, row, col) & 0xFFFF));
    }
#endif
    float complex max_val = 0;
    int16_t max_x = -1;
    int16_t max_y = -1;
    IMG_PIXEL_LATCH_INDEX(max_val, max_y, max_x, latch_comp, frame);
    test_assert(max_x == 1);
    test_assert(max_y == 1);
    return 0;
}

static int test_full_chain_simulation() {
    ImageMatrix src_img = {malloc(30 * 30), {30, 30}};
    ImageMatrix img = {malloc(32 * 32), {32, 32}};
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
        bm32_extract_axiscodes(&row_code, &col_code, bit_matrix, bit_mask, 3);

        // overwrite with simulate extracted code
        row_code.bits = mlsq_code_from_position(MLS_INDEX.sequence, 32, i);
        row_code.mask = ~0;
        col_code = row_code;

        // decode positions
        AxisPosition row_pos = decode_axis_position(row_code);
        AxisPosition col_pos = decode_axis_position(col_code);

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

static int test_localization_loop_run() {
    LocalizationContext* ctx = malloc(sizeof(LocalizationContext));
    ctx->original_image = (ImageMatrix){malloc(64 * 64), {64, 64}};
    ctx->unrotated_image = (ImageMatrix){malloc(64 * 64), {64, 64}};
    ctx->sharpened_image = ctx->unrotated_image;
    ctx->correlation_image =
            (ImageMatrixComplex){malloc(64 * 64 * sizeof(float complex)), {64, 64}};
    ctx->correlation_buffer =
            (ImageMatrixComplex){malloc(64 * 64 * sizeof(float complex)), {64, 64}};
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
            bm64_to_img(&ctx->original_image, ctx->binary_image, ctx->binary_mask);

            // simulate real life pipe line
            localization_loop_run(ctx);

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
    free(ctx->original_image.data);
    free(ctx->unrotated_image.data);
    free(ctx->correlation_image.data);
    free(ctx->correlation_buffer.data);
    free(ctx);
    return 0;
}

int test_localization_loop() {
    test_run(test_phase_correlation);
    test_run(test_full_chain_simulation);
    test_run(test_localization_loop_run);
    return 0;
}
