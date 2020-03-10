#include "tests.h"
#include "localization_loop.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#define FRAME_SIZE 64
#define MALLOC_IMAGE(SIZE)                   \
    (ImageMatrix) {                          \
        calloc(SQR(SIZE), 1), { SIZE, SIZE } \
    }
#define MALLOC_IMAGE_COMPLEX(SIZE)                          \
    (ImageMatrixComplex) {                                  \
        calloc(SQR(SIZE), sizeof(Vector2f)), { SIZE, SIZE } \
    }

int main(int argc, char** argv) {
    // run unit tests by default
    if (argc < 2) {
        return run_all_tests();
    }
    // open frame data
    FILE* fp = fopen(argv[1], "r");
    if (!fp) {
        perror("could not open file");
        return errno;
    }
    // allocate context
    LocalizationContext loc_ctx = {};
    ImageMatrix raw_image = MALLOC_IMAGE(FRAME_SIZE);
    ImageMatrix output_image = MALLOC_IMAGE(2 * FRAME_SIZE);
    loc_ctx.unrotated_image = MALLOC_IMAGE(FRAME_SIZE);
    loc_ctx.sharpened_image = MALLOC_IMAGE(FRAME_SIZE);
    loc_ctx.correlation.image = MALLOC_IMAGE_COMPLEX(FRAME_SIZE / 2);
    loc_ctx.correlation.buffer = MALLOC_IMAGE_COMPLEX(FRAME_SIZE / 2);
    // setup configs
    loc_ctx.rotation_scale = 1.0f;
    loc_ctx.scale_query.lower_bound = 0.8f;
    loc_ctx.scale_query.upper_bound = 1.2f;
    loc_ctx.scale_query.step_size = 0.02f;
    // print csv headers
    printf("frame, "
           "match_size, "
           "thresh0, "
           "thresh1, "
           "scale, "
           "cor_x, "
           "cor_y, "
           "loc_x, "
           "loc_y, "
           "rot_x, "
           "rot_y, "
           "row_bits, "
           "row_mask, "
           "row_err, "
           "row_tot, "
           "col_bits, "
           "col_mask, "
           "col_err, "
           "col_tot"
           "\n");
    // loop through frames
    uint32_t n_frames = 0;
    for (size_t read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp);
            read_bytes == SQR(FRAME_SIZE);
            read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp), ++n_frames) {
        // process frame
        localization_loop_run(&loc_ctx, raw_image);
        // print csv values
        printf("%u, %d, %u, %u, %f, %f, %f, %u, %u, %f, %f, %u, %u, %u, %u, %u, %u, %u, %u\n",
                n_frames, loc_ctx.scale_match.location.match_size, loc_ctx.threshold[0],
                loc_ctx.threshold[1], (double) loc_ctx.scale_match.scale,
                (double) loc_ctx.correlation.translation.x,
                (double) loc_ctx.correlation.translation.y, loc_ctx.scale_match.location.x,
                loc_ctx.scale_match.location.y, (double) loc_ctx.scale_match.location.rotation.x,
                (double) loc_ctx.scale_match.location.rotation.y, loc_ctx.scale_match.row_code.bits,
                loc_ctx.scale_match.row_code.mask, loc_ctx.scale_match.row_code.n_errors,
                loc_ctx.scale_match.row_code.n_samples, loc_ctx.scale_match.col_code.bits,
                loc_ctx.scale_match.col_code.mask, loc_ctx.scale_match.col_code.n_errors,
                loc_ctx.scale_match.col_code.n_samples);
        // write raw image to top left
        ImagePoint top_left = {0, 0};
        IMG_PASTE(output_image, raw_image, top_left);
        // write sharpened image to top right
        top_left.x += FRAME_SIZE;
        IMG_PASTE(output_image, loc_ctx.sharpened_image, top_left);
        // write correlation image to bottom right
        FOR_EACH_PIXEL(raw_image) {
            // normalize to 255
            PIXEL(raw_image, row, col) = 255 *
                                         PIXEL(loc_ctx.correlation.image, row / 2, col / 2).x /
                                         loc_ctx.correlation.max_squared_magnitude;
        }
        // roll to center
        for (int16_t row = 0; row < FRAME_SIZE / 2; ++row) {
            for (int16_t col = 0; col < FRAME_SIZE / 2; ++col) {
                SWAP(PIXEL(raw_image, row, col),
                        PIXEL(raw_image, row + FRAME_SIZE / 2, col + FRAME_SIZE / 2));
                SWAP(PIXEL(raw_image, row + FRAME_SIZE / 2, col),
                        PIXEL(raw_image, row, col + FRAME_SIZE / 2));
            }
        }
        top_left.y += FRAME_SIZE;
        IMG_PASTE(output_image, raw_image, top_left);
        // write decoded image to bottom left
        AXISCODE_COPY(loc_ctx.scale_query.row_code, loc_ctx.scale_match.row_code);
        AXISCODE_COPY(loc_ctx.scale_query.col_code, loc_ctx.scale_match.col_code);
        loc_ctx.scale_query.row_code = scale_axiscode64(loc_ctx.scale_query.row_code, 2);
        loc_ctx.scale_query.col_code = scale_axiscode64(loc_ctx.scale_query.col_code, 2);
        bm64_from_axiscodes(loc_ctx.binary_image, loc_ctx.binary_mask, loc_ctx.scale_query.row_code,
                loc_ctx.scale_query.col_code);
        bm64_to_img(&raw_image, loc_ctx.binary_image, loc_ctx.binary_mask);
        top_left.x -= FRAME_SIZE;
        IMG_PASTE(output_image, raw_image, top_left);
        // write pgm
        char file_name[32];
        sprintf(file_name, "%05d.pgm", n_frames);
        img_save_to_pgm(output_image, file_name);
    }
    // deallocate context
    free(loc_ctx.correlation.buffer.data);
    free(loc_ctx.correlation.image.data);
    free(loc_ctx.sharpened_image.data);
    free(loc_ctx.unrotated_image.data);
    free(output_image.data);
    free(raw_image.data);
    return 0;
}
