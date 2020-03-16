#include "tests.h"
#include "localization_loop.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#define FRAME_SIZE 64
#define MALLOC_IMAGE(SIZE)      \
    (ImageMatrix) {             \
        calloc(SQR(SIZE), 1), { \
            { SIZE, SIZE }      \
        }                       \
    }
#define MALLOC_IMAGE_COMPLEX(SIZE)             \
    (ImageMatrixComplex) {                     \
        calloc(SQR(SIZE), sizeof(Vector2f)), { \
            { SIZE, SIZE }                     \
        }                                      \
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
    loc_ctx.odom.correlation.image = MALLOC_IMAGE_COMPLEX(FRAME_SIZE / 2);
    loc_ctx.odom.correlation.buffer = MALLOC_IMAGE_COMPLEX(FRAME_SIZE / 2);
    // setup configs
    loc_ctx.rotation_scale = 1.0f;
    loc_ctx.scale_query.lower_bound = 0.8f;
    loc_ctx.scale_query.upper_bound = 1.2f;
    loc_ctx.scale_query.step_size = 0.02f;
    loc_ctx.outlier_filter.distance_threshold = 200;
    loc_ctx.outlier_filter.match_size_threshold = 20;
    loc_ctx.outlier_filter.bit_error_ratio_threshold = 5;
    loc_ctx.outlier_filter.max_rejection_count = 10;
    loc_ctx.odom.correlation.squared_magnitude_threshold = 0.01;
    // print csv headers
    printf("frame, "
           "updated, "
           "thresh0, "
           "thresh1, "
           "match_size, "
           "row_err_ratio, "
           "col_err_ratio, "
           "scale, "
           "loc_x, "
           "loc_y, "
           "loc_dir, "
           "odom_dir, "
           "odom_x, "
           "odom_y, "
           "corr_x, "
           "corr_y, "
           "corr_err_ratio"
           "orientation, "
           "\n");
    // loop through frames
    uint32_t n_frames = 0;

    for (size_t read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp);
            read_bytes == SQR(FRAME_SIZE);
            read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp), ++n_frames) {
        // process frame
        bool updated = localization_loop_run(&loc_ctx, raw_image);
        Vector2f odom_rot = loc_ctx.odom.quadrant_rotation;
        odom_rot.z *= QUADRANT_LOOKUP[loc_ctx.odom.quadrant_count & 3].z;
        printf("%u, %u, %u, %u, %u, %f, %f, %f, %u, %u, %u, %d, %f, %f, %f, %f, %f, %f\n", n_frames,
                updated, loc_ctx.threshold[0], loc_ctx.threshold[1],
                loc_ctx.scale_match.location.match_size,
                (double) loc_ctx.scale_match.row_code.n_errors /
                        (loc_ctx.scale_match.row_code.n_samples + 1),
                (double) loc_ctx.scale_match.col_code.n_errors /
                        (loc_ctx.scale_match.col_code.n_samples + 1),
                (double) loc_ctx.scale_match.scale,
                loc_ctx.outlier_filter.filtered_match.location.x,
                loc_ctx.outlier_filter.filtered_match.location.y,
                loc_ctx.outlier_filter.filtered_match.location.direction,
                loc_ctx.odom.quadrant_count, (double) loc_ctx.odom.position.x,
                (double) loc_ctx.odom.position.y, (double) loc_ctx.odom.correlation.translation.x,
                (double) loc_ctx.odom.correlation.translation.y,
                (double) (loc_ctx.odom.correlation.squared_magnitude_max /
                          (loc_ctx.odom.correlation.squared_magnitude_sum + 0.0001f)),
                (double) cargf(odom_rot.z));

        // write raw image to top left
        ImagePoint top_left = {{0, 0}};
        IMG_PASTE(output_image, raw_image, top_left);
        // write sharpened image to top right
        top_left.x += FRAME_SIZE;
        IMG_PASTE(output_image, loc_ctx.sharpened_image, top_left);
        // write correlation image to bottom right
        FOR_EACH_PIXEL(raw_image) {
            // normalize to 255
            PIXEL(raw_image, row, col) = 255 *
                                         PIXEL(loc_ctx.odom.correlation.image, row / 2, col / 2).x /
                                         loc_ctx.odom.correlation.squared_magnitude_max;
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
    free(loc_ctx.odom.correlation.buffer.data);
    free(loc_ctx.odom.correlation.image.data);
    free(loc_ctx.sharpened_image.data);
    free(loc_ctx.unrotated_image.data);
    free(output_image.data);
    free(raw_image.data);
    return 0;
}
