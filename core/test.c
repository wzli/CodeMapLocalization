#include "tests.h"
#include "localization_loop.h"
#include "debug_prints.h"
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
    LocalizationContext loc_ctx = {0};
    ImageMatrix raw_image = MALLOC_IMAGE(FRAME_SIZE);
    ImageMatrix output_image = MALLOC_IMAGE(2 * FRAME_SIZE);
    loc_ctx.derotated_image = MALLOC_IMAGE(FRAME_SIZE);
    loc_ctx.sharpened_image = MALLOC_IMAGE(FRAME_SIZE);
    loc_ctx.odom.correlation.image = MALLOC_IMAGE_COMPLEX(FRAME_SIZE / 2);
    loc_ctx.odom.correlation.buffer = MALLOC_IMAGE_COMPLEX(FRAME_SIZE / 2);
    char* text_buf = malloc(512);
    LocalizationMsg loc_msg = {0};
    // setup configs
    loc_ctx.rotation_scale = 1.0f;
    loc_ctx.scale_query.lower_bound = 0.8f;
    loc_ctx.scale_query.upper_bound = 1.2f;
    loc_ctx.scale_query.step_size = 0.02f;
    loc_ctx.outlier_filter.distance_threshold = 200;
    loc_ctx.outlier_filter.match_length_threshold = 21 - MLS_INDEX.code_length;
    loc_ctx.outlier_filter.bit_error_ratio_threshold = 5;
    loc_ctx.outlier_filter.max_rejection_count = 10;
    loc_ctx.odom.correlation.squared_magnitude_threshold = 0.01f;

    // write csv header
    LocalizationMsg_to_csv_header(0, text_buf);
    puts(text_buf);

    for (size_t read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp);
            read_bytes == SQR(FRAME_SIZE);
            read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp)) {
        // process frame
        localization_loop_run(&loc_ctx, raw_image);
        Vector2f odom_rot = loc_ctx.odom.quadrant_rotation;
        odom_rot.z *= QUADRANT_LOOKUP[loc_ctx.odom.quadrant_count & 3].z;
        // write csv entry
        write_localization_msg(&loc_msg, &loc_ctx);
        LocalizationMsg_to_csv_entry(&loc_msg, text_buf);
        puts(text_buf);
        // write raw image to top left
        ImagePoint top_left = {0, 0};
        IMG_PASTE(output_image, raw_image, top_left);
        // write sharpened image to top right
        top_left.x += FRAME_SIZE;
        IMG_PASTE(output_image, loc_ctx.sharpened_image, top_left);
        // write correlation image to bottom right
        FOR_EACH_PIXEL(raw_image) {
            // normalize to 255
            PIXEL(raw_image, row, col) =
                    255 * PIXEL(loc_ctx.odom.correlation.image, row / 2, col / 2).xy[0] /
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
        loc_ctx.scale_query.row_code = scale_axiscode64(loc_ctx.scale_query.row_code, 3);
        loc_ctx.scale_query.col_code = scale_axiscode64(loc_ctx.scale_query.col_code, 3);
        bm64_from_axiscodes(loc_ctx.binary_image, loc_ctx.binary_mask, loc_ctx.scale_query.row_code,
                loc_ctx.scale_query.col_code);
        bm64_to_img(&raw_image, loc_ctx.binary_image, loc_ctx.binary_mask);
        top_left.x -= FRAME_SIZE;
        IMG_PASTE(output_image, raw_image, top_left);
        // write pgm
        char file_name[32];
        sprintf(file_name, "%05d.pgm", loc_ctx.frame_count);
        img_save_to_pgm(output_image, file_name);
    }
    // deallocate context
    free(text_buf);
    free(loc_ctx.odom.correlation.buffer.data);
    free(loc_ctx.odom.correlation.image.data);
    free(loc_ctx.sharpened_image.data);
    free(loc_ctx.derotated_image.data);
    free(output_image.data);
    free(raw_image.data);
    return 0;
}
