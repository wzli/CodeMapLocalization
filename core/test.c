#include "tests.h"
#include "localization_loop.h"
#include "mxgen.h"
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

#define STRUCT_LocalizationLog(FIELD) \
    FIELD(uint32_t, frame, )          \
    FIELD(bool, updated, )            \
    FIELD(uint8_t, thresh0, )         \
    FIELD(uint8_t, thresh1, )         \
    FIELD(uint16_t, match_size, )     \
    FIELD(float, row_err_ratio, )     \
    FIELD(float, col_err_ratio, )     \
    FIELD(float, scale, )             \
    FIELD(uint16_t, loc_x, )          \
    FIELD(uint16_t, loc_y, )          \
    FIELD(uint8_t, loc_dir, )         \
    FIELD(int32_t, odom_dir, )        \
    FIELD(float, odom_x, )            \
    FIELD(float, odom_y, )            \
    FIELD(float, corr_x, )            \
    FIELD(float, corr_y, )            \
    FIELD(float, corr_err_ratio, )    \
    FIELD(float, orientation, )
GEN_STRUCT(LocalizationLog);

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
    char* text_buf = malloc(512);
    LocalizationLog loc_log = {};
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

    // write csv header
    LocalizationLog_to_csv_header(text_buf, 0, 0);
    puts(text_buf);

    for (size_t read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp);
            read_bytes == SQR(FRAME_SIZE);
            read_bytes = fread(raw_image.data, 1, SQR(FRAME_SIZE), fp), ++loc_log.frame) {
        // process frame
        loc_log.updated = localization_loop_run(&loc_ctx, raw_image);
        Vector2f odom_rot = loc_ctx.odom.quadrant_rotation;
        odom_rot.z *= QUADRANT_LOOKUP[loc_ctx.odom.quadrant_count & 3].z;

        // write csv entry
        loc_log.thresh0 = loc_ctx.threshold[0];
        loc_log.thresh1 = loc_ctx.threshold[1];
        loc_log.match_size = loc_ctx.scale_match.location.match_size;
        loc_log.row_err_ratio = (float) loc_ctx.scale_match.row_code.n_errors /
                                (loc_ctx.scale_match.row_code.n_samples + 1);
        loc_log.col_err_ratio = (float) loc_ctx.scale_match.col_code.n_errors /
                                (loc_ctx.scale_match.col_code.n_samples + 1);
        loc_log.scale = loc_ctx.scale_match.scale;
        loc_log.loc_x = loc_ctx.outlier_filter.filtered_match.location.x;
        loc_log.loc_y = loc_ctx.outlier_filter.filtered_match.location.y;
        loc_log.loc_dir = loc_ctx.outlier_filter.filtered_match.location.direction;
        loc_log.odom_dir = loc_ctx.odom.quadrant_count;
        loc_log.odom_x = loc_ctx.odom.position.x;
        loc_log.odom_y = loc_ctx.odom.position.y;
        loc_log.corr_x = loc_ctx.odom.correlation.translation.x;
        loc_log.corr_y = loc_ctx.odom.correlation.translation.y;
        loc_log.corr_err_ratio = loc_ctx.odom.correlation.squared_magnitude_max /
                                 (loc_ctx.odom.correlation.squared_magnitude_sum + 0.0001f);
        loc_log.orientation = cargf(odom_rot.z);
        LocalizationLog_to_csv_entry(&loc_log, text_buf);
        puts(text_buf);

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
        loc_ctx.scale_query.row_code = scale_axiscode64(loc_ctx.scale_query.row_code, 3);
        loc_ctx.scale_query.col_code = scale_axiscode64(loc_ctx.scale_query.col_code, 3);
        bm64_from_axiscodes(loc_ctx.binary_image, loc_ctx.binary_mask, loc_ctx.scale_query.row_code,
                loc_ctx.scale_query.col_code);
        bm64_to_img(&raw_image, loc_ctx.binary_image, loc_ctx.binary_mask);
        top_left.x -= FRAME_SIZE;
        IMG_PASTE(output_image, raw_image, top_left);
        // write pgm
        char file_name[32];
        sprintf(file_name, "%05d.pgm", loc_log.frame);
        img_save_to_pgm(output_image, file_name);
    }
    // deallocate context
    free(text_buf);
    free(loc_ctx.odom.correlation.buffer.data);
    free(loc_ctx.odom.correlation.image.data);
    free(loc_ctx.sharpened_image.data);
    free(loc_ctx.unrotated_image.data);
    free(output_image.data);
    free(raw_image.data);
    return 0;
}
