#include "debug_prints.h"
#include <stdio.h>
#include <assert.h>

void write_location_match_msg(LocationMatchMsg* msg, const ScaleMatch* match) {
    assert(msg && match);
    msg->x = match->location.x;
    msg->y = match->location.y;
    msg->match_size = match->location.match_size;
    msg->downsample_errors = match->bit_errors;
    msg->xor_err_ratio = ((float) match->col_code.n_errors / match->col_code.n_samples +
                                 (float) match->row_code.n_errors / match->row_code.n_samples) *
                         0.5f;
    msg->quality = match->quality;
    msg->scale = match->scale;
}

void write_odometry_msg(OdometryMsg* msg, const VisualOdometry* odom) {
    assert(msg && odom);
    msg->x = odom->position.xy[0];
    msg->y = odom->position.xy[1];
    msg->rot = cargf(odom->quadrant_rotation.z * QUADRANT_LOOKUP[odom->quadrant_count & 3].z);
    msg->quadrants = odom->quadrant_count;
    msg->steps = odom->step_count;
}

void write_correlation_msg(CorrelationMsg* msg, const Correlation* corr) {
    assert(msg && corr);
    msg->x = corr->translation.xy[0];
    msg->y = corr->translation.xy[1];
    msg->err_ratio = 1.0f - (corr->squared_magnitude_max / (corr->squared_magnitude_sum + 0.0001f));
}

void write_localization_msg(LocalizationMsg* msg, const LocalizationContext* loc_ctx) {
    assert(msg && loc_ctx);
    msg->frame = loc_ctx->frame_count;
    msg->thresh[0] = loc_ctx->threshold[0];
    msg->thresh[1] = loc_ctx->threshold[1];
    write_location_match_msg(&msg->loc, &loc_ctx->scale_match);
    write_odometry_msg(&msg->odom, &loc_ctx->odom);
    write_correlation_msg(&msg->corr, &loc_ctx->odom.correlation);
}

void print_bits(uint64_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf(" %u", (uint32_t)(word >> word_length) & 1);
    }
    puts("");
}

void print_bit_vector(const uint32_t* vector, uint32_t len) {
    for (uint32_t i = 0; i < len; ++i) {
        printf("%u", bv32_get_bit(vector, i));
    }
    puts("");
}

void print_bit_matrix(BitMatrix32 matrix) {
    for (uint8_t row = 0; row < 32; ++row) {
        print_bits(matrix[row], 32);
    }
}

void print_image_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.size.y; ++row) {
        for (int16_t col = 0; col < src.size.x; ++col) {
            printf("%6d ", PIXEL(src, row, col));
        }
        puts("");
    }
    puts("");
}

void print_axiscode(AxisCode axiscode) {
    uint8_t leading_zeros = count_trailing_zeros(reverse_bits(axiscode.mask.x64, 64));
    printf("samples %d errors %d\n", axiscode.n_samples, axiscode.n_errors);
    print_bits(axiscode.bits.x64, 64 - leading_zeros);
}

void print_axis_position(AxisPosition position) {
    printf("center %d span %d inverted %d reversed %d\n", position.center, position.span,
            position.inverted, position.reversed);
}

void print_location(const Location* location) {
    printf("x %d y %d d %d match %d\n", location->x, location->y, location->direction,
            location->match_size);
}

void bm64_save_to_pgm(BitMatrix64 bit_matrix, BitMatrix64 bit_mask, const char* file_name) {
    ImageMatrix image = {(uint8_t[64 * 64]){0}, {64, 64}};
    bm64_to_img(&image, bit_matrix, bit_mask);
    img_save_to_pgm(image, file_name);
}

void print_location_match(const ScaleMatch* match) {
    char buf[256];
    LocationMatchMsg msg;
    write_location_match_msg(&msg, match);
    LocationMatchMsg_to_json(&msg, buf);
    printf("Location Match\t");
    puts(buf);
}

void print_odometry(const VisualOdometry* odom) {
    char buf[128];
    OdometryMsg msg;
    write_odometry_msg(&msg, odom);
    OdometryMsg_to_json(&msg, buf);
    printf("Odom Estimate\t");
    puts(buf);
}

void print_correlation(const Correlation* corr) {
    char buf[128];
    CorrelationMsg msg;
    write_correlation_msg(&msg, corr);
    CorrelationMsg_to_json(&msg, buf);
    printf("Correlation\t");
    puts(buf);
}

void print_localization(const LocalizationContext* loc_ctx) {
    char buf[512];
    LocalizationMsg msg;
    write_localization_msg(&msg, loc_ctx);
    LocalizationMsg_to_json(&msg, buf);
    puts(buf);
}

void generate_pipeline_montage(
        ImageMatrix* dst, const ImageMatrix raw, const LocalizationContext* loc_ctx) {
    assert(dst && ctx);
    assert(raw.size.x == 64 && raw.size.y == 64);
    // declare buffer area
    uint64_t* matrix = (uint64_t*) dst->data;
    uint64_t* mask = matrix + 64;
    ImageMatrix tmp_img = {(uint8_t*) (mask + 64), raw.size};
    IMG_SET_SIZE(*dst, raw.size.x * 3, raw.size.y * 2);
    // write correlation image to bottom left
    FOR_EACH_PIXEL(tmp_img) {
        // normalize to 255
        PIXEL(tmp_img, row, col) = 255 *
                                   PIXEL(loc_ctx->odom.correlation.image, row / 2, col / 2).xy[0] /
                                   loc_ctx->odom.correlation.squared_magnitude_max;
    }
    // roll to center
    for (int16_t row = 0; row < tmp_img.size.y / 2; ++row) {
        for (int16_t col = 0; col < tmp_img.size.x / 2; ++col) {
            SWAP(PIXEL(tmp_img, row, col),
                    PIXEL(tmp_img, row + tmp_img.size.y / 2, col + tmp_img.size.x / 2));
            SWAP(PIXEL(tmp_img, row + tmp_img.size.y / 2, col),
                    PIXEL(tmp_img, row, col + tmp_img.size.x / 2));
        }
    }
    ImagePoint top_left = {0, raw.size.y};
    IMG_PASTE(*dst, tmp_img, top_left);
    // write decoded image to bottom middle
    bm64_from_axiscodes(
            matrix, mask, &loc_ctx->scale_match.row_code, &loc_ctx->scale_match.col_code);
    bm64_to_img(&tmp_img, matrix, mask);
    top_left.x += raw.size.x;
    IMG_PASTE(*dst, tmp_img, top_left);
    // write extracted image to bottom right
    bm64_from_axiscodes(matrix, mask, &loc_ctx->row_code, &loc_ctx->col_code);
    bm64_to_img(&tmp_img, matrix, mask);
    top_left.x += raw.size.x;
    IMG_PASTE(*dst, tmp_img, top_left);
    // fill out top as gray
    dst->size.y >>= 1;
    IMG_FILL(*dst, 127);
    dst->size.y <<= 1;
    // write raw image to top left
    top_left.x = 0;
    top_left.y = 0;
    IMG_PASTE(*dst, raw, top_left);
    // write derotated image to top middle
    top_left.x += raw.size.x;
    IMG_PASTE(*dst, loc_ctx->derotated_image, top_left);
    // write sharpened image to top right
    top_left.x += raw.size.x;
    IMG_PASTE(*dst, loc_ctx->sharpened_image, top_left);
}
