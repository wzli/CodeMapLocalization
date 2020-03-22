#include "debug_prints.h"
#include <stdio.h>
#include <assert.h>

void write_location_match_msg(LocationMatchMsg* msg, const ScaleMatch* match) {
    assert(msg && match);
    msg->x = match->location.x;
    msg->y = match->location.y;
    msg->match_size = match->location.match_size;
    msg->bit_err_ratio = ((float) match->col_code.n_errors / match->col_code.n_samples +
                                 (float) match->row_code.n_errors / match->row_code.n_samples) *
                         0.5f;
    msg->scale = match->scale;
}

void write_odometry_msg(OdometryMsg* msg, const VisualOdometry* odom) {
    assert(msg && odom);
    msg->x = odom->position.x;
    msg->y = odom->position.y;
    msg->rot = cargf(odom->quadrant_rotation.z * QUADRANT_LOOKUP[odom->quadrant_count & 3].z);
    msg->quadrants = odom->quadrant_count;
    msg->steps = odom->step_count;
}

void write_correlation_msg(CorrelationMsg* msg, const Correlation* corr) {
    assert(msg && corr);
    msg->x = corr->translation.x;
    msg->y = corr->translation.y;
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

void print_axiscode(AxisCode32 axiscode) {
    uint8_t leading_zeros = count_trailing_zeros(reverse_bits(axiscode.mask, 32));
    printf("samples %d errors %d\n", axiscode.n_samples, axiscode.n_errors);
    print_bits(axiscode.bits, 32 - leading_zeros);
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
    ImageMatrix image = {(uint8_t[64 * 64]){}, {{64, 64}}};
    bm64_to_img(&image, bit_matrix, bit_mask);
    img_save_to_pgm(image, file_name);
}

void print_location_match(const ScaleMatch* match) {
    char buf[128];
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
