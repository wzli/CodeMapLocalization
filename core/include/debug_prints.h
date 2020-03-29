#pragma once
#include "localization_loop.h"
#include "msg_defs.h"

// write message helpers
void write_location_match_msg(LocationMatchMsg* msg, const ScaleMatch* match);
void write_odometry_msg(OdometryMsg* msg, const VisualOdometry* odom);
void write_correlation_msg(CorrelationMsg* msg, const Correlation* corr);
void write_localization_msg(LocalizationMsg* msg, const LocalizationContext* loc_ctx);

// debug prints for internal data structures
void print_bits(uint64_t word, int8_t word_length);
void print_bit_vector(const uint32_t* vector, uint32_t len);
void print_bit_matrix(BitMatrix32 matrix);
void print_image_matrix(ImageMatrix src);
void print_axiscode(AxisCode axiscode);
void print_axis_position(AxisPosition position);
void print_location(const Location* location);
void print_location_match(const ScaleMatch* match);
void print_odometry(const VisualOdometry* odom);
void print_correlation(const Correlation* corr);
void print_localization(const LocalizationContext* loc_ctx);
void bm64_save_to_pgm(BitMatrix64 bit_matrix, BitMatrix64 bit_mask, const char* file_name);
void generate_pipeline_montage(
        ImageMatrix* dst, const ImageMatrix raw, const LocalizationContext* loc_ctx);
