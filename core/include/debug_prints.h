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
void print_bit_matrix(BitMatrix32 matrix);
void print_image_matrix(ImageMatrix src);
void print_axiscode(AxisCode32 axiscode);
void print_axis_position(AxisPosition position);
void print_location(const Location* location);
void bm64_save_to_pgm(BitMatrix64 bit_matrix, BitMatrix64 bit_mask, const char* file_name);
