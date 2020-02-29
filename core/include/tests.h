#pragma once
#include "test_micro_image_utils.h"

int test_bitwise_utils();
int test_mls_query();
int test_code_extraction();
int test_location_decode();
int test_optical_flow();
int test_localization_loop();

static inline int run_all_tests() {
    return test_micro_image_utils() || test_bitwise_utils() || test_mls_query() ||
           test_code_extraction() || test_location_decode() || test_optical_flow() ||
           test_localization_loop();
}
