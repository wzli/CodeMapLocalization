#pragma once

int test_math_utils();
int test_image_utils();
int test_bitwise_utils();
int test_mls_query();
int test_code_extraction();
int test_location_decode();
int test_full_chain();

static inline int run_all_tests() {
    return test_math_utils() || test_image_utils() || test_bitwise_utils() || test_mls_query() ||
           test_code_extraction() || test_location_decode() || test_full_chain();
}
