#include "tests.h"

int run_all_tests() {
    test_run(test_math_utils);
    test_run(test_image_utils);
    test_run(test_bitwise_utils);
    test_run(test_mls_query);
    test_run(test_code_extract_64);
    test_run(test_next_valid_code_segment);
    test_run(test_location_decode);
    test_run(test_full_chain);
    printf("ALL TESTS PASS\n");
    return 0;
}
