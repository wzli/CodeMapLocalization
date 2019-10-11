#include "tests.h"

int run_all_tests() {
    test_run(test_bitwise_utils);
    test_run(test_mls_query);
    test_run(test_decode_location);
    return 0;
}
