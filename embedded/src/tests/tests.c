#include "tests.h"

int test_count = 0;
char test_error[200] = "ALL TESTS PASSED";

int run_all_tests() {
    test_run(test_bitwise_utils);
    test_run(test_mls_query);
    //mu_run_test(test_decode_location);
    return 0;
}
