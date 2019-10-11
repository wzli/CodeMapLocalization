#include "test_decode_location.h"

int tests_run = 0;

static char* run_all_tests() {
    mu_run_test(test_decode_location);
    return 0;
}

int main() {
    char* error = run_all_tests();
    printf("%s\nTests run: %d\n", error ? error : "ALL TESTS PASSED", tests_run);
    return 0;
}
