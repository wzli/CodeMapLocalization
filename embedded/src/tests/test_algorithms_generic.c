#include "test_utils.h"
#include "generic_algorithms.h"

static int test_swap() {
    int8_t ai = 3;
    int8_t bi = 7;
    SWAP(ai, bi);
    test_assert(ai == 7 && bi == 3);
    uint8_t ab[] = {33, 56};
    SWAP(ab[0], ab[1]);
    test_assert(ab[0] == 56 && ab[1] == 33);
    int64_t a64 = 3ull << 40;
    int64_t b64 = 7ull << 40;
    SWAP(a64, b64);
    test_assert(a64 == 7ull << 40 && b64 == 3ull << 40);
    return 0;
}

static int test_quick_select() {
    int8_t test_array[] = {0, 33, 33, 4, 6, 7, 1, 2, 8, 4, 3, 40};
    QUICK_SELECT(test_array, sizeof(test_array), 6);
    test_assert(test_array[6] == 6);
    return 0;
}

int test_generic_algorithms() {
    test_run(test_swap);
    test_run(test_quick_select);
    return 0;
}
