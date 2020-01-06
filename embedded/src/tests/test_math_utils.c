#include "test_utils.h"
#include "math_utils.h"

static int test_is_signed() {
    test_assert(!IS_SIGNED(uint8_t));
    test_assert(!IS_SIGNED(uint16_t));
    test_assert(!IS_SIGNED(uint32_t));
    test_assert(!IS_SIGNED(uint64_t));
    test_assert(IS_SIGNED(int8_t));
    test_assert(IS_SIGNED(int16_t));
    test_assert(IS_SIGNED(int32_t));
    test_assert(IS_SIGNED(int64_t));
    return 0;
}

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

static int test_matrix2f_inverse() {
    Matrix2f mat = {{0, 1, 2, 3}};
    Matrix2f identity = m2f_multiply(mat, m2f_inverse(mat));
    test_assert(identity.elements[0] == 1);
    test_assert(identity.elements[1] == 0);
    test_assert(identity.elements[2] == 0);
    test_assert(identity.elements[3] == 1);
    return 0;
}

int test_math_utils() {
    test_run(test_is_signed);
    test_run(test_swap);
    test_run(test_quick_select);
    test_run(test_matrix2f_inverse);
    return 0;
}
