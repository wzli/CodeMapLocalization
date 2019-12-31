#include "tests.h"
#include "math_utils.h"

int test_math_utils() {
    Matrix2f mat = {{0, 1, 2, 3}};
    Matrix2f identity = m2f_multiply(mat, m2f_inverse(mat));
    test_assert(identity.elements[0] == 1);
    test_assert(identity.elements[1] == 0);
    test_assert(identity.elements[2] == 0);
    test_assert(identity.elements[3] == 1);

    test_assert(!IS_SIGNED(uint8_t));
    test_assert(!IS_SIGNED(uint16_t));
    test_assert(!IS_SIGNED(uint32_t));
    test_assert(!IS_SIGNED(uint64_t));
    test_assert(IS_SIGNED(int8_t));
    test_assert(IS_SIGNED(int16_t));
    test_assert(IS_SIGNED(int32_t));
    test_assert(IS_SIGNED(int64_t));

    int8_t ai = 3;
    int8_t bi = 7;
    SWAP(ai, bi);
    test_assert(ai == 7 && bi == 3);
    uint8_t ab[] = {33, 56};
    SWAP(ab[0], ab[1]);
    test_assert(ab[0] == 56 && ab[1] == 33);

    int8_t test_array[] = {0, 33, 33, 4, 6, 7, 1, 2, 8, 4, 3, 40};
    QUICK_SELECT(test_array, sizeof(test_array), 6);
    test_assert(test_array[6] == 6);

    return 0;
}
