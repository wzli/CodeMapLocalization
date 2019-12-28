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

    return 0;
}
