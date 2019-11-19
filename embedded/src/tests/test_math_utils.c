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

    test_assert(INT_TYPE_MAX(uint8_t) == UINT8_MAX);
    test_assert(INT_TYPE_MIN(uint8_t) == 0);
    test_assert(INT_TYPE_MAX(uint16_t) == UINT16_MAX);
    test_assert(INT_TYPE_MIN(uint16_t) == 0);
    test_assert(INT_TYPE_MAX(uint32_t) == UINT32_MAX);
    test_assert(INT_TYPE_MIN(uint32_t) == 0);
    test_assert(INT_TYPE_MAX(uint64_t) == UINT64_MAX);
    test_assert(INT_TYPE_MIN(uint64_t) == 0);
    test_assert(INT_TYPE_MAX(int8_t) == INT8_MAX);
    test_assert(INT_TYPE_MIN(int8_t) == INT8_MIN);
    test_assert(INT_TYPE_MAX(int16_t) == INT16_MAX);
    test_assert(INT_TYPE_MIN(int16_t) == INT16_MIN);
    test_assert(INT_TYPE_MAX(int32_t) == INT32_MAX);
    test_assert(INT_TYPE_MIN(int32_t) == INT32_MIN);
    test_assert(INT_TYPE_MAX(int64_t) == INT64_MAX);
    test_assert(INT_TYPE_MIN(int64_t) == INT64_MIN);
    return 0;
}
