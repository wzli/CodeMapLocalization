#include "tests.h"
#include "bitwise_utils.h"

int test_bitwise_utils() {
    test_assert(invert_bits(0, 2) == 3);
    test_assert(invert_bits(0, 32) == ~0u);
    test_assert(invert_bits(0xf0f0f0f0, 32) == ~0xf0f0f0f0);
    test_assert(invert_bits(0xF0000000, 3) == 7);
    test_assert(reverse_bits(1, 3) == (1 << 2));
    test_assert(reverse_bits(1, 32) == (1u << 31));
    test_assert(reverse_bits(0xf0f0f0f0, 32) == 0x0f0f0f0f);
    test_assert(reverse_bits(0xF00000f0, 8) == 0xf);
    test_assert(sum_bits(0) == 0);
    test_assert(sum_bits(1) == 1);
    test_assert(sum_bits(~0) == 32);
    test_assert(sum_bits(0xf0f0f0f0) == 16);
    test_assert(first_set_bit(0) == 32);
    test_assert(first_set_bit(~0) == 0);
    test_assert(first_set_bit(1 << 7) == 7);
    test_assert(first_set_bit(0xf0f0f0f0) == 4);
    return 0;
}
