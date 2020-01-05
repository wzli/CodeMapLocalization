#include "test_utils.h"
#include "bitwise_utils.h"

static int test_invert_bits() {
    test_assert(invert_bits(0, 2) == 3);
    test_assert(invert_bits(0, 32) == ~0u);
    test_assert(invert_bits(0xf0f0f0f0, 32) == ~0xf0f0f0f0);
    test_assert(invert_bits(0xF0000000, 3) == 7);
    return 0;
}

static int test_reverse_bits() {
    test_assert(reverse_bits(1, 3) == (1 << 2));
    test_assert(reverse_bits(1, 32) == (1u << 31));
    test_assert(reverse_bits(0xf0f0f0f0, 32) == 0x0f0f0f0f);
    test_assert(reverse_bits(0xF00000f0, 8) == 0xf);
    return 0;
}

static int test_count_bits() {
    test_assert(count_bits(0) == 0);
    test_assert(count_bits(1) == 1);
    test_assert(count_bits(~0) == 32);
    test_assert(count_bits(0xf0f0f0f0) == 16);
    test_assert(count_bits(~0ull) == 64);
    test_assert(count_bits(0xf0f0f0f000000000) == 16);
    return 0;
}

static int test_count_trailing_zeros() {
    test_assert(count_trailing_zeros(0) == 32);
    test_assert(count_trailing_zeros(~0) == 0);
    test_assert(count_trailing_zeros(1 << 7) == 7);
    test_assert(count_trailing_zeros(0xf0f0f0f0) == 4);
    test_assert(count_trailing_zeros(0ull) == 64);
    test_assert(count_trailing_zeros(1ull << 40) == 40);
    test_assert(count_trailing_zeros(0xf0f0f0f000000000) == 36);
    return 0;
}

static int test_bit_matrix_transpose() {
    BitMatrix32 bm32 = {};
    bm32[0] = ~0;
    bm32_transpose(bm32);
    for (int i = 0; i < 32; ++i) {
        test_assert(bm32[i] == 1);
    }
    bm32_transpose(bm32);
    test_assert(bm32[0] == ~0u);
    for (int i = 1; i < 32; ++i) {
        test_assert(bm32[i] == 0);
    }

    BitMatrix64 bm64 = {};
    bm64[0] = ~0ull;
    bm64_transpose(bm64);
    for (int i = 0; i < 64; ++i) {
        test_assert(bm64[i] == 1);
    }
    bm64_transpose(bm64);
    test_assert(bm64[0] == ~0ull);
    for (int i = 1; i < 64; ++i) {
        test_assert(bm64[i] == 0);
    }
    return 0;
}

static int test_bit_matrix_set_clear_bits() {
    BitMatrix32 bm32 = {};
    bm32_set_bit(bm32, 30, 30);
    test_assert(bm32_get_bit(bm32, 30, 30) == 1);
    bm32_clear_bit(bm32, 30, 30);
    test_assert(bm32_get_bit(bm32, 30, 30) == 0);

    BitMatrix64 bm64 = {};
    bm64_set_bit(bm64, 60, 60);
    test_assert(bm64_get_bit(bm64, 60, 60) == 1);
    bm64_clear_bit(bm64, 60, 60);
    test_assert(bm64_get_bit(bm64, 60, 60) == 0);
    return 0;
}

int test_bitwise_utils() {
    test_run(test_invert_bits);
    test_run(test_reverse_bits);
    test_run(test_count_bits);
    test_run(test_count_trailing_zeros);
    test_run(test_bit_matrix_transpose);
    test_run(test_bit_matrix_set_clear_bits);
    return 0;
}
