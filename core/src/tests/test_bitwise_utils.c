#include "test_utils.h"
#include "bitwise_utils.h"

static int test_bv32_clear_all() {
    uint32_t v[3] = {0xFFFF, 0xFFFF, 0xFFFF};
    bv32_clear_all(v, 32 * 3);
    test_assert(v[0] == 0);
    test_assert(v[1] == 0);
    test_assert(v[2] == 0);
    return 0;
}

static int test_bv32_get_slice() {
    uint32_t v[3] = {0xFFFF, 0xFFFF, 0xFFFF};
    test_assert(bv32_get_slice(v, 0, 4) == 0xF);
    test_assert(bv32_get_slice(v, 0 + 32, 4) == 0xF);
    test_assert(bv32_get_slice(v, 0, 20) == 0xFFFF);
    test_assert(bv32_get_slice(v, 0 + 32, 20) == 0xFFFF);
    test_assert(bv32_get_slice(v, 8, 20) == 0xFF);
    test_assert(bv32_get_slice(v, 8 + 32, 20) == 0xFF);
    test_assert(bv32_get_slice(v, 8, 20) == 0xFF);
    test_assert(bv32_get_slice(v, 8 + 32, 20) == 0xFF);
    test_assert(bv32_get_slice(v, 24, 20) == 0xFFF00);
    test_assert(bv32_get_slice(v, 24 + 32, 20) == 0xFFF00);

    test_assert(bv32_get_slice_64(v, 0, 64) == 0xFFFF0000FFFFull);
    test_assert(bv32_get_slice_64(v, 16, 48) == 0x0000FFFF0000ull);
    return 0;
}

static int test_bv32_scale() {
    uint32_t src[1] = {0xF0F0};
    uint32_t dst[6] = {0};
    bv32_scale(dst, src, 16, 16, 1);
    test_assert(dst[0] == src[0]);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 8, 16, 1);
    test_assert(dst[0] == 0xF0);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 16, 8, 1);
    test_assert(dst[0] == 0xF0);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 32, 16, 2);
    test_assert(dst[0] == 0xFF00FF00);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 64, 16, 4);
    test_assert(dst[0] == 0xFFFF0000);
    test_assert(dst[1] == 0xFFFF0000);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 64, 16, 0.5f);
    test_assert(dst[0] == 0xCC);
    test_assert(dst[1] == 0);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 64, 16, 0.5f);
    test_assert(dst[0] == 0xCC);
    test_assert(dst[1] == 0);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 64, 16, 0.25f);
    test_assert(dst[0] == 0xA);
    test_assert(dst[1] == 0);

    bv32_clear_all(dst, 32 * 6);
    bv32_scale(dst, src, 64, 16, 100);
    test_assert(dst[0] == 0);
    test_assert(dst[1] == 0);
    return 0;
}

static int test_invert_bits() {
    test_assert(invert_bits(0, 2) == 3);
    test_assert(invert_bits(0, 32) == ~0u);
    test_assert(invert_bits(0xf0f0f0f0, 32) == ~0xf0f0f0f0);
    test_assert(invert_bits(0xF0000000, 3) == 7);
    // test 64 bit
    test_assert(invert_bits(0ull, 64) == ~0ull);
    test_assert(invert_bits(0xf0f0f0f0ull << 32, 64) == ~(0xf0f0f0f0ull << 32));
    test_assert(invert_bits_64(0xF0000000ull, 3) == 7);
    return 0;
}

static int test_reverse_bits() {
    test_assert(reverse_bits(1, 3) == (1 << 2));
    test_assert(reverse_bits(1, 32) == (1u << 31));
    test_assert(reverse_bits(0xf0f0f0f0, 32) == 0x0f0f0f0f);
    test_assert(reverse_bits(0xF00000f0, 8) == 0xf);
    // test 64 bit
    test_assert(reverse_bits(1ull, 64) == (1ull << 63));
    test_assert(reverse_bits(0xf0f0f0f0ull << 32, 64) == reverse_bits(0x0f0f0f0f0, 32));
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
    BitMatrix32 bm32 = {0};
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

    BitMatrix64 bm64 = {0};
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
    BitMatrix32 bm32 = {0};
    bm32_set_bit(bm32, 30, 30);
    test_assert(bm32_get_bit(bm32, 30, 30) == 1);
    bm32_clear_bit(bm32, 30, 30);
    test_assert(bm32_get_bit(bm32, 30, 30) == 0);

    BitMatrix64 bm64 = {0};
    bm64_set_bit(bm64, 60, 60);
    test_assert(bm64_get_bit(bm64, 60, 60) == 1);
    bm64_clear_bit(bm64, 60, 60);
    test_assert(bm64_get_bit(bm64, 60, 60) == 0);
    return 0;
}

int test_bitwise_utils() {
    test_run(test_bv32_clear_all);
    test_run(test_bv32_get_slice);
    test_run(test_bv32_scale);
    test_run(test_invert_bits);
    test_run(test_reverse_bits);
    test_run(test_count_bits);
    test_run(test_count_trailing_zeros);
    test_run(test_bit_matrix_transpose);
    test_run(test_bit_matrix_set_clear_bits);
    return 0;
}
