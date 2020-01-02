#include "bitwise_utils.h"
#include <assert.h>

// bunch of bit twiddling hacks
// see https://graphics.stanford.edu/~seander/bithacks.html

void bm32_transpose(BitMatrix32 A) {
    uint32_t m = 0xFFFF0000;
    for (uint8_t j = 16; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for (uint8_t k = 0; k < 32; k = (k + j + 1) & ~j) {
            uint32_t t = (A[k] ^ (A[k + j] << j)) & m;
            A[k] ^= t;
            A[k + j] ^= (t >> j);
        }
    }
}

void bm64_transpose(BitMatrix64 A) {
    uint64_t m = 0xFFFFFFFF00000000ULL;
    for (uint8_t j = 32; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for (uint8_t k = 0; k < 64; k = (k + j + 1) & ~j) {
            uint64_t t = (A[k] ^ (A[k + j] << j)) & m;
            A[k] ^= t;
            A[k + j] ^= (t >> j);
        }
    }
}

uint32_t reverse_bits(uint32_t x, uint32_t n) {
    assert(n <= 32);
    x &= mask_bits(n);
    uint32_t r = x & 1;
    for (x >>= 1; x; x >>= 1) {
        r <<= 1;
        r |= x & 1;
        n--;
    }
    return r << (n - 1);
}

uint8_t perfect_log2(uint32_t x) {
    static const uint8_t debruijn_table[32] = {0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17,
            4, 8, 31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9};
    return debruijn_table[(x * 0x077CB531) >> 27];
}

uint8_t perfect_log2_64(uint64_t x) {
    static const uint8_t debruijn_table[64] = {0, 1, 2, 53, 3, 7, 54, 27, 4, 38, 41, 8, 34, 55, 48,
            28, 62, 5, 39, 46, 44, 42, 22, 9, 24, 35, 59, 56, 49, 18, 29, 11, 63, 52, 6, 26, 37, 40,
            33, 47, 61, 45, 43, 21, 23, 58, 17, 10, 51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15,
            30, 14, 13, 12};
    return debruijn_table[(x * 0x022fdd63cc95386dULL) >> 58];
}

uint8_t count_bits(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    return (((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}
