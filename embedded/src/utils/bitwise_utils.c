#include "bitwise_utils.h"
#include <assert.h>

// bunch of bit twiddling hacks
// see https://graphics.stanford.edu/~seander/bithacks.html

void bm32_transpose(BitMatrix32 A) {
    uint32_t m = 0xFFFF0000;
    for (uint32_t j = 16; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for (uint32_t k = 0; k < 32; k = (k + j + 1) & ~j) {
            uint32_t t = (A[k] ^ (A[k + j] << j)) & m;
            A[k] ^= t;
            A[k + j] ^= (t >> j);
        }
    }
}

void bm64_transpose(BitMatrix64 A) {
    uint64_t m = 0xFFFFFFFF00000000;
    for (uint64_t j = 32; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for (uint64_t k = 0; k < 64; k = (k + j + 1) & ~j) {
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

uint8_t first_set_bit(uint32_t x) {
    static const int MultiplyDeBruijnBitPosition[32] = {0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15,
            25, 17, 4, 8, 31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9};
    return x ? MultiplyDeBruijnBitPosition[((uint32_t)((x & -x) * 0x077CB531U)) >> 27] : 32;
}

uint8_t sum_bits(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    return (((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}
