#include "hilbert_gen.h"
#include "assert.h"

void hilbert_curve_generate(uint8_t* curve, uint8_t order) {
    assert(curve);
    // store 1st order curve as the first byte
    curve[0] = HILBERT_CURVE_RIGHT | (HILBERT_CURVE_UP << 2) | (HILBERT_CURVE_LEFT << 4) |
               (HILBERT_CURVE_UP << 6);
    // store 2nd order curve as the first word
    // these operations are explained in comments below
    curve[1] = curve[0] ^ 0x55;
    curve[2] = curve[1] ^ 0xC0;
    curve[3] = ~curve[2];

    // main procedure for order 2 and above
    for (int8_t i = 2; i < order; ++i) {
        // block size in units of 32-bit words
        uint32_t block_size = 1 << (2 * i - 4);
        uint32_t* blocks = (uint32_t*) curve;
        // obtain the second and third quadrant by copying the transpose of the first quadrant
        for (uint32_t j = 0; j < block_size; ++j) {
            // XOR with 0x55555555 eg 0b01010101... flips every other bit
            // this maps to a transpose operation in directional bit encoding
            blocks[j + block_size] = blocks[j + 2 * block_size] = blocks[j] ^ 0x55555555;
        }
        // anti-transpose the tail of the third quadrant to connect to the fourth quadrant
        curve[3 * 4 * block_size - 1] ^= 0xC0;
        // obtain the fourth quadrant by copy the anti-transpose of the third quadrant
        blocks += 2 * block_size;
        for (uint32_t j = 0; j < block_size; ++j) {
            // inverting bits map to the anti-transpose operation in directional bit encoding
            blocks[j + block_size] = ~blocks[j];
        }
    }
}
