#include "stdio.h"
#include "string.h"
#include "stdint.h"

typedef enum {
    RIGHT = 0,
    DOWN,
    LEFT,
    UP
} HilbertDirection;

static inline HilbertDirection hilbert_get_direction(const uint8_t * dirs, uint32_t index) {
    return 3 & dirs[index / 4] >> (2 * (index & 3));
};

static void hilbert_build_directions(uint8_t* dirs, uint8_t order) {
    dirs[0] = RIGHT | (DOWN << 2) | (LEFT << 4) | (DOWN << 6);
    dirs[1] = dirs[0] ^ 0x55;
    dirs[2] = dirs[1] ^ 0xC0;
    dirs[3] = dirs[2] ^ 0xFF;

  for(uint8_t i = 2; i < order; ++i) {
        uint32_t size = 1 << (2 * (i - 1) );
        uint32_t src_idx = 0;
        uint32_t dst_idx = size;
        memcpy(dirs + dst_idx, dirs + src_idx, size);
        for(uint32_t j = 0; j < size/4; ++j) {
            ((uint32_t*)(dirs + dst_idx))[j] ^= 0x55555555;
        }
        src_idx += size;
        dst_idx += size;
        memcpy(dirs + dst_idx, dirs + src_idx, size);
        dirs[dst_idx + size - 1] ^= 0xC0;
        src_idx += size;
        dst_idx += size;
        memcpy(dirs + dst_idx, dirs + src_idx, size);
        for(uint32_t j = 0; j < size/4; ++j) {
            ((uint32_t*)(dirs + dst_idx))[j] ^= 0xFFFFFFFF;
        }
    }
}

uint32_t n = 0;
uint8_t dirs[1000] = {0};

void test_hilbert() {
    hilbert_build_directions(dirs, 3);
    for(int k = 0; k < 4; ++k) {
    for(int i = 0; i < 4; ++i) {
        for(int j = 0; j < 4; ++j) {
            HilbertDirection dir = hilbert_get_direction(dirs, k * 16 +  4 * i + j);
            //printf("%c", dir == UP ? 'y' : dir == DOWN ? 'Y' : dir == LEFT ? 'x': 'X');
        }
        //puts("");
    }
        //puts("");
    }

    int x = 0;
    int y = 0;
    printf("0,0\n");
    for(int i = 0; i < (1 << (2 * 3)) - 1; ++i) {
        HilbertDirection dir = hilbert_get_direction(dirs, i);
        switch (dir) {
            case UP:
                --y;
                break;
            case DOWN:
                ++y;
                break;
            case LEFT:
                --x;
                break;
            case RIGHT:
                ++x;
                break;
        }
        printf("%d,%d\n", x, y);
    }
};
