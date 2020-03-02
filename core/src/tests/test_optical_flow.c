#include "test_utils.h"
#include "optical_flow.h"

#define CONV_SIZE (8)

static inline bool latch_comp(float complex cur_val, float complex max_val) {
    return cabsf(cur_val) > cabsf(max_val);
}

static int test_phase_correlation() {
    ImageMatrixComplex frame = {(float complex[SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};
    ImageMatrixComplex next_frame = {(float complex[SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};

    IMG_FILL(frame, 0);
    IMG_FILL(next_frame, 0);

    PIXEL(frame, 2, 2) = 1;
    PIXEL(next_frame, 3, 3) = 1;

    img_phase_correlation(frame, next_frame, false);

#if 0
    FOR_EACH_PIXEL(frame) {
        if (!col) { puts(""); }
        printf("%3d ", (int16_t)(PIXEL(frame, row, col) & 0xFFFF));
    }
#endif
    float complex max_val = 0;
    int16_t max_x = -1;
    int16_t max_y = -1;
    IMG_PIXEL_LATCH_INDEX(max_val, max_y, max_x, latch_comp, frame);
    test_assert(max_x == 1);
    test_assert(max_y == 1);
    return 0;
}

int test_optical_flow() {
    test_run(test_phase_correlation);
    return 0;
}
