#include "test_utils.h"
#include "optical_flow.h"

#define CONV_SIZE (8)

static int test_phase_correlation() {
    ImageMatrixInt32 frame = {(int32_t[2 * SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};
    ImageMatrixInt32 next_frame = {(int32_t[2 * SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};

    IMG_FILL(frame, 0);
    IMG_FILL(next_frame, 0);

    PIXEL(frame, 3, 3) = 1;
    PIXEL(next_frame, 2, 2) = 1;

    img_phase_correlation(frame, next_frame, false);

#if 0
    FOR_EACH_PIXEL(frame) {
        if (!col) { puts(""); }
        printf("%3d ", (int16_t)(PIXEL(frame, row, col) & 0xFFFF));
    }
#endif
    int32_t max_val = -1;
    int16_t max_x = -1;
    int16_t max_y = -1;
    IMG_PIXEL_LATCH_INDEX(max_val, max_y, max_x, >, frame);
    test_assert(max_x == 1);
    test_assert(max_y == 1);
    return 0;
}

int test_optical_flow() {
    test_run(test_phase_correlation);
    return 0;
}
