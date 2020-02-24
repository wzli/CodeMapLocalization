#include "test_utils.h"
#include "optical_flow.h"

#define CONV_SIZE (8)

static int test_phase_correlation() {
    ImageMatrixInt32 frame = {(int32_t[2 * SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};
    ImageMatrixInt32 next_frame = {(int32_t[2 * SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};

    IMG_FILL(frame, 0);
    IMG_FILL(next_frame, 0);

    PIXEL(frame, 1, 1) = 1;
    PIXEL(next_frame, 0, 0) = 1;

    img_phase_correlation(frame, next_frame, false);

    FOR_EACH_PIXEL(next_frame) { test_assert(PIXEL(frame, row, col) == (row == 1 && col == 1)); }
    return 0;
}

int test_optical_flow() {
    test_run(test_phase_correlation);
    return 0;
}
