#include "test_utils.h"
#include "visual_odometry.h"

#define CONV_SIZE (8)

static inline bool latch_comp(Vector2f cur_val, Vector2f max_val) {
    return cabsf(cur_val.z) > cabsf(max_val.z);
}

static int test_phase_correlation() {
    ImageMatrixComplex frame = {(Vector2f[SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};
    ImageMatrixComplex next_frame = {(Vector2f[SQR(CONV_SIZE)]){}, {CONV_SIZE, CONV_SIZE}};

    IMG_FILL(frame, (Vector2f){});
    IMG_FILL(next_frame, (Vector2f){});

    PIXEL(frame, 2, 2).x = 1;
    PIXEL(next_frame, 3, 3).x = 1;

    img_phase_correlation(frame, next_frame, false);

#if 0
    FOR_EACH_PIXEL(frame) {
        if (!col) { puts(""); }
        printf("%3d ", (int16_t)(PIXEL(frame, row, col) & 0xFFFF));
    }
#endif
    Vector2f max_val = {};
    int16_t max_x = -1;
    int16_t max_y = -1;
    IMG_PIXEL_LATCH_INDEX(max_val, max_y, max_x, latch_comp, frame);
    test_assert(max_x == 1);
    test_assert(max_y == 1);
    return 0;
}

int test_visual_odometry() {
    test_run(test_phase_correlation);
    return 0;
}
