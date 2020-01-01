#include "tests.h"
#include "image_utils.h"

static uint32_t histogram[256];

int test_image_utils() {
    ImageMatrix test_img = {(uint8_t[5 * 5]){}, 5, 5};
    ImageMatrix buf_img = {(uint8_t[5 * 5 * 2]){}, 5, 5};
    test_assert(IMG_SIZE(test_img) == 25);

    for (int i = 0; i < IMG_SIZE(test_img); ++i) {
        test_img.data[i] = i;
    }

    int i = 0;
    FOR_EACH_PIXEL(test_img) { test_assert(PIXEL(test_img, row, col) == i++); };

    IMG_COPY(&buf_img, test_img);
    FOR_EACH_PIXEL(test_img) {
        test_assert(PIXEL(test_img, row, col) == PIXEL(buf_img, row, col));
    };

    IMG_FILL(buf_img, 0);
    FOR_EACH_PIXEL(test_img) { test_assert(PIXEL(buf_img, row, col) == 0); };

    int sum = 0;
    IMG_SUM(sum, test_img);
    test_assert(sum == 300);

    int avg = 0;
    IMG_AVERAGE(avg, test_img);
    test_assert(avg == 12);

    uint8_t max = 0;
    IMG_MAX(max, test_img);
    test_assert(max == 24);

    uint8_t min = max;
    IMG_MIN(min, test_img);
    test_assert(min == 0);

    int32_t accumulator = 0;
    IMG_APPLY_KERNEL(accumulator, edge_kernel, test_img, 0, 0);
    test_assert(accumulator == 0);

    IMG_THRESHOLD(&buf_img, test_img, 4);
    FOR_EACH_PIXEL(buf_img) { test_assert(PIXEL(buf_img, row, col) == (row > 0) * UINT8_MAX); }

    ImageWindow win = {3, 3, 5, 5};
    IMG_CROP(&buf_img, test_img, win);
    test_assert(IMG_SIZE(buf_img) == 4);
    FOR_EACH_PIXEL(buf_img) {
        test_assert(PIXEL(buf_img, row, col) == PIXEL(test_img, row + 3, col + 3));
    }

    IMG_VFLIP(&buf_img, test_img);
    FOR_EACH_PIXEL(buf_img) {
        test_assert(PIXEL(buf_img, row, col) == PIXEL(test_img, test_img.n_rows - 1 - row, col));
    }

    IMG_VFLIP(&buf_img, buf_img);
    FOR_EACH_PIXEL(test_img) {
        test_assert(PIXEL(test_img, row, col) == PIXEL(buf_img, row, col));
    };

    IMG_HFLIP(&buf_img, test_img);
    FOR_EACH_PIXEL(buf_img) {
        test_assert(PIXEL(buf_img, row, col) == PIXEL(test_img, row, test_img.n_cols - 1 - col));
    }

    IMG_HFLIP(&buf_img, buf_img);
    FOR_EACH_PIXEL(test_img) {
        test_assert(PIXEL(test_img, row, col) == PIXEL(buf_img, row, col));
    };

    IMG_TRANSPOSE(&buf_img, test_img);
    FOR_EACH_PIXEL(buf_img) { test_assert(PIXEL(buf_img, row, col) == PIXEL(test_img, col, row)); }

    IMG_TRANSPOSE(&buf_img, buf_img);
    FOR_EACH_PIXEL(test_img) {
        test_assert(PIXEL(test_img, row, col) == PIXEL(buf_img, row, col));
    };
    buf_img.n_cols = 4;
    IMG_TRANSPOSE(&buf_img, buf_img);
    test_assert(IMG_SIZE(buf_img) == 0);

    IMG_NORMALIZE_RANGE(&buf_img, test_img, 5, 20);
    test_assert(buf_img.data[5] == 0);
    test_assert(buf_img.data[20] == UINT8_MAX);

    IMG_NORMALIZE(&buf_img, test_img);
    test_assert(PIXEL(buf_img, 0, 0) == 0);
    test_assert(PIXEL(buf_img, 4, 4) == UINT8_MAX);

    img_histogram(histogram, test_img);
    for (int i = 0; i < 256; ++i) {
        test_assert(histogram[i] == (i < 25));
    }
    test_assert(img_otsu_histogram_threshold(histogram) == 12);

    buf_img.n_cols *= 2;
    img_resize(buf_img, test_img, img_nearest_interpolation);
    FOR_EACH_PIXEL(buf_img) {
        test_assert(PIXEL(buf_img, row, col) == PIXEL(test_img, row, col / 2));
    }
    return 0;
}
