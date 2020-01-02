#include "tests.h"
#include "location_decode.h"
#include "mls_query.h"

int test_full_chain() {
    for (int i = 0; i < 10; ++i) {
        // src image setup
        static uint8_t src_img_buf[30 * 30];
        static ImageMatrix src_img = {src_img_buf, 30, 30};
        FOR_EACH_PIXEL(src_img) { PIXEL(src_img, row, col) = row + col; }

        // unrotate image
        static uint8_t img_buf[32 * 32];
        static ImageMatrix img = {img_buf, 32, 32};
        Vector2f rot = img_estimate_rotation(src_img);
        img_rotate(img, src_img, rot, 127, img_bilinear_interpolation);

        // convert to bit matrix
        static BitMatrix32 bit_matrix, bit_mask;
        img_to_bm32(bit_matrix, bit_mask, img, 125, 130);

        // extract codes
        static AxisCode32 row_code, col_code;
        bm32_extract_axis_codes(&row_code, &col_code, bit_matrix, bit_mask, 3);

        // overwrite with simulate extracted code
        row_code.bits = mlsq_code_from_position(MLS_INDEX.sequence, 32, i);
        row_code.mask = ~0;
        col_code = row_code;

        // decode positions
        AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
        AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

        // deduce location
        Location loc = deduce_location(row_pos, col_pos);

        // expect a valid location
        test_assert(loc.x == i + row_pos.span / 2);
        test_assert(loc.y == i + col_pos.span / 2);
    }
    return 0;
}
