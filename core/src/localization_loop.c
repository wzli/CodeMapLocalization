#include "localization_loop.h"
#include <assert.h>

bool localization_loop_run(LocalizationContext* ctx, const ImageMatrix image) {
    assert(ctx);
    assert(ctx->sharpened_image.data);
    ++ctx->frame_count;
    // find threshold of original image
    img_histogram(ctx->histogram, image);
    ctx->otsu_threshold = img_compute_otsu_threshold(ctx->histogram);
    // estimate rotation of original image and derotate it
    Vector2f quadrant_rotation = img_derotate(ctx->derotated_image, image, ctx->otsu_threshold);
    // run visual odometry
    odom_update(&ctx->odom, ctx->derotated_image, quadrant_rotation,
            ctx->outlier_filter.filtered_match.scale);
    // sharpen derotated image and remove edge effects
    img_hyper_sharpen(&(ctx->sharpened_image), ctx->derotated_image);
    ImagePoint image_center = {ctx->sharpened_image.size.x / 2, ctx->sharpened_image.size.y / 2};
    Vector2f vertex = {{2 + image_center.x, 2 + image_center.y}};
    vertex.z *= quadrant_rotation.z;
    img_draw_regular_polygon(ctx->sharpened_image, image_center, vertex, 4, ctx->otsu_threshold, 5);
    // binarize to bit matrix
    img_to_bm64(ctx->binary_image, ctx->binary_mask, ctx->sharpened_image, ctx->otsu_threshold,
            ctx->otsu_threshold);
    // extract row and column codes
    bm64_extract_axiscodes(
            &(ctx->row_code), &(ctx->col_code), ctx->binary_image, ctx->binary_mask, 5);
    // scale search and decode
    ctx->scale_match = (ScaleMatch){0};
    ac64_scale_search_location(
            &(ctx->scale_match), &(ctx->row_code), &(ctx->col_code), ctx->scale_decay_rate);
    // outlier rejection filter
    if (outlier_filter_location(&(ctx->outlier_filter), &(ctx->scale_match))) {
        // update odom
        odom_set_location(&(ctx->odom), ctx->outlier_filter.filtered_match.location);
        return true;
    }
    return false;
}

Vector2f img_derotate(ImageMatrix dst, const ImageMatrix src, uint8_t bg_fill) {
    assert(dst.data && src.data);
    Vector2f rotation = img_estimate_rotation(src);
    if (rotation.z != 0) {
        rotation.xy[1] = -rotation.xy[1];
        img_rotate(dst, src, rotation, bg_fill, img_bilinear_interpolation);
    }
    return rotation;
}
