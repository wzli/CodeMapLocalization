#include "localization_loop.h"
#include <assert.h>

void localization_loop_run(LocalizationContext* ctx, const ImageMatrix image) {
    assert(ctx);
    assert(ctx->sharpened_image.data);
    // find threshold of original image
    img_histogram(ctx->histogram, image);
    ctx->threshold[0] = img_compute_otsu_threshold(ctx->histogram);
    // estimate rotation of original image and derotate it
    Vector2f quadrant_rotation =
            img_derotate(ctx->unrotated_image, image, ctx->rotation_scale, ctx->threshold[0]);
    // run visual odometry
    odom_update(&ctx->odom_ctx, ctx->unrotated_image, quadrant_rotation,
            ctx->outlier_filter.filtered_match.scale);
    // sharpen unrotated image and remove edge effects
    img_hyper_sharpen(&(ctx->sharpened_image), ctx->unrotated_image);
    ImagePoint image_center = {{ctx->sharpened_image.size.x / 2, ctx->sharpened_image.size.y / 2}};
    Vector2f vertex = {{2 * image_center.x, 2 * image_center.y}};
    vertex.z *= quadrant_rotation.z * ctx->rotation_scale;
    img_draw_regular_polygon(ctx->sharpened_image, image_center, vertex, 4, ctx->threshold[0], 5);
    // find threshold of filtered image
    img_histogram(ctx->histogram, ctx->sharpened_image);
    ctx->histogram[ctx->threshold[0]] = 0;
    ctx->threshold[1] = img_compute_otsu_threshold(ctx->histogram);
    if (ctx->threshold[1] < ctx->threshold[0]) {
        SWAP(ctx->threshold[1], ctx->threshold[0]);
    }
    // binarize to bit matrix
    img_to_bm64(ctx->binary_image, ctx->binary_mask, ctx->sharpened_image, ctx->threshold[0],
            ctx->threshold[1]);
    // extract row and column codes
    bm64_extract_axiscodes(&(ctx->scale_query.row_code), &(ctx->scale_query.col_code),
            ctx->binary_image, ctx->binary_mask, 5);
    ctx->scale_match = (ScaleMatch){};
    scale_search_location(&(ctx->scale_match), &(ctx->scale_query));
    if (outlier_filter_location(&(ctx->outlier_filter), &(ctx->scale_match))) {
        ctx->odom_ctx.quadrant_count &= ~3u;
        ctx->odom_ctx.quadrant_count |= ctx->outlier_filter.filtered_match.location.direction;
    }
}

Vector2f img_derotate(ImageMatrix dst, const ImageMatrix src, float scale, uint8_t bg_fill) {
    assert(dst.data && src.data);
    Vector2f rotation_estimate = img_estimate_rotation(src);
    rotation_estimate.y = -rotation_estimate.y;
    if (rotation_estimate.z != 0) {
        img_rotate(dst, src, (Vector2f)(rotation_estimate.z * scale), bg_fill,
                img_bilinear_interpolation);
    }
    return rotation_estimate;
}
