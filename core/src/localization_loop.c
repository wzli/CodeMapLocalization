#include "localization_loop.h"
#include <assert.h>

void localization_loop_run(LocalizationContext* ctx, const ImageMatrix image) {
    assert(ctx);
    assert(ctx->sharpened_image.data);
    // find threshold of original image
    img_histogram(ctx->histogram, image);
    ctx->threshold[0] = img_compute_otsu_threshold(ctx->histogram);
    // unrotate the image
    ctx->rotation_estimate = img_derotation_filter(
            ctx->unrotated_image, image, ctx->rotation_scale, ctx->threshold[0]);
    // run optical flow
    img_estimate_translation(&(ctx->correlation), ctx->unrotated_image);
    // sharpen unrotated image and remove edge effects
    img_hyper_sharpen(&(ctx->sharpened_image), ctx->unrotated_image);
    ImagePoint image_center = {{ctx->sharpened_image.size.x / 2, ctx->sharpened_image.size.y / 2}};
    Vector2f vertex = {{2 * image_center.x, 2 * image_center.y}};
    vertex.z *= ctx->rotation_estimate.z;
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
    // compensate rotation estimate
    ctx->scale_match.location.rotation.z *= ctx->rotation_estimate.z;
}

Vector2f img_derotation_filter(
        ImageMatrix dst, const ImageMatrix src, float rotation_scale, uint8_t bg_fill) {
    assert(dst.data && src.data);
    // estimate rotation of original image
    Vector2f rotation_estimate = img_estimate_rotation(src);
    if (rotation_estimate.z != 0) {
        // unrotate
        rotation_estimate.y *= -rotation_scale;
        rotation_estimate.x *= rotation_scale;
        img_rotate(dst, src, rotation_estimate, bg_fill, img_bilinear_interpolation);
    }
    return rotation_estimate;
}
