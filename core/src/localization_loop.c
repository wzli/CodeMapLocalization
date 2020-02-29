#include "localization_loop.h"

void localization_loop_run(LocalizationContext* ctx) {
    // find threshold of original image
    img_histogram(ctx->histogram, ctx->original_image);
    ctx->threshold[0] = img_compute_otsu_threshold(ctx->histogram);

    // estimate rotation of original image
    ctx->rotation_estimate = img_estimate_rotation(ctx->original_image);
    if (!v2f_is_zero(ctx->rotation_estimate)) {
        // unrotate
        ctx->rotation_estimate.y *= -1;
        ctx->unrotated_image.size = ctx->original_image.size;
        img_rotate(ctx->unrotated_image, ctx->original_image, ctx->rotation_estimate,
                ctx->threshold[0], img_bilinear_interpolation);
    }
    // sharpen unrotated image and remove edge effects
    img_hyper_sharpen(&(ctx->sharpened_image), ctx->unrotated_image);
    ImagePoint image_center = {ctx->sharpened_image.size.x / 2, ctx->sharpened_image.size.y / 2};
    Vector2f vertex =
            v2f_rotate(ctx->rotation_estimate, (Vector2f){2 + image_center.x, 2 + image_center.y});
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
    ctx->scale_match.location.rotation =
            v2f_add_angle(ctx->scale_match.location.rotation, ctx->rotation_estimate);
}
