#include "location_decode.h"
#include "mls_query.h"
#include <assert.h>

#define WIDTH 32
#include "position_decode_impl.h"

#define WIDTH 64
#include "position_decode_impl.h"

Location deduce_location(AxisPosition row_position, AxisPosition col_position) {
    Location loc = {0};
    loc.match_size = row_position.inverted == col_position.inverted
                             ? row_position.span * col_position.span
                             : 0;
    assert(loc.match_size >= 0);
    loc.direction = (row_position.reversed << 1) | (col_position.reversed ^ row_position.reversed);
    if (row_position.reversed ^ col_position.reversed) {
        loc.x = col_position.center;
        loc.y = row_position.center;
    } else {
        loc.x = row_position.center;
        loc.y = col_position.center;
    }
    return loc;
}

void scale_search_location(ScaleMatch* match, const ScaleQuery* query) {
    assert(match && query);
    assert(query->lower_bound > 0 && query->upper_bound > 0 && query->step_size > 0);
    for (float scale = query->lower_bound; scale <= query->upper_bound; scale += query->step_size) {
        // scale and down sample axis codes
        AxisCode64 row_code = query->row_code;
        AxisCode64 col_code = query->col_code;
        ac64_downsample(&row_code, scale / 3);
        ac64_downsample(&col_code, scale / 3);
        // decode posiiton
        AxisPosition row_pos = ac64_decode_position(row_code);
        AxisPosition col_pos = ac64_decode_position(col_code);
        Location loc = deduce_location(row_pos, col_pos);
        if (loc.match_size >= match->location.match_size) {
            *match = (ScaleMatch){loc, row_code, col_code, scale};
        }
    }
}

bool outlier_filter_location(OutlierFilter* ctx, const ScaleMatch* new_match) {
    assert(ctx && new_match);
    if (new_match->location.x == ctx->filtered_match.location.x &&
            new_match->location.y == ctx->filtered_match.location.y) {
        return true;
    }
    if (new_match->location.match_size < SQR(ctx->match_length_threshold)) {
        return false;
    }
    if ((new_match->row_code.n_errors * ctx->bit_error_ratio_threshold >
                new_match->row_code.n_samples) ||
            (new_match->col_code.n_errors * ctx->bit_error_ratio_threshold >
                    new_match->col_code.n_samples)) {
        return false;
    }
    int32_t dx = new_match->location.x - ctx->filtered_match.location.x;
    int32_t dy = new_match->location.y - ctx->filtered_match.location.y;
    if (ABS(dx) > ctx->distance_threshold || ABS(dy) > ctx->distance_threshold) {
        if (++ctx->rejection_count > ctx->max_rejection_count) {
            ctx->rejection_count = 0;
            ctx->filtered_match = *new_match;
            return true;
        }
        return false;
    }
    ctx->rejection_count = 0;
    ctx->filtered_match = *new_match;
    return true;
}
