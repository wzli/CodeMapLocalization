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

void scale_search_location(
        ScaleMatch* match, const AxisCode* row_code, const AxisCode* col_code, float decay_rate) {
    assert(match && row_code && col_code && decay_rate > 0 && decay_rate <= 1);
    ScaleMatch candidate;
    for (candidate.scale = 1.0f; candidate.scale >= 1.0f / 3;
            candidate.scale *= 1 - decay_rate) {
        // scale and down sample axis codes
        candidate.row_code = *row_code;
        candidate.col_code = *col_code;
        candidate.row_scale_errors = ac64_downsample(&candidate.row_code, candidate.scale);
        candidate.col_scale_errors = ac64_downsample(&candidate.col_code, candidate.scale);
        // decode posiiton
        AxisPosition row_pos = ac64_decode_position(candidate.row_code);
        AxisPosition col_pos = ac64_decode_position(candidate.col_code);
        candidate.location = deduce_location(row_pos, col_pos);
        if (candidate.location.match_size >= match->location.match_size) {
            *match = candidate;
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
