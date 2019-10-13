#pragma once
#include <stdint.h>
#include <math.h>

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define SQR(x) ((x) * (x))
#define CUBE(x) (SQR(x) * (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : (x) > (max) ? (max) : (x))
#define IS_SIGNED(Type) ((Type) 0 - 1 <= 0)

#ifndef M_PI
#define M_PI (3.1415926f)
#endif

#ifndef M_SQRT1_2
#define M_SQRT1_2 (0.707106781f)
#endif

typedef struct {
    float x;
    float y;
} Vector2f;

static inline uint8_t v2f_is_zero(Vector2f vec) {
    return vec.x == 0 && vec.y == 0;
}

static inline uint8_t v2f_is_nan(Vector2f vec) {
    return isnan(vec.x) || isnan(vec.y);
}

static inline Vector2f v2f_add(Vector2f a, Vector2f b) {
    return (Vector2f){a.x + b.x, a.y + b.y};
}

static inline Vector2f v2f_subtract(Vector2f a, Vector2f b) {
    return (Vector2f){a.x - b.x, a.y - b.y};
}

static inline Vector2f v2f_scale(Vector2f vec, float scale) {
    return (Vector2f){scale * vec.x, scale * vec.y};
}

static inline float v2f_dot(Vector2f a, Vector2f b) {
    return a.x * b.x + a.y * b.y;
}

static inline float v2f_norm_sqr(Vector2f vec) {
    return SQR(vec.x) + SQR(vec.y);
}

static inline float v2f_norm(Vector2f vec) {
    return sqrtf(v2f_norm_sqr(vec));
}

static inline Vector2f v2f_normalize(Vector2f vec) {
    return v2f_scale(vec, 1.0f / v2f_norm(vec));
}

static inline Vector2f v2f_midpoint(Vector2f a, Vector2f b) {
    return v2f_scale(v2f_add(a, b), 0.5f);
}

static inline float v2f_distance_sqr(Vector2f a, Vector2f b) {
    return v2f_norm_sqr(v2f_subtract(a, b));
}

static inline float v2f_distance(Vector2f a, Vector2f b) {
    return sqrtf(v2f_distance_sqr(a, b));
}

static inline Vector2f v2f_flip_rotation(Vector2f rot) {
    float inv_r2 = 1.0f / v2f_norm_sqr(rot);
    return (Vector2f){inv_r2 * rot.x, -inv_r2 * rot.y};
}

static inline Vector2f v2f_rotate(Vector2f vec, Vector2f rot) {
    return (Vector2f){(rot.x * vec.x) - (rot.y * vec.y), (rot.y * vec.x) + (rot.x * vec.y)};
}

static inline Vector2f v2f_double_angle(Vector2f rot) {
    return (Vector2f){SQR(rot.x) - SQR(rot.y), 2.0f * rot.x * rot.y};
}

static inline Vector2f v2f_half_angle(Vector2f rot) {
    return (Vector2f){sqrtf((1.0f + rot.x) * 0.5f), copysignf(sqrtf((1.0f - rot.x) * 0.5f), rot.y)};
}

static inline Vector2f v2f_add_angle(Vector2f rot_a, Vector2f rot_b) {
    return (Vector2f){rot_a.x * rot_b.x - rot_a.y * rot_b.y, rot_a.y * rot_b.x + rot_a.x * rot_b.y};
}
