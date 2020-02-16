#pragma once
#include <stdint.h>
#include <limits.h>
#include <math.h>

#ifndef M_PI
#define M_PI (3.1415926f)
#endif

#ifndef M_SQRT1_2
#define M_SQRT1_2 (0.707106781f)
#endif

#define SGN(X) (((X) > 0) - ((X) < 0))
#define ABS(X) ((X) < 0 ? -(X) : (X))
#define SQR(X) ((X) * (X))
#define CUBE(X) ((x) * (X) * (X))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#define CLAMP(X, MIN, MAX) ((X) < (MIN) ? (MIN) : (X) > (MAX) ? (MAX) : (X))
#define IS_SIGNED(Type) ((Type) -1 < 0x7F)
#define IS_POWER_OF_TWO(X) !(X & (X - 1))

typedef struct {
    float x;
    float y;
} Vector2f;

typedef union {
    float elements[4];
    Vector2f rows[2];
} Matrix2f;

// vector operations
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

static inline Vector2f v2f_multiply(Vector2f a, Vector2f b) {
    return (Vector2f){a.x * b.x, a.y * b.y};
}

static inline Vector2f v2f_scale(Vector2f vec, float scale) {
    return (Vector2f){scale * vec.x, scale * vec.y};
}

static inline float v2f_dot(Vector2f a, Vector2f b) {
    return a.x * b.x + a.y * b.y;
}

static inline float v2f_cross(Vector2f a, Vector2f b) {
    return a.x * b.y - a.y * b.x;
}

static inline float v2f_norm_l1(Vector2f vec) {
    return ABS(vec.x) + ABS(vec.y);
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

// matrix operations

static inline Vector2f m2f_transform(Matrix2f mat, Vector2f vec) {
    return (Vector2f){v2f_dot(mat.rows[0], vec), v2f_dot(mat.rows[1], vec)};
}

static inline uint8_t m2f_is_zero(Matrix2f mat) {
    return v2f_is_zero(mat.rows[0]) && v2f_is_zero(mat.rows[1]);
}

static inline uint8_t m2f_is_nan(Matrix2f mat) {
    return v2f_is_nan(mat.rows[0]) || v2f_is_nan(mat.rows[1]);
}

static inline float m2f_determinant(Matrix2f mat) {
    return mat.elements[0] * mat.elements[3] - mat.elements[1] * mat.elements[2];
}

static inline Matrix2f m2f_transpose(Matrix2f mat) {
    return (Matrix2f){{mat.elements[0], mat.elements[2], mat.elements[1], mat.elements[3]}};
}

static inline Matrix2f m2f_scale(Matrix2f mat, float scale) {
    return (Matrix2f){.rows = {v2f_scale(mat.rows[0], scale), v2f_scale(mat.rows[1], scale)}};
}

static inline Matrix2f m2f_inverse(Matrix2f mat) {
    float inv_det = 1.0f / m2f_determinant(mat);
    return (Matrix2f){{
            mat.elements[3] * inv_det,
            -mat.elements[1] * inv_det,
            -mat.elements[2] * inv_det,
            mat.elements[0] * inv_det,
    }};
}

static inline Matrix2f m2f_multiply(Matrix2f a, Matrix2f b) {
    Matrix2f b_t = m2f_transpose(b);
    return (Matrix2f){{
            v2f_dot(a.rows[0], b_t.rows[0]),
            v2f_dot(a.rows[0], b_t.rows[1]),
            v2f_dot(a.rows[1], b_t.rows[0]),
            v2f_dot(a.rows[1], b_t.rows[1]),
    }};
}
