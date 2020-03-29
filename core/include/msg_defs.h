#pragma once
#include "mxgen.h"

#define TYPEDEF_LocationMatchMsg(X, _) \
    X(uint16_t, x, )                   \
    X(uint16_t, y, )                   \
    X(uint16_t, match_size, )          \
    X(uint16_t, downsample_errors, )   \
    X(float, xor_err_ratio, )          \
    X(float, quality, )                \
    X(float, scale, )
MXGEN(struct, LocationMatchMsg)

#define TYPEDEF_OdometryMsg(X, _) \
    X(float, x, )                 \
    X(float, y, )                 \
    X(float, rot, )               \
    X(int32_t, quadrants, )       \
    X(uint32_t, steps, )
MXGEN(struct, OdometryMsg)

#define TYPEDEF_CorrelationMsg(X, _) \
    X(float, x, )                    \
    X(float, y, )                    \
    X(float, err_ratio, )
MXGEN(struct, CorrelationMsg)

#define TYPEDEF_LocalizationMsg(X, _) \
    X(uint32_t, frame, )              \
    X(uint8_t, thresh, [2])           \
    X(LocationMatchMsg, loc, )        \
    X(OdometryMsg, odom, )            \
    X(CorrelationMsg, corr, )
MXGEN(struct, LocalizationMsg)
