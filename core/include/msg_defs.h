#pragma once
#include "mxgen.h"

#define TYPEDEF_LocationMatchMsg(X, _) \
    X(uint16_t, x, )                   \
    X(uint16_t, y, )                   \
    X(uint16_t, match_size, )          \
    X(uint16_t, downsample_errors, )   \
    X(float, xor_error_ratio, )        \
    X(float, quality, )                \
    X(float, scale, )
MXGEN(struct, LocationMatchMsg)

#define TYPEDEF_OdometryMsg(X, _) \
    X(float, x, )                 \
    X(float, y, )                 \
    X(float, rotation, )          \
    X(int32_t, quadrant_count, )  \
    X(uint32_t, drift_count, )
MXGEN(struct, OdometryMsg)

#define TYPEDEF_CorrelationMsg(X, _) \
    X(float, x, )                    \
    X(float, y, )                    \
    X(float, error_ratio, )
MXGEN(struct, CorrelationMsg)

#define TYPEDEF_LocalizationMsg(X, _) \
    X(uint32_t, frame, )              \
    X(OdometryMsg, odometry, )        \
    X(CorrelationMsg, correlation, )  \
    X(LocationMatchMsg, location, )   \
    X(uint8_t, threshold, )
MXGEN(struct, LocalizationMsg)
