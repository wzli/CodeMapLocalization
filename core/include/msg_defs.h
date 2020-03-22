#pragma once
#include "mxgen.h"

#define STRUCT_LocationMatchMsg(X) \
    X(uint16_t, x, )               \
    X(uint16_t, y, )               \
    X(uint16_t, match_size, )      \
    X(float, x_err_ratio, )        \
    X(float, y_err_ratio, )        \
    X(float, scale, )
GEN_STRUCT(LocationMatchMsg);

#define STRUCT_OdometryMsg(X) \
    X(float, x, )             \
    X(float, y, )             \
    X(float, rot, )           \
    X(int32_t, quadrants, )   \
    X(uint32_t, steps, )
GEN_STRUCT(OdometryMsg);

#define STRUCT_CorrelationMsg(X) \
    X(float, x, )                \
    X(float, y, )                \
    X(float, err_ratio, )
GEN_STRUCT(CorrelationMsg);

#define STRUCT_LocalizationMsg(FIELD) \
    FIELD(uint32_t, frame, )          \
    FIELD(uint8_t, thresh, [2])       \
    FIELD(LocationMatchMsg, loc, )    \
    FIELD(OdometryMsg, odom, )        \
    FIELD(CorrelationMsg, corr, )
GEN_STRUCT(LocalizationMsg);
