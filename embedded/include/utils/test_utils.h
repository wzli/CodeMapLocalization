#pragma once
#include <stdio.h>

#define test_success(COND)                                                           \
    do {                                                                             \
        int error = (COND);                                                          \
        if (error) {                                                                 \
            printf("FAIL %s\n  %s:%d\n  %s\n", __func__, __FILE__, __LINE__, #COND); \
            return error;                                                            \
        }                                                                            \
    } while (0)

#define test_assert(COND) test_success(!(COND))

#define test_run(TEST)                    \
    do {                                  \
        printf("RUNNING %s --> ", #TEST); \
        int error = (TEST)();             \
        if (error) {                      \
            return error;                 \
        }                                 \
        printf("PASS\n");                 \
    } while (0)
