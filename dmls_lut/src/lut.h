#pragma once
#include <stdint.h>

typedef enum {
    LUT_SUCCESS = 0,
    LUT_INSERT_COLLISION
} LutError;

uint16_t lut_hash(uint8_t len, uint64_t salt, uint32_t key);
LutError lut_insert(uint16_t* lut, uint8_t hash_len, uint32_t hash_multiplier, uint32_t key, uint16_t value);
LutError lut_access(const uint16_t* lut, uint8_t hash_len, uint32_t hash_multiplier, uint32_t key, uint16_t* value);

