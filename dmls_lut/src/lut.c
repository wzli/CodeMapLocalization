#include "lut.h"
#include <stdio.h>

uint16_t lut_hash(uint8_t len, uint64_t salt, uint32_t key) {
    return (salt * key) >> (64 - len);
}

LutError lut_insert(uint16_t* lut, uint8_t hash_len, uint32_t hash_salt, uint32_t key, uint16_t value) {
    uint16_t hash = lut_hash(hash_len, hash_salt, key);
    if(lut[hash]) {
        return LUT_INSERT_COLLISION;
    }
    lut[hash] = value;
    return LUT_SUCCESS;
}
