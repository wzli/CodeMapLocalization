#pragma once
#include <stdint.h>

#define LUT_KEY_ERROR (0xFFFF)

typedef struct {
  uint32_t key : 18;
  uint16_t value : 14;
} LutEntry;

extern const uint8_t LUT_KEY_LENGTH;
extern const uint32_t LUT_SIZE;
extern const uint64_t LUT_UNIQUE_ID;
extern const LutEntry *LUT_DATA;

void lut_sort(LutEntry *lut, uint16_t len);
uint16_t lut_search(const LutEntry *lut, uint16_t len, uint32_t key);