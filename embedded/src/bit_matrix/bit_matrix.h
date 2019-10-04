#pragma once
#include <stdint.h>

typedef uint32_t BitMatrix32[32];
void extract_codes(uint32_t* row_code, uint32_t* col_code, BitMatrix32 matrix,
        BitMatrix32 mask);
