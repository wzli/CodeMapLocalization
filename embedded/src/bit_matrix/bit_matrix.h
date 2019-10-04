#pragma once
#include <stdint.h>

typedef uint32_t BitMatrix[32];
void extract_codes(uint32_t* row_code, uint32_t* col_code, BitMatrix matrix,
        BitMatrix mask);
