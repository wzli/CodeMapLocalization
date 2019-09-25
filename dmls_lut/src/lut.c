#include "lut.h"
#include <stdlib.h>

static int comp (const void * elem1, const void * elem2) 
{
    uint32_t key1 = ((LutEntry*)elem1)->key;
    uint32_t key2 = ((LutEntry*)elem2)->key;
    return key1 == key2 ? 0 : key1 > key2 ? 1 : -1;
}

void lut_sort(LutEntry* lut, uint16_t len) {
    qsort(lut, len, sizeof(LutEntry), comp);
}

uint16_t lut_search(const LutEntry* lut, uint16_t len, uint32_t key) {
    const void* result = bsearch(&key, lut, len, sizeof(LutEntry), comp);
    return result ? ((LutEntry*)result)->value : LUT_KEY_ERROR;
}
