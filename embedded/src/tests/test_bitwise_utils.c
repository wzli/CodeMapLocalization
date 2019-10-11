#include "tests.h"
#include "bitwise_utils.h"

int test_bitwise_utils() {
    test_assert(inverse_bits(0, 2) ==  3);    
    test_assert(inverse_bits(0, 32) ==  ~0);    
    test_assert(reverse_bits(1, 3) ==  (1 << 2));    
    test_assert(reverse_bits(1, 32) ==  (1 << 31));    
    test_assert(sum_bits(0) ==  0);    
    test_assert(sum_bits(1) ==  1);    
    test_assert(sum_bits(~0) ==  32);    
    test_assert(first_set_bit(0) == 32);
    test_assert(first_set_bit(~0) == 0);
    test_assert(first_set_bit(1 << 7) == 7);
    return 0;
}
