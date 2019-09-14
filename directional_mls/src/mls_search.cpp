#include "mls_search.hpp"

MlsSearch::MlsSearch(uint8_t n)
  : _words(1 << n)
  , _banned_words(1<< n, false) {
    for(uint32_t i = 0; i < _words.size(); ++i) {
        _banned_words[i] = i > reverse_bits(i, n);
    }
}

uint32_t MlsSearch::reverse_bits(uint32_t v, uint8_t n) {
    unsigned int r = v & 1;
    int s = n - 1;
    for (v >>= 1; v; v >>= 1) {   
        r <<= 1;
        r |= v & 1;
        s--;
    }
    return r << s;

}
