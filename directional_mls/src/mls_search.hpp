#pragma once
#include <cstdint>
#include <vector>

class MlsSearch {
  public:
    MlsSearch(uint8_t n);

    static uint32_t reverse_bits(uint32_t v, uint8_t n);


  private:

    struct Word {
        uint32_t longest_sequence_length = 0;
        uint32_t visit_count = 0;
    };

    std::vector<Word> _words;
    std::vector<bool> _banned_words;
};
