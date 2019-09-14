#pragma once
#include <cstdint>
#include <vector>

class MlsSearch {
  public:
    void generate_new_sequence(std::vector<bool>& sequence, uint32_t start_word, uint8_t n);
    void generate_mls(const uint8_t n);

    static uint32_t reverse_bits(uint32_t v, uint8_t n);

  private:

    struct Word {
        uint32_t longest_sequence_length = 0;
        uint32_t visit_count = 0;
    };

    std::vector<Word> _words;
    std::vector<uint32_t> _least_visited_words;
};
