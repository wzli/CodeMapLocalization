#pragma once
#include <cstdint>
#include <vector>

class DmlsGen {
public:
  void generate_new_sequence(std::vector<bool> &sequence,
                             std::vector<uint32_t> &visited_words,
                             uint32_t start_word, uint8_t n);
  void generate_mls(const uint8_t n);

  static uint32_t reverse_bits(uint32_t v, uint8_t n);
  static uint32_t inverse_bits(uint32_t v, uint8_t n);

private:
  std::vector<uint32_t> _word_visit_counts;
  std::vector<uint32_t> _least_visited_words;
};
