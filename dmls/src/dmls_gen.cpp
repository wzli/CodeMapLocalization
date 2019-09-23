#include "dmls_gen.hpp"
#include <algorithm>

#include <bitset>
#include <iostream>

uint32_t DmlsGen::inverse_bits(uint32_t v, uint8_t n) {
  return ~(v | (0xFFFFFFFF << n));
}

uint32_t DmlsGen::reverse_bits(uint32_t v, uint8_t n) {
  v &= ~(0xFFFFFFFF << n);
  unsigned int r = v & 1;
  for (v >>= 1; v; v >>= 1) {
    r <<= 1;
    r |= v & 1;
    n--;
  }
  return r << (n - 1);
}

void DmlsGen::generate_mls(uint8_t n) {
  std::vector<bool> start_string{0,1,0};
  generate_dmls(start_string, n);
}

void DmlsGen::generate_dmls(std::vector<bool> &sequence, uint8_t n) {
  thread_local std::vector<uint32_t> word_visit_table;
  thread_local uint16_t visit_id = 0;
  const uint32_t L_BIT = 1 << (n - 1);
  word_visit_table.resize(1 << n, visit_id);
  ++visit_id;
  // build start word
  uint32_t word = 0;
  for (bool bit : sequence) {
    word = (word >> 1) | (bit << (n - 1));
  }
  if (sequence.size() < n) {
    word >>= sequence.size() - n;
  }
#if 0
  word_visit_table[start_word] = visit_id;
  auto is_word_visited = [&](uint32_t word) {
    return word_visit_table[word] == visit_id ||
           word_visit_table[inverse_bits(word, n)] == visit_id ||
           word_visit_table[reverse_bits(word, n)] == visit_id;
  };
  auto extend_sequence = [&](uint32_t word, bool rev) {
    while (true) {
      word = rev ? (word & ~L_BIT) << 1 : word >> 1;
      uint32_t other_word = word | (rev ? 1 : L_BIT);
      bool word_visited = is_word_visited(word);
      bool other_word_visited = is_word_visited(other_word);
      if (other_word_visited) {
        if (word_visited) {
          break;
        }
      } else if (word_visited ||
                 _word_visit_counts[other_word] < _word_visit_counts[word]) {
        word = other_word;
      }
      sequence.emplace_back(word & (rev ? 1 : L_BIT));
      word_visit_table[word] = visit_id;
    }
  };
  for (uint32_t bit_mask = 1; bit_mask <= L_BIT; bit_mask <<= 1) {
    sequence.emplace_back(bit_mask & start_word);
  }
  extend_sequence(start_word, false);
  std::reverse(sequence.begin(), sequence.end());
  extend_sequence(start_word, true);
#endif
}
