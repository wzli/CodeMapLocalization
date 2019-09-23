#include "dmls_gen.hpp"
#include <algorithm>
#include <bitset>
#include <cassert>
#include <iostream>

uint32_t DmlsGen::inverse_bits(uint32_t v) const {
  return ~(v | (0xFFFFFFFF << _word_len));
}

uint32_t DmlsGen::reverse_bits(uint32_t v) const {
  uint8_t n = _word_len;
  v &= ~(0xFFFFFFFF << n);
  unsigned int r = v & 1;
  for (v >>= 1; v; v >>= 1) {
    r <<= 1;
    r |= v & 1;
    n--;
  }
  return r << (n - 1);
}

bool DmlsGen::is_word_visited(uint32_t word) const {
  return _word_visit_table[word] == _visit_id ||
         _word_visit_table[inverse_bits(word)] == _visit_id ||
         _word_visit_table[reverse_bits(word)] == _visit_id;
};

uint32_t DmlsGen::select_new_node(uint32_t word, uint32_t node) const {
    if(_nodes.child[0]) {
      if(_nodes.child[1]) {
      }
    } else {
      if(_nodes.child[1]) {
      } else {
        assert();
      }
    }
    return 0;
}

void DmlsGen::generate_mls(uint8_t n) {
  _word_len = n;
  std::vector<bool> start_string{0,1,0,1,0,1};
  std::cout << "error code " << (int)generate_dmls(start_string) << std::endl;
}

DmlsGen::Error DmlsGen::generate_dmls(std::vector<bool> &sequence) {
  const uint32_t L_BIT = 1 << (_word_len - 1);
  // initalize word visit table
  _word_visit_table.resize(1 << _word_len, _visit_id);
  ++_visit_id;
  // build start word
  uint32_t word = 0;
  for (uint32_t i = 0; i < sequence.size(); ++i) {
    word = (word >> 1) | (sequence[i] << (_word_len - 1));
    // insert words parsed in the start sequence
    if(i >= _word_len) {
      if(is_word_visited(word)) {
        return INVALID_START_SEQUENCE;
      }
      _word_visit_table[word] = _visit_id;
    }
  }
  if (sequence.size() < _word_len) {
    word >>= _word_len - sequence.size();
  }
  // create root node
  _nodes.clear();
  _nodes.emplace_back();
  uint32_t node = 0;
  // select the least explored node

#if 0
  while (true) {
    word = word >> 1;
    _nodes[node].child[0] = _nodes.size();
    _nodes.emplace_back();
    uint32_t other_word = word | L_BIT;
    _nodes[node].child[1] = _nodes.size();
    _nodes.emplace_back();
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
    _word_visit_table[word] = _visit_id;
  }

  _word_visit_table[start_word] = _visit_id;
  auto extend_sequence = [&](uint32_t word, bool rev) {
    while (true) {
      word = rev ? (word & ~L_BIT) << 1 : word >> 1;
      uint32_t other_word = word | (rev ? 1 : L_BIT);
      bool word_visited = is_word_visited(word, n);
      bool other_word_visited = is_word_visited(other_word, n);
      if (other_word_visited) {
        if (word_visited) {
          break;
        }
      } else if (word_visited ||
                 _word_visit_counts[other_word] < _word_visit_counts[word]) {
        word = other_word;
      }
      sequence.emplace_back(word & (rev ? 1 : L_BIT));
      _word_visit_table[word] = _visit_id;
    }
  };
  for (uint32_t bit_mask = 1; bit_mask <= L_BIT; bit_mask <<= 1) {
    sequence.emplace_back(bit_mask & start_word);
  }
  extend_sequence(start_word, false);
  std::reverse(sequence.begin(), sequence.end());
  extend_sequence(start_word, true);
#endif
  return SUCCESS;
}
