#pragma once
#include <cstdint>
#include <vector>

class DmlsGen {
public:
  enum Error {
    SUCCESS,
    INVALID_START_SEQUENCE,
  };

  struct Node {
    double explored_rate;
    uint32_t parent;
    uint32_t child[2];
  };

  void generate_mls(uint8_t n);

  Error generate_dmls(std::vector<bool> &sequence);

private:
  uint32_t inverse_bits(uint32_t v) const;
  uint32_t reverse_bits(uint32_t v) const;
  bool is_word_visited(uint32_t word) const;
  uint32_t select_new_node(uint32_t word, uint32_t node) const;

  std::vector<Node> _nodes;
  std::vector<uint32_t> _word_visit_table;
  uint16_t _visit_id = 0;
  uint8_t _word_len;
};
