#pragma once
#include <cstdint>
#include <vector>

class DmlsGen {
public:
  struct Node {
    enum Index {
      UNINITIALIZED = 0,
      NONEXISTANT = 0xFFFFFFFF,
    };
    double explored_rate;
    uint32_t parent;
    uint32_t child[2];
  };

  double generate_dmls(std::vector<bool> &sequence, uint8_t word_length,
                       uint32_t iterations);

private:
  uint32_t inverse_bits(uint32_t v) const;
  uint32_t reverse_bits(uint32_t v) const;
  bool is_word_visited(uint32_t word) const;
  uint32_t add_node(uint32_t parent);
  bool parse_start_word(uint32_t &start_word,
                        const std::vector<bool> &sequence);
  void propagate_explored_rates(uint32_t node);

  std::vector<Node> _nodes;
  std::vector<uint32_t> _deleted_nodes;
  std::vector<uint32_t> _word_visit_table;
  uint16_t _visit_id = 0;
  uint16_t _initial_visit_id;
  uint8_t _word_len;
};
