#pragma once
#include <cstdint>
#include <vector>

class DmlsGen {
public:
  struct Node {
    double explored_rate;
    uint32_t parent;
    uint32_t child[2];
  };

  void generate_mls(uint8_t n);

  void generate_dmls(std::vector<bool> &sequence, uint8_t n);

private:
  static uint32_t inverse_bits(uint32_t v, uint8_t n);
  static uint32_t reverse_bits(uint32_t v, uint8_t n);

  std::vector<Node> _nodes;
};
