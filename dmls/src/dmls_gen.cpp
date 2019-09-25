#include "dmls_gen.hpp"
#include <algorithm>

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
  uint32_t rword = reverse_bits(word);
  uint32_t last_visit_ids[] = {_word_visit_ids[word],
                               _word_visit_ids[rword],
                               _word_visit_ids[inverse_bits(word)],
                               _word_visit_ids[inverse_bits(rword)]};
  for (uint32_t last_visit_id : last_visit_ids) {
    if (last_visit_id == _visit_id || last_visit_id == _initial_visit_id) {
      return true;
    }
  }
  return false;
}

uint32_t DmlsGen::add_node(uint32_t parent) {
  uint32_t node;
  if (_deleted_nodes.empty()) {
    node = _nodes.size();
    _nodes.emplace_back();
  } else {
    node = _deleted_nodes.back();
    _deleted_nodes.pop_back();
    _nodes[node] = {};
  }
  _nodes[node].parent = parent;
  return node;
}

bool DmlsGen::parse_start_word(uint32_t &start_word,
                               const std::vector<bool> &sequence) {
  start_word = 0;
  for (uint32_t i = 0; i < sequence.size(); ++i) {
    start_word = (start_word >> 1) | (sequence[i] << (_word_len - 1));
    // insert words parsed in the start sequence
    if (i + 1 >= _word_len) {
      if (is_word_visited(start_word)) {
        return false;
      }
      _word_visit_ids[start_word] = _visit_id;
    }
  }
  // shift first bit of sequence to LSB
  if (sequence.size() < _word_len) {
    start_word >>= _word_len - sequence.size();
  }
  return true;
}

void DmlsGen::propagate_explored_rates(uint32_t node) {
  for (double explored_rate_increment = 1.0;
       node != Node::Index::NONEXISTANT && explored_rate_increment > 0;
       node = _nodes[node].parent) {
    explored_rate_increment *= 0.5;
    _nodes[node].explored_rate += explored_rate_increment;
    // delete child nodes if the parent is fully explorered
    if (_nodes[node].explored_rate >= 1.0) {
      for (uint32_t &child : _nodes[node].child) {
        if (child != Node::Index::NONEXISTANT) {
          _deleted_nodes.emplace_back(child);
          child = Node::Index::NONEXISTANT;
        }
      }
    }
  }
}

double DmlsGen::generate_dmls(std::vector<bool> &sequence, uint8_t word_length,
                              uint32_t iterations, uint32_t size_limit) {
  _word_len = word_length;
  // initalize word visit table
  _word_visit_counts.clear();
  _word_visit_counts.resize(1 << _word_len);
  _word_visit_ids.resize(1 << _word_len, _visit_id);
  ++_visit_id;
  _initial_visit_id = _visit_id;
  // parse start word
  uint32_t start_word;
  if (!parse_start_word(start_word, sequence)) {
    return 0.0;
  }
  // create root node
  _deleted_nodes.clear();
  _nodes.clear();
  add_node(Node::Index::NONEXISTANT);
  // keep track of best sequence found
  const uint32_t start_sequence_len = sequence.size();
  std::vector<bool> max_length_sequence = sequence;
  // run search
  while (_nodes[0].explored_rate < 1.0 && --iterations && _nodes.size() <= size_limit) {
    uint32_t node = 0;
    uint32_t word = start_word;
    uint32_t next_word[2];
    double next_word_explored_rate[2] = {0.0, 0.0};
    sequence.resize(start_sequence_len);
    ++_visit_id;
    while (next_word_explored_rate[0] < 1.0 ||
           next_word_explored_rate[1] < 1.0) {
      if (sequence.size() < _word_len) {
        next_word[0] = word;
        next_word[1] = word | (1 << (sequence.size()));
      } else {
        next_word[0] = word >> 1;
        next_word[1] = next_word[0] | (1 << (_word_len - 1));
      }
      // do for both 0 and 1 extentions
      for (uint8_t i = 0; i < 2; ++i) {
        switch (_nodes[node].child[i]) {
        // previously explored
        case Node::Index::NONEXISTANT:
          next_word_explored_rate[i] = 1.0;
          break;
        // initialize child node if not already
        case Node::Index::UNINITIALIZED:
          // check if word is traversable
          if (sequence.size() >= _word_len && is_word_visited(next_word[i])) {
            // node is not traversable, mark as explored and bubble up
            _nodes[node].child[i] = Node::Index::NONEXISTANT;
            propagate_explored_rates(node);
          } else {
            // node is traversable, add new child node
            _nodes[node].child[i] = add_node(node);
          }
        default:
          next_word_explored_rate[i] =
              _nodes[node].child[i] == Node::Index::NONEXISTANT
                  ? 1.0
                  : _nodes[_nodes[node].child[i]].explored_rate;
        }
      }
      uint8_t next_bit =
          next_word_explored_rate[0] == next_word_explored_rate[1]
              ? _word_visit_counts[next_word[0]] >
                    _word_visit_counts[next_word[1]]
              : next_word_explored_rate[0] > next_word_explored_rate[1];
      word = next_word[next_bit];
      node = _nodes[node].child[next_bit];
      sequence.emplace_back(next_bit);
      if (sequence.size() >= _word_len) {
        ++_word_visit_counts[word];
        _word_visit_ids[word] = _visit_id;
      }
    }
    sequence.pop_back();
    if (sequence.size() > max_length_sequence.size()) {
      max_length_sequence = sequence;
    }
  }
  sequence = std::move(max_length_sequence);
  return _nodes[0].explored_rate;
}
