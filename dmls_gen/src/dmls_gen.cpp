#include "dmls_gen.hpp"
#include "roulette.hpp"
extern "C" {
#include "bitwise_utils.h"
}

bool DmlsGen::is_word_visited(uint32_t word) const {
    return _word_visit_ids[word] == _visit_id || _word_visit_ids[word] == PALINDROME;
}

void DmlsGen::set_word_visited(uint32_t word) {
    ++_word_visit_counts[word];
    uint32_t rword = reverse_bits(word, _word_length);
    _word_visit_ids[word] = _visit_id;
    _word_visit_ids[rword] = _visit_id;
    _word_visit_ids[invert_bits(word, _word_length)] = _visit_id;
    _word_visit_ids[invert_bits(rword, _word_length)] = _visit_id;
}

void DmlsGen::generate_dmls(std::vector<bool>& sequence, uint8_t word_length, uint64_t iterations,
        std::function<void(std::vector<bool>& sequence, uint8_t word_length)> new_record_callback) {
    if (word_length > 32) {
        return;
    }
    _word_length = word_length;
    _word_visit_counts.clear();
    _word_visit_counts.resize(1 << _word_length);
    _word_visit_ids.resize(1 << _word_length, _visit_id);
    for (uint32_t word = 0; word < (1u << _word_length); ++word) {
        uint32_t rword = reverse_bits(word, _word_length);
        uint32_t irword = invert_bits(rword, _word_length);
        if (word == rword || word == irword) {
            _word_visit_ids[word] = PALINDROME;
        }
    }
    const uint32_t MSB = 1 << (_word_length - 1);
    std::vector<float> next_word_weights(4);
    while (--iterations) {
        _gen.seed((static_cast<uint64_t>(_rd()) << 32) | (_rd() & 0xFFFFFFFF));
        uint32_t l_word = _gen() & mask_bits(_word_length);
        while (l_word == reverse_bits(l_word, _word_length)) {
            l_word = _gen() & mask_bits(_word_length);
        }
        uint32_t r_word = l_word;
        _l_sequence.clear();
        _r_sequence.clear();
        for (uint32_t i = 0, buf = l_word; i < _word_length; ++i, buf >>= 1) {
            _l_sequence.emplace_back(buf & 1);
        }
        ++_visit_id;
        _visit_id += _visit_id == PALINDROME;
        set_word_visited(l_word);
        while (true) {
            uint32_t next_words[4] = {
                    l_word >> 1,
                    (l_word >> 1) | MSB,
                    (r_word & ~MSB) << 1,
                    ((r_word & ~MSB) << 1) | 1,
            };
            float weight_sum = 0.0f;
            for (uint8_t i = 0; i < 4; ++i) {
                next_word_weights[i] = is_word_visited(next_words[i])
                                               ? 0.0f
                                               : 1.0f / (_word_visit_counts[next_words[i]] + 1);
                weight_sum += next_word_weights[i];
            }
            if (weight_sum == 0.0f) {
                break;
            }
            uint8_t next_word_index = roulette(next_word_weights, _gen());
            if (next_word_index & 2) {
                r_word = next_words[next_word_index];
                _r_sequence.emplace_back(next_word_index & 1);
            } else {
                l_word = next_words[next_word_index];
                _l_sequence.emplace_back(next_word_index & 1);
            }
            set_word_visited(next_words[next_word_index]);
        }
        if (_l_sequence.size() + _r_sequence.size() > sequence.size()) {
            sequence.clear();
            sequence.insert(sequence.end(), _l_sequence.rbegin(), _l_sequence.rend());
            sequence.insert(sequence.end(), _r_sequence.begin(), _r_sequence.end());
            if (new_record_callback) {
                new_record_callback(sequence, _word_length);
            }
        }
    }
}
