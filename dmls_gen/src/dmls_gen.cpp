#include "dmls_gen.hpp"

uint32_t DmlsGen::inverse_bits(uint32_t v) const {
    return ~(v | (0xFFFFFFFF << _word_length));
}

uint32_t DmlsGen::reverse_bits(uint32_t v) const {
    uint8_t n = _word_length;
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
    return _word_visit_ids[word] == _visit_id;
}

void DmlsGen::set_word_visited(uint32_t word) {
    ++_word_visit_counts[word];
    uint32_t rword = reverse_bits(word);
    _word_visit_ids[word] = _visit_id;
    _word_visit_ids[rword] = _visit_id;
    _word_visit_ids[inverse_bits(word)] = _visit_id;
    _word_visit_ids[inverse_bits(rword)] = _visit_id;
}

void DmlsGen::generate_dmls(std::vector<bool>& sequence, uint8_t word_length, uint32_t iterations,
        std::function<void(std::vector<bool>& sequence, uint8_t word_length)> new_record_callback) {
    if (word_length > 32) {
        return;
    }
    _word_length = word_length;
    _word_visit_counts.clear();
    _word_visit_counts.resize(1 << _word_length);
    _word_visit_ids.resize(1 << _word_length, _visit_id);
    _start_word_selector.param(std::uniform_int_distribution<uint32_t>::param_type(0, (1 << _word_length) - 1));
    const uint32_t MSB = 1 << (_word_length - 1);
    std::vector<double> next_word_weights(4);
    while (--iterations) {
        _gen.seed(_rd());
        uint32_t l_word = _start_word_selector(_gen);
        uint32_t r_word = l_word;
        _l_sequence.clear();
        _r_sequence.clear();
        for (uint32_t i = 0, buf = l_word; i < _word_length; ++i, buf >>= 1) {
            _l_sequence.emplace_back(buf & 1);
        }
        ++_visit_id;
        set_word_visited(l_word);
        while (true) {
            uint32_t next_words[4] = {
                    l_word >> 1,
                    (l_word >> 1) | MSB,
                    (r_word & ~MSB) << 1,
                    ((r_word & ~MSB) << 1) | 1,
            };
            double weight_sum = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                next_word_weights[i] =
                        is_word_visited(next_words[i]) ? 0.0 : 1.0 / (_word_visit_counts[next_words[i]] + 1);
                weight_sum += next_word_weights[i];
            }
            if (weight_sum == 0.0) {
                break;
            }
            _next_word_selector.param(std::discrete_distribution<uint16_t>::param_type(
                    next_word_weights.begin(), next_word_weights.end()));
            int next_word_index = _next_word_selector(_gen);
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
