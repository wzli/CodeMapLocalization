#include "dmls_gen.hpp"
#include <algorithm>

#include <iostream>
#include <bitset>

uint32_t MlsSearch::inverse_bits(uint32_t v, uint8_t n) {
    return ~(v | (0xFFFFFFFF << n));
}

uint32_t MlsSearch::reverse_bits(uint32_t v, uint8_t n) {
    v &= ~(0xFFFFFFFF << n);
    unsigned int r = v & 1;
    for (v >>= 1; v; v >>= 1) {   
        r <<= 1;
        r |= v & 1;
        n--;
    }
    return r << (n - 1);
}

void MlsSearch::generate_mls(uint8_t n) {
    _word_visit_counts.clear();
    _least_visited_words.clear();
    _word_visit_counts.resize(1 << n);
    _least_visited_words.reserve(_word_visit_counts.size());
    for(uint32_t i = 0; i < _word_visit_counts.size(); ++i) {
        _least_visited_words.emplace_back(i);
    }
    std::vector<bool> sequence, max_length_sequence;
    std::vector<uint32_t> visited_words;
    for(int i = 0; i < 2000; ++i) {
        std::partial_sort(_least_visited_words.begin(), _least_visited_words.begin() + 1,  _least_visited_words.end(),
            [this] (uint32_t a, uint32_t b) {return _word_visit_counts[a] < _word_visit_counts[b];}
        );
        generate_new_sequence(sequence, visited_words, _least_visited_words.front(), n);
        for(uint32_t word : visited_words) {
            ++_word_visit_counts[word];
        }
        if(sequence.size() > max_length_sequence.size()) {
            max_length_sequence = sequence;
        }
    }
    for(bool bit : max_length_sequence) {
        std::cout << (int16_t)bit;
    }
    uint32_t optimal_length = 0;
    for(uint32_t i = 0; i < _word_visit_counts.size(); ++i) {
        optimal_length += (i <= reverse_bits(i, n));
    }
    std::cout << " size " << max_length_sequence.size() << "/" << optimal_length << " n " << (int16_t)n << std::endl;
}

void MlsSearch::generate_new_sequence(std::vector<bool>& sequence, std::vector<uint32_t>& visited_words, uint32_t start_word, uint8_t n) {
    thread_local std::vector<uint32_t> last_visit_id;
    thread_local uint16_t visit_id = 0;
    const uint32_t L_BIT = 1 << (n -1);
    last_visit_id.resize(_word_visit_counts.size(), visit_id);
    ++visit_id;
    sequence.clear();
    visited_words.clear();
    last_visit_id[start_word] = visit_id;
    visited_words.emplace_back(start_word);
    auto is_word_visited = [&] (uint32_t word) {
        return last_visit_id[word] == visit_id
                    || last_visit_id[inverse_bits(word, n)] == visit_id
                    || last_visit_id[reverse_bits(word, n)] == visit_id;
    };
    auto extend_sequence = [&] (uint32_t word, bool rev) {
        while(true) {
            word = rev ? (word & ~L_BIT) << 1 : word >> 1;
            uint32_t other_word = word | (rev ? 1 : L_BIT);
            bool word_visited = is_word_visited(word);
            bool other_word_visited = is_word_visited(other_word);
            if(other_word_visited) {
                if(word_visited) {
                    break;
                } 
            } else if(word_visited || _word_visit_counts[other_word] < _word_visit_counts[word]) {
                word = other_word;
            }
            sequence.emplace_back(word & (rev ? 1 : L_BIT));
            last_visit_id[word] = visit_id;
            visited_words.emplace_back(word);
        }
    };
    for(uint32_t bit_mask = 1; bit_mask <= L_BIT ; bit_mask <<= 1) {
        sequence.emplace_back(bit_mask & start_word);
    }
    extend_sequence(start_word, false);
    std::reverse(sequence.begin(), sequence.end());
    extend_sequence(start_word, true);
}
