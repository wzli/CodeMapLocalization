#include "mls_search.hpp"
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
    _words.clear();
    _least_visited_words.clear();
    _words.resize(1 << n);
    _least_visited_words.reserve(_words.size());
    for(uint32_t i = 0; i < _words.size(); ++i) {
        _least_visited_words.emplace_back(i);
    }
    std::vector<bool> sequence, max_length_sequence;
    for(int i = 0; i < 1000; ++i) {
        std::partial_sort(_least_visited_words.begin(), _least_visited_words.begin() + 1,  _least_visited_words.end(),
            [this] (uint32_t a, uint32_t b) {return _words[a].visit_count < _words[b].visit_count;}
        );
        generate_new_sequence(sequence, _least_visited_words.front(), n);
        if(sequence.size() > max_length_sequence.size()) {
            max_length_sequence = sequence;
        }
    }
    for(bool bit : max_length_sequence) {
        std::cout << (int16_t)bit;
    }
    uint32_t optimal_length = 0;
    for(uint32_t i = 0; i < _words.size(); ++i) {
        optimal_length += (i <= reverse_bits(i, n));
    }
    std::cout << " size " << max_length_sequence.size() << "/" << optimal_length << " n " << (int16_t)n << std::endl;
}

void MlsSearch::generate_new_sequence(std::vector<bool>& sequence, uint32_t start_word, uint8_t n) {
    thread_local std::vector<uint32_t> last_visit_id;
    thread_local uint16_t visit_id = 0;
    const uint32_t L_BIT = 1 << (n -1);
    last_visit_id.resize(_words.size(), visit_id);
    ++visit_id;
    sequence.clear();
    for(uint32_t bit_mask = 1; bit_mask <= L_BIT ; bit_mask <<= 1) {
        sequence.emplace_back(bit_mask & start_word);
    }
    last_visit_id[start_word] = visit_id;
    ++_words[start_word].visit_count;
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
            } else if(word_visited || _words[other_word].visit_count < _words[word].visit_count) {
                word = other_word;
            }
            sequence.emplace_back(word & (rev ? 1 : L_BIT));
            last_visit_id[word] = visit_id;
            ++_words[word].visit_count;
        }
    };
    extend_sequence(start_word, false);
    std::reverse(sequence.begin(), sequence.end());
    extend_sequence(start_word, true);
    start_word = 0;
    for(auto bit = sequence.begin(); bit != sequence.begin() + n - 1; ++bit) {
        start_word = (start_word << 1) ^ *bit;
    }
    for(auto bit = sequence.begin() + n - 1; bit != sequence.end(); ++bit) {
        start_word = ((start_word & ~L_BIT) << 1) ^ *bit;
        if(_words[start_word].longest_sequence_length < sequence.size()) {
            _words[start_word].longest_sequence_length = sequence.size();
        }
    }

    //for(bool bit : sequence) {
    //    std::cout << (int16_t)bit;
    //}
    //std::cout << " size " << sequence.size() << " n " << (int16_t)n << std::endl;
}
