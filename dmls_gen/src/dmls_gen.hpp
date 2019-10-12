#pragma once
#include <cstdint>
#include <functional>
#include <random>
#include <vector>

class DmlsGen {
public:
    void generate_dmls(std::vector<bool>& sequence, uint8_t word_length, uint32_t iterations,
            std::function<void(std::vector<bool>& sequence, uint8_t word_length)>
                    new_record_callback = nullptr);

private:
    static constexpr uint32_t PALINDROME = ~0;

    bool is_word_visited(uint32_t word) const;
    void set_word_visited(uint32_t word);

    std::vector<uint32_t> _word_visit_counts, _word_visit_ids;
    std::vector<bool> _l_sequence, _r_sequence;
    std::random_device _rd;
    std::mt19937 _gen;
    std::discrete_distribution<uint16_t> _next_word_selector;
    std::uniform_int_distribution<uint32_t> _start_word_selector;
    uint32_t _visit_id = 0;
    uint8_t _word_length;
};
