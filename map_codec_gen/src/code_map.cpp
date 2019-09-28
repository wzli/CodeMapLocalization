#include "code_map.hpp"
#include <fstream>
#include <iostream>

void CodeMap::generate(const std::string& sequence) {
    dmls.clear();
    dmls.reserve(sequence.size());
    for (char c : sequence) {
        dmls.emplace_back(c - '0');
    }
    data.clear();
    data.reserve(dmls.size() * dmls.size());
    for (size_t y = 0; y < dmls.size(); ++y) {
        for (size_t x = 0; x < dmls.size(); ++x) {
            data.emplace_back(dmls[x] ^ dmls[y]);
        }
    }
    sequence_hash = std::hash<std::string>{}(sequence);
};

void CodeMap::save_pbm(const std::string& file_name) {
    std::ofstream f(file_name, std::ios::binary);
    f << "P4\n#0x" << std::hex << sequence_hash << '\n' << std::dec << dmls.size() << ' ' << dmls.size() << '\n';
    std::vector<char> linebits((dmls.size() + 7) / 8);
    for (size_t y = 0; y < dmls.size(); ++y) {
        std::fill(linebits.begin(), linebits.end(), 0);
        for (size_t x = 0; x < dmls.size(); ++x) {
            const int byte_pos = x / 8;
            const int bit_pos = (8 - 1) - (x % 8);
            linebits[byte_pos] |= (data[y * dmls.size() + x] << bit_pos);
        }
        f.write(&linebits[0], linebits.size());
    }
}
