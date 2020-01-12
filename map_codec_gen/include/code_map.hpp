#pragma once
#include <vector>
#include <fstream>

namespace CodeMap {

template <class SequenceType>
void save_pbm(const std::string& file_name, const SequenceType& sequence) {
    std::ofstream f(file_name, std::ios::binary);
    f << "P4\n#0x" << std::hex << std::hash<std::string>{}(sequence) << '\n'
      << std::dec << sequence.size() << ' ' << sequence.size() << '\n';
    std::vector<char> linebits((sequence.size() + 7) / 8);
    for (size_t y = 0; y < sequence.size(); ++y) {
        std::fill(linebits.begin(), linebits.end(), 0);
        for (size_t x = 0; x < sequence.size(); ++x) {
            const int byte_pos = x / 8;
            const int bit_pos = (8 - 1) - (x % 8);
            linebits[byte_pos] |= ((sequence[x] == sequence[y]) << bit_pos);
        }
        f.write(&linebits[0], linebits.size());
    }
}

}  // namespace CodeMap
