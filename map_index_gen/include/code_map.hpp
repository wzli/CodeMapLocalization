#pragma once
#include <vector>
#include <fstream>

namespace CodeMap {

template <class SequenceType>
void save_pbm(const std::string& file_name, const SequenceType& x_sequence,
        const SequenceType& y_sequence) {
    std::ofstream f(file_name, std::ios::binary);
    f << "P4\n#0x" << std::hex << std::hash<std::string>{}(x_sequence) << ' ' << std::hex
      << std::hash<std::string>{}(y_sequence) << '\n'
      << std::dec << x_sequence.size() << ' ' << y_sequence.size() << '\n';
    std::vector<char> linebits((x_sequence.size() + 7) / 8);
    std::vector<char> linebits_inv(linebits.size());
    std::fill(linebits.begin(), linebits.end(), 0);
    std::fill(linebits_inv.begin(), linebits_inv.end(), 0);
    for (size_t x = 0; x < x_sequence.size(); ++x) {
        const int byte_pos = x / 8;
        const int bit_pos = (8 - 1) - (x % 8);
        linebits[byte_pos] |= (x_sequence[x] == '1') << bit_pos;
        linebits_inv[byte_pos] |= (x_sequence[x] == '0') << bit_pos;
    }
    for (char y_symbol : y_sequence) {
        f.write(y_symbol == '1' ? linebits.data() : linebits_inv.data(), linebits.size());
    }
}

}  // namespace CodeMap
