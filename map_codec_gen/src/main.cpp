extern "C" {
#include "mls_query/mls_query.h"
}
#include "code_map.hpp"
#include <cstdlib>
#include <bitset>
#include <fstream>
#include <iostream>

uint32_t inverse_bits(uint32_t v, uint32_t n) {
    return ~(v | (0xFFFFFFFF << n));
}

uint32_t reverse_bits(uint32_t v, uint32_t n) {
    v &= ~(0xFFFFFFFF << n);
    unsigned int r = v & 1;
    for (v >>= 1; v; v >>= 1) {
        r <<= 1;
        r |= v & 1;
        n--;
    }
    return r << (n - 1);
}

int main(int argc, char** argv) {
    std::string s;
    std::ifstream file(argv[1]);

    if (!file.is_open()) {
        std::cout << "Unable to open the dmls yaml file" << std::endl;
        return -1;
    }

    std::getline(file, s, ' ');
    std::getline(file, s);
    int word_length = atoi(s.c_str());
    std::cout << "parsed word_length: " << word_length << std::endl;

    std::getline(file, s, ' ');
    std::getline(file, s);
    size_t sequence_length = std::stoul(s);
    std::cout << "parsed sequence_length: " << sequence_length << std::endl;

    std::getline(file, s, ' ');
    std::getline(file, s);
    size_t sequence_hash = 0;
    try {
        sequence_hash = std::stoul(s, 0, 16);
        std::cout << "parsed sequence_hash: 0x" << std::hex << sequence_hash << std::endl;
    } catch (std::exception& e) {
        std::cout << e.what();
    }

    std::getline(file, s, '\'');
    std::getline(file, s, '\'');

    if (s.size() != sequence_length) {
        std::cout << "WARNING: sequence length doesn't match, actual is " << std::dec << s.size()
                  << std::endl;
    }
    if (std::hash<std::string>{}(s) != sequence_hash) {
        std::cout << "WARNING: sequence hash doesn't match, actual is 0x" << std::hex
                  << std::hash<std::string>{}(s) << std::endl;
    }
    std::cout << "Generating Map Codec ..." << std::endl;

    // below is word size specific (since the look up table optimized for
    // 16-bits)

    uint32_t* bit_array = new uint32_t[(s.size()/32) + 1];

    const size_t LUT_SIZE = s.size() - word_length + 1;
    uint16_t* lut = new uint16_t[LUT_SIZE];
    uint16_t* test_lut = new uint16_t[1 << word_length];
    for (int32_t i = 0; i < (1 << word_length); i++) {
        test_lut[i] = MLSQ_NOT_FOUND;
    }

    for (uint32_t i = 0, word = 0; i < s.size(); ++i) {
        word >>= 1;
        if (s[i] == '1') {
            word |= 1 << (word_length - 1);
            ba32_set_bit(bit_array, i);
        } else if (s[i] == '0') {
            ba32_clear_bit(bit_array, i);
        } else {
            std::cout << "Error: sequence contains invalid character " << s[i] << std::endl;
            return -2;
        }
        int position = i - word_length + 1;
        if (position >= 0) {
            uint32_t rword = reverse_bits(word, word_length);
            if (test_lut[word] != MLSQ_NOT_FOUND) {
                std::cout << "Error: sequence doesn't satisfy dmls constraints" << std::endl;
                return -3;
            }
            test_lut[word] = position;
            test_lut[rword] = position;
            test_lut[inverse_bits(word, word_length)] = position;
            test_lut[inverse_bits(rword, word_length)] = position;
            lut[position] = position;
        }
    };


    for (uint32_t i = 0, word = 0; i < s.size(); ++i) {
        word >>= 1;
        if (s[i] == '1') {
            if(!ba32_get_bit(bit_array, i)) {
                std::cout << "Error: bit_array_mismatch" << std::endl;
            }
            word |= 1 << (word_length - 1);
        }
        int position = i - word_length + 1;
        if(position >= 0) {
            uint32_t got_word = mlsq_position_to_code(bit_array, word_length, position);
            if(word != got_word) {
                std::cout << "Error: word fetch mismatch " << (int)i << ' ' << std::bitset<17>(word) << " " << std::bitset<17>(got_word) << std::endl;
            }
        }
    }

    MlsQueryIndex query_index = {bit_array, lut, s.size(), word_length};
    uint16_t len = mlsq_sort_code_positions(query_index);
    while(len--) {
        std::cout << query_index.sorted_code_positions[len] << std::endl;
    }
    for (uint32_t i = 0; i < (1 << word_length); ++i) {
        uint16_t position = mlsq_code_to_position(query_index, i);
        if (position != MLSQ_NOT_FOUND && position != test_lut[i]) {
            std::cout << "Internal Error: LUT search result doesn't match the "
                         "original i " << i << " p " << position << " t " << test_lut[i]
                      << std::endl;
            return -4;
        }
    }
    #if 0
    std::ofstream lut_file("lut_dat.c");
    if (!lut_file.is_open()) {
        std::cout << "Unable to write LUT file" << std::endl;
        return -5;
    }
    lut_file << "#include \"lut.h\"\n";
    lut_file << "const uint8_t LUT_KEY_LENGTH = " << word_length << ";\n";
    lut_file << "const uint32_t LUT_SIZE = " << LUT_SIZE << ";\n";
    lut_file << "const uint64_t LUT_UNIQUE_ID = 0x" << std::hex << std::hash<std::string>{}(s)
             << ";\n";
    lut_file << "const LutEntry __LUT_DATA[] = {\n";
    for (uint32_t i = 0; i < LUT_SIZE; ++i) {
        lut_file << "  { 0x" << std::hex << lut[i].key << ", " << std::dec << lut[i].value
                 << " },\n";
    }
    lut_file << "};\n";
    lut_file << "const LutEntry* LUT_DATA = __LUT_DATA;\n";
    lut_file.close();
    std::cout << "LookupTable file succesfully generated" << std::endl;
    #endif

    delete bit_array;
    delete lut;
    delete test_lut;

    //CodeMap code_map;
    //code_map.generate(s);
    //code_map.save_pbm("code_map.pbm");
    //std::cout << "CodeMap file succesfully generated" << std::endl;

    return 0;
}
