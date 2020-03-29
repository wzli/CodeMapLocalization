extern "C" {
#include "bitwise_utils.h"
#include "mls_query.h"
}
#include "code_map.hpp"
#include <memory>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <bitset>

enum Error {
    SUCCESS = 0,
    ERROR_INVALID_ARGUMENT,
    ERROR_OPENING_FILE,
    ERROR_WRITING_FILE,
    ERROR_INVALID_SEQUENCE_CHARACTER,
    ERROR_DMLS_CONSTRAINS_UNSATISFIED,
    INTERNAL_ERROR_SEQUENCE_VECTOR_MISMATCH,
    INTERNAL_ERROR_POSITION_LOOKUP_MISMATCH,
    INTERNAL_ERROR_SORTED_POSITIONS_LENGTH_MISMATCH,
    INTERNAL_ERROR_CODE_POSITION_LOOKUP_MISMATCH,
    INTERNAL_ERROR_MISSING_CODE_POSITION_ENTRY,
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "provide path dmls yaml file as argument" << std::endl;
        return ERROR_INVALID_ARGUMENT;
    }

    std::string s;
    std::ifstream file(argv[1]);

    if (!file.is_open()) {
        std::cout << "Unable to open the dmls yaml file" << std::endl;
        return ERROR_OPENING_FILE;
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

    const size_t positions_length = s.size() - word_length + 1;
    const uint16_t sequence_chunks = (s.size() / 32) + ((s.size() % 32) > 0);
    std::unique_ptr<uint32_t[]> sequence(new uint32_t[sequence_chunks]);
    std::unique_ptr<uint16_t[]> sorted_positions(new uint16_t[positions_length]);
    std::unique_ptr<uint16_t[]> test_lookup(new uint16_t[1 << word_length]);
    sequence[sequence_chunks - 1] = 0;

    for (int32_t i = 0; i < (1 << word_length); i++) {
        test_lookup[i] = MLSQ_NOT_FOUND;
    }

    for (uint32_t i = 0, word = 0; i < s.size(); ++i) {
        word >>= 1;
        if (s[i] == '1') {
            word |= 1 << (word_length - 1);
            bv32_set_bit(sequence.get(), i);
        } else if (s[i] == '0') {
            bv32_clear_bit(sequence.get(), i);
        } else {
            std::cout << "Error: sequence contains invalid character " << s[i] << std::endl;
            return ERROR_INVALID_SEQUENCE_CHARACTER;
        }
        int position = i - word_length + 1;
        if (position >= 0) {
            uint32_t rword = reverse_bits(word, word_length);
            if (word == rword || test_lookup[word] != MLSQ_NOT_FOUND) {
                std::cout << "Error: sequence doesn't satisfy dmls constraints" << std::endl;
                return ERROR_DMLS_CONSTRAINS_UNSATISFIED;
            }
            if (word == invert_bits(rword, word_length)) {
                std::cout << "Warning: word " << std::bitset<32>(word) << " at position "
                          << std::dec << position << " has reverse invert ambiguity" << std::endl;
            }
            test_lookup[word] = position;
            test_lookup[rword] = position;
            test_lookup[invert_bits(word, word_length)] = position;
            test_lookup[invert_bits(rword, word_length)] = position;
        }
    };

    for (uint32_t i = 0, word = 0; i < s.size(); ++i) {
        uint32_t new_bit = s[i] - '0';
        if (new_bit != bv32_get_bit(sequence.get(), i)) {
            std::cout << "Internal Error: sequence bit array doesn't match source string"
                      << std::endl;
            return INTERNAL_ERROR_SEQUENCE_VECTOR_MISMATCH;
        }
        word >>= 1;
        word |= new_bit << (word_length - 1);
        int position = i - word_length + 1;
        if (position >= 0) {
            if (word != bv32_get_slice(sequence.get(), position, word_length)) {
                std::cout << "Internal Error: position lookup doesn't match expected word"
                          << std::endl;
                return INTERNAL_ERROR_POSITION_LOOKUP_MISMATCH;
            }
        }
    }

    MlsIndex query_index = {sequence.get(), sorted_positions.get(), static_cast<uint16_t>(s.size()),
            static_cast<uint8_t>(word_length)};
    if (mlsq_sort_code_positions(query_index) != positions_length) {
        std::cout << "Internal Error: sorted positions length doesn't match expected length"
                  << std::endl;
        return INTERNAL_ERROR_SORTED_POSITIONS_LENGTH_MISMATCH;
    }
    uint32_t match_count = 0;
    for (uint32_t word = 0; word < (1u << word_length); ++word) {
        uint16_t position = mlsq_position_from_code(query_index, word);
        if (position != MLSQ_NOT_FOUND && position != test_lookup[word]) {
            std::cout << "Internal Error: MLS query result doesn't match the "
                         "original word"
                      << word << " position " << position << " expected position "
                      << test_lookup[word] << std::endl;
            return INTERNAL_ERROR_CODE_POSITION_LOOKUP_MISMATCH;
        }
        match_count += position != MLSQ_NOT_FOUND;
    }
    if (match_count != positions_length) {
        std::cout << "Internal Error: Not all code to position lookups were found" << std::endl;
        return INTERNAL_ERROR_MISSING_CODE_POSITION_ENTRY;
    }

    std::ofstream mlsq_index_file("mls_index.c");
    if (!mlsq_index_file.is_open()) {
        std::cout << "Unable to write MLS index file" << std::endl;
        return ERROR_WRITING_FILE;
    }
    mlsq_index_file << "#include \"mls_query.h\"\n";

    mlsq_index_file << "\nconst uint64_t MLS_ID = 0x" << std::hex << std::hash<std::string>{}(s)
                    << ";\n";

    mlsq_index_file << "\nstatic const uint32_t MLS_SEQUENCE[" << std::dec << sequence_chunks
                    << "] = {\n";
    for (uint32_t i = 0; i < sequence_chunks; ++i) {
        mlsq_index_file << "    0x" << std::hex << query_index.sequence[i] << ",\n";
    }
    mlsq_index_file << "};\n";

    mlsq_index_file << "\nstatic const uint16_t MLS_SORTED_CODE_POSITIONS[" << std::dec
                    << positions_length << "] = {\n";
    for (uint32_t i = 0; i < positions_length; ++i) {
        mlsq_index_file << "    0x" << std::hex << query_index.sorted_code_positions[i] << ",\n";
    }
    mlsq_index_file << "};\n";

    mlsq_index_file << "\nconst MlsIndex MLS_INDEX = {\n";
    mlsq_index_file << "    MLS_SEQUENCE,\n";
    mlsq_index_file << "    MLS_SORTED_CODE_POSITIONS,\n";
    mlsq_index_file << "    " << std::dec << s.size() << ",\n";
    mlsq_index_file << "    " << std::dec << word_length << ",\n};\n";
    mlsq_index_file.close();
    std::cout << "LookupTable file succesfully generated" << std::endl;

    CodeMap::save_pbm("code_map.pbm", s);
    std::cout << "CodeMap file succesfully generated" << std::endl;

    return SUCCESS;
}
