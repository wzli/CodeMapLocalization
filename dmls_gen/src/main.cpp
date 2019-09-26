#include "dmls_gen.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char **argv) {
  std::cout << "Generating ...\n" << std::endl;
  DmlsGen dmls_gen;
  std::vector<bool> sequence;
  dmls_gen.generate_dmls(
      sequence, atoi(argv[1]), 0,
      [](std::vector<bool> &sequence, uint8_t word_length) {
        std::stringstream sequence_string, file_name;
        std::ofstream file;
        for (bool bit : sequence) {
          sequence_string << bit;
        }
        auto sequence_hash = std::hash<std::string>{}(sequence_string.str());
        file_name << "dmls_" << (int)word_length << "_" << sequence.size()
                  << "_" << std::hex << sequence_hash << ".yaml";
        file.open(file_name.str());
        file << "word_length: " << (int)word_length << '\n';
        file << "sequence_length: " << sequence.size() << '\n';
        file << "sequence_hash: 0x" << std::hex << sequence_hash << '\n';
        file << "sequence: \"" << sequence_string.str() << "\"" << '\n';
        file.close();
        std::cout << sequence_string.str()
                  << "\nword_length: " << (int)word_length
                  << " sequence_length: " << sequence.size() << '\n'
                  << std::endl;
      });
  if (sequence.empty()) {
    std::cout << "word length must be between 1 to 32" << std::endl;
  }
  return 0;
}
