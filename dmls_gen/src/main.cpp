#include "dmls_gen.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;
int main(int argc, char **argv) {
  cout << "Generating ..." << std::endl << std::endl;
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
        file_name << "dmls_" << (int)word_length << "_" << sequence.size()
                  << "_" << std::hex
                  << std::hash<std::string>{}(sequence_string.str());
        file.open(file_name.str());
        file << sequence_string.str();
        file.close();
        std::cout << sequence_string.str() << std::endl
                  << "bits " << (int)word_length << " length "
                  << sequence.size() << std::endl
                  << std::endl;
      });
  return 0;
}
