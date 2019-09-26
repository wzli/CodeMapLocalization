#include "dmls_gen.hpp"
#include <cstdlib>
#include <iostream>

using namespace std;
int main(int argc, char **argv) {
  cout << "Generating ..." << std::endl << std::endl;
  DmlsGen dmls_gen;
  std::vector<bool> sequence;
  dmls_gen.generate_dmls(sequence, atoi(argv[1]), 0,
                         [](std::vector<bool> &sequence) {
                           for (bool bit : sequence) {
                             std::cout << bit;
                           }
                           std::cout << std::endl
                                     << "len " << sequence.size() << std::endl
                                     << std::endl;
                         });
  return 0;
}
