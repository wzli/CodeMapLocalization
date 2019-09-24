#include <iostream>
#include "dmls_gen.hpp"

using namespace std;
int main() {
  cout << "Generating ..." << std::endl;
  DmlsGen dmls_gen;
  std::vector<bool> sequence = {0};
  double explored = dmls_gen.generate_dmls(sequence, 5, 1000);
  for(bool bit : sequence) {
      std::cout << (int)bit;
  }
  std::cout << std::endl << "len " << sequence.size() << " explored " << explored << std::endl;
  return 0;
}
