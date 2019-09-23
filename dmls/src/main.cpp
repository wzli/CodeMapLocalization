#include <iostream>

#include "dmls_gen.hpp"

using namespace std;
int main() {
  cout << "Starting ..." << std::endl;
  DmlsGen mls_search;
  mls_search.generate_mls(5);
  return 0;
}
