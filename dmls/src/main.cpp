#include <iostream>

#include "dmls_gen.hpp"

using namespace std;
int main() 
{
  cout << "Starting ..." << std::endl;
  MlsSearch mls_search;
  mls_search.generate_mls(16);
  return 0;
}
