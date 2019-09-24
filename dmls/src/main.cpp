#include "dmls_gen.hpp"
#include <iostream>
#include <thread>

using namespace std;
int main() {
  cout << "Generating ..." << std::endl;
  DmlsGen dmls_gen;
  std::vector<std::vector<bool>> sequences = {
      {0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1}};
  std::vector<double> explored_rates(sequences.size());
  std::vector<std::thread> threads;
  for (uint8_t i = 0; i < sequences.size(); ++i) {
    threads.emplace_back(
        [&sequences, &explored_rates](int idx) {
          DmlsGen dmls_gen;
          explored_rates[idx] =
              dmls_gen.generate_dmls(sequences[idx], 15, 30000);
        },
        i);
  }
  for (auto &thread : threads) {
    thread.join();
  }

  uint8_t max_len_idx = 0;
  for (uint8_t i = 1; i < sequences.size(); ++i) {
    if (sequences[i].size() > sequences[max_len_idx].size()) {
      max_len_idx = i;
    }
    explored_rates[0] += explored_rates[i];
  }
  for (bool bit : sequences[max_len_idx]) {
    std::cout << (int)bit;
  }
  std::cout << std::endl
            << "len " << sequences[max_len_idx].size() << " explored "
            << explored_rates[0] / sequences.size() << std::endl;
  return 0;
}
