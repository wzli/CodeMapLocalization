#include <fstream>
#include <iostream>
#include <cstdlib>

extern "C" {
  #include "lut/lut.h"
}

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

int main(int argc, char **argv) {
  std::string s;
  std::ifstream file(argv[1]);

  if(!file.is_open()) {
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
  size_t sequence_hash = std::stoul(s, 0, 16);
  std::cout << "parsed sequence_hash: 0x" << std::hex << sequence_hash << std::endl;

  std::getline(file, s, '"');
  std::getline(file, s, '"');

  if(s.size() != sequence_length) {
      std::cout << "WARNING: sequence length doesn't match" << std::endl;
  }
  if(std::hash<std::string>{}(s) != sequence_hash) {
      std::cout << "WARNING: sequence hash doesn't match" << std::endl;
  }
  std::cout << "Generating Map Codec ..." << std::endl;

  // below is word size specific (since the look up table optimized for 16-bits)
  const size_t LUT_SIZE = s.size() - word_length + 1;
  LutEntry* lut = new LutEntry[LUT_SIZE];
  uint16_t* test_lut = new uint16_t[1 << word_length];
  for(int32_t i = 0; i < (1 << word_length); i++) {
      test_lut[i] = LUT_KEY_ERROR;
  }

  for(uint32_t i = 0, word = 0; i < s.size(); ++i) {
      word >>= 1;
      if(s[i] == '1') {
          word |= 1 << (word_length - 1);
      } else if (s[i] != '0' ){
          std::cout << "Error: sequence contains invalid character " << s[i] << std::endl;
          return -2;
      }
      int position = i - word_length + 1;
      if(position >= 0) {
          uint32_t rword = reverse_bits(word, word_length);
          if(test_lut[word] != LUT_KEY_ERROR) {
              std::cout << "Error: sequence doesn't satisfy dmls constraints" << std::endl;
              return -3;
          }
          test_lut[word] = position;
          test_lut[rword] = position;
          test_lut[inverse_bits(word, word_length)] = position;
          test_lut[inverse_bits(rword, word_length)] = position;
          lut[position] = (LutEntry){word, static_cast<uint16_t>(position)};
      }
  };
  
  lut_sort(lut, LUT_SIZE) ;
  for(uint32_t i = 0; i < (1 << 16); ++i) {
      uint16_t position = lut_search(lut, LUT_SIZE, i);
      if(position != LUT_KEY_ERROR && position != test_lut[i]) {
          std::cout << "Internal Error: LUT search result doesn't match the original" << std::endl;
          return -4;
      }
  }

  std::ofstream lut_file("lut_dat.c");
  if(!lut_file.is_open()) {
      std::cout << "Unable to write LUT file" << std::endl;
      return -5;
  }
  lut_file << "#include \"lut.h\"" << std::endl;
  lut_file << "const uint8_t LUT_KEY_LENGTH = " << word_length << ";" << std::endl;
  lut_file << "const uint32_t LUT_SIZE = " << LUT_SIZE << ";" << std::endl;
  lut_file << "const uint64_t LUT_UNIQUE_ID = 0x" << std::hex << std::hash<std::string>{}(s) << ";" << std::endl;
  lut_file << "const LutEntry __LUT_DATA[] = {" << std::endl;
  for(uint32_t i = 0; i < LUT_SIZE; ++i) {
      lut_file << "  { 0x" << std::hex << lut[i].key << ", " << std::dec << lut[i].value << " }," << std::endl;
  }
  lut_file << "};" << std::endl;
  lut_file << "const LutEntry* LUT_DATA = __LUT_DATA;" << std::endl;
  lut_file.close();

  std::cout << "LUT file succesfully generated" << std::endl;

  delete lut;
  delete test_lut;

  return 0;
}
