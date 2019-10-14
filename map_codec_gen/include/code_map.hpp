#pragma once
#include <string>
#include <vector>

struct CodeMap {
    void generate(const std::string& sequence);
    void save_pbm(const std::string& file_name);

    std::vector<bool> dmls;
    std::vector<bool> data;
    size_t sequence_hash;
};
