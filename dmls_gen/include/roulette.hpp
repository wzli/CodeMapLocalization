#include <cstdint>
#include <numeric>
#include <cassert>

template <class WEIGHT_CONTAINER>
size_t roulette(const WEIGHT_CONTAINER& weights, uint32_t random_number) {
    assert(weights.size() > 0);
    const float total = std::accumulate(weights.begin(), weights.end(), 0.0f);
    assert(total > 0);
    float scaled_num = total * random_number / ~0u;
    size_t i = 0;
    size_t j = weights.size();
    while(weights[--j] == 0);
    do {
        scaled_num -= weights[i];
    } while((scaled_num > 0 || weights[i] == 0) && (++i < j));
    assert(i >= 0);
    assert(i < weights.size());
    assert(weights[i] != 0);
    return i;
}
