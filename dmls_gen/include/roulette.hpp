#include <cstdint>
#include <numeric>

template <class WEIGHT_CONTAINER>
size_t roulette(const WEIGHT_CONTAINER& weights, uint32_t random_number) {
    float total = std::accumulate(weights.begin(), weights.end(), 0.0f);
    const float scaled_num = total * random_number / ~0u;
    size_t i = weights.size() - 1;
    ;
    do {
        total -= weights[i];
    } while (scaled_num < total && --i > 0);
    return i;
}
