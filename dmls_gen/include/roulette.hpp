#include <cstdint>
#include <numeric>

template <class WEIGHT_CONTAINER>
size_t roulette(const WEIGHT_CONTAINER& weights, uint32_t random_number) {
    float total = std::accumulate(weights.begin(), weights.end(), 0.0f);
    float scaled_num = total * random_number / ~0u;
    size_t i = 0;
    for (float threshold = weights[0]; scaled_num > threshold && i < weights.size();
            threshold += weights[++i])
        ;
    return i;
}
