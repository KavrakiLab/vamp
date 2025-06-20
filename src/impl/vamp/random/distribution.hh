#pragma once

#include <cmath>
#include <random>
#include <algorithm>

namespace vamp::rng
{
    struct Distribution
    {
        Distribution() noexcept : rng(0)
        {
        }

        template <typename T>
        inline auto uniform_integer(T low, T high) noexcept ->
            typename std::enable_if<std::is_integral_v<T>, T>::type
        {
            auto random = static_cast<T>(std::floor(uniform_real(low, high + 1.0)));
            return (random > high) ? high : random;
        }

        inline auto uniform_01() noexcept -> float
        {
            return uniform(rng);
        }

        inline auto uniform_real(float low, float high) noexcept -> float
        {
            return (high - low) * uniform_01() + low;
        }

        template <typename T>
        inline auto shuffle(T &iterable)
        {
            std::shuffle(iterable.begin(), iterable.end(), rng);
        }

        inline void reset() noexcept
        {
            rng.seed(0);
        }

        std::mt19937 rng;
        std::uniform_real_distribution<float> uniform{0, 1};
    };
}  // namespace vamp::rng
