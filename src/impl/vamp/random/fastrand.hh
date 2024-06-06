#pragma once

#include <limits>

namespace vamp::rng
{

    struct FastRNG
    {
        FastRNG() noexcept : seed(0)
        {
        }

        template <typename T>
        inline auto uniform_integer(T low, T high) -> typename std::enable_if<std::is_integral_v<T>, T>::type
        {
            auto random = static_cast<T>(std::floor(uniform_real(low, high + 1.0)));
            return (random > high) ? high : random;
        }

        inline auto uniform_01() -> float
        {
            return static_cast<float>(fastrand()) / static_cast<float>(std::numeric_limits<uint32_t>::max());
        }

        inline auto uniform_real(float low, float high) -> float
        {
            return (high - low) * uniform_01() + low;
        }

        inline auto fastrand() -> uint32_t
        {
            seed = (214013 * seed + 2531011);
            return (seed >> 16) & 0x7FFF;
        }

        uint32_t seed{0};
    };
}  // namespace vamp::rng
