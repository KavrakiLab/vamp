#pragma once

extern "C"
{
#include <simdxorshift128plus.h>
}

#include <array>
#include <vamp/vector.hh>
#include <vamp/vector/interface.hh>

namespace vamp::rng
{
    template <std::size_t dim>
    struct XORShift
    {
        explicit XORShift(std::size_t skip_iterations = 0)
          : XORShift(2, 3UL, 0.F, 1.F, skip_iterations) noexcept
        {
        }

        explicit XORShift(
            uint64_t key1 = 2UL,
            uint64_t key2 = 3UL,
            float min_val = 0.0,
            float max_val = 1.0,
            std::size_t skip_iterations = 0) noexcept
          : min_val{min_val}, max_val{max_val}
        {
            avx_xorshift128plus_init(key1, key2, &key);

            for (auto i = 0U; i < skip_iterations; ++i)
            {
                next();
            }
        }

        avx_xorshift128plus_key_t key{};
        float min_val;
        float max_val;
        using IntVector = Vector<SIMDVector<__m256i>, 1, dim>;
        IntVector buffer;

        inline auto next() noexcept -> FloatVector<dim>
        {
            for (auto i = 0U; i < IntVector::num_vectors; ++i)
            {
                buffer.data[i] = avx_xorshift128plus(&key);
            }

            return FloatVector<dim>::map_to_range(buffer, min_val, max_val);
        }
    };
}  // namespace vamp::rng
