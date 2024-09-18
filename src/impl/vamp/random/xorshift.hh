#pragma once

extern "C"
{
#include <simdxorshift128plus.h>
}

#include <vamp/vector.hh>
#include <vamp/vector/interface.hh>
#include <vamp/random/rng.hh>

namespace vamp::rng
{
    template <std::size_t dim>
    struct XORShift : public RNG<dim>
    {
        explicit XORShift(
            uint64_t key1 = 2UL,
            uint64_t key2 = 3UL,
            float min_val = 0.0,
            float max_val = 1.0) noexcept
          : key1_init(key1), key2_init(key2), min_val(min_val), max_val(max_val)
        {
            avx_xorshift128plus_init(key1, key2, &key);
        }

        uint64_t key1_init;
        uint64_t key2_init;
        float min_val;
        float max_val;

        avx_xorshift128plus_key_t key{};
        using IntVector = Vector<SIMDVector<__m256i>, 1, dim>;
        IntVector buffer;

        inline void reset() noexcept override
        {
            avx_xorshift128plus_init(key1_init, key2_init, &key);
        }

        inline auto next() noexcept -> FloatVector<dim> override
        {
            for (auto i = 0U; i < IntVector::num_vectors; ++i)
            {
                buffer.data[i] = avx_xorshift128plus(&key);
            }

            return FloatVector<dim>::map_to_range(buffer, min_val, max_val);
        }
    };
}  // namespace vamp::rng
