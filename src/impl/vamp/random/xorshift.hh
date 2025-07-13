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
    template <typename Robot>
    struct XORShift : public RNG<Robot>
    {
        explicit XORShift(uint64_t key1 = 2UL, uint64_t key2 = 3UL) noexcept
          : key1_init(key1), key2_init(key2)
        {
            avx_xorshift128plus_init(key1, key2, &key);
        }

        uint64_t key1_init;
        uint64_t key2_init;

        avx_xorshift128plus_key_t key{};
        using IntVector = Vector<SIMDVector<__m256i>, 1, Robot::dimension>;
        IntVector buffer;

        inline void reset() noexcept override final
        {
            avx_xorshift128plus_init(key1_init, key2_init, &key);
        }

        inline auto next() noexcept -> FloatVector<Robot::dimension> override final
        {
            for (auto i = 0U; i < IntVector::num_vectors; ++i)
            {
                buffer.data[i] = avx_xorshift128plus(&key);
            }

            auto result = FloatVector<Robot::dimension>::map_to_range(buffer, 0.F, 1.F);
            Robot::scale_configuration(result);
            return result;
        }
    };
}  // namespace vamp::rng
