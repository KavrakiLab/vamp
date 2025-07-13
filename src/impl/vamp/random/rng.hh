#pragma once

#include <memory>
#include <vamp/vector.hh>
#include <vamp/random/distribution.hh>

namespace vamp::rng
{
    template <typename Robot>
    struct RNG
    {
        using Ptr = std::shared_ptr<RNG<Robot>>;
        virtual inline void reset() noexcept = 0;
        virtual inline auto next() noexcept -> FloatVector<Robot::dimension> = 0;

        Distribution dist;
    };
}  // namespace vamp::rng
