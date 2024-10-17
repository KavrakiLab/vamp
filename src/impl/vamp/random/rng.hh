#pragma once

#include <memory>
#include <vamp/vector.hh>
#include <vamp/random/distribution.hh>

namespace vamp::rng
{
    template <std::size_t dim>
    struct RNG
    {
        using Ptr = std::shared_ptr<RNG<dim>>;
        virtual inline void reset() noexcept = 0;
        virtual inline auto next() noexcept -> FloatVector<dim> = 0;

        Distribution dist;
    };
}  // namespace vamp::rng
