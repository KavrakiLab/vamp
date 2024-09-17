#pragma once

#include <memory>
#include <vamp/vector.hh>

namespace vamp::rng
{
    template <std::size_t dim>
    struct ConfigurationRNG
    {
        using Ptr = std::shared_ptr<ConfigurationRNG<dim>>;
        virtual inline auto next() noexcept -> FloatVector<dim> = 0;
    };
}  // namespace vamp::rng
