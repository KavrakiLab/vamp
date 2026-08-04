#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <unordered_set>

#include <vamp/constants.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    namespace utils
    {
        // Ported from OMPL: src/ompl/util/src/GeometricEquations.cpp
        inline auto unit_ball_measure(std::size_t dim) noexcept -> double
        {
            return std::pow(std::sqrt(vamp::utils::constants::pi), static_cast<double>(dim)) /
                   std::tgamma(static_cast<double>(dim) / 2.0 + 1.0);
        }
    }  // namespace utils

    struct ConstantNeighborParams
    {
        [[nodiscard]] inline auto _max_neighbors_impl(std::size_t /*num_states*/) const noexcept
            -> std::size_t
        {
            return k;
        }

        [[nodiscard]] inline auto _neighbor_radius_impl(std::size_t /*num_states*/) const noexcept -> float
        {
            return r;
        }

        std::size_t k = std::numeric_limits<std::size_t>::max();
        float r = std::numeric_limits<float>::infinity();
    };

    struct PRMStarNeighborParams
    {
        explicit PRMStarNeighborParams(std::size_t dim, double space_measure) noexcept
          : dim(dim), space_measure(space_measure)
        {
        }

        [[nodiscard]] inline constexpr auto max_neighbors(std::size_t num_states) const noexcept
            -> std::size_t
        {
            const auto prmstar_constant =
                vamp::utils::constants::e + (vamp::utils::constants::e / dimension());
            return static_cast<std::size_t>(
                vamp::utils::c_ceil(prmstar_constant * std::log(static_cast<double>(num_states))));
        }

        [[nodiscard]] inline auto neighbor_radius(std::size_t num_states) const noexcept -> float
        {
            const auto inverse_dim = 1.0 / dimension();
            // TODO: Implement online estimation of free space measure
            const auto space_measure_ratio = space_measure / utils::unit_ball_measure(dim);
            const auto prm_constant =
                2.0 * std::pow(1.0 + inverse_dim, inverse_dim) * std::pow(space_measure_ratio, inverse_dim);
            return gamma_scale * prm_constant *
                   std::pow(std::log(num_states) / static_cast<double>(num_states), inverse_dim);
        }

        [[nodiscard]] inline constexpr auto dimension() const noexcept -> double
        {
            return static_cast<double>(dim);
        }

        std::size_t dim;
        double space_measure;
        double gamma_scale = 2.0;
    };

    struct FCITStarNeighborParams
    {
        explicit FCITStarNeighborParams(std::size_t dim, double space_measure) noexcept
          : dim(dim), space_measure(space_measure)
        {
        }

        // We use *all* the neighbors
        [[nodiscard]] inline constexpr auto max_neighbors(std::size_t /* num_states */) const noexcept
            -> std::size_t
        {
            return std::numeric_limits<size_t>::max();
        }

        // I said we use *all* the neighbors
        [[nodiscard]] inline auto neighbor_radius(std::size_t /* num_states */) const noexcept -> float
        {
            return std::numeric_limits<float>::infinity();
        }

        [[nodiscard]] inline constexpr auto dimension() const noexcept -> double
        {
            return static_cast<double>(dim);
        }

        std::size_t dim;
        double space_measure;
        double gamma_scale = 2.0;
    };

    struct FMTStarNeighborParams
    {
        explicit FMTStarNeighborParams(std::size_t dim, double space_measure)
          : dim(dim), space_measure(space_measure)
        {
        }

        [[nodiscard]] inline constexpr auto max_neighbors(std::size_t num_states) const noexcept
            -> std::size_t
        {
            // NOTE: Adapted from OMPL
            const auto fmtstar_constant =
                vamp::utils::c_ceil(std::pow(2.0 * radius_multiplier, dimension())) *
                (vamp::utils::constants::e / dimension());
            return fmtstar_constant * std::log(static_cast<double>(num_states));
        }

        [[nodiscard]] inline auto neighbor_radius(std::size_t num_states) const noexcept -> float
        {
            // NOTE: Adapted from OMPL
            const auto inverse_dim = 1.0 / dimension();
            // TODO: Implement online estimation of free space measure
            const auto space_measure_ratio = space_measure / utils::unit_ball_measure(dim);
            const auto fmtstar_constant = radius_multiplier * 2.0 * std::pow(inverse_dim, inverse_dim) *
                                          std::pow(space_measure_ratio, inverse_dim);
            return fmtstar_constant *
                   std::pow(
                       std::log(static_cast<double>(num_states)) / static_cast<double>(num_states),
                       inverse_dim);
        }

        [[nodiscard]] inline constexpr auto dimension() const noexcept -> double
        {
            return static_cast<double>(dim);
        }

        std::size_t dim;
        double space_measure;
        // NOTE: Default value taken from OMPL
        double radius_multiplier = 1.1;
    };

    template <typename NeighborParams>
    struct RoadmapSettings
    {
        explicit RoadmapSettings(NeighborParams neighbor_params_) noexcept
          : neighbor_params(std::move(neighbor_params_))
        {
        }

        inline auto max_neighbors(std::size_t num_states) const noexcept
        {
            return neighbor_params.max_neighbors(num_states);
        }

        inline auto neighbor_radius(std::size_t num_states) const noexcept
        {
            return neighbor_params.neighbor_radius(num_states);
        }

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
        std::size_t batch_size = 1000;
        bool optimize = false;
        NeighborParams neighbor_params;
    };

    struct RoadmapNode
    {
        RoadmapNode(
            unsigned int index,
            unsigned int component,
            float g = std::numeric_limits<float>::infinity())
          : index(index), component(component), g(g)
        {
        }

        unsigned int index;
        unsigned int component;
        float g;

        struct Neighbor
        {
            unsigned int index;
            float distance;
        };

        std::vector<Neighbor> neighbors;
    };

}  // namespace vamp::planning
