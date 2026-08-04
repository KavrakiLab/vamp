#pragma once

#include <limits>
#include <vamp/planning/validate.hh>
#include <vamp/planning/nn.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    namespace path_helpers
    {
        template <typename Robot, typename Container>
        [[nodiscard]] inline auto cost_robot(const Container &wps) noexcept -> float
        {
            const auto n = wps.size();
            if (n > 2)
            {
                float total = 0;
                for (auto i = 0U; i + 1 < n; ++i)
                {
                    total += Robot::distance(wps[i], wps[i + 1]);
                }
                return total;
            }
            if (n == 2)
            {
                return Robot::distance(wps.front(), wps.back());
            }
            return std::numeric_limits<float>::infinity();
        }

        template <typename Robot, typename Container>
        inline auto subdivide_robot(Container &wps) noexcept
        {
            const auto n = wps.size();
            if (n < 2)
            {
                return;
            }
            Container next;
            next.reserve(n * 2);
            for (auto i = 0U; i + 1 < n; ++i)
            {
                next.emplace_back(wps[i]);
                next.emplace_back(Robot::interpolate(wps[i], wps[i + 1], 0.5F));
            }
            next.emplace_back(wps.back());
            wps = std::move(next);
        }

        template <typename Robot, typename Container>
        inline auto interpolate_to_n_states_robot(Container &wps, std::size_t n) noexcept
        {
            const auto n_p = wps.size();
            if (n_p < 2 or n < n_p)
            {
                return;
            }
            std::vector<float> seg_lengths(n_p - 1);
            float remaining_length = 0.;
            for (auto i = 0U; i + 1 < n_p; ++i)
            {
                seg_lengths[i] = Robot::distance(wps[i], wps[i + 1]);
                remaining_length += seg_lengths[i];
            }
            if (remaining_length < std::numeric_limits<float>::epsilon())
            {
                return;
            }
            Container next;
            next.reserve(n);
            const auto n1 = n_p - 1;
            for (auto i = 0U; i < n1; ++i)
            {
                const auto &a = wps[i];
                const auto &b = wps[i + 1];
                next.emplace_back(a);
                const auto max_n_states = n + i - n_p;
                if (max_n_states > 0)
                {
                    auto ns = (i + 1 == n1) ? (max_n_states + 2) :
                                              (static_cast<std::size_t>(
                                                   std::floor(0.5 + n * seg_lengths[i] / remaining_length)) +
                                               1);
                    ns = (ns > 2) ? std::min(ns - 2, max_n_states) : 0;
                    for (auto k = 1U; k <= ns; ++k)
                    {
                        next.emplace_back(
                            Robot::interpolate(a, b, static_cast<float>(k) / static_cast<float>(ns)));
                    }
                    n -= ns + 1;
                    remaining_length -= seg_lengths[i];
                }
                else
                {
                    n -= 1;
                }
            }
            next.emplace_back(wps.back());
            wps = std::move(next);
        }

        template <typename Robot, typename Container>
        inline auto interpolate_to_resolution_robot(Container &wps, std::size_t resolution) noexcept
        {
            const auto n_p = wps.size();
            if (n_p < 2)
            {
                return;
            }
            Container next;
            for (auto i = 0U; i + 1 < n_p; ++i)
            {
                const auto &a = wps[i];
                const auto &b = wps[i + 1];
                const float segment_cost = Robot::distance(a, b);
                const auto segment_states =
                    static_cast<std::size_t>(segment_cost * static_cast<float>(resolution));
                next.emplace_back(a);
                if (segment_cost < 1.F / static_cast<float>(resolution))
                {
                    continue;
                }
                for (auto k = 1U; k < segment_states; ++k)
                {
                    next.emplace_back(
                        Robot::interpolate(a, b, static_cast<float>(k) / static_cast<float>(segment_states)));
                }
            }
            next.emplace_back(wps.back());
            wps = std::move(next);
        }
    }  // namespace path_helpers

    template <typename Robot>
    struct Path : public std::vector<FloatVector<Robot::dimension>>
    {
        [[nodiscard]] inline auto cost() const noexcept -> float
        {
            return path_helpers::cost_robot<Robot>(*this);
        }

        inline auto subdivide() noexcept
        {
            path_helpers::subdivide_robot<Robot>(*this);
        }

        inline auto interpolate_to_n_states(std::size_t n) noexcept
        {
            path_helpers::interpolate_to_n_states_robot<Robot>(*this, n);
        }

        inline auto interpolate_to_resolution(std::size_t resolution) noexcept
        {
            path_helpers::interpolate_to_resolution_robot<Robot>(*this, resolution);
        }

        template <std::size_t rake>
        inline auto validate(const collision::Environment<FloatVector<rake>> &environment) noexcept -> bool
        {
            for (auto i = 0U; i < this->size() - 1; ++i)
            {
                const auto &current = this->operator[](i);
                const auto &next = this->operator[](i + 1);
                if (not validate_motion<Robot, rake, Robot::resolution>(current, next, environment))
                {
                    return false;
                }
            }
            return true;
        }
    };

    template <typename Robot>
    struct PlanningResult
    {
        Path<Robot> path;
        float cost{0.};
        std::size_t nanoseconds{0};
        std::size_t iterations{0};
        std::vector<std::size_t> size;
    };

    template <typename Robot>
    struct Roadmap
    {
        std::vector<FloatVector<Robot::dimension>> vertices;
        std::vector<std::vector<std::size_t>> edges;
        std::size_t nanoseconds{0};
        std::size_t iterations{0};
    };
}  // namespace vamp::planning
