#pragma once

#include <limits>
#include <vamp/planning/validate.hh>
#include <vamp/planning/nn.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot>
    struct Path : public std::vector<FloatVector<Robot::dimension>>
    {
        [[nodiscard]] inline auto cost() const noexcept -> float
        {
            if (this->size() > 2)
            {
                float distance = 0;
                for (auto i = 0U; i < this->size() - 1; ++i)
                {
                    distance += this->operator[](i).distance(this->operator[](i + 1));
                }

                return distance;
            }

            if (this->size() == 2)
            {
                return this->front().distance(this->back());
            }

            return std::numeric_limits<float>::infinity();
        }

        inline auto subdivide() noexcept
        {
            Path<Robot> new_path;
            new_path.reserve(this->size() * 2);

            for (auto i = 0U; i < this->size() - 1; ++i)
            {
                const auto &current = this->operator[](i);
                const auto &next = this->operator[](i + 1);
                new_path.emplace_back(current);
                new_path.emplace_back(current.interpolate(next, 0.5));
            }

            new_path.emplace_back(this->back());
            this->swap(new_path);
        }

        inline auto interpolate_to_n_states(std::size_t n) noexcept
        {
            const std::size_t n_p = this->size();
            if (this->size() < 2 or n < n_p)
            {
                return;
            }

            Path<Robot> new_path;
            new_path.reserve(n);

            std::vector<float> segment_lengths(n_p - 1);
            float remaining_length = 0.;

            for (auto i = 0U; i < n_p - 1; ++i)
            {
                remaining_length += segment_lengths[i] =
                    this->operator[](i).distance(this->operator[](i + 1));
            }

            if (remaining_length < std::numeric_limits<float>::epsilon())
            {
                return;
            }

            const auto n1 = n_p - 1;
            for (auto i = 0U; i < n1; ++i)
            {
                const auto &a = this->operator[](i);
                const auto &b = this->operator[](i + 1);

                new_path.emplace_back(a);
                const auto max_n_states = n + i - n_p;

                if (max_n_states > 0)
                {
                    auto ns =
                        (i + 1 == n1) ?
                            (max_n_states + 2) :
                            (static_cast<int>(std::floor(0.5 + n * segment_lengths[i] / remaining_length)) +
                             1);

                    // more than endpoints needed
                    ns = (ns > 2) ? std::min(ns - 2, max_n_states) : 0;

                    const auto &v = b - a;
                    for (auto k = 1U; k <= ns; ++k)
                    {
                        new_path.emplace_back(a + (static_cast<float>(k) / ns) * v);
                    }

                    n -= ns + 1;
                    remaining_length -= segment_lengths[i];
                }
                else
                {
                    n -= 1;
                }
            }

            new_path.emplace_back(this->back());
            this->swap(new_path);
        }

        inline auto interpolate_to_resolution(std::size_t resolution) noexcept
        {
            if (this->size() < 2)
            {
                return;
            }

            const float path_cost = cost();
            const auto n_states = static_cast<std::size_t>(path_cost * static_cast<float>(resolution));

            Path<Robot> new_path;
            new_path.reserve(n_states);

            for (auto i = 0U; i < this->size() - 1; ++i)
            {
                const auto &current = this->operator[](i);
                const auto &next = this->operator[](i + 1);

                const float segment_cost = current.distance(next);
                const auto segment_states =
                    static_cast<std::size_t>(segment_cost * static_cast<float>(resolution));

                new_path.emplace_back(current);

                if (segment_cost < 1.F / static_cast<float>(resolution))
                {
                    continue;
                }

                for (auto i = 1U; i < segment_states; ++i)
                {
                    new_path.emplace_back(current.interpolate(
                        next, static_cast<float>(i) / static_cast<float>(segment_states)));
                }
            }

            new_path.emplace_back(this->back());
            this->swap(new_path);
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
