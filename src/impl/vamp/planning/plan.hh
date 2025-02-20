#pragma once

#include <limits>
#include <vamp/planning/nn.hh>
#include <vamp/vector.hh>
#include <vector>

namespace vamp::planning
{
    template <std::size_t dim>
    struct Path : public std::vector<FloatVector<dim>>
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
            Path<dim> new_path;
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

        inline auto interpolate(std::size_t resolution) noexcept
        {
            Path<dim> new_path;

            if (this->size() < 2)
            {
                return;
            }

            float path_cost = cost();
            auto n_states = static_cast<std::size_t>(path_cost * static_cast<float>(resolution));
            new_path.reserve(n_states);

            for (auto i = 0U; i < this->size() - 1; ++i)
            {
                const auto &current = this->operator[](i);
                const auto &next = this->operator[](i + 1);

                float segment_cost = current.distance(next);
                auto segment_states = static_cast<std::size_t>(segment_cost * static_cast<float>(resolution));

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
    };

    template <std::size_t dim>
    struct PlanningResult
    {
        Path<dim> path;
        float cost{0.};
        std::size_t nanoseconds{0};
        std::size_t iterations{0};
        std::vector<std::size_t> size;
    };

    template <std::size_t dim>
    struct Roadmap
    {
        std::vector<FloatVector<dim>> vertices;
        std::vector<std::vector<std::size_t>> edges;
        std::size_t nanoseconds{0};
        std::size_t iterations{0};
    };
}  // namespace vamp::planning
