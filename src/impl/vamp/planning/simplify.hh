#pragma once

#include <map>

#include <vamp/collision/environment.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto smooth_bspline(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const BSplineSettings &settings) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        bool changed = false;
        for (auto step = 0U; step < settings.max_steps; ++step)
        {
            path.subdivide();

            bool updated = false;
            for (auto index = 2U; index < path.size() - 1; index += 2)
            {
                const auto temp_1 = path[index].interpolate(path[index - 1], settings.midpoint_interpolation);
                const auto temp_2 = path[index].interpolate(path[index + 1], settings.midpoint_interpolation);
                const auto midpoint = temp_1.interpolate(temp_2, 0.5);

                if (path[index].distance(midpoint) > settings.min_change and
                    validate_motion<Robot, rake, resolution>(path[index - 1], midpoint, environment) and
                    validate_motion<Robot, rake, resolution>(midpoint, path[index + 1], environment))
                {
                    path[index] = midpoint;
                    changed |= (updated = true);
                }
            }

            if (not updated)
            {
                break;
            }
        }

        return changed;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto reduce_path_vertices(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const ReduceSettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
        const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() : settings.max_empty_steps;

        bool result = false;
        for (auto i = 0U, no_change = 0U; i < max_steps or no_change < max_empty_steps; ++i, ++no_change)
        {
            int initial_size = path.size();
            int max_n = initial_size - 1;

            int range = 1 + static_cast<int>(
                                std::floor(0.5F + static_cast<float>(initial_size) * settings.range_ratio));

            auto point_0 = rng->dist.uniform_integer(0, max_n);
            auto point_1 =
                rng->dist.uniform_integer(std::max(point_0 - range, 0), std::min(max_n, point_0 + range));

            if (std::abs(point_0 - point_1) < 2)
            {
                if (point_0 < max_n - 1)
                {
                    point_1 = point_0 + 2;
                }
                else if (point_0 > 1)
                {
                    point_1 = point_0 - 2;
                }
                else
                {
                    continue;
                }
            }

            if (point_0 > point_1)
            {
                std::swap(point_0, point_1);
            }

            if (validate_motion<Robot, rake, resolution>(path[point_0], path[point_1], environment))
            {
                path.erase(path.begin() + point_0 + 1, path.begin() + point_1);
                no_change = 0;
                result = true;
            }
        }

        return result;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto shortcut_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const ShortcutSettings & /*settings*/) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        bool result = false;
        for (auto i = 0U; i < path.size() - 2; ++i)
        {
            for (auto j = path.size() - 1; j > i + 1; --j)
            {
                if (validate_motion<Robot, rake, resolution>(path[i], path[j], environment))
                {
                    path.erase(path.begin() + i + 1, path.begin() + j);
                    result = true;
                    break;
                }
            }
        }

        return result;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto perturb_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const PerturbSettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
        const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() : settings.max_empty_steps;

        bool changed = false;
        for (auto step = 0U, no_change = 0U; step < max_steps and no_change < max_empty_steps;
             ++step, ++no_change)
        {
            auto to_perturb_idx = rng->dist.uniform_integer(1UL, path.size() - 2);
            auto perturb_state = path[to_perturb_idx];
            auto before_state = path[to_perturb_idx - 1];
            auto after_state = path[to_perturb_idx + 1];

            float old_cost = perturb_state.distance(before_state) + perturb_state.distance(after_state);

            for (auto attempt = 0U; attempt < settings.perturbation_attempts; ++attempt)
            {
                auto perturbation = rng->next();
                Robot::scale_configuration(perturbation);

                const auto new_state = perturb_state.interpolate(perturbation, settings.range);
                float new_cost = new_state.distance(before_state) + new_state.distance(after_state);

                if (new_cost < old_cost and
                    validate_motion<Robot, rake, resolution>(before_state, new_state, environment) and
                    validate_motion<Robot, rake, resolution>(after_state, new_state, environment))
                {
                    no_change = 0;
                    changed = true;
                    path[to_perturb_idx] = new_state;
                    break;
                }
            }
        }

        return changed;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline auto simplify(
        const Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const SimplifySettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng) -> PlanningResult<Robot>
    {
        auto start_time = std::chrono::steady_clock::now();

        PlanningResult<Robot> result;

        const auto bspline = [&result, &environment, settings]()
        { return smooth_bspline<Robot, rake, resolution>(result.path, environment, settings.bspline); };
        const auto reduce = [&result, &environment, settings, rng]()
        {
            return reduce_path_vertices<Robot, rake, resolution>(
                result.path, environment, settings.reduce, rng);
        };
        const auto shortcut = [&result, &environment, settings]()
        { return shortcut_path<Robot, rake, resolution>(result.path, environment, settings.shortcut); };
        const auto perturb = [&result, &environment, settings, rng]()
        { return perturb_path<Robot, rake, resolution>(result.path, environment, settings.perturb, rng); };

        const std::map<SimplifyRoutine, std::function<bool()>> operations = {
            {BSPLINE, bspline},
            {REDUCE, reduce},
            {SHORTCUT, shortcut},
            {PERTURB, perturb},
        };

        // Check if straight line is valid
        if (path.size() == 2 or (path.size() > 2 and validate_motion<Robot, rake, resolution>(
                                                         path.front(), path.back(), environment)))
        {
            result.path.emplace_back(path.front());
            result.path.emplace_back(path.back());
            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            return result;
        }

        result.path = path;

        if (settings.interpolate)
        {
            result.path.interpolate_to_n_states(settings.interpolate);
        }

        if (path.size() > 2)
        {
            for (auto i = 0U; i < settings.max_iterations; ++i)
            {
                result.iterations++;

                bool any = false;
                for (const auto &op : settings.operations)
                {
                    any |= operations.find(op)->second();
                }

                if (not any)
                {
                    break;
                }
            }
        }

        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
        return result;
    }
}  // namespace vamp::planning
