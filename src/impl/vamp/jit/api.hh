#pragma once

#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/simplify_settings.hh>

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vamp::jit
{
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    struct DynamicPath
    {
        std::shared_ptr<DynamicRobot> robot;
        std::vector<std::vector<float>> waypoints;
        std::size_t dim{0};

        DynamicPath() = default;

        explicit DynamicPath(std::shared_ptr<DynamicRobot> r)
          : robot(std::move(r)), dim(robot ? robot->dimension() : 0)
        {
        }

        auto size() const -> std::size_t
        {
            return waypoints.size();
        }

        auto cost() const -> float
        {
            return vamp::planning::path_helpers::cost(waypoints);
        }

        auto subdivide() -> void
        {
            vamp::planning::path_helpers::subdivide(waypoints);
        }

        auto interpolate_to_n_states(std::size_t n) -> void
        {
            vamp::planning::path_helpers::interpolate_to_n_states(waypoints, n);
        }

        auto interpolate_to_resolution(std::size_t resolution) -> void
        {
            vamp::planning::path_helpers::interpolate_to_resolution(waypoints, resolution);
        }

        auto validate(const vamp::collision::Environment<float> &env) -> bool
        {
            if (not robot)
            {
                throw std::runtime_error("DynamicPath has no associated robot");
            }
            const auto &ops = robot->ops();
            for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
            {
                if (not ops.validate_motion(
                        waypoints[i].data(),
                        waypoints[i + 1].data(),
                        static_cast<const void *>(&env),
                        0))
                {
                    return false;
                }
            }
            return true;
        }
    };

    struct DynamicSampler
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::SamplerHandle *handle{nullptr};

        DynamicSampler() = default;

        DynamicSampler(std::shared_ptr<DynamicRobot> r, ffi::SamplerHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicSampler()
        {
            if (handle != nullptr and robot)
            {
                robot->ops().sampler_destroy(handle);
            }
        }

        DynamicSampler(const DynamicSampler &) = delete;
        DynamicSampler &operator=(const DynamicSampler &) = delete;

        auto reset() -> void
        {
            robot->ops().sampler_reset(handle);
        }

        auto skip(std::uint64_t n) -> void
        {
            robot->ops().sampler_skip(handle, n);
        }

        // Writes `dim` floats into `out`.
        auto next(float *out) -> void
        {
            robot->ops().sampler_next(handle, out);
        }
    };

    struct DynamicPhs
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::PhsHandle *handle{nullptr};

        DynamicPhs() = default;

        DynamicPhs(std::shared_ptr<DynamicRobot> r, ffi::PhsHandle *h) : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicPhs()
        {
            if (handle != nullptr and robot)
            {
                robot->ops().phs_destroy(handle);
            }
        }

        DynamicPhs(const DynamicPhs &) = delete;
        DynamicPhs &operator=(const DynamicPhs &) = delete;

        auto set_transverse_diameter(float diameter) -> void
        {
            robot->ops().phs_set_diameter(handle, diameter);
        }

        auto transform(const float *in, float *out) const -> void
        {
            robot->ops().phs_transform(handle, in, out);
        }
    };

    struct DynamicPlanResult
    {
        std::shared_ptr<DynamicPath> path;
        std::size_t iterations{0};
        std::uint64_t nanoseconds{0};

        auto solved() const -> bool
        {
            return path and path->waypoints.size() >= 2;
        }
    };

    inline auto make_halton_sampler(std::shared_ptr<DynamicRobot> robot) -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->ops().sampler_halton();
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto make_xorshift_sampler(std::shared_ptr<DynamicRobot> robot, std::uint64_t seed = 0)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->ops().sampler_xorshift(seed);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto make_phs(std::shared_ptr<DynamicRobot> robot, const float *focus_a, const float *focus_b)
        -> std::shared_ptr<DynamicPhs>
    {
        auto *h = robot->ops().phs_new(focus_a, focus_b);
        return std::make_shared<DynamicPhs>(std::move(robot), h);
    }

    inline auto
    make_phs_sampler(std::shared_ptr<DynamicRobot> robot, const DynamicPhs &phs, DynamicSampler &inner)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->ops().sampler_phs(phs.handle, inner.handle);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    // Drain a result handle: copy meta + waypoints into a DynamicPlanResult,
    // then destroy the handle. Works uniformly for planner and simplify
    // results since they share result_meta / result_copy_waypoint / result_destroy.
    inline auto drain_handle(std::shared_ptr<DynamicRobot> robot, ffi::PlanResultHandle *handle)
        -> DynamicPlanResult
    {
        const auto &ops = robot->ops();
        auto meta = ops.result_meta(handle);

        DynamicPlanResult result;
        result.iterations = meta.iterations;
        result.nanoseconds = meta.nanoseconds;
        result.path = std::make_shared<DynamicPath>(robot);
        result.path->waypoints.reserve(meta.waypoints);

        std::vector<float> wp(meta.dimension);
        for (auto i = 0U; i < meta.waypoints; ++i)
        {
            ops.result_copy_waypoint(handle, i, wp.data());
            result.path->waypoints.emplace_back(wp);
        }

        ops.result_destroy(handle);
        return result;
    }

    template <typename SettingsT>
    inline auto solve(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        const auto &p = robot->ops().planners[static_cast<std::size_t>(planner)];
        if (p.solve == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        auto *handle = p.solve(
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        return drain_handle(std::move(robot), handle);
    }

    template <typename SettingsT>
    inline auto solve_multi(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        const auto &p = robot->ops().planners[static_cast<std::size_t>(planner)];
        if (p.solve_multi == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        auto *handle = p.solve_multi(
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        return drain_handle(std::move(robot), handle);
    }

    inline auto simplify(
        std::shared_ptr<DynamicRobot> robot,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vamp::planning::SimplifySettings &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        auto *handle = robot->ops().simplify(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);
        return drain_handle(std::move(robot), handle);
    }

    inline auto
    debug(DynamicRobot &robot, const float *config, const vamp::collision::Environment<float> &env)
        -> DebugType
    {
        const auto &ops = robot.ops();
        auto *handle = ops.debug(config, static_cast<const void *>(&env));
        DebugType copy = *reinterpret_cast<DebugType *>(handle);
        ops.debug_destroy(handle);
        return copy;
    }

    inline auto eefk(DynamicRobot &robot, const float *config) -> Eigen::Matrix4f
    {
        Eigen::Matrix4f result;
        robot.ops().eefk(config, result.data());
        return result;
    }

    inline auto fk(DynamicRobot &robot, const float *config) -> std::vector<vamp::collision::Sphere<float>>
    {
        const auto n = robot.n_spheres();
        std::vector<float> buf(n * 4);
        robot.ops().fk(config, buf.data());

        std::vector<vamp::collision::Sphere<float>> out;
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            out.emplace_back(buf[i * 4 + 0], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]);
        }
        return out;
    }

    inline auto validate(
        DynamicRobot &robot,
        const float *config,
        const vamp::collision::Environment<float> &env,
        bool check_bounds) -> bool
    {
        return robot.ops().validate(config, static_cast<const void *>(&env), check_bounds ? 1 : 0) != 0;
    }

    inline auto validate_motion(
        DynamicRobot &robot,
        const float *c_in,
        const float *c_out,
        const vamp::collision::Environment<float> &env,
        bool check_bounds) -> bool
    {
        return robot.ops().validate_motion(
                   c_in, c_out, static_cast<const void *>(&env), check_bounds ? 1 : 0) != 0;
    }

    inline auto filter_self_from_pointcloud(
        DynamicRobot &robot,
        const float *points,
        std::uint64_t n_points,
        float point_radius,
        const float *config,
        const vamp::collision::Environment<float> &env) -> std::vector<vamp::collision::Point>
    {
        std::vector<vamp::collision::Point> out;
        robot.ops().filter_pointcloud(
            points, n_points, point_radius, config, static_cast<const void *>(&env), &out);
        return out;
    }
}  // namespace vamp::jit
