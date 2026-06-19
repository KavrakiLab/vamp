#pragma once

#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>

#include <Eigen/Dense>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Reusable C++ surface around DynamicRobot. Both vamp/bindings/python/dynamic.cc
// (nanobind bindings) and any pure-C++ caller use the types and free functions
// declared here — the bindings should be a thin wrapper, not a re-implementation.
//
// Lifetime ownership: DynamicSampler / DynamicPhs / DynamicPath hold a
// shared_ptr back to the originating DynamicRobot so the JIT session outlives
// any handle they own. The opaque handles (ffi::SamplerHandle*, etc.) are
// released in the destructor.

namespace vamp::jit
{
    // Type R::Debug isn't templated on R — same layout across all robots
    // (pair of vector<vector<string>> and vector<pair<size_t, size_t>>).
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    // ---- DynamicPath -------------------------------------------------------
    //
    // Mirrors vamp::planning::Path<Robot> for the dynamic case. Holds a
    // shared_ptr back to the robot so validate() can dispatch to the JIT'd
    // validate_motion. cost / subdivide / interpolate_* delegate to the
    // shared vamp::planning::path_helpers templates so the implementation is
    // bit-for-bit identical to the static side.

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
            for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
            {
                if (not robot->validate_motion(
                        waypoints[i].data(),
                        waypoints[i + 1].data(),
                        static_cast<const void *>(&env),
                        false))
                {
                    return false;
                }
            }
            return true;
        }
    };

    // ---- DynamicSampler ----------------------------------------------------
    //
    // RAII wrapper around an ffi::SamplerHandle from one of the per-robot
    // factories (sampler_halton, sampler_xorshift, sampler_phs).

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
                robot->sampler_destroy(handle);
            }
        }

        DynamicSampler(const DynamicSampler &) = delete;
        DynamicSampler &operator=(const DynamicSampler &) = delete;

        auto reset() -> void
        {
            robot->sampler_reset(handle);
        }
        auto skip(std::uint64_t n) -> void
        {
            robot->sampler_skip(handle, n);
        }
        // Writes `dim` floats into `out`.
        auto next(float *out) -> void
        {
            robot->sampler_next(handle, out);
        }
    };

    // ---- DynamicPhs --------------------------------------------------------
    //
    // RAII wrapper around an ffi::PhsHandle for the per-robot
    // ProlateHyperspheroid<R>.

    struct DynamicPhs
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::PhsHandle *handle{nullptr};

        DynamicPhs() = default;
        DynamicPhs(std::shared_ptr<DynamicRobot> r, ffi::PhsHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicPhs()
        {
            if (handle != nullptr and robot)
            {
                robot->phs_destroy(handle);
            }
        }

        DynamicPhs(const DynamicPhs &) = delete;
        DynamicPhs &operator=(const DynamicPhs &) = delete;

        auto set_transverse_diameter(float diameter) -> void
        {
            robot->phs_set_transverse_diameter(handle, diameter);
        }
        // Writes `dim` floats into `out`.
        auto transform(const float *in, float *out) const -> void
        {
            robot->phs_transform(handle, in, out);
        }
    };

    // ---- DynamicPlanResult -------------------------------------------------
    //
    // Mirrors vamp::planning::PlanningResult<Robot>. `solved` is the standard
    // path-size-ge-2 predicate; path-cost lives on path->cost() like the
    // static side.

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

    // ---- Sampler / PHS factories ------------------------------------------

    inline auto make_halton_sampler(std::shared_ptr<DynamicRobot> robot)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->sampler_halton();
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto make_xorshift_sampler(std::shared_ptr<DynamicRobot> robot, std::uint64_t seed = 0)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->sampler_xorshift(seed);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto make_phs(
        std::shared_ptr<DynamicRobot> robot,
        const float *focus_a,
        const float *focus_b) -> std::shared_ptr<DynamicPhs>
    {
        auto *h = robot->phs_new(focus_a, focus_b);
        return std::make_shared<DynamicPhs>(std::move(robot), h);
    }

    inline auto make_phs_sampler(
        std::shared_ptr<DynamicRobot> robot,
        const DynamicPhs &phs,
        DynamicSampler &inner) -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->sampler_phs(phs.handle, inner.handle);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    // ---- internal: drain an FFI result handle into DynamicPlanResult ------

    template <typename MetaFn, typename CopyFn, typename DestroyFn>
    inline auto drain_handle(
        std::shared_ptr<DynamicRobot> robot,
        ffi::PlanResultHandle *handle,
        MetaFn &&meta_call,
        CopyFn &&copy_call,
        DestroyFn &&destroy_call) -> DynamicPlanResult
    {
        auto meta = meta_call(handle);

        DynamicPlanResult result;
        result.iterations = meta.iterations;
        result.nanoseconds = meta.nanoseconds;
        result.path = std::make_shared<DynamicPath>(robot);
        result.path->waypoints.reserve(meta.waypoints);

        std::vector<float> wp(meta.dimension);
        for (auto i = 0U; i < meta.waypoints; ++i)
        {
            copy_call(handle, i, wp.data());
            result.path->waypoints.emplace_back(wp);
        }

        destroy_call(handle);
        return result;
    }

    // ---- solve / solve_multi / simplify ----------------------------------
    //
    // Templated on the settings type. SettingsT must be the concrete vamp
    // planning settings struct the planner expects (e.g. RRTCSettings).

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
        auto *handle = robot->solve(
            planner,
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        if (handle == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        auto *raw = robot.get();
        return drain_handle(
            robot,
            handle,
            [&](auto *h) { return raw->result_meta(planner, h); },
            [&](auto *h, std::uint64_t i, float *out) { raw->result_copy_waypoint(planner, h, i, out); },
            [&](auto *h) { raw->result_destroy(planner, h); });
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
        auto *handle = robot->solve_multi(
            planner,
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        if (handle == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        auto *raw = robot.get();
        return drain_handle(
            robot,
            handle,
            [&](auto *h) { return raw->result_meta(planner, h); },
            [&](auto *h, std::uint64_t i, float *out) { raw->result_copy_waypoint(planner, h, i, out); },
            [&](auto *h) { raw->result_destroy(planner, h); });
    }

    inline auto simplify(
        std::shared_ptr<DynamicRobot> robot,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vamp::planning::SimplifySettings &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        auto *handle = robot->simplify(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);
        auto *raw = robot.get();
        return drain_handle(
            robot,
            handle,
            [&](auto *h) { return raw->simplify_result_meta(h); },
            [&](auto *h, std::uint64_t i, float *out) { raw->simplify_result_copy_waypoint(h, i, out); },
            [&](auto *h) { raw->simplify_result_destroy(h); });
    }

    // ---- debug / eefk / fk / pointcloud filter ----------------------------

    inline auto debug(DynamicRobot &robot, const float *config, const vamp::collision::Environment<float> &env)
        -> DebugType
    {
        auto *handle = robot.debug(config, static_cast<const void *>(&env));
        DebugType copy = *reinterpret_cast<DebugType *>(handle);
        robot.debug_destroy(handle);
        return copy;
    }

    inline auto eefk(DynamicRobot &robot, const float *config) -> Eigen::Matrix4f
    {
        Eigen::Matrix4f result;
        robot.eefk(config, result.data());
        return result;
    }

    inline auto fk(DynamicRobot &robot, const float *config)
        -> std::vector<vamp::collision::Sphere<float>>
    {
        const auto n = robot.n_spheres();
        std::vector<float> buf(n * 4);
        robot.fk(config, buf.data());

        std::vector<vamp::collision::Sphere<float>> out;
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            out.emplace_back(buf[i * 4 + 0], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]);
        }
        return out;
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
        robot.filter_self_from_pointcloud(
            points, n_points, point_radius, config, static_cast<const void *>(&env), &out);
        return out;
    }
}  // namespace vamp::jit
