#pragma once

#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>

#include <cricket/jit/session.hh>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace vamp::jit
{
    struct LoadOptions
    {
        std::string robot_source;
        std::string robot_name;
        std::size_t dimension;
        std::size_t rake = 8;
        std::size_t resolution = 32;

        std::vector<vamp::planning::Planner> planners;

        std::vector<std::string> include_dirs;
        std::vector<std::string> system_include_dirs;
        std::vector<std::string> extra_flags;
        std::vector<std::string> defines;
    };

    auto default_load_options() -> LoadOptions;

    class DynamicRobot
    {
    public:
        explicit DynamicRobot(
            const LoadOptions &opts,
            std::shared_ptr<cricket::jit::DiskObjectCache> cache = nullptr);
        ~DynamicRobot();

        DynamicRobot(const DynamicRobot &) = delete;
        DynamicRobot &operator=(const DynamicRobot &) = delete;

        auto has_planner(vamp::planning::Planner p) const -> bool;

        auto dimension() const -> std::size_t
        {
            return dimension_;
        }

        auto rake() const -> std::size_t
        {
            return rake_;
        }

        auto solve(
            vamp::planning::Planner p,
            const float *start,
            const float *goal,
            const void *env,
            const void *settings,
            ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *;

        auto solve_multi(
            vamp::planning::Planner p,
            const float *start,
            const float *goals,
            std::uint64_t n_goals,
            const void *env,
            const void *settings,
            ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *;

        auto result_meta(vamp::planning::Planner p, const ffi::PlanResultHandle *h) -> ffi::PlanResultMeta;
        auto result_copy_waypoint(
            vamp::planning::Planner p,
            const ffi::PlanResultHandle *h,
            std::uint64_t idx,
            float *out) -> void;
        auto result_destroy(vamp::planning::Planner p, ffi::PlanResultHandle *h) -> void;

        auto simplify(
            const float *path,
            std::uint64_t n_waypoints,
            const void *env,
            const void *settings,
            ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *;

        auto simplify_result_meta(const ffi::PlanResultHandle *h) -> ffi::PlanResultMeta;
        auto simplify_result_copy_waypoint(const ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
            -> void;
        auto simplify_result_destroy(ffi::PlanResultHandle *h) -> void;

        auto sampler_halton() -> ffi::SamplerHandle *;
        auto sampler_xorshift(std::uint64_t seed) -> ffi::SamplerHandle *;
        auto sampler_reset(ffi::SamplerHandle *h) -> void;
        auto sampler_skip(ffi::SamplerHandle *h, std::uint64_t n) -> void;
        auto sampler_next(ffi::SamplerHandle *h, float *out) -> void;
        auto sampler_destroy(ffi::SamplerHandle *h) -> void;

        auto debug(const float *config, const void *env) -> ffi::DebugHandle *;
        auto debug_destroy(ffi::DebugHandle *h) -> void;

        // ---- end-effector forward kinematics ----------------------------------

        // Writes a 4x4 column-major matrix (16 floats) into out_matrix.
        auto eefk(const float *config, float *out_matrix) -> void;

        // ---- sphere FK / validity / pointcloud filter -------------------------

        // Writes 4 * n_spheres() floats (x, y, z, r) into out_spheres.
        auto fk(const float *config, float *out_spheres) -> void;

        auto validate(const float *config, const void *env, bool check_bounds) -> bool;
        auto validate_motion(const float *c_in, const float *c_out, const void *env, bool check_bounds)
            -> bool;

        // out_filtered is a host-owned std::vector<vamp::collision::Point>*.
        auto filter_self_from_pointcloud(
            const float *points,
            std::uint64_t n_points,
            float point_radius,
            const float *config,
            const void *env,
            void *out_filtered) -> void;

        // ---- static per-robot metadata ----------------------------------------
        //
        // Queried once at construction and cached in the fields below.

        auto n_spheres() const -> std::size_t
        {
            return n_spheres_;
        }
        auto space_measure() const -> float
        {
            return space_measure_;
        }
        auto min_radius() const -> float
        {
            return min_radius_;
        }
        auto max_radius() const -> float
        {
            return max_radius_;
        }
        auto joint_names() const -> const std::vector<std::string> &
        {
            return joint_names_;
        }
        auto upper_bounds() const -> const std::vector<float> &
        {
            return upper_bounds_;
        }
        auto lower_bounds() const -> const std::vector<float> &
        {
            return lower_bounds_;
        }

        // ---- PHS --------------------------------------------------------------

        auto phs_new(const float *focus_a, const float *focus_b) -> ffi::PhsHandle *;
        auto phs_destroy(ffi::PhsHandle *h) -> void;
        auto phs_set_transverse_diameter(ffi::PhsHandle *h, float diameter) -> void;
        auto phs_transform(const ffi::PhsHandle *h, const float *in, float *out) -> void;
        auto sampler_phs(const ffi::PhsHandle *phs, ffi::SamplerHandle *inner) -> ffi::SamplerHandle *;

    private:
        struct PlannerEntry
        {
            ffi::SolveFn solve;
            ffi::SolveMultiFn solve_multi;
            ffi::ResultMetaFn meta;
            ffi::ResultCopyWaypointFn copy;
            ffi::ResultDestroyFn destroy;
        };

        struct SimplifyEntry
        {
            ffi::SimplifyFn simplify;
            ffi::ResultMetaFn meta;
            ffi::ResultCopyWaypointFn copy;
            ffi::ResultDestroyFn destroy;
        };

        struct SamplerEntry
        {
            ffi::SamplerHaltonFn halton;
            ffi::SamplerXorshiftFn xorshift;
            ffi::SamplerResetFn reset;
            ffi::SamplerSkipFn skip;
            ffi::SamplerNextFn next;
            ffi::SamplerDestroyFn destroy;
        };

        struct DebugEntry
        {
            ffi::DebugFn debug;
            ffi::DebugDestroyFn destroy;
        };

        struct InspectEntry
        {
            ffi::FkFn fk;
            ffi::ValidateFn validate;
            ffi::ValidateMotionFn validate_motion;
            ffi::FilterPointcloudFn filter_pointcloud;
        };

        struct PhsEntry
        {
            ffi::PhsNewFn create;
            ffi::PhsDestroyFn destroy;
            ffi::PhsSetDiameterFn set_diameter;
            ffi::PhsTransformFn transform;
            ffi::SamplerPhsFn sampler;
        };

        std::size_t dimension_;
        std::size_t rake_;
        std::unique_ptr<cricket::jit::JitSession> session_;
        std::unordered_map<vamp::planning::Planner, PlannerEntry> planners_;
        SimplifyEntry simplify_{};
        SamplerEntry sampler_{};
        DebugEntry debug_{};
        InspectEntry inspect_{};
        PhsEntry phs_{};
        ffi::EefkFn eefk_{nullptr};

        // Cached static metadata.
        std::size_t n_spheres_{0};
        float space_measure_{0.0F};
        float min_radius_{0.0F};
        float max_radius_{0.0F};
        std::vector<std::string> joint_names_;
        std::vector<float> upper_bounds_;
        std::vector<float> lower_bounds_;
    };
}  // namespace vamp::jit
