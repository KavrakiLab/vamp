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
        // Full C++ source defining the robot struct (cricket-generated or hand-written).
        std::string robot_source;
        std::string robot_name;
        std::size_t dimension;        // for FFI buffer sizing on the host side
        std::size_t rake = 8;         // SIMD lanes
        std::size_t resolution = 32;  // collision check resolution

        std::vector<Planner> planners;

        // Include paths and extra cc1 flags the JIT compiler should use. The
        // caller (typically vamp's Python entrypoint) is responsible for
        // filling these with the vamp + Eigen + nigh include dirs that match
        // this vamp build.
        std::vector<std::string> include_dirs;
        std::vector<std::string> system_include_dirs;
        std::vector<std::string> extra_flags;
        std::vector<std::string> defines;
    };

    // Returns a LoadOptions with include_dirs / system_include_dirs / defines
    // / extra_flags pre-populated from vamp's build-time configuration (see
    // <vamp/jit/build_paths.hh>). The caller fills in robot_source /
    // robot_name / planners / etc.
    auto default_load_options() -> LoadOptions;

    // Owns a JIT session compiled for one robot. Function pointers for the
    // configured planners (plus per-robot simplify and sampler ops) are
    // resolved at construction time; per-planner solve/simplify/sampler
    // calls dispatch through them.
    class DynamicRobot
    {
    public:
        explicit DynamicRobot(
            const LoadOptions &opts,
            std::shared_ptr<cricket::jit::DiskObjectCache> cache = nullptr);
        ~DynamicRobot();

        DynamicRobot(const DynamicRobot &) = delete;
        DynamicRobot &operator=(const DynamicRobot &) = delete;

        auto has_planner(Planner p) const -> bool;

        auto dimension() const -> std::size_t
        {
            return dimension_;
        }

        auto rake() const -> std::size_t
        {
            return rake_;
        }

        // ---- planner solve / multi-goal solve ---------------------------------

        // Returns nullptr if the planner was not loaded.
        auto solve(
            Planner p,
            const float *start,
            const float *goal,
            const void *env,
            const void *settings,
            ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *;

        auto solve_multi(
            Planner p,
            const float *start,
            const float *goals,
            std::uint64_t n_goals,
            const void *env,
            const void *settings,
            ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *;

        auto result_meta(Planner p, const ffi::PlanResultHandle *h) -> ffi::PlanResultMeta;
        auto result_copy_waypoint(Planner p, const ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
            -> void;
        auto result_destroy(Planner p, ffi::PlanResultHandle *h) -> void;

        // ---- simplify ---------------------------------------------------------

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

        // ---- samplers ---------------------------------------------------------

        auto sampler_halton() -> ffi::SamplerHandle *;
        auto sampler_xorshift(std::uint64_t seed) -> ffi::SamplerHandle *;
        auto sampler_reset(ffi::SamplerHandle *h) -> void;
        auto sampler_skip(ffi::SamplerHandle *h, std::uint64_t n) -> void;
        auto sampler_next(ffi::SamplerHandle *h, float *out) -> void;
        auto sampler_destroy(ffi::SamplerHandle *h) -> void;

        // ---- debug ------------------------------------------------------------

        auto debug(const float *config, const void *env) -> ffi::DebugHandle *;
        auto debug_destroy(ffi::DebugHandle *h) -> void;

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

        std::size_t dimension_;
        std::size_t rake_;
        std::unique_ptr<cricket::jit::JitSession> session_;
        std::unordered_map<int, PlannerEntry> planners_;  // key = Planner enum int
        SimplifyEntry simplify_{};
        SamplerEntry sampler_{};
        DebugEntry debug_{};
    };
}  // namespace vamp::jit
