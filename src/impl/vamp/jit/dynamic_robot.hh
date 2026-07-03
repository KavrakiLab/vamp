#pragma once

#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>
#include <vamp/planning/planner.hh>

#include <cricket/jit/compiler.hh>
#include <cricket/jit/session.hh>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
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

        cricket::jit::CompileOptions compile_options;
    };

    auto default_load_options() -> LoadOptions;

    struct RobotOps
    {
        struct PlannerEntry
        {
            ffi::SolveFn solve{nullptr};
            ffi::SolveMultiFn solve_multi{nullptr};
        };

        std::array<PlannerEntry, vamp::planning::N_PLANNERS> planners{};

        ffi::ResultMetaFn result_meta{nullptr};
        ffi::ResultCopyWaypointFn result_copy_waypoint{nullptr};
        ffi::ResultDestroyFn result_destroy{nullptr};
        ffi::ResultSizesFn result_sizes{nullptr};

        ffi::SimplifyFn simplify{nullptr};

        ffi::SamplerHaltonFn sampler_halton{nullptr};
        ffi::SamplerXorshiftFn sampler_xorshift{nullptr};
        ffi::SamplerResetFn sampler_reset{nullptr};
        ffi::SamplerSkipFn sampler_skip{nullptr};
        ffi::SamplerNextFn sampler_next{nullptr};
        ffi::SamplerDestroyFn sampler_destroy{nullptr};

        ffi::DebugFn debug{nullptr};
        ffi::DebugDestroyFn debug_destroy{nullptr};
        ffi::EefkFn eefk{nullptr};
        ffi::FkFn fk{nullptr};
        ffi::ValidateFn validate{nullptr};
        ffi::ValidateMotionFn validate_motion{nullptr};
        ffi::FilterPointcloudFn filter_pointcloud{nullptr};

        ffi::PhsNewFn phs_new{nullptr};
        ffi::PhsDestroyFn phs_destroy{nullptr};
        ffi::PhsSetDiameterFn phs_set_diameter{nullptr};
        ffi::PhsTransformFn phs_transform{nullptr};
        ffi::SamplerPhsFn sampler_phs{nullptr};
    };

    class DynamicRobot
    {
    public:
        explicit DynamicRobot(
            const LoadOptions &opts,
            std::shared_ptr<cricket::jit::DiskObjectCache> cache = nullptr);
        ~DynamicRobot();

        DynamicRobot(const DynamicRobot &) = delete;
        DynamicRobot &operator=(const DynamicRobot &) = delete;

        auto ops() const -> const RobotOps &
        {
            return ops_;
        }

        auto has_planner(vamp::planning::Planner p) const -> bool
        {
            return ops_.planners[static_cast<std::size_t>(p)].solve != nullptr;
        }

        auto dimension() const -> std::size_t
        {
            return dimension_;
        }

        auto rake() const -> std::size_t
        {
            return rake_;
        }

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

    private:
        std::size_t dimension_;
        std::size_t rake_;
        std::unique_ptr<cricket::jit::JitSession> session_;
        RobotOps ops_{};

        std::size_t n_spheres_{0};
        float space_measure_{0.0F};
        float min_radius_{0.0F};
        float max_radius_{0.0F};
        std::vector<std::string> joint_names_;
        std::vector<float> upper_bounds_;
        std::vector<float> lower_bounds_;
    };
}  // namespace vamp::jit
