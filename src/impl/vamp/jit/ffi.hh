#pragma once

#include <cstddef>
#include <cstdint>

namespace vamp::jit::ffi
{
    struct PlanResultHandle;
    struct SamplerHandle;
    struct DebugHandle;
    struct PhsHandle;

    struct PlanResultMeta
    {
        std::int32_t success;
        std::uint32_t dimension;
        std::uint64_t waypoints;
        std::uint64_t nanoseconds;
        std::uint64_t iterations;
        float cost;
    };

    using SolveFn = PlanResultHandle *(*)(const float *start,
                                          const float *goal,
                                          const void *env_ptr,
                                          const void *settings_ptr,
                                          SamplerHandle *sampler);

    using SolveMultiFn = PlanResultHandle *(*)(const float *start,
                                               const float *goals_ptr,
                                               std::uint64_t n_goals,
                                               const void *env_ptr,
                                               const void *settings_ptr,
                                               SamplerHandle *sampler);

    using SimplifyFn = PlanResultHandle *(*)(const float *path_ptr,
                                             std::uint64_t n_waypoints,
                                             const void *env_ptr,
                                             const void *settings_ptr,
                                             SamplerHandle *sampler);

    using ResultMetaFn = PlanResultMeta (*)(const PlanResultHandle *);
    using ResultCopyWaypointFn = void (*)(const PlanResultHandle *, std::uint64_t idx, float *out);
    using ResultDestroyFn = void (*)(PlanResultHandle *);

    using SamplerHaltonFn = SamplerHandle *(*)();
    using SamplerXorshiftFn = SamplerHandle *(*)(std::uint64_t seed);
    using SamplerResetFn = void (*)(SamplerHandle *);
    using SamplerSkipFn = void (*)(SamplerHandle *, std::uint64_t n);
    using SamplerNextFn = void (*)(SamplerHandle *, float *out);
    using SamplerDestroyFn = void (*)(SamplerHandle *);

    using DebugFn = DebugHandle *(*)(const float *config, const void *env_ptr);
    using DebugDestroyFn = void (*)(DebugHandle *);

    using EefkFn = void (*)(const float *config, float *out_matrix);
    using FkFn = void (*)(const float *config, float *out_spheres);
    using ValidateFn = std::int32_t (*)(const float *config, const void *env_ptr, std::int32_t check_bounds);
    using ValidateMotionFn = std::int32_t (
            *)(const float *c_in, const float *c_out, const void *env_ptr, std::int32_t check_bounds);

    using FilterPointcloudFn = void (*)(
        const float *points_in,
        std::uint64_t n_points,
        float point_radius,
        const float *config,
        const void *env_ptr,
        void *out_filtered);

    using SpaceMeasureFn = float (*)();
    using MinMaxRadiiFn = void (*)(float *out_min, float *out_max);
    using NSpheresFn = std::uint64_t (*)();
    using BoundsFn = void (*)(float *out);
    using JointNamesFn = void (*)(void *out_strings);

    using PhsNewFn = PhsHandle *(*)(const float *focus_a, const float *focus_b);
    using PhsDestroyFn = void (*)(PhsHandle *);
    using PhsSetDiameterFn = void (*)(PhsHandle *, float diameter);
    using PhsTransformFn = void (*)(const PhsHandle *, const float *in, float *out);
    using SamplerPhsFn = SamplerHandle *(*)(const PhsHandle *, SamplerHandle *inner);
}  // namespace vamp::jit::ffi
