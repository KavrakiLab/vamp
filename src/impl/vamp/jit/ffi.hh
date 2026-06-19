#pragma once

#include <cstddef>
#include <cstdint>

namespace vamp::jit::ffi
{
    struct PlanResultHandle;
    struct SamplerHandle;
    struct DebugHandle;

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
}  // namespace vamp::jit::ffi
