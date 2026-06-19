#pragma once

#include <cstddef>
#include <cstdint>

// vamp-specific FFI types for the JIT boundary: opaque planner-result and
// sampler handles, plain-POD metadata view, and the function-pointer
// signatures the host calls into.
//
// All POD with stable layout so the host and the JIT'd .o agree on them.

namespace vamp::jit::ffi
{
    // Opaque planner-result handle. The JIT'd module owns the underlying
    // (templated) result object and exposes it as an opaque pointer here.
    struct PlanResultHandle;

    // Opaque sampler handle. The JIT'd module owns the underlying
    // RNG<Robot>::Ptr and exposes it as an opaque pointer here. Lifetime is
    // managed via the sampler factory / destroy entrypoints below; the host
    // shares it across solve / simplify calls just like in the static API.
    struct SamplerHandle;

    // Opaque debug-result handle. Internally a heap-allocated R::Debug
    // (which is `std::pair<vector<vector<string>>, vector<pair<size_t,
    // size_t>>>` — same type across robots, so the host can interpret it
    // by the type alias below without naming R).
    struct DebugHandle;

    // Flat metadata the host can read without knowing the templated result type.
    struct PlanResultMeta
    {
        std::int32_t success;       // 0 = no path, non-zero = found
        std::uint32_t dimension;    // robot config dimension
        std::uint64_t waypoints;    // number of waypoints in path
        std::uint64_t nanoseconds;  // planner wall time
        std::uint64_t iterations;   // planner iterations
        float cost;
    };

    // Single-goal solve. Every per-planner JIT'd module exports this shape
    // under a planner-specific symbol (e.g. vamp_jit_rrtc_solve).
    //
    //   start, goal      — flat float[dimension]; dimension is known at JIT time
    //   env_ptr          — vamp::collision::Environment<float>* (host owns)
    //   settings_ptr     — pointer to the planner's settings struct (host owns)
    //   sampler          — SamplerHandle from one of the sampler factories
    using SolveFn = PlanResultHandle *(*)(const float *start,
                                          const float *goal,
                                          const void *env_ptr,
                                          const void *settings_ptr,
                                          SamplerHandle *sampler);

    // Multi-goal solve. goals_ptr is a contiguous float[n_goals * dimension].
    using SolveMultiFn = PlanResultHandle *(*)(const float *start,
                                               const float *goals_ptr,
                                               std::uint64_t n_goals,
                                               const void *env_ptr,
                                               const void *settings_ptr,
                                               SamplerHandle *sampler);

    // Path simplification. path_ptr is a contiguous float[n_waypoints * dimension].
    using SimplifyFn = PlanResultHandle *(*)(const float *path_ptr,
                                             std::uint64_t n_waypoints,
                                             const void *env_ptr,
                                             const void *settings_ptr,
                                             SamplerHandle *sampler);

    // Result accessors. Each JIT'd module exports the same shape under
    // module-suffixed names so multiple robots can coexist in one process.
    using ResultMetaFn = PlanResultMeta (*)(const PlanResultHandle *);
    using ResultCopyWaypointFn = void (*)(const PlanResultHandle *, std::uint64_t idx, float *out);
    using ResultDestroyFn = void (*)(PlanResultHandle *);

    // Sampler factories + ops. Halton takes no seed (deterministic sequence);
    // XORShift takes a seed (0 == default). reset/skip/next mirror the static
    // Python RNG bindings; destroy releases the heap-allocated handle.
    using SamplerHaltonFn = SamplerHandle *(*)();
    using SamplerXorshiftFn = SamplerHandle *(*)(std::uint64_t seed);
    using SamplerResetFn = void (*)(SamplerHandle *);
    using SamplerSkipFn = void (*)(SamplerHandle *, std::uint64_t n);
    using SamplerNextFn = void (*)(SamplerHandle *, float *out);
    using SamplerDestroyFn = void (*)(SamplerHandle *);

    // Per-robot debug entrypoint. Mirrors vamp.<robot>.debug(): returns
    // per-sphere environment-collision descriptions and a list of
    // self-colliding sphere pairs.
    using DebugFn = DebugHandle *(*)(const float *config, const void *env_ptr);
    using DebugDestroyFn = void (*)(DebugHandle *);
}  // namespace vamp::jit::ffi
