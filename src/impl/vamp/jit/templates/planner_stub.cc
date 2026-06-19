// Generic per-planner stub. Instantiates the planner class for
// vamp::robots::{{robot_name}} and exposes single- and multi-goal extern "C"
// entrypoints matching the vamp::jit::ffi shape.
//
// Multiple planners coexist in the same JIT'd TU; per-planner state lives in
// a uniquely-named namespace (VAMP_JIT_PLANNER_NS) to avoid clashing on R /
// WrappedResult / etc. Per-robot helpers (R, load_config, deref_sampler) are
// defined once in vamp_jit_robot (see sampler_stub.cc).
//
// inja substitutions (handled by vamp::jit::generate_stub_source):
//   {{robot_name}}      — struct name inside vamp::robots::
//   {{rake}}            — SIMD lane width (compile-time constant)
//   {{resolution}}      — collision-check substep count
//   {{planner_name}}    — lowercase, used in the extern "C" symbol prefix
//                         and the per-planner namespace name
//   {{planner_class}}   — vamp::planning template class
//   {{settings_class}}  — settings struct/typedef
//   {{planner_header}}  — relative path under vamp/planning/
//   {{settings_header}} — relative path under vamp/planning/

// clang-format off
#include <vamp/planning/{{planner_header}}>
#include <vamp/planning/{{settings_header}}>

#define VAMP_JIT_PLANNER_NS         vamp_jit_{{planner_name}}
#define VAMP_JIT_PLANNER_CLASS      vamp::planning::{{planner_class}}
#define VAMP_JIT_SETTINGS_CLASS     vamp::planning::{{settings_class}}
#define VAMP_JIT_RAKE               {{rake}}
#define VAMP_JIT_RESOLUTION         {{resolution}}

#define VAMP_JIT_FN_SOLVE           vamp_jit_{{planner_name}}_solve
#define VAMP_JIT_FN_SOLVE_MULTI     vamp_jit_{{planner_name}}_solve_multi
#define VAMP_JIT_FN_RESULT_META     vamp_jit_{{planner_name}}_result_meta
#define VAMP_JIT_FN_RESULT_COPY     vamp_jit_{{planner_name}}_result_copy_waypoint
#define VAMP_JIT_FN_RESULT_DESTROY  vamp_jit_{{planner_name}}_result_destroy
// clang-format on

#include <vector>

namespace VAMP_JIT_PLANNER_NS
{
    using R = vamp_jit_robot::R;
    using ResultT = vamp::planning::PlanningResult<R>;
    using PlannerT = VAMP_JIT_PLANNER_CLASS<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>;
    using SettingsT = VAMP_JIT_SETTINGS_CLASS;

    struct WrappedResult
    {
        ResultT inner;
    };
}  // namespace VAMP_JIT_PLANNER_NS

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE(
    const float *start_ptr,
    const float *goal_ptr,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = vamp_jit_robot::load_config(start_ptr);
    auto goal = vamp_jit_robot::load_config(goal_ptr);

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new WrappedResult{PlannerT::solve(start, goal, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_MULTI(
    const float *start_ptr,
    const float *goals_ptr,
    std::uint64_t n_goals,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = vamp_jit_robot::load_config(start_ptr);
    std::vector<typename R::Configuration> goals;
    goals.reserve(n_goals);
    for (std::uint64_t i = 0; i < n_goals; ++i)
    {
        goals.emplace_back(vamp_jit_robot::load_config(goals_ptr + i * R::dimension));
    }

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new WrappedResult{PlannerT::solve(start, goals, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultMeta VAMP_JIT_FN_RESULT_META(const vamp::jit::ffi::PlanResultHandle *h)
{
    const auto *w = reinterpret_cast<const VAMP_JIT_PLANNER_NS::WrappedResult *>(h);
    vamp::jit::ffi::PlanResultMeta m{};
    m.success = w->inner.path.empty() ? 0 : 1;
    m.dimension = VAMP_JIT_PLANNER_NS::R::dimension;
    m.waypoints = w->inner.path.size();
    m.nanoseconds = w->inner.nanoseconds;
    m.iterations = w->inner.iterations;
    m.cost = w->inner.path.cost();
    return m;
}

extern "C" void
VAMP_JIT_FN_RESULT_COPY(const vamp::jit::ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
{
    const auto *w = reinterpret_cast<const VAMP_JIT_PLANNER_NS::WrappedResult *>(h);
    auto arr = w->inner.path[idx].to_array();
    std::memcpy(out, arr.data(), VAMP_JIT_PLANNER_NS::R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_RESULT_DESTROY(vamp::jit::ffi::PlanResultHandle *h)
{
    delete reinterpret_cast<VAMP_JIT_PLANNER_NS::WrappedResult *>(h);
}

#undef VAMP_JIT_PLANNER_NS
#undef VAMP_JIT_PLANNER_CLASS
#undef VAMP_JIT_SETTINGS_CLASS
#undef VAMP_JIT_RAKE
#undef VAMP_JIT_RESOLUTION
#undef VAMP_JIT_FN_SOLVE
#undef VAMP_JIT_FN_SOLVE_MULTI
#undef VAMP_JIT_FN_RESULT_META
#undef VAMP_JIT_FN_RESULT_COPY
#undef VAMP_JIT_FN_RESULT_DESTROY
