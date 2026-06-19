// Generic per-planner stub. Instantiates
//   vamp::planning::{{planner_class}}<R, {{rake}}, {{resolution}}>
// for vamp::robots::{{robot_name}} and exposes single- and multi-goal
// extern "C" entrypoints matching the vamp::jit::ffi shape.
//
// Multiple planners coexist in the same JIT'd TU; per-planner state lives
// in named namespace vamp_jit_{{planner_name}} to avoid clashing on R /
// WrappedResult / etc. Per-robot helpers (R, load_config, deref_sampler)
// are defined once in vamp_jit_robot (see sampler_stub.cc).
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
#include <vamp/planning/{{planner_header}}>
#include <vamp/planning/{{settings_header}}>

#include <vector>

namespace vamp_jit_{{planner_name}}
{
    using R = vamp_jit_robot::R;
    using ResultT = vamp::planning::PlanningResult<R>;
    using PlannerT = vamp::planning::{{planner_class}}<R, {{rake}}, {{resolution}}>;
    using SettingsT = vamp::planning::{{settings_class}};

    struct WrappedResult
    {
        ResultT inner;
    };
}  // namespace vamp_jit_{{planner_name}}

extern "C" vamp::jit::ffi::PlanResultHandle *vamp_jit_{{planner_name}}_solve(
    const float *start_ptr,
    const float *goal_ptr,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using namespace vamp_jit_{{planner_name}};

    auto start = vamp_jit_robot::load_config(start_ptr);
    auto goal = vamp_jit_robot::load_config(goal_ptr);

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<{{rake}}>> env_rake(*env_in);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new WrappedResult{PlannerT::solve(start, goal, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultHandle *vamp_jit_{{planner_name}}_solve_multi(
    const float *start_ptr,
    const float *goals_ptr,
    std::uint64_t n_goals,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using namespace vamp_jit_{{planner_name}};

    auto start = vamp_jit_robot::load_config(start_ptr);
    std::vector<typename R::Configuration> goals;
    goals.reserve(n_goals);
    for (std::uint64_t i = 0; i < n_goals; ++i)
    {
        goals.emplace_back(vamp_jit_robot::load_config(goals_ptr + i * R::dimension));
    }

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<{{rake}}>> env_rake(*env_in);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new WrappedResult{PlannerT::solve(start, goals, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultMeta vamp_jit_{{planner_name}}_result_meta(
    const vamp::jit::ffi::PlanResultHandle *h)
{
    const auto *w = reinterpret_cast<const vamp_jit_{{planner_name}}::WrappedResult *>(h);
    vamp::jit::ffi::PlanResultMeta m{};
    m.success = w->inner.path.empty() ? 0 : 1;
    m.dimension = vamp_jit_{{planner_name}}::R::dimension;
    m.waypoints = w->inner.path.size();
    m.nanoseconds = w->inner.nanoseconds;
    m.iterations = w->inner.iterations;
    m.cost = w->inner.path.cost();
    return m;
}

extern "C" void vamp_jit_{{planner_name}}_result_copy_waypoint(
    const vamp::jit::ffi::PlanResultHandle *h,
    std::uint64_t idx,
    float *out)
{
    const auto *w = reinterpret_cast<const vamp_jit_{{planner_name}}::WrappedResult *>(h);
    auto arr = w->inner.path[idx].to_array();
    std::memcpy(out, arr.data(), vamp_jit_{{planner_name}}::R::dimension * sizeof(float));
}

extern "C" void vamp_jit_{{planner_name}}_result_destroy(vamp::jit::ffi::PlanResultHandle *h)
{
    delete reinterpret_cast<vamp_jit_{{planner_name}}::WrappedResult *>(h);
}
