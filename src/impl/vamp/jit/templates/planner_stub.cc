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
// clang-format on

#include <vector>

namespace VAMP_JIT_PLANNER_NS
{
    using R = vamp_jit_robot::R;
    using PlannerT = VAMP_JIT_PLANNER_CLASS<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>;
    using SettingsT = VAMP_JIT_SETTINGS_CLASS;
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

    auto *wrapped = new vamp_jit_robot::WrappedResult{PlannerT::solve(start, goal, env_rake, settings, rng)};
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

    auto *wrapped = new vamp_jit_robot::WrappedResult{PlannerT::solve(start, goals, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}