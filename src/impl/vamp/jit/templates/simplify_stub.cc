// Per-robot simplify stub. Wraps vamp::planning::simplify<R, rake, resolution>
// behind an extern "C" entrypoint with the same shape as a planner solve.
// Returns a PlanResultHandle so the host can reuse the per-planner result
// accessors (any planner's would work — same underlying PlanningResult<R>),
// or call the dedicated simplify result symbols below.
//
// inja substitutions:
//   {{robot_name}}      — struct name inside vamp::robots::
//   {{rake}}            — SIMD lane width
//   {{resolution}}      — collision-check substep count
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>

namespace vamp_jit_simplify
{
    using R = vamp_jit_robot::R;
    using ResultT = vamp::planning::PlanningResult<R>;

    struct WrappedResult
    {
        ResultT inner;
    };
}  // namespace vamp_jit_simplify

extern "C" vamp::jit::ffi::PlanResultHandle *vamp_jit_{{robot_name}}_simplify(
    const float *path_ptr,
    std::uint64_t n_waypoints,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using R = vamp_jit_robot::R;
    vamp::planning::Path<R> path;
    path.reserve(n_waypoints);
    for (std::uint64_t i = 0; i < n_waypoints; ++i)
    {
        path.emplace_back(vamp_jit_robot::load_config(path_ptr + i * R::dimension));
    }

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<{{rake}}>> env_rake(*env_in);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new vamp_jit_simplify::WrappedResult{
        vamp::planning::simplify<R, {{rake}}, {{resolution}}>(path, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultMeta vamp_jit_{{robot_name}}_simplify_result_meta(
    const vamp::jit::ffi::PlanResultHandle *h)
{
    const auto *w = reinterpret_cast<const vamp_jit_simplify::WrappedResult *>(h);
    vamp::jit::ffi::PlanResultMeta m{};
    m.success = w->inner.path.empty() ? 0 : 1;
    m.dimension = vamp_jit_robot::R::dimension;
    m.waypoints = w->inner.path.size();
    m.nanoseconds = w->inner.nanoseconds;
    m.iterations = w->inner.iterations;
    m.cost = w->inner.path.cost();
    return m;
}

extern "C" void vamp_jit_{{robot_name}}_simplify_result_copy_waypoint(
    const vamp::jit::ffi::PlanResultHandle *h,
    std::uint64_t idx,
    float *out)
{
    const auto *w = reinterpret_cast<const vamp_jit_simplify::WrappedResult *>(h);
    auto arr = w->inner.path[idx].to_array();
    std::memcpy(out, arr.data(), vamp_jit_robot::R::dimension * sizeof(float));
}

extern "C" void vamp_jit_{{robot_name}}_simplify_result_destroy(vamp::jit::ffi::PlanResultHandle *h)
{
    delete reinterpret_cast<vamp_jit_simplify::WrappedResult *>(h);
}
