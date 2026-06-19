// Per-robot sampler stub + shared per-robot helpers.
//
// Type-erases vamp::rng::RNG<R>::Ptr behind vamp::jit::ffi::SamplerHandle so
// the host can construct, share, and destroy samplers across solve/simplify
// calls without naming the robot's RNG template instantiation.
//
// Also exposes the per-robot helpers (R, SamplerPtr, load_config, deref) that
// planner_stub.cc and simplify_stub.cc use — they're all in one TU, so a
// shared named namespace beats redeclaring inside each planner namespace.
//
// inja substitutions:
//   {{robot_name}} — struct name inside vamp::robots::

namespace vamp_jit_robot
{
    using R = vamp::robots::{{robot_name}};
    using SamplerPtr = typename vamp::rng::RNG<R>::Ptr;
    // ResultT (vamp::planning::PlanningResult<R>) is NOT declared here:
    // sampler_stub.cc is rendered before any vamp/planning/* header is
    // included, so we'd reference an unknown namespace. Per-planner and
    // simplify stubs declare it themselves.

    // Sized-array copy avoids the rounded-size Vector ctor OOB-reading the
    // caller's dimension-sized buffer.
    inline auto load_config(const float *data) -> typename R::Configuration
    {
        std::array<float, R::dimension> arr;
        std::memcpy(arr.data(), data, R::dimension * sizeof(float));
        return typename R::Configuration(arr);
    }

    inline auto deref_sampler(vamp::jit::ffi::SamplerHandle *h) -> SamplerPtr &
    {
        return *reinterpret_cast<SamplerPtr *>(h);
    }
}  // namespace vamp_jit_robot

extern "C" vamp::jit::ffi::SamplerHandle *vamp_jit_{{robot_name}}_sampler_halton()
{
    auto *p = new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::Halton<vamp_jit_robot::R>>());
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" vamp::jit::ffi::SamplerHandle *vamp_jit_{{robot_name}}_sampler_xorshift(std::uint64_t seed)
{
    // XORShift's two-key ctor needs both non-zero. Seed 0 → use class default.
    auto *p =
        (seed == 0)
            ? new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>())
            : new vamp_jit_robot::SamplerPtr(
                  std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>(seed, seed + 1));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" void vamp_jit_{{robot_name}}_sampler_reset(vamp::jit::ffi::SamplerHandle *h)
{
    vamp_jit_robot::deref_sampler(h)->reset();
}

extern "C" void vamp_jit_{{robot_name}}_sampler_skip(vamp::jit::ffi::SamplerHandle *h, std::uint64_t n)
{
    auto &rng = vamp_jit_robot::deref_sampler(h);
    for (std::uint64_t i = 0; i < n; ++i)
    {
        rng->next();
    }
}

extern "C" void vamp_jit_{{robot_name}}_sampler_next(vamp::jit::ffi::SamplerHandle *h, float *out)
{
    auto arr = vamp_jit_robot::deref_sampler(h)->next().to_array();
    std::memcpy(out, arr.data(), vamp_jit_robot::R::dimension * sizeof(float));
}

extern "C" void vamp_jit_{{robot_name}}_sampler_destroy(vamp::jit::ffi::SamplerHandle *h)
{
    delete reinterpret_cast<vamp_jit_robot::SamplerPtr *>(h);
}
