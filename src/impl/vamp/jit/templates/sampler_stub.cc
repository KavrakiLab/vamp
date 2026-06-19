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

// clang-format off
// All inja placeholders live in this block so clang-format never reflows
// them. The rest of the file uses these aliases / macros so it can be
// formatted normally.
namespace vamp_jit_robot { using R = vamp::robots::{{robot_name}}; }

#define VAMP_JIT_SAMPLER_HALTON   vamp_jit_{{robot_name}}_sampler_halton
#define VAMP_JIT_SAMPLER_XORSHIFT vamp_jit_{{robot_name}}_sampler_xorshift
#define VAMP_JIT_SAMPLER_RESET    vamp_jit_{{robot_name}}_sampler_reset
#define VAMP_JIT_SAMPLER_SKIP     vamp_jit_{{robot_name}}_sampler_skip
#define VAMP_JIT_SAMPLER_NEXT     vamp_jit_{{robot_name}}_sampler_next
#define VAMP_JIT_SAMPLER_DESTROY  vamp_jit_{{robot_name}}_sampler_destroy
// clang-format on

namespace vamp_jit_robot
{
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

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_SAMPLER_HALTON()
{
    auto *p = new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::Halton<vamp_jit_robot::R>>());
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_SAMPLER_XORSHIFT(std::uint64_t seed)
{
    // XORShift's two-key ctor needs both non-zero. Seed 0 → use class default.
    auto *p = (seed == 0) ?
                  new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>()) :
                  new vamp_jit_robot::SamplerPtr(
                      std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>(seed, seed + 1));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" void VAMP_JIT_SAMPLER_RESET(vamp::jit::ffi::SamplerHandle *h)
{
    vamp_jit_robot::deref_sampler(h)->reset();
}

extern "C" void VAMP_JIT_SAMPLER_SKIP(vamp::jit::ffi::SamplerHandle *h, std::uint64_t n)
{
    auto &rng = vamp_jit_robot::deref_sampler(h);
    for (std::uint64_t i = 0; i < n; ++i)
    {
        rng->next();
    }
}

extern "C" void VAMP_JIT_SAMPLER_NEXT(vamp::jit::ffi::SamplerHandle *h, float *out)
{
    auto arr = vamp_jit_robot::deref_sampler(h)->next().to_array();
    std::memcpy(out, arr.data(), vamp_jit_robot::R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_SAMPLER_DESTROY(vamp::jit::ffi::SamplerHandle *h)
{
    delete reinterpret_cast<vamp_jit_robot::SamplerPtr *>(h);
}

#undef VAMP_JIT_SAMPLER_HALTON
#undef VAMP_JIT_SAMPLER_XORSHIFT
#undef VAMP_JIT_SAMPLER_RESET
#undef VAMP_JIT_SAMPLER_SKIP
#undef VAMP_JIT_SAMPLER_NEXT
#undef VAMP_JIT_SAMPLER_DESTROY
