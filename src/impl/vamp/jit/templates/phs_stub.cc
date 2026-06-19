// Per-robot PHS stub. Wraps vamp::planning::ProlateHyperspheroid<R> behind
// the vamp::jit::ffi::PhsHandle opaque pointer, plus a sampler_phs factory
// that constructs a ProlateHyperspheroidRNG<R> wrapping an inner sampler.
//
// inja substitutions:
//   {{robot_name}} — struct name inside vamp::robots::

#include <vamp/planning/phs.hh>

// clang-format off
#define VAMP_JIT_PHS_NEW           vamp_jit_{{robot_name}}_phs_new
#define VAMP_JIT_PHS_DESTROY       vamp_jit_{{robot_name}}_phs_destroy
#define VAMP_JIT_PHS_SET_DIAMETER  vamp_jit_{{robot_name}}_phs_set_transverse_diameter
#define VAMP_JIT_PHS_TRANSFORM     vamp_jit_{{robot_name}}_phs_transform
#define VAMP_JIT_SAMPLER_PHS       vamp_jit_{{robot_name}}_sampler_phs
// clang-format on

extern "C" vamp::jit::ffi::PhsHandle *VAMP_JIT_PHS_NEW(const float *focus_a, const float *focus_b)
{
    using R = vamp_jit_robot::R;
    auto fa = vamp_jit_robot::load_config(focus_a);
    auto fb = vamp_jit_robot::load_config(focus_b);
    auto *phs = new vamp::planning::ProlateHyperspheroid<R>(fa, fb);
    return reinterpret_cast<vamp::jit::ffi::PhsHandle *>(phs);
}

extern "C" void VAMP_JIT_PHS_DESTROY(vamp::jit::ffi::PhsHandle *h)
{
    delete reinterpret_cast<vamp::planning::ProlateHyperspheroid<vamp_jit_robot::R> *>(h);
}

extern "C" void VAMP_JIT_PHS_SET_DIAMETER(vamp::jit::ffi::PhsHandle *h, float diameter)
{
    reinterpret_cast<vamp::planning::ProlateHyperspheroid<vamp_jit_robot::R> *>(h)
        ->set_transverse_diameter(diameter);
}

extern "C" void VAMP_JIT_PHS_TRANSFORM(const vamp::jit::ffi::PhsHandle *h, const float *in, float *out)
{
    using R = vamp_jit_robot::R;
    auto in_v = vamp_jit_robot::load_config(in);
    auto out_v =
        reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(h)->transform(in_v);
    auto arr = out_v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_SAMPLER_PHS(
    const vamp::jit::ffi::PhsHandle *phs_h,
    vamp::jit::ffi::SamplerHandle *inner_h)
{
    using R = vamp_jit_robot::R;
    // ProlateHyperspheroidRNG takes phs by value; copy out of the opaque
    // handle. inner_h is the underlying RNG::Ptr (sampler_stub's holder).
    auto phs = *reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(phs_h);
    auto inner_rng = vamp_jit_robot::deref_sampler(inner_h);
    auto *holder = new vamp_jit_robot::SamplerPtr(
        std::make_shared<vamp::planning::ProlateHyperspheroidRNG<R>>(phs, inner_rng));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(holder);
}

#undef VAMP_JIT_PHS_NEW
#undef VAMP_JIT_PHS_DESTROY
#undef VAMP_JIT_PHS_SET_DIAMETER
#undef VAMP_JIT_PHS_TRANSFORM
#undef VAMP_JIT_SAMPLER_PHS
