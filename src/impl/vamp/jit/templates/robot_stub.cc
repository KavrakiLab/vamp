#include <vamp/collision/sphere_sphere.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>

// clang-format off
namespace vamp_jit_robot { using R = vamp::robots::{{robot_name}}; }
#define VAMP_JIT_RAKE       {{rake}}
#define VAMP_JIT_RESOLUTION {{resolution}}
// clang-format on

namespace vamp_jit_robot
{
    using SamplerPtr = typename vamp::rng::RNG<R>::Ptr;

    // Shared result wrapper. The same struct is returned by every planner's
    // solve / solve_multi and by simplify — PlanningResult<R> doesn't depend
    // on the planner, so all of them can share a single (robot-scoped) set of
    // meta / copy / destroy functions instead of minting one per planner.
    struct WrappedResult
    {
        vamp::planning::PlanningResult<R> inner;
    };

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

// ===== Shared result helpers ==============================================

extern "C" vamp::jit::ffi::PlanResultMeta
vamp_jit_{{robot_name}}_result_meta(const vamp::jit::ffi::PlanResultHandle *h)
{
    const auto *w = reinterpret_cast<const vamp_jit_robot::WrappedResult *>(h);
    vamp::jit::ffi::PlanResultMeta m{};
    m.success = w->inner.path.empty() ? 0 : 1;
    m.dimension = vamp_jit_robot::R::dimension;
    m.waypoints = w->inner.path.size();
    m.nanoseconds = w->inner.nanoseconds;
    m.iterations = w->inner.iterations;
    m.cost = w->inner.path.cost();
    return m;
}

extern "C" void vamp_jit_{{robot_name}}_result_copy_waypoint(
    const vamp::jit::ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
{
    const auto *w = reinterpret_cast<const vamp_jit_robot::WrappedResult *>(h);
    auto arr = w->inner.path[idx].to_array();
    std::memcpy(out, arr.data(), vamp_jit_robot::R::dimension * sizeof(float));
}

extern "C" void vamp_jit_{{robot_name}}_result_destroy(vamp::jit::ffi::PlanResultHandle *h)
{
    delete reinterpret_cast<vamp_jit_robot::WrappedResult *>(h);
}

// ===== Sampler ============================================================

extern "C" vamp::jit::ffi::SamplerHandle *vamp_jit_{{robot_name}}_sampler_halton()
{
    auto *p = new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::Halton<vamp_jit_robot::R>>());
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" vamp::jit::ffi::SamplerHandle *vamp_jit_{{robot_name}}_sampler_xorshift(std::uint64_t seed)
{
    // XORShift's two-key ctor needs both non-zero. Seed 0 → use class default.
    auto *p = (seed == 0) ?
                  new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>()) :
                  new vamp_jit_robot::SamplerPtr(
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

// ===== Simplify ===========================================================

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
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new vamp_jit_robot::WrappedResult{
        vamp::planning::simplify<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>(path, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

// ===== Debug / introspection ==============================================

extern "C" vamp::jit::ffi::DebugHandle *
vamp_jit_{{robot_name}}_debug(const float *config, const void *env_ptr)
{
    using R = vamp_jit_robot::R;

    typename R::template ConfigurationBlock<VAMP_JIT_RAKE> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    auto *result = new typename R::Debug(R::template fkcc_debug<VAMP_JIT_RAKE>(env_rake, block));
    return reinterpret_cast<vamp::jit::ffi::DebugHandle *>(result);
}

extern "C" void vamp_jit_{{robot_name}}_debug_destroy(vamp::jit::ffi::DebugHandle *h)
{
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
    delete reinterpret_cast<DebugType *>(h);
}

extern "C" void vamp_jit_{{robot_name}}_eefk(const float *config, float *out_matrix)
{
    using R = vamp_jit_robot::R;
    typename R::ConfigurationArray cfg;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        cfg[i] = config[i];
    }

    Eigen::Matrix4f mat = R::eefk(cfg).matrix();
    std::memcpy(out_matrix, mat.data(), 16 * sizeof(float));
}

extern "C" void vamp_jit_{{robot_name}}_fk(const float *config, float *out_spheres)
{
    using R = vamp_jit_robot::R;
    typename R::template ConfigurationBlock<1> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }

    typename R::template Spheres<1> out;
    R::template sphere_fk<1>(block, out);

    for (std::size_t i = 0; i < R::n_spheres; ++i)
    {
        out_spheres[i * 4 + 0] = out.x[{i, 0}];
        out_spheres[i * 4 + 1] = out.y[{i, 0}];
        out_spheres[i * 4 + 2] = out.z[{i, 0}];
        out_spheres[i * 4 + 3] = out.r[{i, 0}];
    }
}

extern "C" std::int32_t
vamp_jit_{{robot_name}}_validate(const float *config_ptr, const void *env_ptr, std::int32_t check_bounds)
{
    using R = vamp_jit_robot::R;

    auto configuration = vamp_jit_robot::load_config(config_ptr);
    auto copy = configuration.trim();
    R::descale_configuration(copy);
    const bool in_bounds = (copy <= 1.F).all() and (copy >= 0.F).all();

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    return (not check_bounds or in_bounds) and
           vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(configuration, configuration, env_rake);
}

extern "C" std::int32_t vamp_jit_{{robot_name}}_validate_motion(
    const float *c_in_ptr, const float *c_out_ptr, const void *env_ptr, std::int32_t check_bounds)
{
    using R = vamp_jit_robot::R;

    auto c_in = vamp_jit_robot::load_config(c_in_ptr);
    auto copy_in = c_in.trim();
    R::descale_configuration(copy_in);
    const bool in_bounds_in = (copy_in <= 1.F).all() and (copy_in >= 0.F).all();

    auto c_out = vamp_jit_robot::load_config(c_out_ptr);
    auto copy_out = c_out.trim();
    R::descale_configuration(copy_out);
    const bool in_bounds_out = (copy_out <= 1.F).all() and (copy_out >= 0.F).all();

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    return (not check_bounds or (in_bounds_in and in_bounds_out)) and
           vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(c_in, c_out, env_rake);
}

extern "C" void vamp_jit_{{robot_name}}_filter_self_from_pointcloud(
    const float *points_in,
    std::uint64_t n_points,
    float point_radius,
    const float *config,
    const void *env_ptr,
    void *out_filtered_vec)
{
    using R = vamp_jit_robot::R;

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    typename R::template ConfigurationBlock<1> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }

    typename R::template Spheres<1> out;
    R::template sphere_fk<1>(block, out);

    auto *filtered = static_cast<std::vector<vamp::collision::Point> *>(out_filtered_vec);
    filtered->clear();
    filtered->reserve(n_points);

    for (std::uint64_t p = 0; p < n_points; ++p)
    {
        const float x = points_in[p * 3 + 0];
        const float y = points_in[p * 3 + 1];
        const float z = points_in[p * 3 + 2];

        bool valid = true;
        for (std::size_t i = 0; i < R::n_spheres; ++i)
        {
            if (vamp::collision::sphere_sphere_sql2(
                    out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}], x, y, z, point_radius) < 0 or
                vamp::sphere_environment_in_collision<>(env_rake, x, y, z, point_radius))
            {
                valid = false;
                break;
            }
        }

        if (valid)
        {
            filtered->emplace_back(vamp::collision::Point{x, y, z});
        }
    }
}

extern "C" float vamp_jit_{{robot_name}}_space_measure()
{
    return vamp_jit_robot::R::space_measure();
}

extern "C" void vamp_jit_{{robot_name}}_min_max_radii(float *out_min, float *out_max)
{
    *out_min = vamp_jit_robot::R::min_radius;
    *out_max = vamp_jit_robot::R::max_radius;
}

extern "C" std::uint64_t vamp_jit_{{robot_name}}_n_spheres()
{
    return vamp_jit_robot::R::n_spheres;
}

extern "C" void vamp_jit_{{robot_name}}_joint_names(void *out_strings)
{
    auto *v = static_cast<std::vector<std::string> *>(out_strings);
    v->clear();
    for (auto sv : vamp_jit_robot::R::joint_names)
    {
        v->emplace_back(sv);
    }
}

extern "C" void vamp_jit_{{robot_name}}_upper_bounds(float *out)
{
    using R = vamp_jit_robot::R;
    std::array<float, R::dimension> ones;
    ones.fill(1.0F);
    typename R::Configuration v(ones);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" void vamp_jit_{{robot_name}}_lower_bounds(float *out)
{
    using R = vamp_jit_robot::R;
    std::array<float, R::dimension> zeros;
    zeros.fill(0.0F);
    typename R::Configuration v(zeros);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

// ===== PHS ================================================================

extern "C" vamp::jit::ffi::PhsHandle *
vamp_jit_{{robot_name}}_phs_new(const float *focus_a, const float *focus_b)
{
    using R = vamp_jit_robot::R;
    auto fa = vamp_jit_robot::load_config(focus_a);
    auto fb = vamp_jit_robot::load_config(focus_b);
    auto *phs = new vamp::planning::ProlateHyperspheroid<R>(fa, fb);
    return reinterpret_cast<vamp::jit::ffi::PhsHandle *>(phs);
}

extern "C" void vamp_jit_{{robot_name}}_phs_destroy(vamp::jit::ffi::PhsHandle *h)
{
    delete reinterpret_cast<vamp::planning::ProlateHyperspheroid<vamp_jit_robot::R> *>(h);
}

extern "C" void
vamp_jit_{{robot_name}}_phs_set_transverse_diameter(vamp::jit::ffi::PhsHandle *h, float diameter)
{
    reinterpret_cast<vamp::planning::ProlateHyperspheroid<vamp_jit_robot::R> *>(h)->set_transverse_diameter(
        diameter);
}

extern "C" void
vamp_jit_{{robot_name}}_phs_transform(const vamp::jit::ffi::PhsHandle *h, const float *in, float *out)
{
    using R = vamp_jit_robot::R;
    auto in_v = vamp_jit_robot::load_config(in);
    auto out_v = reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(h)->transform(in_v);
    auto arr = out_v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" vamp::jit::ffi::SamplerHandle *vamp_jit_{{robot_name}}_sampler_phs(
    const vamp::jit::ffi::PhsHandle *phs_h, vamp::jit::ffi::SamplerHandle *inner_h)
{
    using R = vamp_jit_robot::R;
    auto phs = *reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(phs_h);
    auto inner_rng = vamp_jit_robot::deref_sampler(inner_h);
    auto *holder = new vamp_jit_robot::SamplerPtr(
        std::make_shared<vamp::planning::ProlateHyperspheroidRNG<R>>(phs, inner_rng));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(holder);
}

#undef VAMP_JIT_RAKE
#undef VAMP_JIT_RESOLUTION
