#include <vamp/collision/sphere_sphere.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>

// clang-format off
#define VAMP_JIT_ROBOT_TYPE                 vamp::robots::{{robot_name}}
#define VAMP_JIT_RAKE                       {{rake}}
#define VAMP_JIT_RESOLUTION                 {{resolution}}

#define VAMP_JIT_FN_RESULT_META             vamp_jit_{{robot_name}}_result_meta
#define VAMP_JIT_FN_RESULT_COPY_WAYPOINT    vamp_jit_{{robot_name}}_result_copy_waypoint
#define VAMP_JIT_FN_RESULT_DESTROY          vamp_jit_{{robot_name}}_result_destroy
#define VAMP_JIT_FN_RESULT_SIZES            vamp_jit_{{robot_name}}_result_sizes

#define VAMP_JIT_FN_SAMPLER_HALTON          vamp_jit_{{robot_name}}_sampler_halton
#define VAMP_JIT_FN_SAMPLER_XORSHIFT        vamp_jit_{{robot_name}}_sampler_xorshift
#define VAMP_JIT_FN_SAMPLER_RESET           vamp_jit_{{robot_name}}_sampler_reset
#define VAMP_JIT_FN_SAMPLER_SKIP            vamp_jit_{{robot_name}}_sampler_skip
#define VAMP_JIT_FN_SAMPLER_NEXT            vamp_jit_{{robot_name}}_sampler_next
#define VAMP_JIT_FN_SAMPLER_DESTROY         vamp_jit_{{robot_name}}_sampler_destroy

#define VAMP_JIT_FN_SIMPLIFY                vamp_jit_{{robot_name}}_simplify

#define VAMP_JIT_FN_DEBUG                   vamp_jit_{{robot_name}}_debug
#define VAMP_JIT_FN_DEBUG_DESTROY           vamp_jit_{{robot_name}}_debug_destroy
#define VAMP_JIT_FN_EEFK                    vamp_jit_{{robot_name}}_eefk
#define VAMP_JIT_FN_FK                      vamp_jit_{{robot_name}}_fk
#define VAMP_JIT_FN_VALIDATE                vamp_jit_{{robot_name}}_validate
#define VAMP_JIT_FN_VALIDATE_MOTION         vamp_jit_{{robot_name}}_validate_motion
#define VAMP_JIT_FN_FILTER_PC               vamp_jit_{{robot_name}}_filter_self_from_pointcloud
#define VAMP_JIT_FN_SPACE_MEASURE           vamp_jit_{{robot_name}}_space_measure
#define VAMP_JIT_FN_MIN_MAX_RADII           vamp_jit_{{robot_name}}_min_max_radii
#define VAMP_JIT_FN_N_SPHERES               vamp_jit_{{robot_name}}_n_spheres
#define VAMP_JIT_FN_JOINT_NAMES             vamp_jit_{{robot_name}}_joint_names
#define VAMP_JIT_FN_UPPER_BOUNDS            vamp_jit_{{robot_name}}_upper_bounds
#define VAMP_JIT_FN_LOWER_BOUNDS            vamp_jit_{{robot_name}}_lower_bounds

#define VAMP_JIT_FN_PHS_NEW                 vamp_jit_{{robot_name}}_phs_new
#define VAMP_JIT_FN_PHS_DESTROY             vamp_jit_{{robot_name}}_phs_destroy
#define VAMP_JIT_FN_PHS_SET_DIAMETER        vamp_jit_{{robot_name}}_phs_set_transverse_diameter
#define VAMP_JIT_FN_PHS_TRANSFORM           vamp_jit_{{robot_name}}_phs_transform
#define VAMP_JIT_FN_SAMPLER_PHS             vamp_jit_{{robot_name}}_sampler_phs
// clang-format on

namespace vamp_jit_robot
{
    using R = VAMP_JIT_ROBOT_TYPE;
    using SamplerPtr = typename vamp::rng::RNG<R>::Ptr;

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

    inline auto load_block_1(const float *data) -> typename R::template ConfigurationBlock<1>
    {
        typename R::template ConfigurationBlock<1> out;
        for (std::size_t i = 0; i < R::dimension; ++i)
        {
            out[i] = data[i];
        }
        return out;
    }

    inline auto raked_env(const void *env_ptr)
    {
        return vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>>(
            *static_cast<const vamp::collision::Environment<float> *>(env_ptr));
    }

    inline auto deref_sampler(vamp::jit::ffi::SamplerHandle *h) -> SamplerPtr &
    {
        return *reinterpret_cast<SamplerPtr *>(h);
    }
}  // namespace vamp_jit_robot

extern "C" vamp::jit::ffi::PlanResultMeta VAMP_JIT_FN_RESULT_META(const vamp::jit::ffi::PlanResultHandle *h)
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

extern "C" void
VAMP_JIT_FN_RESULT_COPY_WAYPOINT(const vamp::jit::ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
{
    const auto *w = reinterpret_cast<const vamp_jit_robot::WrappedResult *>(h);
    auto arr = w->inner.path[idx].to_array();
    std::memcpy(out, arr.data(), vamp_jit_robot::R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_RESULT_DESTROY(vamp::jit::ffi::PlanResultHandle *h)
{
    delete reinterpret_cast<vamp_jit_robot::WrappedResult *>(h);
}

extern "C" void VAMP_JIT_FN_RESULT_SIZES(const vamp::jit::ffi::PlanResultHandle *h, void *out_sizes_vec)
{
    const auto *w = reinterpret_cast<const vamp_jit_robot::WrappedResult *>(h);
    auto *out = static_cast<std::vector<std::size_t> *>(out_sizes_vec);
    *out = w->inner.size;
}

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_FN_SAMPLER_HALTON()
{
    auto *p = new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::Halton<vamp_jit_robot::R>>());
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_FN_SAMPLER_XORSHIFT(std::uint64_t seed)
{
#if defined(__x86_64__)
    auto *p = (seed == 0) ?
                  new vamp_jit_robot::SamplerPtr(std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>()) :
                  new vamp_jit_robot::SamplerPtr(
                      std::make_shared<vamp::rng::XORShift<vamp_jit_robot::R>>(seed, seed + 1));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
#else
    throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
}

extern "C" void VAMP_JIT_FN_SAMPLER_RESET(vamp::jit::ffi::SamplerHandle *h)
{
    vamp_jit_robot::deref_sampler(h)->reset();
}

extern "C" void VAMP_JIT_FN_SAMPLER_SKIP(vamp::jit::ffi::SamplerHandle *h, std::uint64_t n)
{
    auto &rng = vamp_jit_robot::deref_sampler(h);
    for (std::uint64_t i = 0; i < n; ++i)
    {
        rng->next();
    }
}

extern "C" void VAMP_JIT_FN_SAMPLER_NEXT(vamp::jit::ffi::SamplerHandle *h, float *out)
{
    auto arr = vamp_jit_robot::deref_sampler(h)->next().to_array();
    std::memcpy(out, arr.data(), vamp_jit_robot::R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_SAMPLER_DESTROY(vamp::jit::ffi::SamplerHandle *h)
{
    delete reinterpret_cast<vamp_jit_robot::SamplerPtr *>(h);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SIMPLIFY(
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

    auto env_rake = vamp_jit_robot::raked_env(env_ptr);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = vamp_jit_robot::deref_sampler(sampler);

    auto *wrapped = new vamp_jit_robot::WrappedResult{
        vamp::planning::simplify<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>(path, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::DebugHandle *VAMP_JIT_FN_DEBUG(const float *config, const void *env_ptr)
{
    using R = vamp_jit_robot::R;
    typename R::template ConfigurationBlock<VAMP_JIT_RAKE> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }
    auto env_rake = vamp_jit_robot::raked_env(env_ptr);
    auto *result = new typename R::Debug(R::template fkcc_debug<VAMP_JIT_RAKE>(env_rake, block));
    return reinterpret_cast<vamp::jit::ffi::DebugHandle *>(result);
}

extern "C" void VAMP_JIT_FN_DEBUG_DESTROY(vamp::jit::ffi::DebugHandle *h)
{
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
    delete reinterpret_cast<DebugType *>(h);
}

extern "C" void VAMP_JIT_FN_EEFK(const float *config, float *out_matrix)
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

extern "C" void VAMP_JIT_FN_FK(const float *config, float *out_spheres)
{
    using R = vamp_jit_robot::R;
    auto block = vamp_jit_robot::load_block_1(config);

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
VAMP_JIT_FN_VALIDATE(const float *config_ptr, const void *env_ptr, std::int32_t check_bounds)
{
    using R = vamp_jit_robot::R;

    auto configuration = vamp_jit_robot::load_config(config_ptr);
    auto copy = configuration.trim();
    R::descale_configuration(copy);
    const bool in_bounds = (copy <= 1.F).all() and (copy >= 0.F).all();

    return (not check_bounds or in_bounds) and
           vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(
               configuration, configuration, vamp_jit_robot::raked_env(env_ptr));
}

extern "C" std::int32_t VAMP_JIT_FN_VALIDATE_MOTION(
    const float *c_in_ptr,
    const float *c_out_ptr,
    const void *env_ptr,
    std::int32_t check_bounds)
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

    return (not check_bounds or (in_bounds_in and in_bounds_out)) and
           vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(
               c_in, c_out, vamp_jit_robot::raked_env(env_ptr));
}

extern "C" void VAMP_JIT_FN_FILTER_PC(
    const float *points_in,
    std::uint64_t n_points,
    float point_radius,
    const float *config,
    const void *env_ptr,
    void *out_filtered_vec)
{
    vamp::collision::filter_self_from_pointcloud<vamp_jit_robot::R, VAMP_JIT_RAKE>(
        points_in,
        n_points,
        point_radius,
        vamp_jit_robot::load_block_1(config),
        vamp_jit_robot::raked_env(env_ptr),
        *static_cast<std::vector<vamp::collision::Point> *>(out_filtered_vec));
}

extern "C" float VAMP_JIT_FN_SPACE_MEASURE()
{
    return vamp_jit_robot::R::space_measure();
}

extern "C" void VAMP_JIT_FN_MIN_MAX_RADII(float *out_min, float *out_max)
{
    *out_min = vamp_jit_robot::R::min_radius;
    *out_max = vamp_jit_robot::R::max_radius;
}

extern "C" std::uint64_t VAMP_JIT_FN_N_SPHERES()
{
    return vamp_jit_robot::R::n_spheres;
}

extern "C" void VAMP_JIT_FN_JOINT_NAMES(void *out_strings)
{
    auto *v = static_cast<std::vector<std::string> *>(out_strings);
    v->clear();
    for (auto sv : vamp_jit_robot::R::joint_names)
    {
        v->emplace_back(sv);
    }
}

extern "C" void VAMP_JIT_FN_UPPER_BOUNDS(float *out)
{
    using R = vamp_jit_robot::R;
    std::array<float, R::dimension> ones;
    ones.fill(1.0F);
    typename R::Configuration v(ones);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_LOWER_BOUNDS(float *out)
{
    using R = vamp_jit_robot::R;
    std::array<float, R::dimension> zeros;
    zeros.fill(0.0F);
    typename R::Configuration v(zeros);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" vamp::jit::ffi::PhsHandle *VAMP_JIT_FN_PHS_NEW(const float *focus_a, const float *focus_b)
{
    using R = vamp_jit_robot::R;
    auto fa = vamp_jit_robot::load_config(focus_a);
    auto fb = vamp_jit_robot::load_config(focus_b);
    auto *phs = new vamp::planning::ProlateHyperspheroid<R>(fa, fb);
    return reinterpret_cast<vamp::jit::ffi::PhsHandle *>(phs);
}

extern "C" void VAMP_JIT_FN_PHS_DESTROY(vamp::jit::ffi::PhsHandle *h)
{
    delete reinterpret_cast<vamp::planning::ProlateHyperspheroid<vamp_jit_robot::R> *>(h);
}

extern "C" void VAMP_JIT_FN_PHS_SET_DIAMETER(vamp::jit::ffi::PhsHandle *h, float diameter)
{
    reinterpret_cast<vamp::planning::ProlateHyperspheroid<vamp_jit_robot::R> *>(h)->set_transverse_diameter(
        diameter);
}

extern "C" void VAMP_JIT_FN_PHS_TRANSFORM(const vamp::jit::ffi::PhsHandle *h, const float *in, float *out)
{
    using R = vamp_jit_robot::R;
    auto in_v = vamp_jit_robot::load_config(in);
    auto out_v = reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(h)->transform(in_v);
    auto arr = out_v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" vamp::jit::ffi::SamplerHandle *
VAMP_JIT_FN_SAMPLER_PHS(const vamp::jit::ffi::PhsHandle *phs_h, vamp::jit::ffi::SamplerHandle *inner_h)
{
    using R = vamp_jit_robot::R;
    auto phs = *reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(phs_h);
    auto inner_rng = vamp_jit_robot::deref_sampler(inner_h);
    auto *holder = new vamp_jit_robot::SamplerPtr(
        std::make_shared<vamp::planning::ProlateHyperspheroidRNG<R>>(phs, inner_rng));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(holder);
}