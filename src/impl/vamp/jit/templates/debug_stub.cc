// Per-robot introspection stub. Wraps the static-binding helpers used for
// inspection / validation / pointcloud filtering: debug(), eefk(), fk(),
// validate(), validate_motion(), filter_self_from_pointcloud(), plus the
// static-data getters (space_measure, min/max radii, joint names, bounds,
// n_spheres).
//
// inja substitutions:
//   {{robot_name}} — struct name inside vamp::robots::
//   {{rake}}       — SIMD lane width (the static binding uses rake too)

#include <vamp/collision/sphere_sphere.hh>
#include <vamp/planning/validate.hh>

// clang-format off
#define VAMP_JIT_DEBUG          vamp_jit_{{robot_name}}_debug
#define VAMP_JIT_DEBUG_DESTROY  vamp_jit_{{robot_name}}_debug_destroy
#define VAMP_JIT_EEFK           vamp_jit_{{robot_name}}_eefk
#define VAMP_JIT_FK             vamp_jit_{{robot_name}}_fk
#define VAMP_JIT_VALIDATE       vamp_jit_{{robot_name}}_validate
#define VAMP_JIT_VALIDATE_MOTION vamp_jit_{{robot_name}}_validate_motion
#define VAMP_JIT_FILTER_PC      vamp_jit_{{robot_name}}_filter_self_from_pointcloud
#define VAMP_JIT_SPACE_MEASURE  vamp_jit_{{robot_name}}_space_measure
#define VAMP_JIT_MIN_MAX_RADII  vamp_jit_{{robot_name}}_min_max_radii
#define VAMP_JIT_N_SPHERES      vamp_jit_{{robot_name}}_n_spheres
#define VAMP_JIT_JOINT_NAMES    vamp_jit_{{robot_name}}_joint_names
#define VAMP_JIT_UPPER_BOUNDS   vamp_jit_{{robot_name}}_upper_bounds
#define VAMP_JIT_LOWER_BOUNDS   vamp_jit_{{robot_name}}_lower_bounds
#define VAMP_JIT_RAKE           {{rake}}
// clang-format on

extern "C" vamp::jit::ffi::DebugHandle *VAMP_JIT_DEBUG(const float *config, const void *env_ptr)
{
    using R = vamp_jit_robot::R;

    // Broadcast the scalar configuration into a rake-wide block, mirroring
    // the static binding's Input::template block<rake>(c).
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

extern "C" void VAMP_JIT_DEBUG_DESTROY(vamp::jit::ffi::DebugHandle *h)
{
    // R::Debug isn't templated on R — it's always pair<vec<vec<string>>,
    // vec<pair<size_t, size_t>>>. Same type the host uses below.
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
    delete reinterpret_cast<DebugType *>(h);
}

extern "C" void VAMP_JIT_EEFK(const float *config, float *out_matrix)
{
    using R = vamp_jit_robot::R;
    typename R::ConfigurationArray cfg;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        cfg[i] = config[i];
    }

    // Static binding does Robot::eefk(array).matrix(); we mirror that, then
    // memcpy the 16 floats out (column-major — Eigen's default layout).
    Eigen::Matrix4f mat = R::eefk(cfg).matrix();
    std::memcpy(out_matrix, mat.data(), 16 * sizeof(float));
}

extern "C" void VAMP_JIT_FK(const float *config, float *out_spheres)
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
VAMP_JIT_VALIDATE(const float *config_ptr, const void *env_ptr, std::int32_t check_bounds)
{
    using R = vamp_jit_robot::R;

    auto configuration = vamp_jit_robot::load_config(config_ptr);
    auto copy = configuration.trim();
    R::descale_configuration(copy);
    const bool in_bounds = (copy <= 1.F).all() and (copy >= 0.F).all();

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    const bool ok = (not check_bounds or in_bounds) and
                    vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(
                        configuration, configuration, env_rake);
    return ok ? 1 : 0;
}

extern "C" std::int32_t VAMP_JIT_VALIDATE_MOTION(
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

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    const bool ok = (not check_bounds or (in_bounds_in and in_bounds_out)) and
                    vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(c_in, c_out, env_rake);
    return ok ? 1 : 0;
}

extern "C" void VAMP_JIT_FILTER_PC(
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
                    out.x[{i, 0}],
                    out.y[{i, 0}],
                    out.z[{i, 0}],
                    out.r[{i, 0}],
                    x,
                    y,
                    z,
                    point_radius) < 0 or
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

extern "C" float VAMP_JIT_SPACE_MEASURE()
{
    return vamp_jit_robot::R::space_measure();
}

extern "C" void VAMP_JIT_MIN_MAX_RADII(float *out_min, float *out_max)
{
    *out_min = vamp_jit_robot::R::min_radius;
    *out_max = vamp_jit_robot::R::max_radius;
}

extern "C" std::uint64_t VAMP_JIT_N_SPHERES()
{
    return vamp_jit_robot::R::n_spheres;
}

extern "C" void VAMP_JIT_JOINT_NAMES(void *out_strings)
{
    auto *v = static_cast<std::vector<std::string> *>(out_strings);
    v->clear();
    for (auto sv : vamp_jit_robot::R::joint_names)
    {
        v->emplace_back(sv);
    }
}

extern "C" void VAMP_JIT_UPPER_BOUNDS(float *out)
{
    using R = vamp_jit_robot::R;
    std::array<float, R::dimension> ones;
    ones.fill(1.0F);
    typename R::Configuration v(ones);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_LOWER_BOUNDS(float *out)
{
    using R = vamp_jit_robot::R;
    std::array<float, R::dimension> zeros;
    zeros.fill(0.0F);
    typename R::Configuration v(zeros);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

#undef VAMP_JIT_DEBUG
#undef VAMP_JIT_DEBUG_DESTROY
#undef VAMP_JIT_EEFK
#undef VAMP_JIT_FK
#undef VAMP_JIT_VALIDATE
#undef VAMP_JIT_VALIDATE_MOTION
#undef VAMP_JIT_FILTER_PC
#undef VAMP_JIT_SPACE_MEASURE
#undef VAMP_JIT_MIN_MAX_RADII
#undef VAMP_JIT_N_SPHERES
#undef VAMP_JIT_JOINT_NAMES
#undef VAMP_JIT_UPPER_BOUNDS
#undef VAMP_JIT_LOWER_BOUNDS
#undef VAMP_JIT_RAKE
