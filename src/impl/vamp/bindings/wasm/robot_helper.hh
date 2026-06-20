#pragma once

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <vamp/planning/rrtc.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/aorrtc.hh>
#include <vamp/planning/aorrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/plan.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/factory.hh>
#include <vamp/random/halton.hh>
#include <vamp/vector.hh>

#include <array>
#include <vector>
#include <memory>
#include <string>

namespace vamp::wasm
{

    namespace em = emscripten;

    using RRTCSettings = vamp::planning::RRTCSettings;
    using AORRTCSettings = vamp::planning::AORRTCSettings;
    using SimplifySettings = vamp::planning::SimplifySettings;
    using BSplineSettings = vamp::planning::BSplineSettings;
    using ReduceSettings = vamp::planning::ReduceSettings;
    using ShortcutSettings = vamp::planning::ShortcutSettings;
    using PerturbSettings = vamp::planning::PerturbSettings;
    using SimplifyRoutine = vamp::planning::SimplifyRoutine;

    using EnvironmentF = vamp::collision::Environment<float>;
    using EnvironmentV = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

    class EnvironmentJS final
    {
    public:
        EnvironmentJS() = default;

        inline auto add_sphere(float x, float y, float z, float radius) -> void
        {
            env.spheres.emplace_back(vamp::collision::factory::sphere::flat(x, y, z, radius));
            env.sort();
        }

        inline auto
        add_cuboid(float cx, float cy, float cz, float rx, float ry, float rz, float hx, float hy, float hz)
            -> void
        {
            auto cuboid = vamp::collision::factory::cuboid::array(
                std::array<float, 3>{cx, cy, cz},
                std::array<float, 3>{rx, ry, rz},
                std::array<float, 3>{hx, hy, hz});

            if (cuboid.axis_3_z == 1.f)
            {
                env.z_aligned_cuboids.emplace_back(cuboid);
            }
            else
            {
                env.cuboids.emplace_back(cuboid);
            }
            env.sort();
        }

        inline auto add_capsule(float x1, float y1, float z1, float x2, float y2, float z2, float radius)
            -> void
        {
            auto capsule = vamp::collision::factory::cylinder::endpoints::array(
                std::array<float, 3>{x1, y1, z1}, std::array<float, 3>{x2, y2, z2}, radius);

            if (capsule.xv == 0.f && capsule.yv == 0.f)
            {
                env.z_aligned_capsules.emplace_back(capsule);
            }
            else
            {
                env.capsules.emplace_back(capsule);
            }
            env.sort();
        }

        inline auto clear() -> void
        {
            env.spheres.clear();
            env.cuboids.clear();
            env.z_aligned_cuboids.clear();
            env.capsules.clear();
            env.z_aligned_capsules.clear();
        }

        inline auto num_spheres() const noexcept -> int
        {
            return env.spheres.size();
        }

        inline auto num_cuboids() const noexcept -> int
        {
            return env.cuboids.size() + env.z_aligned_cuboids.size();
        }

        inline auto num_capsules() const noexcept -> int
        {
            return env.capsules.size() + env.z_aligned_capsules.size();
        }

        EnvironmentF env;
    };

    struct FKResultJS final
    {
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> z;
        std::vector<float> r;
    };

    template <typename Robot>
    class RNGJS final
    {
        using Configuration = typename Robot::Configuration;
        using RNG = vamp::rng::RNG<Robot>;
        using HaltonRNG = vamp::rng::Halton<Robot>;

    public:
        RNGJS() : rng(std::make_shared<HaltonRNG>())
        {
        }

        inline auto reset() -> void
        {
            rng->reset();
        }

        inline auto next() -> std::vector<float>
        {
            auto config = rng->next();
            auto arr = config.to_array();
            std::vector<float> result(Robot::dimension);
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                result[i] = arr[i];
            }
            return result;
        }

        typename RNG::Ptr rng;
    };

    template <typename Robot>
    class PathJS final
    {
        using Configuration = typename Robot::Configuration;
        using ConfigArray = std::array<float, Robot::dimension>;
        using Path = vamp::planning::Path<Robot>;

    public:
        PathJS() = default;

        explicit PathJS(const Path &p) : path(p)
        {
        }

        inline auto size() const noexcept -> int
        {
            return path.size();
        }

        inline auto get_waypoint(int idx) const -> std::vector<float>
        {
            if (idx < 0 || idx >= static_cast<int>(path.size()))
            {
                return {};
            }
            auto config = path[idx];
            auto arr = config.to_array();
            std::vector<float> result(Robot::dimension);
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                result[i] = arr[i];
            }
            return result;
        }

        inline auto append(const std::vector<float> &config) -> void
        {
            if (config.size() != Robot::dimension)
            {
                return;
            }
            ConfigArray arr;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                arr[i] = config[i];
            }
            path.emplace_back(Configuration(arr));
        }

        inline auto cost() const noexcept -> float
        {
            return path.cost();
        }

        inline auto subdivide() -> void
        {
            path.subdivide();
        }

        inline auto interpolate_to_resolution(int resolution) -> void
        {
            path.interpolate_to_resolution(resolution);
        }

        inline auto interpolate_to_n_states(int n) -> void
        {
            path.interpolate_to_n_states(n);
        }

        inline auto get_all_waypoints() const -> std::vector<float>
        {
            std::vector<float> result(path.size() * Robot::dimension);
            for (std::size_t i = 0; i < path.size(); ++i)
            {
                auto arr = path[i].to_array();
                for (std::size_t j = 0; j < Robot::dimension; ++j)
                {
                    result[i * Robot::dimension + j] = arr[j];
                }
            }
            return result;
        }

        Path path;
    };

    template <typename Robot>
    struct PlanningResultJS final
    {
        bool solved;
        PathJS<Robot> path;
        double nanoseconds;
        int iterations;

        PlanningResultJS() : solved(false), nanoseconds(0), iterations(0)
        {
        }
    };

    template <typename Robot>
    inline auto js_array_to_config(const std::vector<float> &arr) -> typename Robot::Configuration
    {
        using Configuration = typename Robot::Configuration;
        using ConfigArray = std::array<float, Robot::dimension>;

        ConfigArray config_arr;
        for (std::size_t i = 0; i < Robot::dimension && i < arr.size(); ++i)
        {
            config_arr[i] = arr[i];
        }

        return Configuration(config_arr);
    }

    template <typename Robot>
    inline auto plan_rrtc(
        const std::vector<float> &start,
        const std::vector<float> &goal,
        EnvironmentJS &env_js,
        const RRTCSettings &settings,
        RNGJS<Robot> &rng) -> PlanningResultJS<Robot>
    {
        PlanningResultJS<Robot> result;

        if (start.size() != Robot::dimension || goal.size() != Robot::dimension)
        {
            return result;
        }

        EnvironmentV env_v(env_js.env);

        auto plan_result = vamp::planning::RRTC<Robot, vamp::FloatVectorWidth, Robot::resolution>::solve(
            js_array_to_config<Robot>(start), js_array_to_config<Robot>(goal), env_v, settings, rng.rng);

        result.solved = !plan_result.path.empty();
        result.path = PathJS<Robot>(plan_result.path);
        result.nanoseconds = plan_result.nanoseconds;
        result.iterations = plan_result.iterations;

        return result;
    }

    template <typename Robot>
    inline auto plan_aorrtc(
        const std::vector<float> &start,
        const std::vector<float> &goal,
        EnvironmentJS &env_js,
        const AORRTCSettings &settings,
        RNGJS<Robot> &rng) -> PlanningResultJS<Robot>
    {
        PlanningResultJS<Robot> result;

        if (start.size() != Robot::dimension || goal.size() != Robot::dimension)
        {
            return result;
        }

        EnvironmentV env_v(env_js.env);

        auto plan_result = vamp::planning::AORRTC<Robot, vamp::FloatVectorWidth, Robot::resolution>::solve(
            js_array_to_config<Robot>(start), js_array_to_config<Robot>(goal), env_v, settings, rng.rng);

        result.solved = !plan_result.path.empty();
        result.path = PathJS<Robot>(plan_result.path);
        result.nanoseconds = plan_result.nanoseconds;
        result.iterations = plan_result.iterations;

        return result;
    }

    template <typename Robot>
    inline auto simplify_path(
        PathJS<Robot> &path_js,
        EnvironmentJS &env_js,
        const SimplifySettings &settings,
        RNGJS<Robot> &rng) -> PlanningResultJS<Robot>
    {
        PlanningResultJS<Robot> result;

        EnvironmentV env_v(env_js.env);

        auto simplify_result = vamp::planning::simplify<Robot, vamp::FloatVectorWidth, Robot::resolution>(
            path_js.path, env_v, settings, rng.rng);

        result.solved = !simplify_result.path.empty();
        result.path = PathJS<Robot>(simplify_result.path);
        result.nanoseconds = simplify_result.nanoseconds;
        result.iterations = simplify_result.iterations;

        return result;
    }

    template <typename Robot>
    inline auto validate_config(const std::vector<float> &config, EnvironmentJS &env_js) -> bool
    {
        if (config.size() != Robot::dimension)
        {
            return false;
        }

        auto configuration = js_array_to_config<Robot>(config);
        EnvironmentV env_v(env_js.env);

        return vamp::planning::validate_motion<Robot, vamp::FloatVectorWidth, 1>(
            configuration, configuration, env_v);
    }

    template <typename Robot>
    inline auto
    validate_motion(const std::vector<float> &start, const std::vector<float> &goal, EnvironmentJS &env_js)
        -> bool
    {
        if (start.size() != Robot::dimension || goal.size() != Robot::dimension)
        {
            return false;
        }

        EnvironmentV env_v(env_js.env);

        return vamp::planning::validate_motion<Robot, vamp::FloatVectorWidth, 1>(
            js_array_to_config<Robot>(start), js_array_to_config<Robot>(goal), env_v);
    }

    template <typename Robot>
    inline auto forward_kinematics(const std::vector<float> &config) -> FKResultJS
    {
        FKResultJS result;
        result.x.resize(Robot::n_spheres);
        result.y.resize(Robot::n_spheres);
        result.z.resize(Robot::n_spheres);
        result.r.resize(Robot::n_spheres);

        if (config.size() != Robot::dimension)
        {
            return result;
        }

        typename Robot::template ConfigurationBlock<1> block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            block[i] = config[i];
        }

        typename Robot::template Spheres<1> spheres;
        Robot::template sphere_fk<1>(block, spheres);

        for (std::size_t i = 0; i < Robot::n_spheres; ++i)
        {
            result.x[i] = spheres.x[{i, 0}];
            result.y[i] = spheres.y[{i, 0}];
            result.z[i] = spheres.z[{i, 0}];
            result.r[i] = spheres.r[{i, 0}];
        }

        return result;
    }

    template <typename Robot>
    inline auto end_effector_fk(const std::vector<float> &config) -> std::vector<float>
    {
        using ConfigArray = std::array<float, Robot::dimension>;

        std::vector<float> result(16, 0);

        if (config.size() != Robot::dimension)
        {
            return result;
        }

        ConfigArray arr;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            arr[i] = config[i];
        }

        auto tf = Robot::eefk(arr);
        auto matrix = tf.matrix();

        // Column-major order
        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                result[col * 4 + row] = matrix(row, col);
            }
        }

        return result;
    }

    template <typename Robot>
    inline auto get_dimension() noexcept -> int
    {
        return Robot::dimension;
    }

    template <typename Robot>
    inline auto get_num_spheres() noexcept -> int
    {
        return Robot::n_spheres;
    }

    template <typename Robot>
    inline auto get_resolution() noexcept -> int
    {
        return Robot::resolution;
    }

    template <typename Robot>
    inline auto get_space_measure() noexcept -> float
    {
        return Robot::space_measure();
    }

    template <typename Robot>
    inline auto get_upper_bounds() -> std::vector<float>
    {
        using Configuration = typename Robot::Configuration;

        std::array<float, Robot::dimension> ones;
        ones.fill(1.0f);
        auto one_v = Configuration(ones);
        Robot::scale_configuration(one_v);
        auto arr = one_v.to_array();

        std::vector<float> result(Robot::dimension);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            result[i] = arr[i];
        }
        return result;
    }

    template <typename Robot>
    inline auto get_lower_bounds() -> std::vector<float>
    {
        using Configuration = typename Robot::Configuration;

        std::array<float, Robot::dimension> zeros;
        zeros.fill(0.0f);
        auto zero_v = Configuration(zeros);
        Robot::scale_configuration(zero_v);
        auto arr = zero_v.to_array();

        std::vector<float> result(Robot::dimension);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            result[i] = arr[i];
        }
        return result;
    }

    template <typename Robot>
    inline auto get_joint_names() -> std::vector<std::string>
    {
        std::vector<std::string> names;
        for (const auto &name : Robot::joint_names)
        {
            names.push_back(std::string(name));
        }
        return names;
    }

    inline auto register_common_types() -> void
    {
        em::register_vector<float>("VectorFloat");
        em::register_vector<std::string>("VectorString");
        em::register_vector<int>("VectorInt");
        em::register_vector<SimplifyRoutine>("VectorSimplifyRoutine");

        em::enum_<SimplifyRoutine>("SimplifyRoutine")
            .value("BSPLINE", SimplifyRoutine::BSPLINE)
            .value("REDUCE", SimplifyRoutine::REDUCE)
            .value("SHORTCUT", SimplifyRoutine::SHORTCUT)
            .value("PERTURB", SimplifyRoutine::PERTURB);

        em::class_<RRTCSettings>("RRTCSettings")
            .constructor<>()
            .property("range", &RRTCSettings::range)
            .property("dynamic_domain", &RRTCSettings::dynamic_domain)
            .property("radius", &RRTCSettings::radius)
            .property("alpha", &RRTCSettings::alpha)
            .property("min_radius", &RRTCSettings::min_radius)
            .property("balance", &RRTCSettings::balance)
            .property("tree_ratio", &RRTCSettings::tree_ratio)
            .property("max_iterations", &RRTCSettings::max_iterations)
            .property("max_samples", &RRTCSettings::max_samples)
            .property("start_tree_first", &RRTCSettings::start_tree_first);

        em::class_<BSplineSettings>("BSplineSettings")
            .constructor<>()
            .property("max_steps", &BSplineSettings::max_steps)
            .property("min_change", &BSplineSettings::min_change)
            .property("midpoint_interpolation", &BSplineSettings::midpoint_interpolation);

        em::class_<ReduceSettings>("ReduceSettings")
            .constructor<>()
            .property("max_steps", &ReduceSettings::max_steps)
            .property("max_empty_steps", &ReduceSettings::max_empty_steps)
            .property("range_ratio", &ReduceSettings::range_ratio);

        em::class_<ShortcutSettings>("ShortcutSettings").constructor<>();

        em::class_<PerturbSettings>("PerturbSettings")
            .constructor<>()
            .property("max_steps", &PerturbSettings::max_steps)
            .property("max_empty_steps", &PerturbSettings::max_empty_steps)
            .property("perturbation_attempts", &PerturbSettings::perturbation_attempts)
            .property("range", &PerturbSettings::range);

        em::class_<SimplifySettings>("SimplifySettings")
            .constructor<>()
            .property("max_iterations", &SimplifySettings::max_iterations)
            .property("interpolate", &SimplifySettings::interpolate)
            .property("operations", &SimplifySettings::operations)
            .property("reduce", &SimplifySettings::reduce)
            .property("shortcut", &SimplifySettings::shortcut)
            .property("bspline", &SimplifySettings::bspline)
            .property("perturb", &SimplifySettings::perturb);

        em::class_<AORRTCSettings>("AORRTCSettings")
            .constructor<>()
            .property("rrtc", &AORRTCSettings::rrtc)
            .property("simplify", &AORRTCSettings::simplify)
            .property("optimize", &AORRTCSettings::optimize)
            .property("cost_bound_resample", &AORRTCSettings::cost_bound_resample)
            .property("simplify_intermediate", &AORRTCSettings::simplify_intermediate)
            .property("use_phs", &AORRTCSettings::use_phs)
            .property("anytime", &AORRTCSettings::anytime)
            .property("max_iterations", &AORRTCSettings::max_iterations)
            .property("max_internal_iterations", &AORRTCSettings::max_internal_iterations)
            .property("max_samples", &AORRTCSettings::max_samples)
            .property("max_cost_bound_resamples", &AORRTCSettings::max_cost_bound_resamples);

        em::class_<EnvironmentJS>("Environment")
            .constructor<>()
            .function("add_sphere", &EnvironmentJS::add_sphere)
            .function("add_cuboid", &EnvironmentJS::add_cuboid)
            .function("add_capsule", &EnvironmentJS::add_capsule)
            .function("clear", &EnvironmentJS::clear)
            .function("num_spheres", &EnvironmentJS::num_spheres)
            .function("num_cuboids", &EnvironmentJS::num_cuboids)
            .function("num_capsules", &EnvironmentJS::num_capsules);

        em::class_<FKResultJS>("FKResult")
            .constructor<>()
            .property("x", &FKResultJS::x)
            .property("y", &FKResultJS::y)
            .property("z", &FKResultJS::z)
            .property("r", &FKResultJS::r);
    }

    template <typename Robot>
    inline auto register_robot_types() -> void
    {
        using RNG = RNGJS<Robot>;
        using Path = PathJS<Robot>;
        using PlanningResult = PlanningResultJS<Robot>;

        em::class_<RNG>("RNG")
            .template constructor<>()
            .function("reset", &RNG::reset)
            .function("next", &RNG::next);

        em::class_<Path>("Path")
            .template constructor<>()
            .function("size", &Path::size)
            .function("get_waypoint", &Path::get_waypoint)
            .function("append", &Path::append)
            .function("cost", &Path::cost)
            .function("subdivide", &Path::subdivide)
            .function("interpolate_to_resolution", &Path::interpolate_to_resolution)
            .function("interpolate_to_n_states", &Path::interpolate_to_n_states)
            .function("get_all_waypoints", &Path::get_all_waypoints);

        em::class_<PlanningResult>("PlanningResult")
            .template constructor<>()
            .property("solved", &PlanningResult::solved)
            .property("path", &PlanningResult::path)
            .property("nanoseconds", &PlanningResult::nanoseconds)
            .property("iterations", &PlanningResult::iterations);

        em::function("plan_rrtc", &plan_rrtc<Robot>);
        em::function("plan_aorrtc", &plan_aorrtc<Robot>);
        em::function("simplify_path", &simplify_path<Robot>);
        em::function("validate", &validate_config<Robot>);
        em::function("validate_motion", &validate_motion<Robot>);
        em::function("forward_kinematics", &forward_kinematics<Robot>);
        em::function("end_effector_fk", &end_effector_fk<Robot>);

        em::function("get_dimension", &get_dimension<Robot>);
        em::function("get_num_spheres", &get_num_spheres<Robot>);
        em::function("get_resolution", &get_resolution<Robot>);
        em::function("get_space_measure", &get_space_measure<Robot>);
        em::function("get_upper_bounds", &get_upper_bounds<Robot>);
        em::function("get_lower_bounds", &get_lower_bounds<Robot>);
        em::function("get_joint_names", &get_joint_names<Robot>);
    }

}  // namespace vamp::wasm
