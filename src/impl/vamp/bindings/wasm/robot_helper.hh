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

class EnvironmentJS
{
public:
    EnvironmentJS() = default;

    void addSphere(float x, float y, float z, float radius)
    {
        env.spheres.emplace_back(vamp::collision::factory::sphere::flat(x, y, z, radius));
        env.sort();
    }

    void addCuboid(
        float cx, float cy, float cz,
        float rx, float ry, float rz,
        float hx, float hy, float hz)
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

    void addCapsule(
        float x1, float y1, float z1,
        float x2, float y2, float z2,
        float radius)
    {
        auto capsule = vamp::collision::factory::cylinder::endpoints::array(
            std::array<float, 3>{x1, y1, z1},
            std::array<float, 3>{x2, y2, z2},
            radius);

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

    void clear()
    {
        env.spheres.clear();
        env.cuboids.clear();
        env.z_aligned_cuboids.clear();
        env.capsules.clear();
        env.z_aligned_capsules.clear();
    }

    int numSpheres() const { return env.spheres.size(); }
    int numCuboids() const { return env.cuboids.size() + env.z_aligned_cuboids.size(); }
    int numCapsules() const { return env.capsules.size() + env.z_aligned_capsules.size(); }

    EnvironmentF env;
};

struct FKResultJS
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> r;
};

template <typename Robot>
class RNGJS
{
    using Configuration = typename Robot::Configuration;
    using RNG = vamp::rng::RNG<Robot>;
    using HaltonRNG = vamp::rng::Halton<Robot>;

public:
    RNGJS() : rng(std::make_shared<HaltonRNG>()) {}

    void reset() { rng->reset(); }

    std::vector<float> next()
    {
        auto config = rng->next();
        auto arr = config.to_array();
        std::vector<float> result(Robot::dimension);
        for (size_t i = 0; i < Robot::dimension; ++i)
        {
            result[i] = arr[i];
        }
        return result;
    }

    typename RNG::Ptr rng;
};

template <typename Robot>
class PathJS
{
    using Configuration = typename Robot::Configuration;
    using ConfigArray = std::array<float, Robot::dimension>;
    using Path = vamp::planning::Path<Robot>;

public:
    PathJS() = default;
    PathJS(const Path &p) : path(p) {}

    int size() const { return path.size(); }

    std::vector<float> getWaypoint(int idx) const
    {
        if (idx < 0 || idx >= static_cast<int>(path.size()))
        {
            return {};
        }
        auto config = path[idx];
        auto arr = config.to_array();
        std::vector<float> result(Robot::dimension);
        for (size_t i = 0; i < Robot::dimension; ++i)
        {
            result[i] = arr[i];
        }
        return result;
    }

    void append(const std::vector<float> &config)
    {
        if (config.size() != Robot::dimension) return;
        ConfigArray arr;
        for (size_t i = 0; i < Robot::dimension; ++i)
        {
            arr[i] = config[i];
        }
        path.emplace_back(Configuration(arr));
    }

    float cost() const { return path.cost(); }

    void subdivide() { path.subdivide(); }

    void interpolateToResolution(int resolution)
    {
        path.interpolate_to_resolution(resolution);
    }

    void interpolateToNStates(int n)
    {
        path.interpolate_to_n_states(n);
    }

    std::vector<float> getAllWaypoints() const
    {
        std::vector<float> result(path.size() * Robot::dimension);
        for (size_t i = 0; i < path.size(); ++i)
        {
            auto arr = path[i].to_array();
            for (size_t j = 0; j < Robot::dimension; ++j)
            {
                result[i * Robot::dimension + j] = arr[j];
            }
        }
        return result;
    }

    Path path;
};

template <typename Robot>
struct PlanningResultJS
{
    bool solved;
    PathJS<Robot> path;
    double nanoseconds;
    int iterations;

    PlanningResultJS() : solved(false), nanoseconds(0), iterations(0) {}
};

template <typename Robot>
typename Robot::Configuration jsArrayToConfig(const std::vector<float> &arr)
{
    using Configuration = typename Robot::Configuration;
    using ConfigArray = std::array<float, Robot::dimension>;

    ConfigArray config_arr;
    for (size_t i = 0; i < Robot::dimension && i < arr.size(); ++i)
    {
        config_arr[i] = arr[i];
    }

    return Configuration(config_arr);
}

template <typename Robot>
PlanningResultJS<Robot> planRRTC(
    const std::vector<float> &start,
    const std::vector<float> &goal,
    EnvironmentJS &envJS,
    const RRTCSettings &settings,
    RNGJS<Robot> &rng)
{
    PlanningResultJS<Robot> result;

    if (start.size() != Robot::dimension || goal.size() != Robot::dimension)
    {
        return result;
    }

    EnvironmentV envV(envJS.env);

    auto planResult = vamp::planning::RRTC<Robot, vamp::FloatVectorWidth, Robot::resolution>::solve(
        jsArrayToConfig<Robot>(start),
        jsArrayToConfig<Robot>(goal),
        envV,
        settings,
        rng.rng);

    result.solved = planResult.solved;
    result.path = PathJS<Robot>(planResult.path);
    result.nanoseconds = planResult.nanoseconds;
    result.iterations = planResult.iterations;

    return result;
}

template <typename Robot>
PlanningResultJS<Robot> planAORRTC(
    const std::vector<float> &start,
    const std::vector<float> &goal,
    EnvironmentJS &envJS,
    const AORRTCSettings &settings,
    RNGJS<Robot> &rng)
{
    PlanningResultJS<Robot> result;

    if (start.size() != Robot::dimension || goal.size() != Robot::dimension)
    {
        return result;
    }

    EnvironmentV envV(envJS.env);

    auto planResult = vamp::planning::AORRTC<Robot, vamp::FloatVectorWidth, Robot::resolution>::solve(
        jsArrayToConfig<Robot>(start),
        jsArrayToConfig<Robot>(goal),
        envV,
        settings,
        rng.rng);

    result.solved = planResult.solved;
    result.path = PathJS<Robot>(planResult.path);
    result.nanoseconds = planResult.nanoseconds;
    result.iterations = planResult.iterations;

    return result;
}

template <typename Robot>
PlanningResultJS<Robot> simplifyPath(
    PathJS<Robot> &pathJS,
    EnvironmentJS &envJS,
    const SimplifySettings &settings,
    RNGJS<Robot> &rng)
{
    PlanningResultJS<Robot> result;

    EnvironmentV envV(envJS.env);

    auto simplifyResult = vamp::planning::simplify<Robot, vamp::FloatVectorWidth, Robot::resolution>(
        pathJS.path,
        envV,
        settings,
        rng.rng);

    result.solved = simplifyResult.solved;
    result.path = PathJS<Robot>(simplifyResult.path);
    result.nanoseconds = simplifyResult.nanoseconds;
    result.iterations = simplifyResult.iterations;

    return result;
}

template <typename Robot>
bool validateConfig(const std::vector<float> &config, EnvironmentJS &envJS)
{
    if (config.size() != Robot::dimension)
    {
        return false;
    }

    auto configuration = jsArrayToConfig<Robot>(config);
    EnvironmentV envV(envJS.env);

    return vamp::planning::validate_motion<Robot, vamp::FloatVectorWidth, 1>(
        configuration, configuration, envV);
}

template <typename Robot>
bool validateMotion(
    const std::vector<float> &start,
    const std::vector<float> &goal,
    EnvironmentJS &envJS)
{
    if (start.size() != Robot::dimension || goal.size() != Robot::dimension)
    {
        return false;
    }

    EnvironmentV envV(envJS.env);

    return vamp::planning::validate_motion<Robot, vamp::FloatVectorWidth, 1>(
        jsArrayToConfig<Robot>(start), jsArrayToConfig<Robot>(goal), envV);
}

template <typename Robot>
FKResultJS forwardKinematics(const std::vector<float> &config)
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
    for (size_t i = 0; i < Robot::dimension; ++i)
    {
        block[i] = config[i];
    }

    typename Robot::template Spheres<1> spheres;
    Robot::template sphere_fk<1>(block, spheres);

    for (size_t i = 0; i < Robot::n_spheres; ++i)
    {
        result.x[i] = spheres.x[{i, 0}];
        result.y[i] = spheres.y[{i, 0}];
        result.z[i] = spheres.z[{i, 0}];
        result.r[i] = spheres.r[{i, 0}];
    }

    return result;
}

template <typename Robot>
std::vector<float> endEffectorFK(const std::vector<float> &config)
{
    using ConfigArray = std::array<float, Robot::dimension>;

    std::vector<float> result(16, 0);

    if (config.size() != Robot::dimension)
    {
        return result;
    }

    ConfigArray arr;
    for (size_t i = 0; i < Robot::dimension; ++i)
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
int getDimension() { return Robot::dimension; }

template <typename Robot>
int getNumSpheres() { return Robot::n_spheres; }

template <typename Robot>
int getResolution() { return Robot::resolution; }

template <typename Robot>
float getSpaceMeasure() { return Robot::space_measure(); }

template <typename Robot>
std::vector<float> getUpperBounds()
{
    using Configuration = typename Robot::Configuration;

    std::array<float, Robot::dimension> ones;
    ones.fill(1.0f);
    auto one_v = Configuration(ones);
    Robot::scale_configuration(one_v);
    auto arr = one_v.to_array();

    std::vector<float> result(Robot::dimension);
    for (size_t i = 0; i < Robot::dimension; ++i)
    {
        result[i] = arr[i];
    }
    return result;
}

template <typename Robot>
std::vector<float> getLowerBounds()
{
    using Configuration = typename Robot::Configuration;

    std::array<float, Robot::dimension> zeros;
    zeros.fill(0.0f);
    auto zero_v = Configuration(zeros);
    Robot::scale_configuration(zero_v);
    auto arr = zero_v.to_array();

    std::vector<float> result(Robot::dimension);
    for (size_t i = 0; i < Robot::dimension; ++i)
    {
        result[i] = arr[i];
    }
    return result;
}

template <typename Robot>
std::vector<std::string> getJointNames()
{
    std::vector<std::string> names;
    for (const auto &name : Robot::joint_names)
    {
        names.push_back(std::string(name));
    }
    return names;
}

inline void register_common_types()
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

    em::class_<ShortcutSettings>("ShortcutSettings")
        .constructor<>();

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
        .function("addSphere", &EnvironmentJS::addSphere)
        .function("addCuboid", &EnvironmentJS::addCuboid)
        .function("addCapsule", &EnvironmentJS::addCapsule)
        .function("clear", &EnvironmentJS::clear)
        .function("numSpheres", &EnvironmentJS::numSpheres)
        .function("numCuboids", &EnvironmentJS::numCuboids)
        .function("numCapsules", &EnvironmentJS::numCapsules);

    em::class_<FKResultJS>("FKResult")
        .constructor<>()
        .property("x", &FKResultJS::x)
        .property("y", &FKResultJS::y)
        .property("z", &FKResultJS::z)
        .property("r", &FKResultJS::r);
}

template <typename Robot>
void register_robot_types()
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
        .function("getWaypoint", &Path::getWaypoint)
        .function("append", &Path::append)
        .function("cost", &Path::cost)
        .function("subdivide", &Path::subdivide)
        .function("interpolateToResolution", &Path::interpolateToResolution)
        .function("interpolateToNStates", &Path::interpolateToNStates)
        .function("getAllWaypoints", &Path::getAllWaypoints);

    em::class_<PlanningResult>("PlanningResult")
        .template constructor<>()
        .property("solved", &PlanningResult::solved)
        .property("path", &PlanningResult::path)
        .property("nanoseconds", &PlanningResult::nanoseconds)
        .property("iterations", &PlanningResult::iterations);

    em::function("planRRTC", &planRRTC<Robot>);
    em::function("planAORRTC", &planAORRTC<Robot>);
    em::function("simplifyPath", &simplifyPath<Robot>);
    em::function("validate", &validateConfig<Robot>);
    em::function("validateMotion", &validateMotion<Robot>);
    em::function("forwardKinematics", &forwardKinematics<Robot>);
    em::function("endEffectorFK", &endEffectorFK<Robot>);

    em::function("getDimension", &getDimension<Robot>);
    em::function("getNumSpheres", &getNumSpheres<Robot>);
    em::function("getResolution", &getResolution<Robot>);
    em::function("getSpaceMeasure", &getSpaceMeasure<Robot>);
    em::function("getUpperBounds", &getUpperBounds<Robot>);
    em::function("getLowerBounds", &getLowerBounds<Robot>);
    em::function("getJointNames", &getJointNames<Robot>);
}

} // namespace vamp::wasm
