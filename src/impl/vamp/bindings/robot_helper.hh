#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vamp/planning/phs.hh>
#include <vamp/random/rng.hh>
#include <vamp/random/halton.hh>

#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#else
#include <stdexcept>
#endif

#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/fcit.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/aorrtc.hh>
#include <vamp/vector.hh>

#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/ndarray.h>

#define DEFINE_HAS_METHOD(method_name)                                                                       \
    template <typename T, typename = void>                                                                   \
    struct has_##method_name : std::false_type                                                               \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    struct has_##method_name<T, std::void_t<decltype(T::method_name)>> : std::true_type                      \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    constexpr bool has_##method_name##_v = has_##method_name<T>::value;

DEFINE_HAS_METHOD(set_lows)
DEFINE_HAS_METHOD(set_highs)
DEFINE_HAS_METHOD(set_radius)

namespace vamp::binding
{
    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    template <typename Robot>
    struct NDArrayInput
    {
        using Type = nanobind::
            ndarray<FloatT, nanobind::numpy, nanobind::shape<Robot::dimension>, nanobind::device::cpu>;

        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        template <std::size_t rake>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;

        inline static auto from(const Configuration &c) -> Type
        {
            auto *arr = new FloatT[Robot::dimension];
            auto c_arr = c.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                arr[i] = c_arr[i];
            }

            nanobind::capsule arr_owner(
                arr, [](void *a) noexcept { delete[] reinterpret_cast<FloatT *>(a); });
            return Type(arr, {Robot::dimension}, arr_owner);
        };

        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(array(a));
        };

        inline static auto array(const Type &a) -> ConfigurationArray
        {
            ConfigurationArray c;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                c[i] = a(i);
            }

            return c;
        };

        template <std::size_t rake>
        inline static auto block(const Type &a) -> ConfigurationBlock<rake>
        {
            ConfigurationBlock<rake> out;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = a(i);
            }

            return out;
        }
    };

    template <typename Robot>
    struct ArrayInput
    {
        using Type = typename Robot::ConfigurationArray;

        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        template <std::size_t rake>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;

        inline static auto from(const Configuration &c) -> Type
        {
            Type a;
            auto c_arr = c.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                a[i] = c_arr[i];
            }

            return a;
        };

        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(a);
        };

        inline static auto array(const Type &a) -> ConfigurationArray
        {
            return a;
        }

        template <std::size_t rake>
        inline static auto block(const Type &a) -> ConfigurationBlock<rake>
        {
            ConfigurationBlock<rake> out;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = a[i];
            }

            return out;
        }
    };

    template <typename Robot, typename Input>
    struct Helper
    {
        using Type = typename Input::Type;

        using Configuration = typename Robot::Configuration;

        using EnvironmentInput = vamp::collision::Environment<float>;
        using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

        using Path = vamp::planning::Path<Robot>;
        using PlanningResult = vamp::planning::PlanningResult<Robot>;
        using Roadmap = vamp::planning::Roadmap<Robot>;

        using RNG = vamp::rng::RNG<Robot>;

        template <typename Planner, typename Settings>
        struct PlannerHelper
        {
            inline static auto single(
                const Type &start,
                const Type &goal,
                const EnvironmentInput &environment,
                const Settings &settings,
                typename RNG::Ptr rng) -> PlanningResult
            {
                return Planner::solve(
                    Input::to(start), Input::to(goal), EnvironmentVector(environment), settings, rng);
            }

            inline static auto multi(
                const Type &start,
                const std::vector<Type> &goals,
                const EnvironmentInput &environment,
                const Settings &settings,
                typename RNG::Ptr rng) -> PlanningResult
            {
                std::vector<Configuration> goals_v;
                goals_v.reserve(goals.size());

                for (const auto &goal : goals)
                {
                    goals_v.emplace_back(Input::to(goal));
                }

                return Planner::solve(
                    Input::to(start), goals_v, EnvironmentVector(environment), settings, rng);
            }

            inline static auto roadmap(
                const Type &start,
                const Type &goal,
                const EnvironmentInput &environment,
                const Settings &settings,
                typename RNG::Ptr rng) -> Roadmap
            {
                return Planner::build_roadmap(
                    Input::to(start), Input::to(goal), EnvironmentVector(environment), settings, rng);
            }
        };

        using PRM = PlannerHelper<
            vamp::planning::PRM<Robot, rake, Robot::resolution>,
            vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams>>;

        using RRTC =
            PlannerHelper<vamp::planning::RRTC<Robot, rake, Robot::resolution>, vamp::planning::RRTCSettings>;

        using FCIT = PlannerHelper<
            vamp::planning::FCIT<Robot, rake, Robot::resolution>,
            vamp::planning::RoadmapSettings<vamp::planning::FCITStarNeighborParams>>;

        using AORRTC = PlannerHelper<
            vamp::planning::AORRTC<Robot, rake, Robot::resolution>,
            vamp::planning::AORRTCSettings>;

        inline static auto fk(const Type &c_in) -> std::vector<vamp::collision::Sphere<float>>
        {
            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(Input::template block<1>(c_in), out);

            std::vector<vamp::collision::Sphere<float>> result(Robot::n_spheres);
            for (auto i = 0U; i < Robot::n_spheres; ++i)
            {
                result[i] = vamp::collision::Sphere<float>{
                    out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}]};
            }

            return result;
        }

        inline static auto debug(const Type &c_in, const EnvironmentInput &environment) ->
            typename Robot::Debug
        {
            return Robot::fkcc_debug(EnvironmentVector(environment), Input::template block<rake>(c_in));
        }

        inline static auto
        validate(const Type &c_in, const EnvironmentInput &environment, bool check_bounds = false) -> bool
        {
            auto configuration = Input::to(c_in);
            auto copy = configuration.trim();
            Robot::descale_configuration(copy);

            const bool in_bounds = (copy <= 1.F).all() and (copy >= 0.F).all();

            return (not check_bounds or in_bounds) and
                   vamp::planning::validate_motion<Robot, rake, 1>(
                       configuration, configuration, EnvironmentVector(environment));
        }

        inline static auto simplify(
            const Path &path,
            const EnvironmentInput &environment,
            const vamp::planning::SimplifySettings &settings,
            typename RNG::Ptr rng) -> PlanningResult
        {
            return vamp::planning::simplify<Robot, rake, Robot::resolution>(
                path, EnvironmentVector(environment), settings, rng);
        }

        inline static auto eefk(const Type &start) -> Eigen::Matrix4f
        {
            return Robot::eefk(Input::array(start)).matrix();
        }

        inline static auto filter_self_from_pointcloud(
            const std::vector<collision::Point> &pc,
            float point_radius,
            const Type &c_in,
            const EnvironmentInput &environment) -> std::vector<collision::Point>
        {
            // TODO: Do this smarter with SIMD CC
            EnvironmentVector ev(environment);

            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(Input::template block<1>(c_in), out);

            std::vector<collision::Point> filtered;
            filtered.reserve(pc.size());

            for (const auto &point : pc)
            {
                const float x = point[0], y = point[1], z = point[2], r = point_radius;

                bool valid = true;
                for (auto i = 0U; i < Robot::n_spheres; ++i)
                {
                    if (collision::sphere_sphere_sql2(
                            out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}], x, y, z, r) < 0 or
                        sphere_environment_in_collision<>(ev, x, y, z, r))
                    {
                        valid = false;
                        break;
                    }
                }

                if (valid)
                {
                    filtered.emplace_back(point);
                }
            }

            return filtered;
        }
    };

    template <typename Robot>
    inline auto init_robot(nanobind::module_ &pymodule) -> nanobind::module_
    {
        namespace nb = nanobind;
        using namespace nb::literals;

        using NA = NDArrayInput<Robot>;
        using CA = ArrayInput<Robot>;
        using NDArray = typename NA::Type;
        using Array = typename CA::Type;
        using HPN = Helper<Robot, NA>;
        using HPA = Helper<Robot, CA>;

        auto submodule = pymodule.def_submodule(Robot::name, "Robot-specific submodule");

        submodule.def(
            "dimension",
            []() { return Robot::dimension; },
            "Dimension of configuration space for this robot.");
        submodule.def(
            "resolution",
            []() { return Robot::resolution; },
            "Collision checking resolution for this robot.");
        submodule.def(
            "n_spheres", []() { return Robot::n_spheres; }, "Number of spheres in robot collision model.");
        submodule.def(
            "space_measure", []() { return Robot::space_measure(); }, "Measure of robot's C-space.");
        submodule.def(
            "min_max_radii",
            []() -> std::pair<float, float> { return {Robot::min_radius, Robot::max_radius}; },
            "Minimum and maximum radii sizes of robot spheres.");
        submodule.def(
            "joint_names", []() { return Robot::joint_names; }, "Joint names for the robot in order of DoF");
        submodule.def("end_effector", []() { return Robot::end_effector; }, "End-effector frame name.");

        using RNG = vamp::rng::RNG<Robot>;
        nb::class_<typename RNG::Ptr>(submodule, "RNG", "RNG for robot configurations.")
            .def(
                "reset",
                [](typename RNG::Ptr rng) { rng->reset(); },
                "Reset the RNG to initial state and seed.")
            .def(
                "next",
                [](typename RNG::Ptr rng) -> NDArray { return NA::from(rng->next()); },
                "Sample the next configuration. Modifies internal RNG state.")
            .def(
                "skip",
                [](typename RNG::Ptr rng, std::size_t n)
                {
                    for (auto i = 0U; i < n; ++i)
                    {
                        rng->next();
                    }
                },
                "Skip the next n iterations.");

        using PHS = vamp::planning::ProlateHyperspheroid<Robot>;
        nb::class_<PHS>(submodule, "ProlateHyperspheroid", "Prolate Hyperspheroid for Robot.")
            .def(
                "__init__",
                [](PHS *t, const NDArray &a, const NDArray &b) { new (t) PHS(NA::to(a), NA::to(b)); },
                "Construct from two loci.")
            .def("set_transverse_diameter", &PHS::set_transverse_diameter)
            .def("transform", &PHS::transform);

        submodule.def(
            "halton",
            []() -> typename RNG::Ptr { return std::make_shared<vamp::rng::Halton<Robot>>(); },
            "Creates a new Halton sampler.");

        submodule.def(
            "phs_sampler",
            [](const planning::ProlateHyperspheroid<Robot> &phs, typename RNG::Ptr rng) -> typename RNG::Ptr
            { return std::make_shared<planning::ProlateHyperspheroidRNG<Robot>>(phs, rng); },
            "Creates a new PHS sampler.");

#if defined(__x86_64__)
        submodule.def(
            "xorshift",
            []() -> typename RNG::Ptr { return std::make_shared<vamp::rng::XORShift<Robot>>(); },
            "Creates a new XORShift sampler.");
#else
        submodule.def(
            "xorshift", []() { throw std::runtime_error("XORShift is not supported on non-x86 systems!"); });
#endif

        using Path = vamp::planning::Path<Robot>;
        nb::class_<Path>(submodule, "Path", "Path in configuration space represented as discrete waypoints.")
            .def(nb::init<>(), "Default constructor, creates empty path.")
            .def(
                "__len__",
                [](const Path &p) { return p.size(); },
                "Return the number of waypoints in the path.")
            .def(
                "__getitem__",
                [](const Path &p, std::size_t i) -> NDArray { return NA::from(p[i]); },
                "Get the i-th configuration in the path.")
            .def(
                "__setitem__",
                [](Path &p, std::size_t i, const NDArray &c) { p[i] = NA::to(c); },
                "Set the i-th configuration of the the path.")
            .def(
                "append",
                [](Path &p, const NDArray &c) { p.emplace_back(NA::to(c)); },
                "Append a configuration to the end of this path.")
            .def(
                "insert",
                [](Path &p, std::size_t i, const NDArray &c) { p.insert(p.cbegin() + i, NA::to(c)); },
                "Append a configuration to the end of this path.")
            .def(
                "__setitem__",
                [](Path &p, std::size_t i, const Array &c) { p[i] = CA::to(c); },
                "Set the i-th configuration of the the path.")
            .def(
                "append",
                [](Path &p, const Array &c) { p.emplace_back(CA::to(c)); },
                "Append a configuration to the end of this path.")
            .def(
                "insert",
                [](Path &p, std::size_t i, const Array &c) { p.insert(p.cbegin() + i, CA::to(c)); },
                "Append a configuration to the end of this path.")
            .def("cost", &Path::cost, "Compute the total path length (by the l2-norm) of the path.")
            .def(
                "subdivide",
                &Path::subdivide,
                "Subdivide the path by inserting a configuration at the midpoint of all existing segments.")
            .def(
                "interpolate_to_resolution",
                &Path::interpolate_to_resolution,
                "Refine the path by interpolating all segments up to the resolution provided.")
            .def(
                "interpolate_to_n_states",
                &Path::interpolate_to_n_states,
                "Refine the path by interpolating to n states as even as possible.")
            .def(
                "validate",
                [](Path &p, const typename HPN::EnvironmentInput &e)
                {
                    const typename HPN::EnvironmentVector ev(e);
                    return p.template validate<rake>(ev);
                },
                "Validate the path in an environment.")
            .def(
                "numpy",
                [](const Path &p) noexcept
                {
                    auto *path_arr = new FloatT
                        [Robot::dimension * p.size() +
                         (Robot::Configuration::num_scalars_rounded - Robot::dimension)];
                    for (auto i = 0U; i < p.size(); ++i)
                    {
                        p[i].to_array_unaligned(path_arr + i * Robot::dimension);
                    }

                    nb::capsule arr_owner(
                        path_arr, [](void *a) noexcept { delete[] reinterpret_cast<FloatT *>(a); });
                    return nb::ndarray<nb::numpy, const FloatT, nb::device::cpu>(
                        path_arr, {p.size(), Robot::dimension}, arr_owner);
                },
                "Convert this path to a numpy matrix.");

        nb::class_<typename HPN::PlanningResult>(submodule, "PlanningResult", "Result of a planning query.")
            .def(nb::init<>(), "Empty constructor.")
            .def_prop_ro(
                "solved",
                [](const typename HPN::PlanningResult &p) { return p.path.size() >= 2; },
                "Returns true if solution found.")
            .def_ro("path", &HPN::PlanningResult::path, "The solution path, if the path is found.")
            .def_ro("nanoseconds", &HPN::PlanningResult::nanoseconds, "Nanoseconds taken to find the path.")
            .def_ro(
                "iterations",
                &HPN::PlanningResult::iterations,
                "Number of planner iterations used to find the path.")
            .def_ro("size", &HPN::PlanningResult::size, "Size of the internal planner datastructures.");

        nb::class_<typename HPN::Roadmap>(submodule, "Roadmap", "Undirected graph in configuration space.")
            .def(nb::init<>(), "Empty constructor.")
            .def(
                "__len__",
                [](const typename HPN::Roadmap &r) { return r.vertices.size(); },
                "Return the number of vertices in the roadmap.")
            .def(
                "__getitem__",
                [](const typename HPN::Roadmap &r, std::size_t i) -> NDArray
                { return NA::from(r.vertices[i]); },
                "Get the i-th vertex.")
            .def_ro(
                "vertices", &HPN::Roadmap::vertices, "List of all vertices (configurations) in the roadmap.")
            .def_ro("edges", &HPN::Roadmap::edges, "List of all undirected edge pairs, by vertex index.")
            .def_ro("nanoseconds", &HPN::Roadmap::nanoseconds, "Nanoseconds taken to construct roadmap.")
            .def_ro(
                "iterations", &HPN::Roadmap::iterations, "Number of iterations taken to construct roadmap.");

        // Doesn't have an Array/NDArray interface, so only once
        submodule.def(
            "simplify",
            HPN::simplify,
            "path"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Simplification heuristics to post-process a path.");

#define MF(name, func, desc, ...)                                                                            \
    submodule.def(name, HPN::func, ##__VA_ARGS__, desc);                                                     \
    submodule.def(name, HPA::func, ##__VA_ARGS__, desc);

        MF("fk",
           fk,
           "Computes the forward kinematics of the robot. Returns array of all collision sphere positions.",
           "configuration"_a);

        MF("eefk",
           eefk,
           "Computes the forward kinematics of the robot's end-effector. Returns XYZ and a XYZW quaternion.",
           "configuration"_a);

        MF("debug",
           debug,
           "Check which spheres of a robot configuration are in collision.",
           "configuration"_a,
           "environment"_a = vamp::collision::Environment<float>());

        MF("validate",
           validate,
           "Check if a configuration is valid. Returns true if valid.",
           "configuration"_a,
           "environment"_a = vamp::collision::Environment<float>(),
           "check_bounds"_a = false);

        MF("filter_self_from_pointcloud",
           filter_self_from_pointcloud,
           "Removes points from pointcloud which collide with the robot and environment.",
           "pc"_a,
           "point_radius"_a,
           "configuration"_a,
           "environment"_a = vamp::collision::Environment<float>());

        MF("roadmap",
           PRM::roadmap,
           "PRM roadmap construction.",
           "start"_a,
           "goal"_a,
           "environment"_a,
           "settings"_a,
           "rng"_a);

#define PLANNER(name, func, desc, ...)                                                                       \
    MF(name, func::single, desc, "start"_a, "goal"_a, "environment"_a, "settings"_a, "rng"_a);               \
    MF(name, func::multi, desc, "start"_a, "goal"_a, "environment"_a, "settings"_a, "rng"_a);

        PLANNER("rrtc", RRTC, "RRTConnect");
        PLANNER("prm", PRM, "PRM");
        PLANNER("fcit", FCIT, "FCIT");
        PLANNER("aorrtc", AORRTC, "AORRTC");

        if constexpr (has_set_lows_v<Robot>)
        {
            submodule.def("set_lows", &Robot::set_lows, "Set lower bounds.");
        }

        if constexpr (has_set_highs_v<Robot>)
        {
            submodule.def("set_highs", &Robot::set_highs, "Set upper bounds.");
        }

        if constexpr (has_set_radius_v<Robot>)
        {
            submodule.def("set_radius", &Robot::set_radius, "Set radius.");
        }

        return submodule;
    }
}  // namespace vamp::binding
