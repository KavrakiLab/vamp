#pragma once

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
#include <nanobind/ndarray.h>

namespace vamp::binding
{
    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    template <typename Robot>
    auto filter_robot_from_pointcloud(
        const std::vector<collision::Point> &pc,
        const typename Robot::ConfigurationArray &configuration,
        const collision::Environment<float> &environment,
        float point_radius) -> std::vector<collision::Point>
    {
        using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

        // TODO: Do this smarter with SIMD CC
        typename Robot::template Spheres<1> out;
        typename Robot::template ConfigurationBlock<1> block;

        EnvironmentVector ev(environment);

        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = configuration[i];
        }

        Robot::template sphere_fk<1>(block, out);

        std::vector<collision::Point> filtered;
        filtered.reserve(pc.size());

        for (const auto &point : pc)
        {
            const auto x = point[0];
            const auto y = point[1];
            const auto z = point[2];
            const auto r = point_radius;

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

    template <typename Robot>
    struct Helper
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;

        template <std::size_t rake>
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        using Path = vamp::planning::Path<Robot>;
        using PlanningResult = vamp::planning::PlanningResult<Robot>;
        using Roadmap = vamp::planning::Roadmap<Robot>;

        using EnvironmentInput = vamp::collision::Environment<float>;
        using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

        using RNG = vamp::rng::RNG<Robot::dimension>;
        using Halton = vamp::rng::Halton<Robot::dimension>;
        using PHS = vamp::planning::ProlateHyperspheroid<Robot::dimension>;
#if defined(__x86_64__)
        using XORShift = vamp::rng::XORShift<Robot::dimension>;
#endif

        using NDArray = nanobind::ndarray<float, nanobind::shape<Robot::dimension>, nanobind::device::cpu>;

        inline static auto c_to_nd(const Configuration &c) -> NDArray
        {
            NDArray a;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                a(i) = c[{i, 0}];
            }

            return a;
        };

        inline static auto nd_to_c(const NDArray &a) -> Configuration
        {
            ConfigurationArray c;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                c[i] = a(i);
            }

            return Configuration(c);
        };

        template <std::size_t rake>
        inline static auto nd_to_b(const NDArray &a) -> ConfigurationBlock<rake>
        {
            ConfigurationBlock<rake> block;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                block[i] = a(i);
            }

            return block;
        }

        inline static auto
        phs_sampler(const planning::ProlateHyperspheroid<Robot::dimension> &phs, typename RNG::Ptr rng) ->
            typename RNG::Ptr
        {
            return std::make_shared<planning::ProlateHyperspheroidRNG<Robot>>(phs, rng);
        }

        inline static auto halton() -> typename RNG::Ptr
        {
            return std::make_shared<Halton>();
        }

#if defined(__x86_64__)
        inline static auto xorshift() -> typename RNG::Ptr
        {
            return std::make_shared<XORShift>();
        }
#endif

        inline static auto fk(const NDArray &configuration) -> std::vector<vamp::collision::Sphere<float>>
        {
            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(nd_to_b<1>(configuration), out);
            std::vector<vamp::collision::Sphere<float>> result;
            result.reserve(Robot::n_spheres);

            for (auto i = 0U; i < Robot::n_spheres; ++i)
            {
                result.emplace_back(out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}]);
            }

            return result;
        }

        inline static auto debug(const NDArray &configuration, const EnvironmentInput &environment) ->
            typename Robot::Debug
        {
            return Robot::fkcc_debug(EnvironmentVector(environment), nd_to_b<rake>(configuration));
        }

        inline static auto
        validate(const NDArray &c_in, const EnvironmentInput &environment, bool check_bounds = false) -> bool
        {
            auto configuration = nd_to_c(c_in);
            auto copy = configuration.trim();
            Robot::descale_configuration(copy);

            const bool in_bounds = (copy <= 1.F).all() and (copy >= 0.F).all();

            return (not check_bounds or in_bounds) and
                   vamp::planning::validate_motion<Robot, rake, 1>(
                       configuration, configuration, EnvironmentVector(environment));
        }

        template <typename Planner, typename Settings>
        struct PlannerHelper
        {
            inline static auto single(
                const NDArray &start,
                const NDArray &goal,
                const EnvironmentInput &environment,
                const Settings &settings,
                typename RNG::Ptr rng) -> PlanningResult
            {
                return Planner::solve(
                    nd_to_c(start), nd_to_c(goal), EnvironmentVector(environment), settings, rng);
            }

            inline static auto multi(
                const NDArray &start,
                const std::vector<NDArray> &goals,
                const EnvironmentInput &environment,
                const Settings &settings,
                typename RNG::Ptr rng) -> PlanningResult
            {
                std::vector<Configuration> goals_v;
                goals_v.reserve(goals.size());

                for (const auto &goal : goals)
                {
                    goals_v.emplace_back(nd_to_c(goal));
                }

                return Planner::solve(nd_to_c(start), goals_v, EnvironmentVector(environment), settings, rng);
            }

            inline static auto roadmap(
                const NDArray &start,
                const NDArray &goal,
                const EnvironmentInput &environment,
                const Settings &settings,
                typename RNG::Ptr rng) -> Roadmap
            {
                return Planner::build_roadmap(
                    nd_to_c(start), nd_to_c(goal), EnvironmentVector(environment), settings, rng);
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

        inline static auto simplify(
            const Path &path,
            const EnvironmentInput &environment,
            const vamp::planning::SimplifySettings &settings,
            typename RNG::Ptr rng) -> PlanningResult
        {
            return vamp::planning::simplify<Robot, rake, Robot::resolution>(
                path, EnvironmentVector(environment), settings, rng);
        }

        inline static auto filter_self_from_pointcloud(
            const std::vector<collision::Point> &pc,
            const ConfigurationArray &start,
            const EnvironmentInput &environment,
            float point_radius) -> std::vector<collision::Point>
        {
            return filter_robot_from_pointcloud<Robot>(pc, start, environment, point_radius);
        }

        inline static auto eefk(const ConfigurationArray &start)
            -> std::pair<std::array<float, 3>, std::array<float, 4>>
        {
            const auto &result = Robot::eefk(start);

            std::array<float, 3> position = {result[0], result[1], result[2]};
            // A (x, y, z, w) quaternion
            std::array<float, 4> orientation = {result[3], result[4], result[5], result[6]};

            return {position, orientation};
        }
    };

    template <typename Robot>
    inline auto init_robot(nanobind::module_ &pymodule) -> nanobind::module_
    {
        namespace nb = nanobind;
        using namespace nb::literals;

        using RH = Helper<Robot>;
        using NA = typename RH::NDArray;

        auto submodule = pymodule.def_submodule(Robot::name, "Robot-specific submodule");

        nb::class_<typename RH::RNG::Ptr>(submodule, "RNG", "RNG for robot configurations.")
            .def(
                "reset",
                [](typename RH::RNG::Ptr rng) { rng->reset(); },
                "Reset the RNG to initial state and seed.")
            .def(
                "next",
                [](typename RH::RNG::Ptr rng) -> NA
                {
                    typename Robot::Configuration x = rng->next();
                    Robot::scale_configuration(x);
                    return RH::c_to_nd(x);
                },
                "Sample the next configuration. Modifies internal RNG state.")
            .def(
                "skip",
                [](typename RH::RNG::Ptr rng, std::size_t n)
                {
                    for (auto i = 0U; i < n; ++i)
                    {
                        auto x = rng->next();
                    }
                },
                "Skip the next n iterations.");

        nb::class_<typename RH::PHS>(submodule, "ProlateHyperspheroid", "Prolate Hyperspheroid for Robot.")
            .def(
                "__init__",
                [](typename RH::PHS *t, const NA &a, const NA &b)
                { new (t) typename RH::PHS(RH::nd_to_c(a), RH::nd_to_c(b)); },
                "Construct from two loci.")
            .def("set_transverse_diameter", &RH::PHS::set_transverse_diameter)
            .def("transform", &RH::PHS::transform);

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

        submodule.def(
            "distance",
            [](const NA &a, const NA &b) { return RH::nd_to_c(a).distance(RH::nd_to_c(b)); },
            "a"_a,
            "b"_a,
            "Distance (l2-norm) between two configurations.");

        nb::class_<typename RH::Path>(
            submodule, "Path", "Path in configuration space represented as discrete waypoints.")
            .def(nb::init<>(), "Default constructor, creates empty path.")
            .def(
                "__len__",
                [](const typename RH::Path &p) { return p.size(); },
                "Return the number of waypoints in the path.")
            .def(
                "__getitem__",
                [](const typename RH::Path &p, std::size_t i) -> NA { return RH::c_to_nd(p[i]); },
                "Get the i-th configuration in the path.")
            .def(
                "__setitem__",
                [](typename RH::Path &p, std::size_t i, const NA &c) { p[i] = RH::nd_to_c(c); },
                "Set the i-th configuration of the the path.")
            // .def(
            //     "__iter__",
            //     [](const typename RH::Path &p)
            //     { return nb::make_iterator(nb::type<typename RH::Path>(), "iterator", p.begin(), p.end());
            //     }, nb::keep_alive<0, 1>(), "Iterate over all configurations in the path.")
            .def(
                "append",
                [](typename RH::Path &p, const NA &c) { p.emplace_back(RH::nd_to_c(c)); },
                "Append a configuration to the end of this path.")
            .def(
                "insert",
                [](typename RH::Path &p, std::size_t i, const NA &c)
                { p.insert(p.cbegin() + i, RH::nd_to_c(c)); },
                "Append a configuration to the end of this path.")
            .def("cost", &RH::Path::cost, "Compute the total path length (by the l2-norm) of the path.")
            .def(
                "subdivide",
                &RH::Path::subdivide,
                "Subdivide the path by inserting a configuration at the midpoint of all existing segments.")
            .def(
                "interpolate_to_resolution",
                &RH::Path::interpolate_to_resolution,
                "Refine the path by interpolating all segments up to the resolution provided.")
            .def(
                "interpolate_to_n_states",
                &RH::Path::interpolate_to_n_states,
                "Refine the path by interpolating to n states as even as possible.")
            .def(
                "validate",
                [](typename RH::Path &p, const typename RH::EnvironmentInput &e)
                {
                    const typename RH::EnvironmentVector ev(e);
                    return p.template validate<rake>(ev);
                },
                "Validate the path in an environment.")
            .def(
                "numpy",
                [](const typename RH::Path &p) noexcept
                {
                    auto *path_arr = new FloatT
                        [Robot::dimension * p.size() +
                         (RH::Configuration::num_scalars_rounded - Robot::dimension)];
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

        nb::class_<typename RH::PlanningResult>(submodule, "PlanningResult", "Result of a planning query.")
            .def(nb::init<>(), "Empty constructor.")
            .def_prop_ro(
                "solved",
                [](const typename RH::PlanningResult &p) { return p.path.size() >= 2; },
                "Returns true if solution found.")
            .def_ro("path", &RH::PlanningResult::path, "The solution path, if the path is found.")
            .def_ro("nanoseconds", &RH::PlanningResult::nanoseconds, "Nanoseconds taken to find the path.")
            .def_ro(
                "iterations",
                &RH::PlanningResult::iterations,
                "Number of planner iterations used to find the path.")
            .def_ro("size", &RH::PlanningResult::size, "Size of the internal planner datastructures.");

        nb::class_<typename RH::Roadmap>(submodule, "Roadmap", "Undirected graph in configuration space.")
            .def(nb::init<>(), "Empty constructor.")
            .def(
                "__len__",
                [](const typename RH::Roadmap &r) { return r.vertices.size(); },
                "Return the number of vertices in the roadmap.")
            .def(
                "__getitem__",
                [](const typename RH::Roadmap &r, std::size_t i) { return r.vertices[i]; },
                "Get the i-th vertex.")
            .def(
                "__iter__",
                [](const typename RH::Roadmap &r)
                {
                    return nb::make_iterator(
                        nb::type<typename RH::Roadmap>(), "iterator", r.vertices.begin(), r.vertices.end());
                },
                nb::keep_alive<0, 1>(),
                "Iterate over all vertices in the roadmap.")
            .def_ro(
                "vertices", &RH::Roadmap::vertices, "List of all vertices (configurations) in the roadmap.")
            .def_ro("edges", &RH::Roadmap::edges, "List of all undirected edge pairs, by vertex index.")
            .def_ro("nanoseconds", &RH::Roadmap::nanoseconds, "Nanoseconds taken to construct roadmap.")
            .def_ro(
                "iterations", &RH::Roadmap::iterations, "Number of iterations taken to construct roadmap.");

        submodule.def("halton", RH::halton, "Creates a new Halton sampler.");
        submodule.def("phs_sampler", RH::phs_sampler, "Creates a new PHS sampler.");

#if defined(__x86_64__)
        submodule.def("xorshift", RH::xorshift, "Creates a new XORShift sampler.");
#else
        submodule.def(
            "xorshift", []() { throw std::runtime_error("XORShift is not supported on non-x86 systems!"); });
#endif

        submodule.def(
            "rrtc",
            RH::RRTC::single,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with RRTConnect.");

        submodule.def(
            "rrtc",
            RH::RRTC::multi,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with RRTConnect.");

        submodule.def(
            "prm",
            RH::PRM::single,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with PRM.");

        submodule.def(
            "prm",
            RH::PRM::multi,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with PRM.");

        submodule.def(
            "fcit",
            RH::FCIT::single,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with FCIT*.");

        submodule.def(
            "fcit",
            RH::FCIT::multi,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with FCIT*.");

        submodule.def(
            "aorrtc",
            RH::AORRTC::single,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with AORRTC.");

        submodule.def(
            "aorrtc",
            RH::AORRTC::multi,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Solve the motion planning problem with AORRTC.");

        submodule.def(
            "roadmap",
            RH::PRM::roadmap,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "PRM roadmap construction.");

        submodule.def(
            "simplify",
            RH::simplify,
            "path"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "Simplification heuristics to post-process a path.");

        submodule.def(
            "validate",
            RH::validate,
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "check_bounds"_a = false,
            "Check if a configuration is valid. Returns true if valid.");

        submodule.def(
            "debug",
            RH::debug,
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Check which spheres of a robot configuration are in collision.");

        submodule.def(
            "fk",
            RH::fk,
            "configuration"_a,
            "Computes the forward kinematics of the robot. Returns array of all collision sphere positions.");

        submodule.def(
            "filter_from_pointcloud",
            RH::filter_self_from_pointcloud,
            "pointcloud"_a,
            "configuration"_a,
            "environment"_a,
            "point_radius"_a,
            "Filters all colliding points from a point cloud.");

        submodule.def(
            "eefk",
            RH::eefk,
            "configuration"_a,
            "Returns the position and orientation (as a xyzw quaternion) of the robot's end-effector.");

        return submodule;
    }
}  // namespace vamp::binding
