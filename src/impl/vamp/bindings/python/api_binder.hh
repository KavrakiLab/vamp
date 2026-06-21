#pragma once

#include <vamp/collision/environment.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/planning/aorrtc_settings.hh>
#include <vamp/planning/grrtstar_settings.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace vamp::binding
{
    namespace nb_ = nanobind;
    namespace vp_ = vamp::planning;
    using namespace nb_::literals;

    template <typename Traits, vp_::Planner P, typename Settings, typename Target>
    inline void register_planner(Target &t, const char *name)
    {
        const auto doc = std::string("Plan using ") + name + ".";
        t.def(
            name,
            &Traits::template solve_single<P, Settings>,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            doc.c_str());
        t.def(
            name,
            &Traits::template solve_multi<P, Settings>,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            doc.c_str());
    }

    template <typename Traits, typename Target>
    inline void bind_robot_methods(Target &t)
    {
        using Env = typename Traits::Env;
        const auto default_env = Env{};

        t.def(
            "fk",
            &Traits::fk,
            "configuration"_a,
            "Forward kinematics. Returns the robot's collision spheres in world frame.");
        t.def(
            "eefk",
            &Traits::eefk,
            "configuration"_a,
            "End-effector forward kinematics. Returns a 4x4 transform.");
        t.def(
            "debug",
            &Traits::debug,
            "configuration"_a,
            "environment"_a = default_env,
            "Per-sphere collision debug: which spheres collide with each other and the environment.");
        t.def(
            "validate",
            &Traits::validate,
            "configuration"_a,
            "environment"_a = default_env,
            "check_bounds"_a = false,
            "Check whether a configuration is valid.");
        t.def(
            "validate_motion",
            &Traits::validate_motion,
            "configuration_in"_a,
            "configuration_out"_a,
            "environment"_a = default_env,
            "check_bounds"_a = true,
            "Check whether a straight-line motion between two configurations is valid.");
        t.def(
            "filter_self_from_pointcloud",
            &Traits::filter_self_from_pointcloud,
            "pc"_a,
            "point_radius"_a,
            "configuration"_a,
            "environment"_a = default_env,
            "Remove pointcloud points colliding with the robot or environment.");
        t.def(
            "simplify",
            &Traits::simplify,
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "Simplification heuristics to post-process a path.");

        t.def("halton", &Traits::make_halton, "Create a Halton sampler for this robot.");
        t.def("xorshift", &Traits::make_xorshift, "seed"_a = std::uint64_t{0}, "Create an XORShift sampler.");
        t.def(
            "phs",
            &Traits::make_phs,
            "focus_a"_a,
            "focus_b"_a,
            "Construct a prolate hyperspheroid from two foci.");
        t.def("phs_sampler", &Traits::make_phs_sampler, "phs"_a, "rng"_a, "Create a PHS sampler.");

        register_planner<Traits, vp_::Planner::RRTC, vp_::RRTCSettings>(t, "rrtc");
        register_planner<Traits, vp_::Planner::PRM, typename Traits::PRMSettings>(t, "prm");
        register_planner<Traits, vp_::Planner::FCIT, typename Traits::FCITSettings>(t, "fcit");
        register_planner<Traits, vp_::Planner::AORRTC, vp_::AORRTCSettings>(t, "aorrtc");
        register_planner<Traits, vp_::Planner::GRRTSTAR, vp_::GRRTStarSettings>(t, "grrtstar");
    }

    template <typename Traits>
    inline auto bind_path_class(nb_::module_ &m, const char *name) -> nb_::class_<typename Traits::Path>
    {
        using P = typename Traits::Path;
        return nb_::class_<P>(m, name, "Path in configuration space represented as discrete waypoints.")
            .def(nb_::init<>(), "Default constructor, creates empty path.")
            .def("__len__", &P::size, "Return the number of waypoints in the path.")
            .def(
                "__getitem__",
                [](const P &p, std::size_t i) { return Traits::path_get(p, i); },
                "Get the i-th configuration in the path.")
            .def("cost", &P::cost, "Compute the total path length (by the l2-norm) of the path.")
            .def(
                "subdivide",
                &P::subdivide,
                "Subdivide the path by inserting a configuration at the midpoint of every segment.")
            .def(
                "interpolate_to_n_states",
                &P::interpolate_to_n_states,
                "n"_a,
                "Refine the path by interpolating to n states as evenly as possible.")
            .def(
                "interpolate_to_resolution",
                &P::interpolate_to_resolution,
                "resolution"_a,
                "Refine the path by interpolating segments up to the given resolution.")
            .def(
                "validate",
                [](P &p, const typename Traits::Env &e) { return Traits::path_validate(p, e); },
                "environment"_a,
                "Validate the path in an environment.")
            .def(
                "numpy",
                [](const P &p) { return Traits::path_numpy(p); },
                "Convert this path to an (n_waypoints, dimension) numpy array.");
    }

    template <typename Traits, typename Klass>
    inline void bind_path_io(Klass &k)
    {
        using P = typename Traits::Path;
        using C = typename Traits::Cfg;
        k.def(
             "__setitem__",
             [](P &p, std::size_t i, const C &c) { Traits::path_set(p, i, c); },
             "Set the i-th configuration of the path.")
            .def(
                "append",
                [](P &p, const C &c) { Traits::path_append(p, c); },
                "Append a configuration to the end of the path.")
            .def(
                "insert",
                [](P &p, std::size_t i, const C &c) { Traits::path_insert(p, i, c); },
                "Insert a configuration at index i.");
    }

    template <typename Traits>
    inline auto bind_planning_result(nb_::module_ &m, const char *name)
        -> nb_::class_<typename Traits::PlanningResult>
    {
        using R = typename Traits::PlanningResult;
        return nb_::class_<R>(m, name, "Result of a planning query.")
            .def(nb_::init<>(), "Empty constructor.")
            .def_prop_ro(
                "solved",
                [](const R &r) { return Traits::result_solved(r); },
                "Returns true if a solution was found.")
            .def_ro("path", &R::path, "The solution path, if found.")
            .def_ro("nanoseconds", &R::nanoseconds, "Nanoseconds taken to find the path.")
            .def_ro("iterations", &R::iterations, "Number of planner iterations used to find the path.")
            .def_ro(
                "size", &R::size, "Sizes of the planner's internal datastructures (e.g. tree node counts).");
    }

    template <typename Traits>
    inline auto bind_sampler(nb_::module_ &m, const char *name) -> nb_::class_<typename Traits::Sampler>
    {
        using S = typename Traits::Sampler;
        return nb_::class_<S>(m, name, "Sampler for robot configurations.")
            .def("reset", &S::reset, "Reset the sampler to its initial state.")
            .def(
                "skip",
                [](S &s, std::size_t n) { Traits::sampler_skip(s, n); },
                "n"_a,
                "Skip the next n samples.")
            .def("next", [](S &s) { return Traits::sampler_next(s); }, "Sample the next configuration.");
    }

    template <typename Traits>
    inline auto bind_phs_class(nb_::module_ &m, const char *name) -> nb_::class_<typename Traits::Phs>
    {
        using P = typename Traits::Phs;
        return nb_::class_<P>(m, name, "Prolate hyperspheroid for sampling.")
            .def(
                "set_transverse_diameter",
                [](P &p, float d) { Traits::phs_set_transverse_diameter(p, d); },
                "diameter"_a,
                "Set the PHS' transverse diameter.");
    }

    template <typename Traits, typename Klass>
    inline void bind_phs_io(Klass &k)
    {
        using P = typename Traits::Phs;
        using C = typename Traits::Cfg;
        k.def(
            "transform",
            [](P &p, const C &x) { return Traits::phs_transform(p, x); },
            "x"_a,
            "Transform a unit-ball point into the PHS.");
    }
}  // namespace vamp::binding
