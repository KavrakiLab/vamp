#include <vamp/planning/roadmap.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/rrt_star_settings.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/bindings/init.hh>

#include <nanobind/stl/vector.h>

namespace nb = nanobind;
namespace vp = vamp::planning;

void vamp::binding::init_settings(nanobind::module_ &pymodule)
{
    nb::class_<vp::RRTCSettings>(pymodule, "RRTCSettings")
        .def(nb::init<>())
        .def_rw("range", &vp::RRTCSettings::range)
        .def_rw("dynamic_domain", &vp::RRTCSettings::dynamic_domain)
        .def_rw("radius", &vp::RRTCSettings::radius)
        .def_rw("alpha", &vp::RRTCSettings::alpha)
        .def_rw("min_radius", &vp::RRTCSettings::min_radius)
        .def_rw("balance", &vp::RRTCSettings::balance)
        .def_rw("tree_ratio", &vp::RRTCSettings::tree_ratio)
        .def_rw("max_iterations", &vp::RRTCSettings::max_iterations)
        .def_rw("max_samples", &vp::RRTCSettings::max_samples)
        .def_rw("start_tree_first", &vp::RRTCSettings::start_tree_first)
        .def_rw("rng_skip_iterations", &vp::RRTCSettings::rng_skip_iterations);
    
    nb::class_<vp::RRT_star_settings>(pymodule, "RRT_star_settings")
        .def(nb::init<>())
        .def_rw("range", &vp::RRT_star_settings::range)
        .def_rw("max_iterations", &vp::RRT_star_settings::max_iterations)
        .def_rw("max_samples", &vp::RRT_star_settings::max_samples)
        .def_rw("rng_skip_iterations", &vp::RRT_star_settings::rng_skip_iterations)
        .def_rw("rewire_factor", &vp::RRT_star_settings::rewire_factor)
        .def_rw("force_max_iters", &vp::RRT_star_settings::force_max_iters);

    // TODO: Redesign a neater form of RoadmapSettings/NeighborParams
    // TODO: Expose the other NeighborParams types
    nb::class_<vp::PRMStarNeighborParams>(pymodule, "PRMNeighborParams")
        .def(nb::init<std::size_t, double>())
        .def_rw("dim", &vp::PRMStarNeighborParams::dim)
        .def_rw("space_measure", &vp::PRMStarNeighborParams::space_measure)
        .def_rw("gamma_scale", &vp::PRMStarNeighborParams::gamma_scale)
        .def("max_neighbors", &vp::PRMStarNeighborParams::max_neighbors)
        .def("neighbor_radius", &vp::PRMStarNeighborParams::neighbor_radius);

    using PRMStarSettings = vp::RoadmapSettings<vp::PRMStarNeighborParams>;
    nb::class_<PRMStarSettings>(pymodule, "PRMSettings")
        .def(nb::init<vp::PRMStarNeighborParams>())
        .def_rw("max_iterations", &PRMStarSettings::max_iterations)
        .def_rw("max_samples", &PRMStarSettings::max_samples)
        .def_rw("rng_skip_iterations", &PRMStarSettings::rng_skip_iterations)
        .def_rw("neighbor_params", &PRMStarSettings::neighbor_params)
        .def("max_neighbors", &PRMStarSettings::max_neighbors)
        .def("neighbor_radius", &PRMStarSettings::neighbor_radius);

    nb::enum_<vp::SimplifyRoutine>(pymodule, "SimplifyRoutine")
        .value("BSPLINE", vp::SimplifyRoutine::BSPLINE)
        .value("REDUCE", vp::SimplifyRoutine::REDUCE)
        .value("SHORTCUT", vp::SimplifyRoutine::SHORTCUT)
        .value("PERTURB", vp::SimplifyRoutine::PERTURB)
        .export_values();

    nb::class_<vp::BSplineSettings>(pymodule, "BSplineSettings")
        .def(nb::init<>())
        .def_rw("max_steps", &vp::BSplineSettings::max_steps)
        .def_rw("min_change", &vp::BSplineSettings::min_change)
        .def_rw("midpoint_interpolation", &vp::BSplineSettings::midpoint_interpolation);

    nb::class_<vp::ReduceSettings>(pymodule, "ReduceSettings")
        .def(nb::init<>())
        .def_rw("max_steps", &vp::ReduceSettings::max_steps)
        .def_rw("max_empty_steps", &vp::ReduceSettings::max_empty_steps)
        .def_rw("range_ratio", &vp::ReduceSettings::range_ratio);

    nb::class_<vp::ShortcutSettings>(pymodule, "ShortcutSettings").def(nb::init<>());

    nb::class_<vp::PerturbSettings>(pymodule, "PerturbSettings")
        .def(nb::init<>())
        .def_rw("max_steps", &vp::PerturbSettings::max_steps)
        .def_rw("max_empty_steps", &vp::PerturbSettings::max_empty_steps)
        .def_rw("perturbation_attempts", &vp::PerturbSettings::perturbation_attempts)
        .def_rw("range_ratio", &vp::PerturbSettings::range);

    nb::class_<vp::SimplifySettings>(pymodule, "SimplifySettings")
        .def(nb::init<>())
        .def_rw("max_iterations", &vp::SimplifySettings::max_iterations)
        .def_rw("operations", &vp::SimplifySettings::operations)
        .def_rw("reduce", &vp::SimplifySettings::reduce)
        .def_rw("shortcut", &vp::SimplifySettings::shortcut)
        .def_rw("perturb", &vp::SimplifySettings::perturb)
        .def_rw("bspline", &vp::SimplifySettings::bspline);
}
