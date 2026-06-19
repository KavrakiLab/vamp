#include <vamp_python_init.hh>

#include <vamp/bindings/python/array_helpers.hh>
#include <vamp/jit/api.hh>
#include <vamp/jit/build_paths.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>

#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/planning/aorrtc_settings.hh>
#include <vamp/planning/grrtstar_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

#include <cricket/codegen.hh>

#include <Eigen/Dense>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <dlfcn.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace
{
    namespace vj = vamp::jit;
    namespace vjf = vamp::jit::ffi;
    namespace vp = vamp::planning;

    // Configuration inputs accept either a Python list (std::vector<float>)
    // or a 1-D numpy ndarray — mirroring the static MF() dual-overload pattern.
    using ConfigNd = nb::ndarray<const float, nb::ndim<1>, nb::device::cpu>;

    // Path / goals inputs accept either a list-of-lists or a 2-D numpy ndarray.
    using PathNd = nb::ndarray<const float, nb::ndim<2>, nb::device::cpu>;

    // ---- input-extraction helpers -----------------------------------------
    //
    // Each `as_config` / `as_path` / `as_points` overload returns a (data*, n)
    // view into either a Python list/list-of-lists or a numpy ndarray. For
    // ndarrays we tolerate any stride layout: if it's not C-contiguous we
    // copy into the supplied scratch buffer. Lists are always contiguous by
    // construction. These are Python-binding-specific and stay here; C++
    // callers use the vamp::jit::* API directly with raw float pointers.

    auto as_config(
        const std::vector<float> &v,
        std::size_t dim,
        std::vector<float> & /*scratch*/,
        const char *what) -> const float *
    {
        if (v.size() != dim)
        {
            throw std::runtime_error(std::string(what) + " has wrong dimension");
        }
        return v.data();
    }

    // ndarray overloads delegate to the shared helpers in
    // vamp/bindings/python/array_helpers.hh (used by the static side too).
    auto as_config(const ConfigNd &a, std::size_t dim, std::vector<float> &scratch, const char *what)
        -> const float *
    {
        return vamp::binding::as_flat_1d(a, dim, scratch, what);
    }

    auto as_path(
        const std::vector<std::vector<float>> &v,
        std::size_t dim,
        std::vector<float> &scratch,
        const char *what) -> std::pair<const float *, std::uint64_t>
    {
        scratch.clear();
        scratch.reserve(dim * v.size());
        for (const auto &wp : v)
        {
            if (wp.size() != dim)
            {
                throw std::runtime_error(std::string(what) + " has wrong waypoint dimension");
            }
            scratch.insert(scratch.end(), wp.begin(), wp.end());
        }
        return {scratch.data(), v.size()};
    }

    auto as_path(const PathNd &a, std::size_t dim, std::vector<float> &scratch, const char *what)
        -> std::pair<const float *, std::uint64_t>
    {
        return vamp::binding::as_flat_2d(a, dim, scratch, what);
    }

    auto as_points(const std::vector<vamp::collision::Point> &v, std::vector<float> &scratch, const char *)
        -> std::pair<const float *, std::uint64_t>
    {
        scratch.clear();
        scratch.reserve(v.size() * 3);
        for (const auto &p : v)
        {
            scratch.insert(scratch.end(), p.begin(), p.end());
        }
        return {scratch.data(), v.size()};
    }

    auto as_points(const PathNd &a, std::vector<float> &scratch, const char *what)
        -> std::pair<const float *, std::uint64_t>
    {
        return vamp::binding::as_flat_2d(a, 3, scratch, what);
    }
}  // namespace

namespace vamp::binding
{
    void init_dynamic(nb::module_ &pymodule)
    {
        // Re-dlopen _core_ext.so with RTLD_GLOBAL so template instantiations
        // become visible to the JIT's process-symbol search. Python loads
        // extension modules with RTLD_LOCAL by default.
        Dl_info info{};
        if (dladdr(reinterpret_cast<void *>(&init_dynamic), &info) == 0 or info.dli_fname == nullptr)
        {
            throw std::runtime_error("Failed to promote RTLD to global.");
        }
        dlopen(info.dli_fname, RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD);

        // ---- DynamicPath -----------------------------------------------
        nb::class_<vj::DynamicPath>(pymodule, "DynamicPath")
            .def(nb::init<>(), "Default constructor, creates empty path.")
            .def(
                "__len__",
                [](const vj::DynamicPath &p) { return p.waypoints.size(); },
                "Return the number of waypoints in the path.")
            .def(
                "__getitem__",
                [](const vj::DynamicPath &p, std::size_t i) -> nb::ndarray<nb::numpy, float, nb::device::cpu>
                {
                    if (i >= p.waypoints.size())
                    {
                        throw nb::index_error();
                    }
                    return vamp::binding::make_ndarray<1>(p.waypoints[i].data(), {p.dim});
                },
                "Get the i-th configuration in the path.")
            .def(
                "__setitem__",
                [](vj::DynamicPath &p, std::size_t i, const std::vector<float> &c)
                {
                    if (i >= p.waypoints.size())
                    {
                        throw nb::index_error();
                    }
                    if (p.dim != 0 and c.size() != p.dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    p.waypoints[i] = c;
                },
                "Set the i-th configuration of the path.")
            .def(
                "__setitem__",
                [](vj::DynamicPath &p, std::size_t i, const ConfigNd &c)
                {
                    if (i >= p.waypoints.size())
                    {
                        throw nb::index_error();
                    }
                    if (p.dim != 0 and c.shape(0) != p.dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    std::vector<float> tmp(c.shape(0));
                    for (std::size_t j = 0; j < c.shape(0); ++j)
                    {
                        tmp[j] = c(j);
                    }
                    p.waypoints[i] = std::move(tmp);
                },
                "Set the i-th configuration of the path.")
            .def(
                "append",
                [](vj::DynamicPath &p, const std::vector<float> &c)
                {
                    if (p.dim == 0)
                    {
                        p.dim = c.size();
                    }
                    else if (c.size() != p.dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    p.waypoints.emplace_back(c);
                },
                "Append a configuration to the end of this path.")
            .def(
                "append",
                [](vj::DynamicPath &p, const ConfigNd &c)
                {
                    if (p.dim == 0)
                    {
                        p.dim = c.shape(0);
                    }
                    else if (c.shape(0) != p.dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    std::vector<float> tmp(c.shape(0));
                    for (std::size_t j = 0; j < c.shape(0); ++j)
                    {
                        tmp[j] = c(j);
                    }
                    p.waypoints.emplace_back(std::move(tmp));
                },
                "Append a configuration to the end of this path.")
            .def(
                "insert",
                [](vj::DynamicPath &p, std::size_t i, const std::vector<float> &c)
                {
                    if (p.dim == 0)
                    {
                        p.dim = c.size();
                    }
                    else if (c.size() != p.dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    p.waypoints.insert(p.waypoints.cbegin() + i, c);
                },
                "Insert a configuration at index i.")
            .def(
                "insert",
                [](vj::DynamicPath &p, std::size_t i, const ConfigNd &c)
                {
                    if (p.dim == 0)
                    {
                        p.dim = c.shape(0);
                    }
                    else if (c.shape(0) != p.dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    std::vector<float> tmp(c.shape(0));
                    for (std::size_t j = 0; j < c.shape(0); ++j)
                    {
                        tmp[j] = c(j);
                    }
                    p.waypoints.insert(p.waypoints.cbegin() + i, std::move(tmp));
                },
                "Insert a configuration at index i.")
            .def(
                "cost", &vj::DynamicPath::cost, "Compute the total path length (by the l2-norm) of the path.")
            .def(
                "subdivide",
                &vj::DynamicPath::subdivide,
                "Subdivide the path by inserting a configuration at the midpoint of every segment.")
            .def(
                "interpolate_to_n_states",
                &vj::DynamicPath::interpolate_to_n_states,
                "n"_a,
                "Refine the path by interpolating to n states as evenly as possible.")
            .def(
                "interpolate_to_resolution",
                &vj::DynamicPath::interpolate_to_resolution,
                "resolution"_a,
                "Refine the path by interpolating segments up to the given resolution.")
            .def(
                "validate",
                &vj::DynamicPath::validate,
                "environment"_a,
                "Validate the path in an environment.")
            .def(
                "numpy",
                [](const vj::DynamicPath &p)
                {
                    // Flatten the row vectors into a contiguous buffer, then
                    // wrap as an (n, dim) ndarray via the shared helper.
                    const auto n = p.waypoints.size();
                    std::vector<float> flat(n * p.dim);
                    for (std::size_t i = 0; i < n; ++i)
                    {
                        std::memcpy(
                            flat.data() + i * p.dim, p.waypoints[i].data(), p.dim * sizeof(float));
                    }
                    return vamp::binding::make_ndarray<2>(flat.data(), {n, p.dim});
                },
                "Convert this path to an (n_waypoints, dimension) numpy array.");

        // ---- DynamicPlanResult ---------------------------------------------
        nb::class_<vj::DynamicPlanResult>(pymodule, "DynamicPlanResult")
            .def(nb::init<>(), "Empty constructor.")
            .def_prop_ro("solved", &vj::DynamicPlanResult::solved, "Returns true if solution found.")
            .def_ro("path", &vj::DynamicPlanResult::path, "The solution path, if found.")
            .def_ro("nanoseconds", &vj::DynamicPlanResult::nanoseconds, "Nanoseconds taken to find the path.")
            .def_ro(
                "iterations",
                &vj::DynamicPlanResult::iterations,
                "Number of planner iterations used to find the path.");

        // ---- DynamicSampler -----------------------------------------------
        nb::class_<vj::DynamicSampler>(pymodule, "DynamicSampler")
            .def("reset", &vj::DynamicSampler::reset, "Reset the sampler to its initial state.")
            .def("skip", &vj::DynamicSampler::skip, "n"_a, "Skip the next n samples.")
            .def(
                "next",
                [](vj::DynamicSampler &s)
                {
                    const auto dim = s.robot->dimension();
                    std::vector<float> out(dim);
                    s.next(out.data());
                    return vamp::binding::make_ndarray<1>(out.data(), {dim});
                },
                "Sample the next configuration.");

        // ---- DynamicPhs ----------------------------------------------------
        nb::class_<vj::DynamicPhs>(pymodule, "DynamicPhs")
            .def("set_transverse_diameter", &vj::DynamicPhs::set_transverse_diameter, "diameter"_a)
            .def(
                "transform",
                [](vj::DynamicPhs &p, const std::vector<float> &x)
                {
                    const auto dim = p.robot->dimension();
                    std::vector<float> scratch;
                    auto *xptr = as_config(x, dim, scratch, "x");
                    std::vector<float> out(dim);
                    p.transform(xptr, out.data());
                    return vamp::binding::make_ndarray<1>(out.data(), {dim});
                },
                "x"_a)
            .def(
                "transform",
                [](vj::DynamicPhs &p, const ConfigNd &x)
                {
                    const auto dim = p.robot->dimension();
                    std::vector<float> scratch;
                    auto *xptr = as_config(x, dim, scratch, "x");
                    std::vector<float> out(dim);
                    p.transform(xptr, out.data());
                    return vamp::binding::make_ndarray<1>(out.data(), {dim});
                },
                "x"_a);

        // ---- DynamicRobot --------------------------------------------------

        using PRMSettings = vp::RoadmapSettings<vp::PRMStarNeighborParams>;
        using FCITSettings = vp::RoadmapSettings<vp::FCITStarNeighborParams>;

        auto klass =
            nb::class_<vj::DynamicRobot>(pymodule, "DynamicRobot")
                .def_prop_ro("dimension", &vj::DynamicRobot::dimension)
                .def_prop_ro("rake", &vj::DynamicRobot::rake)
                .def_prop_ro("n_spheres", &vj::DynamicRobot::n_spheres)
                .def_prop_ro("space_measure", &vj::DynamicRobot::space_measure, "Measure of robot's C-space.")
                .def_prop_ro(
                    "joint_names",
                    &vj::DynamicRobot::joint_names,
                    "Joint names for the robot in order of DoF.")
                .def(
                    "min_max_radii",
                    [](const vj::DynamicRobot &self) -> std::pair<float, float>
                    { return {self.min_radius(), self.max_radius()}; },
                    "Minimum and maximum radii sizes of robot spheres.")
                .def(
                    "upper_bounds",
                    [](const vj::DynamicRobot &self) { { const auto &b = self.upper_bounds(); return vamp::binding::make_ndarray<1>(b.data(), {b.size()}); } })
                .def(
                    "lower_bounds",
                    [](const vj::DynamicRobot &self) { { const auto &b = self.lower_bounds(); return vamp::binding::make_ndarray<1>(b.data(), {b.size()}); } });

        // Per-planner overloads: list × {single, multi} and ndarray × {single, multi}.
        // Each lambda just normalises inputs and calls vamp::jit::solve / solve_multi.
#define VAMP_JIT_DEF_PLANNER(KLASS, NAME, ENUM, SETTINGS)                                                    \
    KLASS                                                                                                    \
        .def(                                                                                                \
            NAME,                                                                                            \
            [](std::shared_ptr<vj::DynamicRobot> self,                                                       \
               const std::vector<float> &start,                                                              \
               const std::vector<float> &goal,                                                               \
               const vamp::collision::Environment<float> &env,                                               \
               const SETTINGS &settings,                                                                     \
               vj::DynamicSampler &sampler) -> vj::DynamicPlanResult                                         \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                return vj::solve(                                                                            \
                    self,                                                                                    \
                    ENUM,                                                                                    \
                    as_config(start, d, s_scratch, "start"),                                                 \
                    as_config(goal, d, g_scratch, "goal"),                                                   \
                    env,                                                                                     \
                    settings,                                                                                \
                    sampler);                                                                                \
            },                                                                                               \
            "start"_a,                                                                                       \
            "goal"_a,                                                                                        \
            "environment"_a,                                                                                 \
            "settings"_a,                                                                                    \
            "sampler"_a,                                                                                     \
            "JIT'd " NAME)                                                                                   \
        .def(                                                                                                \
            NAME,                                                                                            \
            [](std::shared_ptr<vj::DynamicRobot> self,                                                       \
               const ConfigNd &start,                                                                        \
               const ConfigNd &goal,                                                                         \
               const vamp::collision::Environment<float> &env,                                               \
               const SETTINGS &settings,                                                                     \
               vj::DynamicSampler &sampler) -> vj::DynamicPlanResult                                         \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                return vj::solve(                                                                            \
                    self,                                                                                    \
                    ENUM,                                                                                    \
                    as_config(start, d, s_scratch, "start"),                                                 \
                    as_config(goal, d, g_scratch, "goal"),                                                   \
                    env,                                                                                     \
                    settings,                                                                                \
                    sampler);                                                                                \
            },                                                                                               \
            "start"_a,                                                                                       \
            "goal"_a,                                                                                        \
            "environment"_a,                                                                                 \
            "settings"_a,                                                                                    \
            "sampler"_a,                                                                                     \
            "JIT'd " NAME)                                                                                   \
        .def(                                                                                                \
            NAME,                                                                                            \
            [](std::shared_ptr<vj::DynamicRobot> self,                                                       \
               const std::vector<float> &start,                                                              \
               const std::vector<std::vector<float>> &goals,                                                 \
               const vamp::collision::Environment<float> &env,                                               \
               const SETTINGS &settings,                                                                     \
               vj::DynamicSampler &sampler) -> vj::DynamicPlanResult                                         \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                auto [gptr, n] = as_path(goals, d, g_scratch, "goals");                                      \
                return vj::solve_multi(                                                                      \
                    self, ENUM, as_config(start, d, s_scratch, "start"), gptr, n, env, settings, sampler);   \
            },                                                                                               \
            "start"_a,                                                                                       \
            "goal"_a,                                                                                        \
            "environment"_a,                                                                                 \
            "settings"_a,                                                                                    \
            "sampler"_a,                                                                                     \
            "JIT'd " NAME)                                                                                   \
        .def(                                                                                                \
            NAME,                                                                                            \
            [](std::shared_ptr<vj::DynamicRobot> self,                                                       \
               const ConfigNd &start,                                                                        \
               const PathNd &goals,                                                                          \
               const vamp::collision::Environment<float> &env,                                               \
               const SETTINGS &settings,                                                                     \
               vj::DynamicSampler &sampler) -> vj::DynamicPlanResult                                         \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                auto [gptr, n] = as_path(goals, d, g_scratch, "goals");                                      \
                return vj::solve_multi(                                                                      \
                    self, ENUM, as_config(start, d, s_scratch, "start"), gptr, n, env, settings, sampler);   \
            },                                                                                               \
            "start"_a,                                                                                       \
            "goal"_a,                                                                                        \
            "environment"_a,                                                                                 \
            "settings"_a,                                                                                    \
            "sampler"_a,                                                                                     \
            "JIT'd " NAME)

        VAMP_JIT_DEF_PLANNER(klass, "rrtc", vp::Planner::RRTC, vp::RRTCSettings);
        VAMP_JIT_DEF_PLANNER(klass, "prm", vp::Planner::PRM, PRMSettings);
        VAMP_JIT_DEF_PLANNER(klass, "fcit", vp::Planner::FCIT, FCITSettings);
        VAMP_JIT_DEF_PLANNER(klass, "aorrtc", vp::Planner::AORRTC, vp::AORRTCSettings);
        VAMP_JIT_DEF_PLANNER(klass, "grrtstar", vp::Planner::GRRTSTAR, vp::GRRTStarSettings);

#undef VAMP_JIT_DEF_PLANNER

        // Sampler factories — mirror vamp.<robot>.halton() / xorshift().
        klass.def(
            "halton",
            [](std::shared_ptr<vj::DynamicRobot> self) { return vj::make_halton_sampler(std::move(self)); },
            "Create a Halton sampler for this robot.");
        klass.def(
            "xorshift",
            [](std::shared_ptr<vj::DynamicRobot> self, std::uint64_t seed)
            { return vj::make_xorshift_sampler(std::move(self), seed); },
            "seed"_a = 0,
            "Create an XORShift sampler for this robot.");

        // debug — mirror vamp.<robot>.debug(). list and ndarray overloads.
        klass.def(
            "debug",
            [](vj::DynamicRobot &self,
               const std::vector<float> &config,
               const vamp::collision::Environment<float> &env)
            {
                std::vector<float> scratch;
                return vj::debug(self, as_config(config, self.dimension(), scratch, "config"), env);
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Check which spheres of a robot configuration are in collision (JIT).");
        klass.def(
            "debug",
            [](vj::DynamicRobot &self, const ConfigNd &config, const vamp::collision::Environment<float> &env)
            {
                std::vector<float> scratch;
                return vj::debug(self, as_config(config, self.dimension(), scratch, "config"), env);
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Check which spheres of a robot configuration are in collision (JIT).");

        // eefk — mirror vamp.<robot>.eefk(). Returns a 4x4 Eigen::Matrix4f
        // (nanobind/eigen marshals to numpy).
        klass.def(
            "eefk",
            [](vj::DynamicRobot &self, const std::vector<float> &config)
            {
                std::vector<float> scratch;
                return vj::eefk(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "End-effector forward kinematics. Returns a 4x4 transform (JIT).");
        klass.def(
            "eefk",
            [](vj::DynamicRobot &self, const ConfigNd &config)
            {
                std::vector<float> scratch;
                return vj::eefk(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "End-effector forward kinematics. Returns a 4x4 transform (JIT).");

        // fk — mirror vamp.<robot>.fk(). Returns vector<Sphere<float>>.
        klass.def(
            "fk",
            [](vj::DynamicRobot &self, const std::vector<float> &config)
            {
                std::vector<float> scratch;
                return vj::fk(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "Computes the forward kinematics of the robot (JIT). Returns spheres.");
        klass.def(
            "fk",
            [](vj::DynamicRobot &self, const ConfigNd &config)
            {
                std::vector<float> scratch;
                return vj::fk(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "Computes the forward kinematics of the robot (JIT). Returns spheres.");

        // validate / validate_motion -----------------------------------------
        klass.def(
            "validate",
            [](vj::DynamicRobot &self,
               const std::vector<float> &config,
               const vamp::collision::Environment<float> &env,
               bool check_bounds)
            {
                std::vector<float> scratch;
                return self.validate(
                    as_config(config, self.dimension(), scratch, "config"),
                    static_cast<const void *>(&env),
                    check_bounds);
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "check_bounds"_a = false,
            "Check if a configuration is valid (JIT).");
        klass.def(
            "validate",
            [](vj::DynamicRobot &self,
               const ConfigNd &config,
               const vamp::collision::Environment<float> &env,
               bool check_bounds)
            {
                std::vector<float> scratch;
                return self.validate(
                    as_config(config, self.dimension(), scratch, "config"),
                    static_cast<const void *>(&env),
                    check_bounds);
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "check_bounds"_a = false,
            "Check if a configuration is valid (JIT).");

        klass.def(
            "validate_motion",
            [](vj::DynamicRobot &self,
               const std::vector<float> &c_in,
               const std::vector<float> &c_out,
               const vamp::collision::Environment<float> &env,
               bool check_bounds)
            {
                std::vector<float> s_in, s_out;
                return self.validate_motion(
                    as_config(c_in, self.dimension(), s_in, "configuration_in"),
                    as_config(c_out, self.dimension(), s_out, "configuration_out"),
                    static_cast<const void *>(&env),
                    check_bounds);
            },
            "configuration_in"_a,
            "configuration_out"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "check_bounds"_a = true,
            "Check if a configuration-to-configuration motion is valid (JIT).");
        klass.def(
            "validate_motion",
            [](vj::DynamicRobot &self,
               const ConfigNd &c_in,
               const ConfigNd &c_out,
               const vamp::collision::Environment<float> &env,
               bool check_bounds)
            {
                std::vector<float> s_in, s_out;
                return self.validate_motion(
                    as_config(c_in, self.dimension(), s_in, "configuration_in"),
                    as_config(c_out, self.dimension(), s_out, "configuration_out"),
                    static_cast<const void *>(&env),
                    check_bounds);
            },
            "configuration_in"_a,
            "configuration_out"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "check_bounds"_a = true,
            "Check if a configuration-to-configuration motion is valid (JIT).");

        // filter_self_from_pointcloud ----------------------------------------
        klass.def(
            "filter_self_from_pointcloud",
            [](vj::DynamicRobot &self,
               const std::vector<vamp::collision::Point> &pc,
               float point_radius,
               const std::vector<float> &config,
               const vamp::collision::Environment<float> &env)
            {
                std::vector<float> cfg_scratch, pc_scratch;
                auto [pptr, n] = as_points(pc, pc_scratch, "pc");
                return vj::filter_self_from_pointcloud(
                    self,
                    pptr,
                    n,
                    point_radius,
                    as_config(config, self.dimension(), cfg_scratch, "config"),
                    env);
            },
            "pc"_a,
            "point_radius"_a,
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Filters points colliding with the robot or environment (JIT).");
        klass.def(
            "filter_self_from_pointcloud",
            [](vj::DynamicRobot &self,
               const PathNd &pc,
               float point_radius,
               const ConfigNd &config,
               const vamp::collision::Environment<float> &env)
            {
                std::vector<float> cfg_scratch, pc_scratch;
                auto [pptr, n] = as_points(pc, pc_scratch, "pc");
                return vj::filter_self_from_pointcloud(
                    self,
                    pptr,
                    n,
                    point_radius,
                    as_config(config, self.dimension(), cfg_scratch, "config"),
                    env);
            },
            "pc"_a,
            "point_radius"_a,
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Filters points colliding with the robot or environment (JIT).");

        // ---- PHS factories + phs_sampler ----------------------------------
        klass.def(
            "phs",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::vector<float> &focus_a,
               const std::vector<float> &focus_b)
            {
                const auto dim = self->dimension();
                std::vector<float> sa, sb;
                return vj::make_phs(
                    self, as_config(focus_a, dim, sa, "focus_a"), as_config(focus_b, dim, sb, "focus_b"));
            },
            "focus_a"_a,
            "focus_b"_a,
            "Construct a prolate hyperspheroid from two foci.");
        klass.def(
            "phs",
            [](std::shared_ptr<vj::DynamicRobot> self, const ConfigNd &focus_a, const ConfigNd &focus_b)
            {
                const auto dim = self->dimension();
                std::vector<float> sa, sb;
                return vj::make_phs(
                    self, as_config(focus_a, dim, sa, "focus_a"), as_config(focus_b, dim, sb, "focus_b"));
            },
            "focus_a"_a,
            "focus_b"_a,
            "Construct a prolate hyperspheroid from two foci.");

        klass.def(
            "phs_sampler",
            [](std::shared_ptr<vj::DynamicRobot> self, const vj::DynamicPhs &phs, vj::DynamicSampler &inner)
            { return vj::make_phs_sampler(std::move(self), phs, inner); },
            "phs"_a,
            "rng"_a,
            "Create a new PHS-rejection sampler wrapping an inner RNG.");

        // ---- simplify ------------------------------------------------------
        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const vj::DynamicPath &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               vj::DynamicSampler &sampler)
            {
                std::vector<float> scratch;
                auto [pptr, n] = as_path(path.waypoints, self->dimension(), scratch, "path");
                return vj::simplify(self, pptr, n, env, settings, sampler);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "JIT'd path simplification.");
        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::vector<std::vector<float>> &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               vj::DynamicSampler &sampler)
            {
                std::vector<float> scratch;
                auto [pptr, n] = as_path(path, self->dimension(), scratch, "path");
                return vj::simplify(self, pptr, n, env, settings, sampler);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "JIT'd path simplification.");
        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const PathNd &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               vj::DynamicSampler &sampler)
            {
                std::vector<float> scratch;
                auto [pptr, n] = as_path(path, self->dimension(), scratch, "path");
                return vj::simplify(self, pptr, n, env, settings, sampler);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "JIT'd path simplification.");

        // ---- load_robot ----------------------------------------------------
        pymodule.def(
            "load_robot",
            [](const std::string &urdf,
               const std::optional<std::string> &srdf,
               const std::string &end_effector,
               const std::vector<std::string> &planners,
               std::size_t rake,
               std::size_t resolution,
               const std::string &name) -> std::shared_ptr<vj::DynamicRobot>
            {
                cricket::GenOptions g;
                g.urdf = urdf;
                if (srdf)
                {
                    g.srdf = std::filesystem::path(*srdf);
                }

                if (not end_effector.empty())
                {
                    g.end_effector = end_effector;
                }

                g.data = {{"name", name}, {"resolution", resolution}};
                auto gen = cricket::generate_robot_source(g);

                vj::LoadOptions opts = vj::default_load_options();
                opts.robot_source = gen.source;
                opts.robot_name = gen.robot_name.empty() ? name : gen.robot_name;
                opts.dimension = gen.dimension;
                opts.rake = rake;
                opts.resolution = resolution;
                for (const auto &p : planners)
                {
                    opts.planners.push_back(vp::planner_from_name(p));
                }

                return std::make_shared<vj::DynamicRobot>(opts);
            },
            "urdf"_a,
            "srdf"_a = nb::none(),
            "end_effector"_a = std::string(""),
            "planners"_a = std::vector<std::string>{"rrtc"},
            "rake"_a = 8,
            "resolution"_a = 32,
            "name"_a = std::string("DynRobot"),
            "JIT-compile a robot from a URDF and return a DynamicRobot handle.");
    }
}  // namespace vamp::binding
