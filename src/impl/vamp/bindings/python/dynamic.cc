#include <vamp_python_init.hh>

#include <vamp/bindings/python/api_binder.hh>
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
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace
{
    namespace vj = vamp::jit;
    namespace vp = vamp::planning;

    using ConfigNd = nb::ndarray<const float, nb::ndim<1>, nb::device::cpu>;
    using PathNd = nb::ndarray<const float, nb::ndim<2>, nb::device::cpu>;
}  // namespace

namespace vamp::binding
{
    struct VectorConfig
    {
        using Type = std::vector<float>;

        static auto as_ptr(const Type &v, std::size_t dim, std::vector<float> & /*scratch*/, const char *what)
            -> const float *
        {
            if (v.size() != dim)
            {
                throw std::runtime_error(std::string(what) + " has wrong dimension");
            }
            return v.data();
        }
    };

    struct NDArrayConfig
    {
        using Type = ConfigNd;

        static auto as_ptr(const Type &a, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> const float *
        {
            return as_flat_1d(a, dim, scratch, what);
        }
    };

    struct VectorPath
    {
        using Type = std::vector<std::vector<float>>;

        static auto as_ptr(const Type &v, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
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
    };

    struct NDArrayPath
    {
        using Type = PathNd;

        static auto as_ptr(const Type &a, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            return as_flat_2d(a, dim, scratch, what);
        }
    };

    struct VectorPointcloud
    {
        using Type = std::vector<vamp::collision::Point>;

        static auto as_ptr(const Type &v, std::vector<float> & /*scratch*/, const char * /*what*/)
            -> std::pair<const float *, std::uint64_t>
        {
            return {v.empty() ? nullptr : v.front().data(), v.size()};
        }
    };

    struct NDArrayPointcloud
    {
        using Type = PathNd;

        static auto as_ptr(const Type &a, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            return as_flat_2d(a, 3, scratch, what);
        }
    };

    inline auto to_waypoint(vj::DynamicPath &p, std::size_t sz, const float *src, bool allow_init)
        -> std::vector<float>
    {
        if (p.dim == 0 and allow_init)
        {
            p.dim = sz;
        }
        else if (sz != p.dim)
        {
            throw std::runtime_error("waypoint has wrong dimension");
        }
        return std::vector<float>(src, src + p.dim);
    }

    inline auto vec_waypoint(vj::DynamicPath &p, const std::vector<float> &c, bool allow_init)
        -> std::vector<float>
    {
        return to_waypoint(p, c.size(), c.data(), allow_init);
    }

    inline auto nd_waypoint(vj::DynamicPath &p, const ConfigNd &c, bool allow_init) -> std::vector<float>
    {
        std::vector<float> scratch;
        const auto sz = c.shape(0);
        const auto *ptr = as_flat_1d(c, p.dim == 0 ? sz : p.dim, scratch, "waypoint");
        return to_waypoint(p, sz, ptr, allow_init);
    }

    inline void check_idx(const vj::DynamicPath &p, std::size_t i)
    {
        if (i >= p.waypoints.size())
        {
            throw nb::index_error();
        }
    }

    template <typename ConfigInput, typename PathInput, typename PcInput>
    struct DynamicRobotTraits
    {
        using Cfg = typename ConfigInput::Type;
        using Pth = typename PathInput::Type;
        using Pc = typename PcInput::Type;
        using Path = vj::DynamicPath;
        using PlanningResult = vj::DynamicPlanResult;
        using Sampler = vj::DynamicSampler;
        using Phs = vj::DynamicPhs;
        using Env = vamp::collision::Environment<float>;
        using PRMSettings = vp::RoadmapSettings<vp::PRMStarNeighborParams>;
        using FCITSettings = vp::RoadmapSettings<vp::FCITStarNeighborParams>;

        template <vp::Planner P, typename Settings>
        static auto solve_single(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            return vj::solve(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                ConfigInput::as_ptr(goal, d, gs, "goal"),
                env,
                settings,
                *sampler);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_multi(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            auto [gptr, n] = PathInput::as_ptr(goals, d, gs, "goals");
            return vj::solve_multi(
                self, P, ConfigInput::as_ptr(start, d, ss, "start"), gptr, n, env, settings, *sampler);
        }

        static auto simplify(
            std::shared_ptr<vj::DynamicRobot> self,
            const Pth &path,
            const Env &env,
            const vp::SimplifySettings &settings,
            std::shared_ptr<Sampler> sampler) -> PlanningResult
        {
            std::vector<float> scratch;
            auto [pptr, n] = PathInput::as_ptr(path, self->dimension(), scratch, "path");
            return vj::simplify(self, pptr, n, env, settings, *sampler);
        }

        static auto fk(vj::DynamicRobot &self, const Cfg &c) -> std::vector<vamp::collision::Sphere<float>>
        {
            std::vector<float> scratch;
            return vj::fk(self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"));
        }

        static auto eefk(vj::DynamicRobot &self, const Cfg &c) -> Eigen::Matrix4f
        {
            std::vector<float> scratch;
            return vj::eefk(self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"));
        }

        static auto debug(vj::DynamicRobot &self, const Cfg &c, const Env &env) -> vj::DebugType
        {
            std::vector<float> scratch;
            return vj::debug(self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"), env);
        }

        static auto validate(vj::DynamicRobot &self, const Cfg &c, const Env &env, bool check_bounds) -> bool
        {
            std::vector<float> scratch;
            return vj::validate(
                self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"), env, check_bounds);
        }

        static auto validate_motion(
            vj::DynamicRobot &self,
            const Cfg &c_in,
            const Cfg &c_out,
            const Env &env,
            bool check_bounds) -> bool
        {
            std::vector<float> s_in, s_out;
            return vj::validate_motion(
                self,
                ConfigInput::as_ptr(c_in, self.dimension(), s_in, "configuration_in"),
                ConfigInput::as_ptr(c_out, self.dimension(), s_out, "configuration_out"),
                env,
                check_bounds);
        }

        static auto filter_self_from_pointcloud(
            vj::DynamicRobot &self,
            const Pc &pc,
            float point_radius,
            const Cfg &c,
            const Env &env) -> std::vector<vamp::collision::Point>
        {
            std::vector<float> cfg_scratch, pc_scratch;
            auto [pptr, n] = PcInput::as_ptr(pc, pc_scratch, "pc");
            return vj::filter_self_from_pointcloud(
                self,
                pptr,
                n,
                point_radius,
                ConfigInput::as_ptr(c, self.dimension(), cfg_scratch, "config"),
                env);
        }

        static auto make_halton(std::shared_ptr<vj::DynamicRobot> self) -> std::shared_ptr<Sampler>
        {
            return vj::make_halton_sampler(std::move(self));
        }

        static auto make_xorshift(std::shared_ptr<vj::DynamicRobot> self, std::uint64_t seed)
            -> std::shared_ptr<Sampler>
        {
            return vj::make_xorshift_sampler(std::move(self), seed);
        }

        static auto make_phs(std::shared_ptr<vj::DynamicRobot> self, const Cfg &focus_a, const Cfg &focus_b)
            -> std::shared_ptr<Phs>
        {
            const auto d = self->dimension();
            std::vector<float> sa, sb;
            return vj::make_phs(
                self,
                ConfigInput::as_ptr(focus_a, d, sa, "focus_a"),
                ConfigInput::as_ptr(focus_b, d, sb, "focus_b"));
        }

        static auto make_phs_sampler(
            std::shared_ptr<vj::DynamicRobot> self,
            const Phs &phs,
            std::shared_ptr<Sampler> inner) -> std::shared_ptr<Sampler>
        {
            return vj::make_phs_sampler(std::move(self), phs, *inner);
        }

        static auto path_len(const Path &p) -> std::size_t
        {
            return p.waypoints.size();
        }

        static auto path_get(const Path &p, std::size_t i)
        {
            check_idx(p, i);
            return make_ndarray<1>(p.waypoints[i].data(), {p.dim});
        }

        static auto path_cost(const Path &p) -> float
        {
            return p.cost();
        }

        static void path_subdivide(Path &p)
        {
            p.subdivide();
        }

        static void path_interpolate_to_n_states(Path &p, std::size_t n)
        {
            p.interpolate_to_n_states(n);
        }

        static void path_interpolate_to_resolution(Path &p, std::size_t r)
        {
            p.interpolate_to_resolution(r);
        }

        static auto path_validate(Path &p, const Env &e) -> bool
        {
            return p.validate(e);
        }

        static auto path_numpy(const Path &p)
        {
            const auto n = p.waypoints.size();
            return make_ndarray_filled<2>(
                {n, p.dim},
                [&](float *dst)
                {
                    for (std::size_t i = 0; i < n; ++i)
                    {
                        std::memcpy(dst + i * p.dim, p.waypoints[i].data(), p.dim * sizeof(float));
                    }
                });
        }

        static void path_set(Path &p, std::size_t i, const Cfg &c)
        {
            check_idx(p, i);
            if constexpr (std::is_same_v<ConfigInput, VectorConfig>)
            {
                p.waypoints[i] = vec_waypoint(p, c, false);
            }
            else
            {
                p.waypoints[i] = nd_waypoint(p, c, false);
            }
        }

        static void path_append(Path &p, const Cfg &c)
        {
            if constexpr (std::is_same_v<ConfigInput, VectorConfig>)
            {
                p.waypoints.emplace_back(vec_waypoint(p, c, true));
            }
            else
            {
                p.waypoints.emplace_back(nd_waypoint(p, c, true));
            }
        }

        static void path_insert(Path &p, std::size_t i, const Cfg &c)
        {
            if constexpr (std::is_same_v<ConfigInput, VectorConfig>)
            {
                p.waypoints.insert(p.waypoints.cbegin() + i, vec_waypoint(p, c, true));
            }
            else
            {
                p.waypoints.insert(p.waypoints.cbegin() + i, nd_waypoint(p, c, true));
            }
        }

        static auto result_solved(const PlanningResult &r) -> bool
        {
            return r.solved();
        }

        static auto result_path(const PlanningResult &r) -> std::shared_ptr<Path>
        {
            return r.path;
        }

        static auto result_nanoseconds(const PlanningResult &r) -> std::uint64_t
        {
            return r.nanoseconds;
        }

        static auto result_iterations(const PlanningResult &r) -> std::size_t
        {
            return r.iterations;
        }

        static auto result_size(const PlanningResult &r) -> std::vector<std::size_t>
        {
            return r.size;
        }

        static void sampler_reset(Sampler &s)
        {
            s.reset();
        }

        static void sampler_skip(Sampler &s, std::size_t n)
        {
            s.skip(n);
        }

        static auto sampler_next(Sampler &s)
        {
            const auto dim = s.robot->dimension();
            std::vector<float> out(dim);
            s.next(out.data());
            return make_ndarray<1>(out.data(), {dim});
        }

        static void phs_set_transverse_diameter(Phs &p, float d)
        {
            p.set_transverse_diameter(d);
        }

        static auto phs_transform(Phs &phs, const Cfg &x) -> nb::ndarray<nb::numpy, float, nb::device::cpu>
        {
            const auto dim = phs.robot->dimension();
            std::vector<float> scratch;
            const auto *xptr = ConfigInput::as_ptr(x, dim, scratch, "x");
            std::vector<float> out(dim);
            phs.transform(xptr, out.data());
            return make_ndarray<1>(out.data(), {dim});
        }
    };

    using DTV = DynamicRobotTraits<VectorConfig, VectorPath, VectorPointcloud>;
    using DTN = DynamicRobotTraits<NDArrayConfig, NDArrayPath, NDArrayPointcloud>;

    void init_dynamic(nb::module_ &pymodule)
    {
        // Re-dlopen _core_ext.so with RTLD_GLOBAL so template instantiations
        // become visible to the JIT's process-symbol search — Python loads
        // extension modules with RTLD_LOCAL by default.
        Dl_info info{};
        if (dladdr(reinterpret_cast<void *>(&init_dynamic), &info) == 0 or info.dli_fname == nullptr)
        {
            throw std::runtime_error("Failed to promote RTLD to global.");
        }
        dlopen(info.dli_fname, RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD);

        auto path_k = bind_path_class<DTV>(pymodule, "DynamicPath");
        bind_path_io<DTV>(path_k);
        bind_path_io<DTN>(path_k);

        bind_planning_result<DTV>(pymodule, "DynamicPlanResult");
        bind_sampler<DTV>(pymodule, "DynamicSampler");

        auto phs_k = bind_phs_class<DTV>(pymodule, "DynamicPhs");
        bind_phs_io<DTV>(phs_k);
        bind_phs_io<DTN>(phs_k);

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
                    [](const vj::DynamicRobot &self)
                    {
                        const auto &b = self.upper_bounds();
                        return make_ndarray<1>(b.data(), {b.size()});
                    })
                .def(
                    "lower_bounds",
                    [](const vj::DynamicRobot &self)
                    {
                        const auto &b = self.lower_bounds();
                        return make_ndarray<1>(b.data(), {b.size()});
                    });

        bind_robot_methods<DTV>(klass);
        bind_robot_methods<DTN>(klass);

        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const vj::DynamicPath &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               std::shared_ptr<vj::DynamicSampler> sampler)
            { return DTV::simplify(self, path.waypoints, env, settings, sampler); },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "JIT'd path simplification (DynamicPath input).");

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
            "name"_a = std::string("DynamicRobot"),
            "JIT-compile a robot from a URDF.");
    }
}  // namespace vamp::binding
