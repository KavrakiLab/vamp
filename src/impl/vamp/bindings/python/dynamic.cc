#include <vamp_python_init.hh>

#include <vamp/jit/build_paths.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>

#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/planning/aorrtc_settings.hh>
#include <vamp/planning/grrtstar_settings.hh>
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

    // Python mirror of an FFI plan result
    struct DynamicPlanResult
    {
        bool success;
        std::size_t dimension;
        std::size_t iterations;
        std::uint64_t nanoseconds;
        float cost;
        std::vector<std::vector<float>> path;
    };

    // Wraps ffi::SamplerHandle along with a shared_ptr to the DynamicRobot that created it,
    // so we can safely free it on dtor.
    struct DynamicSampler
    {
        std::shared_ptr<vj::DynamicRobot> robot;
        vjf::SamplerHandle *handle{nullptr};

        DynamicSampler(std::shared_ptr<vj::DynamicRobot> r, vjf::SamplerHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicSampler()
        {
            if (handle != nullptr and robot)
            {
                robot->sampler_destroy(handle);
            }
        }

        DynamicSampler(const DynamicSampler &) = delete;
        DynamicSampler &operator=(const DynamicSampler &) = delete;
    };

    // Same pattern for vamp::planning::ProlateHyperspheroid<R> behind the
    // ffi::PhsHandle opaque pointer.
    struct DynamicPhs
    {
        std::shared_ptr<vj::DynamicRobot> robot;
        vjf::PhsHandle *handle{nullptr};

        DynamicPhs(std::shared_ptr<vj::DynamicRobot> r, vjf::PhsHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicPhs()
        {
            if (handle != nullptr and robot)
            {
                robot->phs_destroy(handle);
            }
        }

        DynamicPhs(const DynamicPhs &) = delete;
        DynamicPhs &operator=(const DynamicPhs &) = delete;
    };

    auto meta_to_result(const vjf::PlanResultMeta &meta) -> DynamicPlanResult
    {
        DynamicPlanResult result;
        result.success = (meta.success != 0);
        result.dimension = meta.dimension;
        result.iterations = meta.iterations;
        result.nanoseconds = meta.nanoseconds;
        result.cost = meta.cost;
        return result;
    }

    // Drains a planner handle into a DynamicPlanResult, then destroys the
    // handle. The {meta, copy, destroy} triple comes from the per-planner or
    // per-robot symbol set so this routine works for both solve and simplify.
    template <typename MetaFn, typename CopyFn, typename DestroyFn>
    auto drain_handle(
        vjf::PlanResultHandle *handle,
        MetaFn &&meta_call,
        CopyFn &&copy_call,
        DestroyFn &&destroy_call) -> DynamicPlanResult
    {
        auto meta = meta_call(handle);
        auto result = meta_to_result(meta);
        result.path.reserve(meta.waypoints);

        std::vector<float> wp(meta.dimension);
        for (auto i = 0U; i < meta.waypoints; ++i)
        {
            copy_call(handle, i, wp.data());
            result.path.emplace_back(wp);
        }

        destroy_call(handle);
        return result;
    }

    // ---- input-extraction helpers --------------------------------------------
    //
    // Each `as_config` / `as_path` overload returns a (data*, n) view into
    // either a Python list or a numpy ndarray. For ndarrays we tolerate any
    // stride layout: if it's not C-contiguous we copy into the supplied
    // scratch buffer. Lists are always contiguous by construction.

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

    auto as_config(const ConfigNd &a, std::size_t dim, std::vector<float> &scratch, const char *what) -> const
        float *
    {
        if (a.shape(0) != dim)
        {
            throw std::runtime_error(std::string(what) + " has wrong dimension");
        }

        // Strided slices (e.g. wide_buffer[::2]) need to be flattened — reading
        // a.data() linearly would step through interleaved elements.
        if (a.stride(0) != 1)
        {
            scratch.resize(dim);
            for (std::size_t i = 0; i < dim; ++i)
            {
                scratch[i] = a(i);
            }
            return scratch.data();
        }
        return a.data();
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
        if (a.shape(1) != dim)
        {
            throw std::runtime_error(std::string(what) + " has wrong waypoint dimension");
        }

        const std::uint64_t n = a.shape(0);
        const bool contiguous = (a.stride(0) == static_cast<int64_t>(dim) and a.stride(1) == 1);
        if (contiguous)
        {
            return {a.data(), n};
        }

        // Strided input — flatten into scratch.
        scratch.resize(n * dim);
        for (std::uint64_t i = 0; i < n; ++i)
        {
            for (std::size_t j = 0; j < dim; ++j)
            {
                scratch[i * dim + j] = a(i, j);
            }
        }
        return {scratch.data(), n};
    }

    // ---- core dispatch (takes raw pointers so all input shapes share one path)

    template <typename SettingsT>
    auto run_planner_impl(
        vj::DynamicRobot &self,
        vp::Planner planner,
        const float *start,
        const float *goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        auto *handle = self.solve(
            planner,
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        if (handle == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        return drain_handle(
            handle,
            [&](auto *h) { return self.result_meta(planner, h); },
            [&](auto *h, std::uint64_t i, float *out) { self.result_copy_waypoint(planner, h, i, out); },
            [&](auto *h) { self.result_destroy(planner, h); });
    }

    template <typename SettingsT>
    auto run_planner_multi_impl(
        vj::DynamicRobot &self,
        vp::Planner planner,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        auto *handle = self.solve_multi(
            planner,
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        if (handle == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        return drain_handle(
            handle,
            [&](auto *h) { return self.result_meta(planner, h); },
            [&](auto *h, std::uint64_t i, float *out) { self.result_copy_waypoint(planner, h, i, out); },
            [&](auto *h) { self.result_destroy(planner, h); });
    }

    auto run_simplify_impl(
        vj::DynamicRobot &self,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vp::SimplifySettings &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        auto *handle = self.simplify(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);
        return drain_handle(
            handle,
            [&](auto *h) { return self.simplify_result_meta(h); },
            [&](auto *h, std::uint64_t i, float *out) { self.simplify_result_copy_waypoint(h, i, out); },
            [&](auto *h) { self.simplify_result_destroy(h); });
    }

    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    auto run_debug_impl(
        vj::DynamicRobot &self,
        const float *config,
        const vamp::collision::Environment<float> &env) -> DebugType
    {
        auto *handle = self.debug(config, static_cast<const void *>(&env));
        DebugType copy = *reinterpret_cast<DebugType *>(handle);
        self.debug_destroy(handle);
        return copy;
    }

    auto run_eefk_impl(vj::DynamicRobot &self, const float *config) -> Eigen::Matrix4f
    {
        Eigen::Matrix4f result;
        self.eefk(config, result.data());
        return result;
    }

    auto run_fk_impl(vj::DynamicRobot &self, const float *config)
        -> std::vector<vamp::collision::Sphere<float>>
    {
        const auto n = self.n_spheres();
        std::vector<float> buf(n * 4);
        self.fk(config, buf.data());

        std::vector<vamp::collision::Sphere<float>> out;
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            out.emplace_back(buf[i * 4 + 0], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]);
        }
        return out;
    }

    auto run_filter_pc_impl(
        vj::DynamicRobot &self,
        const float *points,
        std::uint64_t n_points,
        float point_radius,
        const float *config,
        const vamp::collision::Environment<float> &env) -> std::vector<vamp::collision::Point>
    {
        std::vector<vamp::collision::Point> out;
        self.filter_self_from_pointcloud(
            points, n_points, point_radius, config, static_cast<const void *>(&env), &out);
        return out;
    }

    // Pointcloud inputs accept either a list of (x, y, z) points or an
    // (n_points, 3) float ndarray. Flattens lists into the scratch buffer.
    auto as_points(
        const std::vector<vamp::collision::Point> &v,
        std::vector<float> &scratch,
        const char *) -> std::pair<const float *, std::uint64_t>
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
        if (a.shape(1) != 3)
        {
            throw std::runtime_error(std::string(what) + " must have shape (N, 3)");
        }
        const std::uint64_t n = a.shape(0);
        const bool contiguous = (a.stride(0) == 3 and a.stride(1) == 1);
        if (contiguous)
        {
            return {a.data(), n};
        }

        scratch.resize(n * 3);
        for (std::uint64_t i = 0; i < n; ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                scratch[i * 3 + j] = a(i, j);
            }
        }
        return {scratch.data(), n};
    }

    // Convert a std::vector<float> of known length into a numpy float32 ndarray
    // backed by a capsule, mirroring the static binding's `from(Configuration)`
    // helper. Used for the bounds accessors.
    auto vec_to_ndarray(const std::vector<float> &v) -> nb::ndarray<nb::numpy, float, nb::device::cpu>
    {
        auto *buf = new float[v.size()];
        std::memcpy(buf, v.data(), v.size() * sizeof(float));
        nb::capsule owner(buf, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
        return nb::ndarray<nb::numpy, float, nb::device::cpu>(buf, {v.size()}, owner);
    }
}  // namespace

namespace vamp::binding
{
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

        nb::class_<DynamicPlanResult>(pymodule, "DynamicPlanResult")
            .def_ro("success", &DynamicPlanResult::success)
            .def_ro("dimension", &DynamicPlanResult::dimension)
            .def_ro("iterations", &DynamicPlanResult::iterations)
            .def_ro("nanoseconds", &DynamicPlanResult::nanoseconds)
            .def_ro("cost", &DynamicPlanResult::cost)
            .def_ro("path", &DynamicPlanResult::path);

        nb::class_<DynamicSampler>(pymodule, "DynamicSampler")
            .def(
                "reset",
                [](DynamicSampler &s) { s.robot->sampler_reset(s.handle); },
                "Reset the sampler to its initial state.")
            .def(
                "skip",
                [](DynamicSampler &s, std::uint64_t n) { s.robot->sampler_skip(s.handle, n); },
                "n"_a,
                "Skip the next n samples.")
            .def(
                "next",
                // Mirrors vamp.<robot>.RNG.next(): returns a numpy ndarray
                // owning a fresh float buffer (capsule-backed).
                [](DynamicSampler &s) -> nb::ndarray<nb::numpy, float, nb::device::cpu>
                {
                    const auto dim = s.robot->dimension();
                    auto *buf = new float[dim];
                    s.robot->sampler_next(s.handle, buf);
                    nb::capsule owner(buf, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                    return nb::ndarray<nb::numpy, float, nb::device::cpu>(buf, {dim}, owner);
                },
                "Sample the next configuration.");

        // ---- DynamicRobot --------------------------------------------------

        using PRMSettings = vp::RoadmapSettings<vp::PRMStarNeighborParams>;
        using FCITSettings = vp::RoadmapSettings<vp::FCITStarNeighborParams>;

        auto klass = nb::class_<vj::DynamicRobot>(pymodule, "DynamicRobot")
                         .def_prop_ro("dimension", &vj::DynamicRobot::dimension)
                         .def_prop_ro("rake", &vj::DynamicRobot::rake)
                         .def_prop_ro("n_spheres", &vj::DynamicRobot::n_spheres)
                         .def_prop_ro(
                             "space_measure",
                             &vj::DynamicRobot::space_measure,
                             "Measure of robot's C-space.")
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
                             [](const vj::DynamicRobot &self) { return vec_to_ndarray(self.upper_bounds()); })
                         .def(
                             "lower_bounds",
                             [](const vj::DynamicRobot &self)
                             { return vec_to_ndarray(self.lower_bounds()); });

        // Per-planner overloads: list × {single, multi} and ndarray × {single, multi}.
        // Mirrors the static MF/PLANNER macros which generate parallel
        // overloads for std::array and numpy ndarray.
#define VAMP_JIT_DEF_PLANNER(KLASS, NAME, ENUM, SETTINGS)                                                    \
    KLASS                                                                                                    \
        .def(                                                                                                \
            NAME,                                                                                            \
            [](std::shared_ptr<vj::DynamicRobot> self,                                                       \
               const std::vector<float> &start,                                                              \
               const std::vector<float> &goal,                                                               \
               const vamp::collision::Environment<float> &env,                                               \
               const SETTINGS &settings,                                                                     \
               DynamicSampler &sampler) -> DynamicPlanResult                                                 \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                return run_planner_impl(                                                                     \
                    *self,                                                                                   \
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
               DynamicSampler &sampler) -> DynamicPlanResult                                                 \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                return run_planner_impl(                                                                     \
                    *self,                                                                                   \
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
               DynamicSampler &sampler) -> DynamicPlanResult                                                 \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                auto [gptr, n] = as_path(goals, d, g_scratch, "goals");                                      \
                return run_planner_multi_impl(                                                               \
                    *self, ENUM, as_config(start, d, s_scratch, "start"), gptr, n, env, settings, sampler);  \
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
               DynamicSampler &sampler) -> DynamicPlanResult                                                 \
            {                                                                                                \
                const auto d = self->dimension();                                                            \
                std::vector<float> s_scratch, g_scratch;                                                     \
                auto [gptr, n] = as_path(goals, d, g_scratch, "goals");                                      \
                return run_planner_multi_impl(                                                               \
                    *self, ENUM, as_config(start, d, s_scratch, "start"), gptr, n, env, settings, sampler);  \
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
            [](std::shared_ptr<vj::DynamicRobot> self) -> std::shared_ptr<DynamicSampler>
            { return std::make_shared<DynamicSampler>(self, self->sampler_halton()); },
            "Create a Halton sampler for this robot.");
        klass.def(
            "xorshift",
            [](std::shared_ptr<vj::DynamicRobot> self, std::uint64_t seed) -> std::shared_ptr<DynamicSampler>
            { return std::make_shared<DynamicSampler>(self, self->sampler_xorshift(seed)); },
            "seed"_a = 0,
            "Create an XORShift sampler for this robot.");

        // debug — mirror vamp.<robot>.debug(). list and ndarray overloads.
        klass.def(
            "debug",
            [](vj::DynamicRobot &self,
               const std::vector<float> &config,
               const vamp::collision::Environment<float> &env) -> DebugType
            {
                std::vector<float> scratch;
                return run_debug_impl(self, as_config(config, self.dimension(), scratch, "config"), env);
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Check which spheres of a robot configuration are in collision (JIT).");
        klass.def(
            "debug",
            [](vj::DynamicRobot &self,
               const ConfigNd &config,
               const vamp::collision::Environment<float> &env) -> DebugType
            {
                std::vector<float> scratch;
                return run_debug_impl(self, as_config(config, self.dimension(), scratch, "config"), env);
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Check which spheres of a robot configuration are in collision (JIT).");

        // eefk — mirror vamp.<robot>.eefk(). Returns a 4x4 Eigen::Matrix4f
        // (nanobind/eigen marshals to numpy).
        klass.def(
            "eefk",
            [](vj::DynamicRobot &self, const std::vector<float> &config) -> Eigen::Matrix4f
            {
                std::vector<float> scratch;
                return run_eefk_impl(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "End-effector forward kinematics. Returns a 4x4 transform (JIT).");
        klass.def(
            "eefk",
            [](vj::DynamicRobot &self, const ConfigNd &config) -> Eigen::Matrix4f
            {
                std::vector<float> scratch;
                return run_eefk_impl(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "End-effector forward kinematics. Returns a 4x4 transform (JIT).");

        // fk — mirror vamp.<robot>.fk(). Returns vector<Sphere<float>>.
        klass.def(
            "fk",
            [](vj::DynamicRobot &self, const std::vector<float> &config)
            {
                std::vector<float> scratch;
                return run_fk_impl(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "Computes the forward kinematics of the robot (JIT). Returns spheres.");
        klass.def(
            "fk",
            [](vj::DynamicRobot &self, const ConfigNd &config)
            {
                std::vector<float> scratch;
                return run_fk_impl(self, as_config(config, self.dimension(), scratch, "config"));
            },
            "configuration"_a,
            "Computes the forward kinematics of the robot (JIT). Returns spheres.");

        // validate — mirror vamp.<robot>.validate().
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

        // validate_motion — mirror vamp.<robot>.validate_motion().
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

        // filter_self_from_pointcloud — mirror vamp.<robot>.filter_self_from_pointcloud().
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
                return run_filter_pc_impl(
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
                return run_filter_pc_impl(
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

        // ---- PHS class + phs_sampler factory --------------------------------
        nb::class_<DynamicPhs>(pymodule, "DynamicPhs")
            .def(
                "set_transverse_diameter",
                [](DynamicPhs &p, float d)
                { p.robot->phs_set_transverse_diameter(p.handle, d); },
                "diameter"_a)
            .def(
                "transform",
                [](DynamicPhs &p, const std::vector<float> &x)
                {
                    const auto dim = p.robot->dimension();
                    std::vector<float> scratch;
                    auto *xptr = as_config(x, dim, scratch, "x");
                    std::vector<float> out(dim);
                    p.robot->phs_transform(p.handle, xptr, out.data());
                    return vec_to_ndarray(out);
                },
                "x"_a)
            .def(
                "transform",
                [](DynamicPhs &p, const ConfigNd &x)
                {
                    const auto dim = p.robot->dimension();
                    std::vector<float> scratch;
                    auto *xptr = as_config(x, dim, scratch, "x");
                    std::vector<float> out(dim);
                    p.robot->phs_transform(p.handle, xptr, out.data());
                    return vec_to_ndarray(out);
                },
                "x"_a);

        klass.def(
            "phs",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::vector<float> &focus_a,
               const std::vector<float> &focus_b)
            {
                const auto dim = self->dimension();
                std::vector<float> sa, sb;
                auto *h = self->phs_new(
                    as_config(focus_a, dim, sa, "focus_a"),
                    as_config(focus_b, dim, sb, "focus_b"));
                return std::make_shared<DynamicPhs>(self, h);
            },
            "focus_a"_a,
            "focus_b"_a,
            "Construct a prolate hyperspheroid from two foci.");
        klass.def(
            "phs",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const ConfigNd &focus_a,
               const ConfigNd &focus_b)
            {
                const auto dim = self->dimension();
                std::vector<float> sa, sb;
                auto *h = self->phs_new(
                    as_config(focus_a, dim, sa, "focus_a"),
                    as_config(focus_b, dim, sb, "focus_b"));
                return std::make_shared<DynamicPhs>(self, h);
            },
            "focus_a"_a,
            "focus_b"_a,
            "Construct a prolate hyperspheroid from two foci.");

        klass.def(
            "phs_sampler",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const DynamicPhs &phs,
               DynamicSampler &inner) -> std::shared_ptr<DynamicSampler>
            {
                auto *handle = self->sampler_phs(phs.handle, inner.handle);
                return std::make_shared<DynamicSampler>(self, handle);
            },
            "phs"_a,
            "rng"_a,
            "Create a new PHS-rejection sampler wrapping an inner RNG.");

        // simplify — list-of-lists or 2-D ndarray.
        klass.def(
            "simplify",
            [](vj::DynamicRobot &self,
               const std::vector<std::vector<float>> &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               DynamicSampler &sampler) -> DynamicPlanResult
            {
                std::vector<float> scratch;
                auto [pptr, n] = as_path(path, self.dimension(), scratch, "path");
                return run_simplify_impl(self, pptr, n, env, settings, sampler);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "JIT'd path simplification.");
        klass.def(
            "simplify",
            [](vj::DynamicRobot &self,
               const PathNd &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               DynamicSampler &sampler) -> DynamicPlanResult
            {
                std::vector<float> scratch;
                auto [pptr, n] = as_path(path, self.dimension(), scratch, "path");
                return run_simplify_impl(self, pptr, n, env, settings, sampler);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "JIT'd path simplification.");

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
