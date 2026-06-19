#pragma once

#include <vamp/bindings/python/array_helpers.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/jit/api.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/simplify_settings.hh>

#include <Eigen/Dense>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// MF-style helper for the dynamic Python bindings. Mirrors the static
// vamp::binding::Helper<Robot, Input> pattern: each user-facing method is
// written ONCE here templated on the input flavour (list vs ndarray), then
// registered twice on the Python class via the VJF_MF / VJF_PLANNER macros.
//
// Three "input adapter" axes:
//   - ConfigInput: std::vector<float> | nb::ndarray<const float, ndim<1>>
//   - PathInput  : std::vector<std::vector<float>> | nb::ndarray<const float, ndim<2>>
//   - PcInput    : std::vector<vamp::collision::Point> | nb::ndarray<const float, ndim<2>>
//
// Each adapter exposes a `Type` typedef (the nanobind-bound input type) and
// an `as_ptr(...)` static that returns either `const float *` (1-D) or
// `std::pair<const float *, n>` (2-D), copying into a caller-owned scratch
// buffer only when the source is non-contiguous.

namespace vamp::binding
{
    namespace nb_ = nanobind;
    namespace vj = vamp::jit;
    namespace vp = vamp::planning;

    using ConfigNd_ = nb_::ndarray<const float, nb_::ndim<1>, nb_::device::cpu>;
    using PathNd_ = nb_::ndarray<const float, nb_::ndim<2>, nb_::device::cpu>;

    // ---- input adapters -----------------------------------------------------

    struct VectorConfig
    {
        using Type = std::vector<float>;
        static auto as_ptr(
            const Type &v,
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
    };

    struct NDArrayConfig
    {
        using Type = ConfigNd_;
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
        using Type = PathNd_;
        static auto as_ptr(const Type &a, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            return as_flat_2d(a, dim, scratch, what);
        }
    };

    struct VectorPointcloud
    {
        using Type = std::vector<vamp::collision::Point>;
        static auto as_ptr(const Type &v, std::vector<float> &scratch, const char * /*what*/)
            -> std::pair<const float *, std::uint64_t>
        {
            scratch.clear();
            scratch.reserve(3 * v.size());
            for (const auto &p : v)
            {
                scratch.insert(scratch.end(), p.begin(), p.end());
            }
            return {scratch.data(), v.size()};
        }
    };

    struct NDArrayPointcloud
    {
        using Type = PathNd_;  // same ndarray flavour as paths; inner_dim fixed at 3
        static auto as_ptr(const Type &a, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            return as_flat_2d(a, 3, scratch, what);
        }
    };

    // ---- DynamicHelper -----------------------------------------------------

    template <typename ConfigInput, typename PathInput, typename PcInput>
    struct DynamicHelper
    {
        using Cfg = typename ConfigInput::Type;
        using Pth = typename PathInput::Type;
        using Pc = typename PcInput::Type;
        using Env = vamp::collision::Environment<float>;

        // ---- planners (templated on enum + Settings) -----------------------

        template <vp::Planner P, typename Settings>
        static auto solve_single(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &settings,
            vj::DynamicSampler &sampler) -> vj::DynamicPlanResult
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
                sampler);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_multi(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &settings,
            vj::DynamicSampler &sampler) -> vj::DynamicPlanResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            auto [gptr, n] = PathInput::as_ptr(goals, d, gs, "goals");
            return vj::solve_multi(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                gptr,
                n,
                env,
                settings,
                sampler);
        }

        // ---- simplify (path is PathInput) ---------------------------------

        static auto simplify(
            std::shared_ptr<vj::DynamicRobot> self,
            const Pth &path,
            const Env &env,
            const vp::SimplifySettings &settings,
            vj::DynamicSampler &sampler) -> vj::DynamicPlanResult
        {
            std::vector<float> scratch;
            auto [pptr, n] = PathInput::as_ptr(path, self->dimension(), scratch, "path");
            return vj::simplify(self, pptr, n, env, settings, sampler);
        }

        // ---- introspection ------------------------------------------------

        static auto fk(vj::DynamicRobot &self, const Cfg &config)
            -> std::vector<vamp::collision::Sphere<float>>
        {
            std::vector<float> scratch;
            return vj::fk(self, ConfigInput::as_ptr(config, self.dimension(), scratch, "configuration"));
        }

        static auto eefk(vj::DynamicRobot &self, const Cfg &config) -> Eigen::Matrix4f
        {
            std::vector<float> scratch;
            return vj::eefk(self, ConfigInput::as_ptr(config, self.dimension(), scratch, "configuration"));
        }

        static auto debug(vj::DynamicRobot &self, const Cfg &config, const Env &env) -> vj::DebugType
        {
            std::vector<float> scratch;
            return vj::debug(self, ConfigInput::as_ptr(config, self.dimension(), scratch, "configuration"), env);
        }

        static auto validate(vj::DynamicRobot &self, const Cfg &config, const Env &env, bool check_bounds)
            -> bool
        {
            std::vector<float> scratch;
            return self.validate(
                ConfigInput::as_ptr(config, self.dimension(), scratch, "configuration"),
                static_cast<const void *>(&env),
                check_bounds);
        }

        static auto validate_motion(
            vj::DynamicRobot &self,
            const Cfg &c_in,
            const Cfg &c_out,
            const Env &env,
            bool check_bounds) -> bool
        {
            std::vector<float> s_in, s_out;
            return self.validate_motion(
                ConfigInput::as_ptr(c_in, self.dimension(), s_in, "configuration_in"),
                ConfigInput::as_ptr(c_out, self.dimension(), s_out, "configuration_out"),
                static_cast<const void *>(&env),
                check_bounds);
        }

        // ---- pointcloud filter --------------------------------------------

        static auto filter_self_from_pointcloud(
            vj::DynamicRobot &self,
            const Pc &pc,
            float point_radius,
            const Cfg &config,
            const Env &env) -> std::vector<vamp::collision::Point>
        {
            std::vector<float> cfg_scratch, pc_scratch;
            auto [pptr, n] = PcInput::as_ptr(pc, pc_scratch, "pc");
            return vj::filter_self_from_pointcloud(
                self,
                pptr,
                n,
                point_radius,
                ConfigInput::as_ptr(config, self.dimension(), cfg_scratch, "config"),
                env);
        }

        // ---- PHS ----------------------------------------------------------

        static auto make_phs(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &focus_a,
            const Cfg &focus_b) -> std::shared_ptr<vj::DynamicPhs>
        {
            const auto d = self->dimension();
            std::vector<float> sa, sb;
            return vj::make_phs(
                self,
                ConfigInput::as_ptr(focus_a, d, sa, "focus_a"),
                ConfigInput::as_ptr(focus_b, d, sb, "focus_b"));
        }

        static auto phs_transform(vj::DynamicPhs &phs, const Cfg &x)
            -> nb_::ndarray<nb_::numpy, float, nb_::device::cpu>
        {
            const auto dim = phs.robot->dimension();
            std::vector<float> scratch;
            const auto *xptr = ConfigInput::as_ptr(x, dim, scratch, "x");
            std::vector<float> out(dim);
            phs.transform(xptr, out.data());
            return make_ndarray<1>(out.data(), {dim});
        }
    };

    // Two pre-built helper instantiations for the two input combinations
    // we surface on the Python side.
    using DHV = DynamicHelper<VectorConfig, VectorPath, VectorPointcloud>;
    using DHN = DynamicHelper<NDArrayConfig, NDArrayPath, NDArrayPointcloud>;
}  // namespace vamp::binding

// ---- MF-style binding macros ----------------------------------------------
//
// VJF_MF(klass, name, func, ...args/desc):
//   registers `func` from both DHV and DHN on `klass` under `name`. Forwards
//   the trailing args (typically nb::arg keyword markers + a docstring).
//
// VJF_PLANNER(klass, name, planner_enum, settings):
//   registers single + multi solve overloads for `planner_enum` (a
//   vp::Planner value) with `settings` type, from both DHV and DHN.

#define VJF_MF(KLASS, NAME, FUNC, ...) \
    KLASS.def(NAME, &::vamp::binding::DHV::FUNC, ##__VA_ARGS__); \
    KLASS.def(NAME, &::vamp::binding::DHN::FUNC, ##__VA_ARGS__)

#define VJF_PLANNER(KLASS, NAME, PLANNER_ENUM, SETTINGS)                                                     \
    KLASS                                                                                                    \
        .def(NAME,                                                                                           \
             &::vamp::binding::DHV::template solve_single<PLANNER_ENUM, SETTINGS>,                           \
             "start"_a, "goal"_a, "environment"_a, "settings"_a, "sampler"_a, "JIT'd " NAME)                 \
        .def(NAME,                                                                                           \
             &::vamp::binding::DHN::template solve_single<PLANNER_ENUM, SETTINGS>,                           \
             "start"_a, "goal"_a, "environment"_a, "settings"_a, "sampler"_a, "JIT'd " NAME)                 \
        .def(NAME,                                                                                           \
             &::vamp::binding::DHV::template solve_multi<PLANNER_ENUM, SETTINGS>,                            \
             "start"_a, "goal"_a, "environment"_a, "settings"_a, "sampler"_a, "JIT'd " NAME)                 \
        .def(NAME,                                                                                           \
             &::vamp::binding::DHN::template solve_multi<PLANNER_ENUM, SETTINGS>,                            \
             "start"_a, "goal"_a, "environment"_a, "settings"_a, "sampler"_a, "JIT'd " NAME)
