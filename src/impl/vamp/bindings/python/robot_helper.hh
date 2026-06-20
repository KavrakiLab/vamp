#pragma once

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>

#include <vamp/collision/filter.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/aorrtc.hh>
#include <vamp/planning/fcit.hh>
#include <vamp/planning/grrtstar.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#else
#include <stdexcept>
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nanobind/eigen/dense.h>
#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

VAMP_DEFINE_HAS_METHOD(set_lows)
VAMP_DEFINE_HAS_METHOD(set_highs)
VAMP_DEFINE_HAS_METHOD(set_radius)

namespace vamp::binding
{
    namespace nb = nanobind;
    using namespace nb::literals;

    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    template <typename Robot>
    struct NDArrayInput
    {
        using Type = nanobind::
            ndarray<FloatT, nanobind::numpy, nanobind::shape<Robot::dimension>, nanobind::device::cpu>;

        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        template <std::size_t r>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<r>;

        inline static auto from(const Configuration &c) -> Type
        {
            auto c_arr = c.to_array();
            return make_ndarray<Type, 1>(c_arr.data(), {Robot::dimension});
        }

        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(array(a));
        }

        inline static auto array(const Type &a) -> ConfigurationArray
        {
            ConfigurationArray c;
            std::vector<float> scratch;
            const auto *ptr = as_flat_1d(a, Robot::dimension, scratch, "configuration");
            std::memcpy(c.data(), ptr, Robot::dimension * sizeof(float));
            return c;
        }

        template <std::size_t r>
        inline static auto block(const Type &a) -> ConfigurationBlock<r>
        {
            ConfigurationBlock<r> out;
            std::vector<float> scratch;
            const auto *ptr = as_flat_1d(a, Robot::dimension, scratch, "configuration");
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = ptr[i];
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
        template <std::size_t r>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<r>;

        inline static auto from(const Configuration &c) -> Type
        {
            Type a;
            auto c_arr = c.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                a[i] = c_arr[i];
            }
            return a;
        }

        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(a);
        }

        inline static auto array(const Type &a) -> ConfigurationArray
        {
            return a;
        }

        template <std::size_t r>
        inline static auto block(const Type &a) -> ConfigurationBlock<r>
        {
            ConfigurationBlock<r> out;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = a[i];
            }
            return out;
        }
    };

    template <typename Robot, typename Input>
    struct StaticRobotTraits
    {
        using Cfg = typename Input::Type;
        using Pth = std::vector<Cfg>;
        using Pc = std::vector<vamp::collision::Point>;
        using Path = vamp::planning::Path<Robot>;
        using PlanningResult = vamp::planning::PlanningResult<Robot>;
        using Sampler = vamp::rng::RNG<Robot>;
        using Phs = vamp::planning::ProlateHyperspheroid<Robot>;
        using Env = vamp::collision::Environment<float>;
        using EnvVec = vamp::collision::Environment<vamp::FloatVector<rake>>;
        using PRMSettings = vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams>;
        using FCITSettings = vamp::planning::RoadmapSettings<vamp::planning::FCITStarNeighborParams>;

        using Configuration = typename Robot::Configuration;
        using RNG = vamp::rng::RNG<Robot>;
        using Roadmap = vamp::planning::Roadmap<Robot>;

        template <vamp::planning::Planner P>
        using PlannerT = vamp::planning::PlannerClass<Robot, rake, Robot::resolution, P>;

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return PlannerT<P>::solve(Input::to(start), Input::to(goal), EnvVec(env), s, rng);
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_multi(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
            }
            return PlannerT<P>::solve(Input::to(start), goals_v, EnvVec(env), s, rng);
        }

        static auto simplify(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return vamp::planning::simplify<Robot, rake, Robot::resolution>(p, EnvVec(env), settings, rng);
        }

        static auto fk(const Cfg &c) -> std::vector<vamp::collision::Sphere<float>>
        {
            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(Input::template block<1>(c), out);

            std::vector<vamp::collision::Sphere<float>> result(Robot::n_spheres);
            for (auto i = 0U; i < Robot::n_spheres; ++i)
            {
                result[i] = vamp::collision::Sphere<float>{
                    out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}]};
            }
            return result;
        }

        static auto eefk(const Cfg &c) -> Eigen::Matrix4f
        {
            return Robot::eefk(Input::array(c)).matrix();
        }

        static auto debug(const Cfg &c, const Env &env) -> typename Robot::Debug
        {
            return Robot::fkcc_debug(EnvVec(env), Input::template block<rake>(c));
        }

        static auto validate(const Cfg &c, const Env &env, bool check_bounds) -> bool
        {
            auto configuration = Input::to(c);
            auto copy = configuration.trim();
            Robot::descale_configuration(copy);
            const bool in_bounds = (copy <= 1.F).all() and (copy >= 0.F).all();
            return (not check_bounds or in_bounds) and
                   vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, EnvVec(env));
        }

        static auto validate_motion(const Cfg &c_in, const Cfg &c_out, const Env &env, bool check_bounds)
            -> bool
        {
            auto cfg_in = Input::to(c_in);
            auto copy_in = cfg_in.trim();
            Robot::descale_configuration(copy_in);
            const bool in_bounds_in = (copy_in <= 1.F).all() and (copy_in >= 0.F).all();

            auto cfg_out = Input::to(c_out);
            auto copy_out = cfg_out.trim();
            Robot::descale_configuration(copy_out);
            const bool in_bounds_out = (copy_out <= 1.F).all() and (copy_out >= 0.F).all();

            return (not check_bounds or (in_bounds_in and in_bounds_out)) and
                   vamp::planning::validate_motion<Robot, rake, 1>(cfg_in, cfg_out, EnvVec(env));
        }

        static auto
        filter_self_from_pointcloud(const Pc &pc, float point_radius, const Cfg &c, const Env &env)
            -> std::vector<vamp::collision::Point>
        {
            std::vector<vamp::collision::Point> out;
            vamp::collision::filter_self_from_pointcloud<Robot, rake>(
                pc.empty() ? nullptr : pc.front().data(),
                pc.size(),
                point_radius,
                Input::template block<1>(c),
                EnvVec(env),
                out);
            return out;
        }

        static auto make_halton() -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::rng::Halton<Robot>>();
        }

        static auto make_xorshift(std::uint64_t /*seed*/) -> std::shared_ptr<Sampler>
        {
#if defined(__x86_64__)
            return std::make_shared<vamp::rng::XORShift<Robot>>();
#else
            throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
        }

        static auto make_phs(const Cfg &focus_a, const Cfg &focus_b) -> std::shared_ptr<Phs>
        {
            return std::make_shared<Phs>(Input::to(focus_a), Input::to(focus_b));
        }

        static auto make_phs_sampler(const Phs &phs, std::shared_ptr<Sampler> inner)
            -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::planning::ProlateHyperspheroidRNG<Robot>>(phs, inner);
        }

        static auto path_len(const Path &p) -> std::size_t
        {
            return p.size();
        }

        static auto path_get(const Path &p, std::size_t i)
        {
            return Input::from(p[i]);
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
            return p.template validate<rake>(EnvVec(e));
        }

        static auto path_numpy(const Path &p)
        {
            using ND = nanobind::ndarray<nanobind::numpy, const FloatT, nanobind::device::cpu>;
            const std::size_t slop = Robot::Configuration::num_scalars_rounded - Robot::dimension;
            return make_ndarray_filled<ND, 2>(
                {p.size(), Robot::dimension},
                [&](FloatT *dst)
                {
                    for (auto i = 0U; i < p.size(); ++i)
                    {
                        p[i].to_array_unaligned(dst + i * Robot::dimension);
                    }
                },
                slop);
        }

        static void path_set(Path &p, std::size_t i, const Cfg &c)
        {
            p[i] = Input::to(c);
        }

        static void path_append(Path &p, const Cfg &c)
        {
            p.emplace_back(Input::to(c));
        }

        static void path_insert(Path &p, std::size_t i, const Cfg &c)
        {
            p.insert(p.cbegin() + i, Input::to(c));
        }

        static auto result_solved(const PlanningResult &r) -> bool
        {
            return r.path.size() >= 2;
        }

        static auto result_path(const PlanningResult &r) -> Path
        {
            return r.path;
        }

        static auto result_nanoseconds(const PlanningResult &r) -> std::size_t
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
            for (auto i = 0U; i < n; ++i)
            {
                s.next();
            }
        }

        static auto sampler_next(Sampler &s)
        {
            return Input::from(s.next());
        }

        static void phs_set_transverse_diameter(Phs &p, float d)
        {
            p.set_transverse_diameter(d);
        }

        static auto phs_transform(Phs &p, const Cfg &x)
        {
            return Input::from(p.transform(Input::to(x)));
        }
    };

    template <typename Robot>
    inline auto init_robot(nanobind::module_ &pymodule) -> nanobind::module_
    {
        using NA = NDArrayInput<Robot>;
        using CA = ArrayInput<Robot>;
        using TA = StaticRobotTraits<Robot, NA>;
        using TC = StaticRobotTraits<Robot, CA>;
        using NDArray = typename NA::Type;

        auto submodule = pymodule.def_submodule(Robot::name, "Robot-specific submodule");

        submodule.def("dimension", []() { return Robot::dimension; });
        submodule.def("resolution", []() { return Robot::resolution; });
        submodule.def("n_spheres", []() { return Robot::n_spheres; });
        submodule.def("space_measure", []() { return Robot::space_measure(); });
        submodule.def(
            "min_max_radii",
            []() -> std::pair<float, float> { return {Robot::min_radius, Robot::max_radius}; });
        submodule.def("joint_names", []() { return Robot::joint_names; });
        submodule.def("end_effector", []() { return Robot::end_effector; });

        submodule.def(
            "upper_bounds",
            []() -> NDArray
            {
                std::array<float, Robot::dimension> ones;
                ones.fill(1.0f);
                auto one_v = typename Robot::Configuration(ones);
                Robot::scale_configuration(one_v);
                return NA::from(one_v);
            });
        submodule.def(
            "lower_bounds",
            []() -> NDArray
            {
                std::array<float, Robot::dimension> zeros;
                zeros.fill(0.0f);
                auto zero_v = typename Robot::Configuration(zeros);
                Robot::scale_configuration(zero_v);
                return NA::from(zero_v);
            });

        bind_sampler<TA>(submodule, "RNG");

        auto phs_k = bind_phs_class<TA>(submodule, "ProlateHyperspheroid");
        bind_phs_io<TA>(phs_k);
        bind_phs_io<TC>(phs_k);
        phs_k.def(
            "__init__",
            [](typename TA::Phs *t, const typename NA::Type &a, const typename NA::Type &b)
            { new (t) typename TA::Phs(NA::to(a), NA::to(b)); },
            "Construct from two foci.");

        auto path_k = bind_path_class<TA>(submodule, "Path");
        bind_path_io<TA>(path_k);
        bind_path_io<TC>(path_k);

        bind_planning_result<TA>(submodule, "PlanningResult");

        bind_robot_methods<TA>(submodule);
        bind_robot_methods<TC>(submodule);

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
