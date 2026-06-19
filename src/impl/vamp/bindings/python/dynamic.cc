#include <vamp_python_init.hh>

#include <vamp/jit/build_paths.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>

#include <vamp/collision/environment.hh>
#include <vamp/planning/aorrtc_settings.hh>
#include <vamp/planning/grrtstar_settings.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

#include <cricket/codegen.hh>

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
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

    auto planner_from_name(const std::string &name) -> vj::Planner
    {
        if (name == "rrtc")
        {
            return vj::Planner::RRTC;
        }

        if (name == "prm")
        {
            return vj::Planner::PRM;
        }

        if (name == "fcit")
        {
            return vj::Planner::FCIT;
        }

        if (name == "aorrtc")
        {
            return vj::Planner::AORRTC;
        }

        if (name == "grrtstar")
        {
            return vj::Planner::GRRTSTAR;
        }

        throw std::runtime_error("vamp.load_robot: unknown planner '" + name + "'");
    }

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

    template <typename SettingsT>
    auto run_planner(
        vj::DynamicRobot &self,
        vj::Planner planner,
        const std::vector<float> &start,
        const std::vector<float> &goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        const auto dim = self.dimension();
        if (start.size() != dim)
        {
            throw std::runtime_error("start has wrong dimension");
        }

        if (goal.size() != dim)
        {
            throw std::runtime_error("goal has wrong dimension");
        }

        auto *handle = self.solve(
            planner,
            start.data(),
            goal.data(),
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
    auto run_planner_multi(
        vj::DynamicRobot &self,
        vj::Planner planner,
        const std::vector<float> &start,
        const std::vector<std::vector<float>> &goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        const auto dim = self.dimension();
        if (start.size() != dim)
        {
            throw std::runtime_error("start has wrong dimension");
        }

        std::vector<float> goals_flat;
        goals_flat.reserve(dim * goals.size());
        for (const auto &g : goals)
        {
            if (g.size() != dim)
            {
                throw std::runtime_error("goal has wrong dimension");
            }

            goals_flat.insert(goals_flat.end(), g.begin(), g.end());
        }

        auto *handle = self.solve_multi(
            planner,
            start.data(),
            goals_flat.data(),
            goals.size(),
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
}  // namespace

namespace vamp::binding
{
    void init_dynamic(nb::module_ &pymodule)
    {
        // Re-dlopen _core_ext.so with RTLD_GLOBAL so template instantiations become
        // visible to the JIT's process-symbol search as Python loads extension
        // modules with RTLD_LOCAL by default
        Dl_info info{};
        if (dladdr(reinterpret_cast<void *>(&promote_self_to_rtld_global), &info) == 0 or
            info.dli_fname == nullptr)
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
                [](DynamicSampler &s) -> std::vector<float>
                {
                    std::vector<float> out(s.robot->dimension());
                    s.robot->sampler_next(s.handle, out.data());
                    return out;
                },
                "Sample the next configuration.");

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
            { return run_planner(*self, ENUM, start, goal, env, settings, sampler); },                       \
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
            { return run_planner_multi(*self, ENUM, start, goals, env, settings, sampler); },                \
            "start"_a,                                                                                       \
            "goal"_a,                                                                                        \
            "environment"_a,                                                                                 \
            "settings"_a,                                                                                    \
            "sampler"_a,                                                                                     \
            "JIT'd " NAME)

        using PRMSettings = vp::RoadmapSettings<vp::PRMStarNeighborParams>;
        using FCITSettings = vp::RoadmapSettings<vp::FCITStarNeighborParams>;

        auto klass = nb::class_<vj::DynamicRobot>(pymodule, "DynamicRobot")
                         .def_prop_ro("dimension", &vj::DynamicRobot::dimension)
                         .def_prop_ro("rake", &vj::DynamicRobot::rake);

        VAMP_JIT_DEF_PLANNER(klass, "rrtc", vj::Planner::RRTC, vp::RRTCSettings);
        VAMP_JIT_DEF_PLANNER(klass, "prm", vj::Planner::PRM, PRMSettings);
        VAMP_JIT_DEF_PLANNER(klass, "fcit", vj::Planner::FCIT, FCITSettings);
        VAMP_JIT_DEF_PLANNER(klass, "aorrtc", vj::Planner::AORRTC, vp::AORRTCSettings);
        VAMP_JIT_DEF_PLANNER(klass, "grrtstar", vj::Planner::GRRTSTAR, vp::GRRTStarSettings);

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

        using DebugType = std::
            pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
        klass.def(
            "debug",
            [](vj::DynamicRobot &self,
               const std::vector<float> &config,
               const vamp::collision::Environment<float> &env) -> DebugType
            {
                if (config.size() != self.dimension())
                {
                    throw std::runtime_error("config has wrong dimension");
                }
                auto *handle = self.debug(config.data(), static_cast<const void *>(&env));
                DebugType copy = *reinterpret_cast<DebugType *>(handle);
                self.debug_destroy(handle);
                return copy;
            },
            "configuration"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Check which spheres of a robot configuration are in collision (JIT).");

        // simplify — mirror vamp.<robot>.simplify. Takes the path as a list
        // of waypoints (each a list/array of floats); we flatten before
        // crossing the FFI boundary.
        klass.def(
            "simplify",
            [](vj::DynamicRobot &self,
               const std::vector<std::vector<float>> &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               DynamicSampler &sampler) -> DynamicPlanResult
            {
                const auto dim = self.dimension();
                std::vector<float> flat;
                flat.reserve(dim * path.size());
                for (const auto &wp : path)
                {
                    if (wp.size() != dim)
                    {
                        throw std::runtime_error("waypoint has wrong dimension");
                    }
                    flat.insert(flat.end(), wp.begin(), wp.end());
                }
                auto *handle = self.simplify(
                    flat.data(),
                    path.size(),
                    static_cast<const void *>(&env),
                    static_cast<const void *>(&settings),
                    sampler.handle);
                return drain_handle(
                    handle,
                    [&](auto *h) { return self.simplify_result_meta(h); },
                    [&](auto *h, std::uint64_t i, float *out)
                    { self.simplify_result_copy_waypoint(h, i, out); },
                    [&](auto *h) { self.simplify_result_destroy(h); });
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
                    opts.planners.push_back(planner_from_name(p));
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
