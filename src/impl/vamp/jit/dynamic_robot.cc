#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/build_paths.hh>

#include <cricket/jit/compiler.hh>

#include <llvm/Support/Error.h>

#include <stdexcept>
#include <utility>

namespace vamp::jit
{
    namespace
    {
        auto lookup(cricket::jit::JitSession &session, const std::string &symbol) -> void *
        {
            auto addr = session.lookup(symbol);
            if (not addr)
            {
                throw std::runtime_error(
                    "vamp::jit: cannot resolve symbol '" + symbol + "': " + llvm::toString(addr.takeError()));
            }
            return addr->toPtr<void *>();
        }

        template <typename F>
        auto resolve_planner(cricket::jit::JitSession &s, vamp::planning::Planner p, std::string_view suffix)
            -> F
        {
            return reinterpret_cast<F>(lookup(s, planner_symbol(p, suffix)));
        }

        template <typename F>
        auto resolve_robot(cricket::jit::JitSession &s, const std::string &robot, std::string_view suffix)
            -> F
        {
            return reinterpret_cast<F>(lookup(s, robot_symbol(robot, suffix)));
        }
    }  // namespace

    auto default_load_options() -> LoadOptions
    {
        LoadOptions opts;
        opts.include_dirs.assign(paths::include_dirs.begin(), paths::include_dirs.end());
        opts.system_include_dirs.assign(paths::system_include_dirs.begin(), paths::system_include_dirs.end());
        opts.defines.assign(paths::defines.begin(), paths::defines.end());
        opts.extra_flags.assign(paths::extra_flags.begin(), paths::extra_flags.end());
        return opts;
    }

    DynamicRobot::DynamicRobot(const LoadOptions &opts, std::shared_ptr<cricket::jit::DiskObjectCache> cache)
      : dimension_(opts.dimension), rake_(opts.rake)
    {
        StubOptions stub_opts;
        stub_opts.robot_source = opts.robot_source;
        stub_opts.robot_name = opts.robot_name;
        stub_opts.rake = opts.rake;
        stub_opts.resolution = opts.resolution;
        stub_opts.planners = opts.planners;

        auto source = generate_stub_source(stub_opts);

        cricket::jit::CompileOptions copts;
        copts.include_dirs = opts.include_dirs;
        copts.system_include_dirs = opts.system_include_dirs;
        copts.defines = opts.defines;
        copts.extra_flags = opts.extra_flags;

        cricket::jit::ClangCompiler compiler;
        auto module = compiler.compile(source, copts);

        session_ = std::make_unique<cricket::jit::JitSession>(std::move(cache));

        if (auto err = session_->add_module(std::move(module)))
        {
            throw std::runtime_error("vamp::jit: add_module failed: " + llvm::toString(std::move(err)));
        }

        for (auto p : opts.planners)
        {
            PlannerEntry e;
            e.solve = resolve_planner<ffi::SolveFn>(*session_, p, "solve");
            e.solve_multi = resolve_planner<ffi::SolveMultiFn>(*session_, p, "solve_multi");
            e.meta = resolve_planner<ffi::ResultMetaFn>(*session_, p, "result_meta");
            e.copy = resolve_planner<ffi::ResultCopyWaypointFn>(*session_, p, "result_copy_waypoint");
            e.destroy = resolve_planner<ffi::ResultDestroyFn>(*session_, p, "result_destroy");
            planners_.emplace(p, e);
        }

        const auto &r = opts.robot_name;
        simplify_.simplify = resolve_robot<ffi::SimplifyFn>(*session_, r, "simplify");
        simplify_.meta = resolve_robot<ffi::ResultMetaFn>(*session_, r, "simplify_result_meta");
        simplify_.copy =
            resolve_robot<ffi::ResultCopyWaypointFn>(*session_, r, "simplify_result_copy_waypoint");
        simplify_.destroy = resolve_robot<ffi::ResultDestroyFn>(*session_, r, "simplify_result_destroy");

        sampler_.halton = resolve_robot<ffi::SamplerHaltonFn>(*session_, r, "sampler_halton");
        sampler_.xorshift = resolve_robot<ffi::SamplerXorshiftFn>(*session_, r, "sampler_xorshift");
        sampler_.reset = resolve_robot<ffi::SamplerResetFn>(*session_, r, "sampler_reset");
        sampler_.skip = resolve_robot<ffi::SamplerSkipFn>(*session_, r, "sampler_skip");
        sampler_.next = resolve_robot<ffi::SamplerNextFn>(*session_, r, "sampler_next");
        sampler_.destroy = resolve_robot<ffi::SamplerDestroyFn>(*session_, r, "sampler_destroy");

        debug_.debug = resolve_robot<ffi::DebugFn>(*session_, r, "debug");
        debug_.destroy = resolve_robot<ffi::DebugDestroyFn>(*session_, r, "debug_destroy");

        // Per-robot: end-effector FK.
        eefk_ = resolve_robot<ffi::EefkFn>(*session_, r, "eefk");
    }

    DynamicRobot::~DynamicRobot() = default;

    auto DynamicRobot::has_planner(vamp::planning::Planner p) const -> bool
    {
        return planners_.count(p) > 0;
    }

    auto DynamicRobot::solve(
        vamp::planning::Planner p,
        const float *start,
        const float *goal,
        const void *env,
        const void *settings,
        ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *
    {
        auto it = planners_.find(p);
        if (it == planners_.end())
        {
            return nullptr;
        }
        return it->second.solve(start, goal, env, settings, sampler);
    }

    auto DynamicRobot::solve_multi(
        vamp::planning::Planner p,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const void *env,
        const void *settings,
        ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *
    {
        auto it = planners_.find(p);
        if (it == planners_.end())
        {
            return nullptr;
        }
        return it->second.solve_multi(start, goals, n_goals, env, settings, sampler);
    }

    auto DynamicRobot::result_meta(vamp::planning::Planner p, const ffi::PlanResultHandle *h)
        -> ffi::PlanResultMeta
    {
        return planners_.at(p).meta(h);
    }

    auto DynamicRobot::result_copy_waypoint(
        vamp::planning::Planner p,
        const ffi::PlanResultHandle *h,
        std::uint64_t idx,
        float *out) -> void
    {
        planners_.at(p).copy(h, idx, out);
    }

    auto DynamicRobot::result_destroy(vamp::planning::Planner p, ffi::PlanResultHandle *h) -> void
    {
        planners_.at(p).destroy(h);
    }

    auto DynamicRobot::simplify(
        const float *path,
        std::uint64_t n_waypoints,
        const void *env,
        const void *settings,
        ffi::SamplerHandle *sampler) -> ffi::PlanResultHandle *
    {
        return simplify_.simplify(path, n_waypoints, env, settings, sampler);
    }

    auto DynamicRobot::simplify_result_meta(const ffi::PlanResultHandle *h) -> ffi::PlanResultMeta
    {
        return simplify_.meta(h);
    }

    auto
    DynamicRobot::simplify_result_copy_waypoint(const ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
        -> void
    {
        simplify_.copy(h, idx, out);
    }

    auto DynamicRobot::simplify_result_destroy(ffi::PlanResultHandle *h) -> void
    {
        simplify_.destroy(h);
    }

    auto DynamicRobot::sampler_halton() -> ffi::SamplerHandle *
    {
        return sampler_.halton();
    }

    auto DynamicRobot::sampler_xorshift(std::uint64_t seed) -> ffi::SamplerHandle *
    {
        return sampler_.xorshift(seed);
    }

    auto DynamicRobot::sampler_reset(ffi::SamplerHandle *h) -> void
    {
        sampler_.reset(h);
    }

    auto DynamicRobot::sampler_skip(ffi::SamplerHandle *h, std::uint64_t n) -> void
    {
        sampler_.skip(h, n);
    }

    auto DynamicRobot::sampler_next(ffi::SamplerHandle *h, float *out) -> void
    {
        sampler_.next(h, out);
    }

    auto DynamicRobot::sampler_destroy(ffi::SamplerHandle *h) -> void
    {
        sampler_.destroy(h);
    }

    auto DynamicRobot::debug(const float *config, const void *env) -> ffi::DebugHandle *
    {
        return debug_.debug(config, env);
    }

    auto DynamicRobot::debug_destroy(ffi::DebugHandle *h) -> void
    {
        debug_.destroy(h);
    }

    auto DynamicRobot::eefk(const float *config, float *out_matrix) -> void
    {
        eefk_(config, out_matrix);
    }
}  // namespace vamp::jit
