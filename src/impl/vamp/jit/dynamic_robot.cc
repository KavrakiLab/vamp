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
        auto resolve(cricket::jit::JitSession &s, vamp::planning::Planner p, std::string_view suffix) -> F
        {
            return reinterpret_cast<F>(lookup(s, planner_symbol(p, suffix)));
        }

        template <typename F>
        auto resolve(cricket::jit::JitSession &s, const std::string &robot, std::string_view suffix) -> F
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
            auto &e = ops_.planners[static_cast<std::size_t>(p)];
            e.solve = resolve<ffi::SolveFn>(*session_, p, "solve");
            e.solve_multi = resolve<ffi::SolveMultiFn>(*session_, p, "solve_multi");
        }

        const auto &r = opts.robot_name;

        // Result helpers are shared across planners + simplify.
        ops_.result_meta = resolve<ffi::ResultMetaFn>(*session_, r, "result_meta");
        ops_.result_copy_waypoint = resolve<ffi::ResultCopyWaypointFn>(*session_, r, "result_copy_waypoint");
        ops_.result_destroy = resolve<ffi::ResultDestroyFn>(*session_, r, "result_destroy");

        ops_.simplify = resolve<ffi::SimplifyFn>(*session_, r, "simplify");

        ops_.sampler_halton = resolve<ffi::SamplerHaltonFn>(*session_, r, "sampler_halton");
        ops_.sampler_xorshift = resolve<ffi::SamplerXorshiftFn>(*session_, r, "sampler_xorshift");
        ops_.sampler_reset = resolve<ffi::SamplerResetFn>(*session_, r, "sampler_reset");
        ops_.sampler_skip = resolve<ffi::SamplerSkipFn>(*session_, r, "sampler_skip");
        ops_.sampler_next = resolve<ffi::SamplerNextFn>(*session_, r, "sampler_next");
        ops_.sampler_destroy = resolve<ffi::SamplerDestroyFn>(*session_, r, "sampler_destroy");

        ops_.debug = resolve<ffi::DebugFn>(*session_, r, "debug");
        ops_.debug_destroy = resolve<ffi::DebugDestroyFn>(*session_, r, "debug_destroy");
        ops_.eefk = resolve<ffi::EefkFn>(*session_, r, "eefk");
        ops_.fk = resolve<ffi::FkFn>(*session_, r, "fk");
        ops_.validate = resolve<ffi::ValidateFn>(*session_, r, "validate");
        ops_.validate_motion = resolve<ffi::ValidateMotionFn>(*session_, r, "validate_motion");
        ops_.filter_pointcloud =
            resolve<ffi::FilterPointcloudFn>(*session_, r, "filter_self_from_pointcloud");

        ops_.phs_new = resolve<ffi::PhsNewFn>(*session_, r, "phs_new");
        ops_.phs_destroy = resolve<ffi::PhsDestroyFn>(*session_, r, "phs_destroy");
        ops_.phs_set_diameter = resolve<ffi::PhsSetDiameterFn>(*session_, r, "phs_set_transverse_diameter");
        ops_.phs_transform = resolve<ffi::PhsTransformFn>(*session_, r, "phs_transform");
        ops_.sampler_phs = resolve<ffi::SamplerPhsFn>(*session_, r, "sampler_phs");

        auto space_measure_fn = resolve<ffi::SpaceMeasureFn>(*session_, r, "space_measure");
        auto min_max_fn = resolve<ffi::MinMaxRadiiFn>(*session_, r, "min_max_radii");
        auto n_spheres_fn = resolve<ffi::NSpheresFn>(*session_, r, "n_spheres");
        auto joint_names_fn = resolve<ffi::JointNamesFn>(*session_, r, "joint_names");
        auto upper_fn = resolve<ffi::BoundsFn>(*session_, r, "upper_bounds");
        auto lower_fn = resolve<ffi::BoundsFn>(*session_, r, "lower_bounds");

        space_measure_ = space_measure_fn();
        min_max_fn(&min_radius_, &max_radius_);
        n_spheres_ = n_spheres_fn();
        joint_names_fn(&joint_names_);

        upper_bounds_.resize(dimension_);
        lower_bounds_.resize(dimension_);
        upper_fn(upper_bounds_.data());
        lower_fn(lower_bounds_.data());
    }

    DynamicRobot::~DynamicRobot() = default;
}  // namespace vamp::jit
