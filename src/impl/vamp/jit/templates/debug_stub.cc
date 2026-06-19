// Per-robot debug stub. Wraps Robot::fkcc_debug<rake>() and Robot::eefk() so
// the host can inspect per-sphere environment-collision / self-collision info
// and compute end-effector poses, mirroring vamp.<robot>.debug() / .eefk() on
// the static side.
//
// inja substitutions:
//   {{robot_name}} — struct name inside vamp::robots::
//   {{rake}}       — SIMD lane width (the static binding uses rake too)

// clang-format off
#define VAMP_JIT_DEBUG         vamp_jit_{{robot_name}}_debug
#define VAMP_JIT_DEBUG_DESTROY vamp_jit_{{robot_name}}_debug_destroy
#define VAMP_JIT_EEFK          vamp_jit_{{robot_name}}_eefk
#define VAMP_JIT_RAKE          {{rake}}
// clang-format on

extern "C" vamp::jit::ffi::DebugHandle *VAMP_JIT_DEBUG(const float *config, const void *env_ptr)
{
    using R = vamp_jit_robot::R;

    // Broadcast the scalar configuration into a rake-wide block, mirroring
    // the static binding's Input::template block<rake>(c).
    typename R::template ConfigurationBlock<VAMP_JIT_RAKE> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>> env_rake(*env_in);

    auto *result = new typename R::Debug(R::template fkcc_debug<VAMP_JIT_RAKE>(env_rake, block));
    return reinterpret_cast<vamp::jit::ffi::DebugHandle *>(result);
}

extern "C" void VAMP_JIT_DEBUG_DESTROY(vamp::jit::ffi::DebugHandle *h)
{
    // R::Debug isn't templated on R — it's always pair<vec<vec<string>>,
    // vec<pair<size_t, size_t>>>. Same type the host uses below.
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
    delete reinterpret_cast<DebugType *>(h);
}

extern "C" void VAMP_JIT_EEFK(const float *config, float *out_matrix)
{
    using R = vamp_jit_robot::R;
    typename R::ConfigurationArray cfg;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        cfg[i] = config[i];
    }

    // Static binding does Robot::eefk(array).matrix(); we mirror that, then
    // memcpy the 16 floats out (column-major — Eigen's default layout).
    Eigen::Matrix4f mat = R::eefk(cfg).matrix();
    std::memcpy(out_matrix, mat.data(), 16 * sizeof(float));
}

#undef VAMP_JIT_DEBUG
#undef VAMP_JIT_DEBUG_DESTROY
#undef VAMP_JIT_EEFK
#undef VAMP_JIT_RAKE
