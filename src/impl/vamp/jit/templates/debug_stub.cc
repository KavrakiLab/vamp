// Per-robot debug stub. Wraps Robot::fkcc_debug<rake>() so the host can
// inspect per-sphere environment-collision and self-collision info, mirroring
// vamp.<robot>.debug() on the static side.
//
// inja substitutions:
//   {{robot_name}} — struct name inside vamp::robots::
//   {{rake}}       — SIMD lane width (the static binding uses rake too)

extern "C" vamp::jit::ffi::DebugHandle *vamp_jit_{{robot_name}}_debug(
    const float *config,
    const void *env_ptr)
{
    using R = vamp_jit_robot::R;

    // Broadcast the scalar configuration into a rake-wide block, mirroring
    // the static binding's Input::template block<rake>(c).
    typename R::template ConfigurationBlock<{{rake}}> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }

    const auto *env_in = static_cast<const vamp::collision::Environment<float> *>(env_ptr);
    vamp::collision::Environment<vamp::FloatVector<{{rake}}>> env_rake(*env_in);

    auto *result = new typename R::Debug(R::template fkcc_debug<{{rake}}>(env_rake, block));
    return reinterpret_cast<vamp::jit::ffi::DebugHandle *>(result);
}

extern "C" void vamp_jit_{{robot_name}}_debug_destroy(vamp::jit::ffi::DebugHandle *h)
{
    // R::Debug isn't templated on R — it's always pair<vec<vec<string>>,
    // vec<pair<size_t, size_t>>>. Same type the host uses below.
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
    delete reinterpret_cast<DebugType *>(h);
}
