#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace vamp::jit
{
    enum class Planner
    {
        RRTC,
        PRM,
        FCIT,
        AORRTC,
        GRRTSTAR,
    };

    struct StubOptions
    {
        std::string robot_source;
        std::string robot_name;
        std::size_t rake;
        std::size_t resolution;
        std::vector<Planner> planners;
    };

    auto generate_stub_source(const StubOptions &opts) -> std::string;

    // extern "C" symbol the stub source exports for a (planner, suffix) pair.
    // suffix is one of: "solve", "solve_multi", "result_meta",
    // "result_copy_waypoint", "result_destroy".
    auto planner_symbol(Planner p, std::string_view suffix) -> std::string;

    // extern "C" symbol the stub source exports for a (robot, suffix) pair.
    // suffix is one of: "simplify", "simplify_result_meta",
    // "simplify_result_copy_waypoint", "simplify_result_destroy",
    // "sampler_halton", "sampler_xorshift", "sampler_reset", "sampler_skip",
    // "sampler_next", "sampler_destroy".
    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string;
}  // namespace vamp::jit
