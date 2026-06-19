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
    auto planner_symbol(Planner p, std::string_view suffix) -> std::string;
    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string;
}  // namespace vamp::jit
