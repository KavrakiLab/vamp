#pragma once

#include <vamp/planning/planner.hh>

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace vamp::jit
{
    struct StubOptions
    {
        std::string robot_source;
        std::string robot_name;
        std::size_t rake;
        std::size_t resolution;
        std::vector<vamp::planning::Planner> planners;
    };

    auto generate_stub_source(const StubOptions &opts) -> std::string;
    auto planner_symbol(vamp::planning::Planner p, std::string_view suffix) -> std::string;
    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string;
}  // namespace vamp::jit
