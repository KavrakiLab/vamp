#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace vamp::planning
{
    enum class Planner
    {
        RRTC,
        PRM,
        FCIT,
        AORRTC,
        GRRTSTAR,
    };

    inline auto planner_name(Planner p) -> std::string_view
    {
        switch (p)
        {
            case Planner::RRTC:
                return "rrtc";
            case Planner::PRM:
                return "prm";
            case Planner::FCIT:
                return "fcit";
            case Planner::AORRTC:
                return "aorrtc";
            case Planner::GRRTSTAR:
                return "grrtstar";
        }
        return "";
    }

    inline auto planner_from_name(std::string_view name) -> Planner
    {
        if (name == "rrtc")
        {
            return Planner::RRTC;
        }
        if (name == "prm")
        {
            return Planner::PRM;
        }
        if (name == "fcit")
        {
            return Planner::FCIT;
        }
        if (name == "aorrtc")
        {
            return Planner::AORRTC;
        }
        if (name == "grrtstar")
        {
            return Planner::GRRTSTAR;
        }
        throw std::runtime_error("vamp::planning: unknown planner '" + std::string(name) + "'");
    }
}  // namespace vamp::planning
