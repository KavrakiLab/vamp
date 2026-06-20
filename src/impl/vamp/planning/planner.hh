#pragma once

#include <vamp/planning/aorrtc.hh>
#include <vamp/planning/fcit.hh>
#include <vamp/planning/grrtstar.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/rrtc.hh>

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

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

    struct PlannerDescriptor
    {
        std::string_view name;
        std::string_view class_name;
        std::string_view settings_class;
        std::string_view header;
        std::string_view settings_header;
    };

    inline constexpr std::array<PlannerDescriptor, 5> PLANNERS = {{
        {"rrtc", "RRTC", "RRTCSettings", "rrtc.hh", "rrtc_settings.hh"},
        {"prm", "PRM", "RoadmapSettings<vamp::planning::PRMStarNeighborParams>", "prm.hh", "roadmap.hh"},
        {"fcit", "FCIT", "RoadmapSettings<vamp::planning::FCITStarNeighborParams>", "fcit.hh", "roadmap.hh"},
        {"aorrtc", "AORRTC", "AORRTCSettings", "aorrtc.hh", "aorrtc_settings.hh"},
        {"grrtstar", "GRRTStar", "GRRTStarSettings", "grrtstar.hh", "grrtstar_settings.hh"},
    }};

    inline constexpr std::size_t N_PLANNERS = PLANNERS.size();

    constexpr auto planner_descriptor(Planner p) -> const PlannerDescriptor &
    {
        return PLANNERS[static_cast<std::size_t>(p)];
    }

    constexpr auto planner_name(Planner p) -> std::string_view
    {
        return planner_descriptor(p).name;
    }

    inline auto planner_from_name(std::string_view name) -> Planner
    {
        for (std::size_t i = 0; i < PLANNERS.size(); ++i)
        {
            if (PLANNERS[i].name == name)
            {
                return static_cast<Planner>(i);
            }
        }
        throw std::runtime_error("vamp::planning: unknown planner '" + std::string(name) + "'");
    }

    namespace detail
    {
        template <typename Robot, std::size_t rake, std::size_t resolution, Planner P>
        constexpr auto pick_planner()
        {
            if constexpr (P == Planner::RRTC)
            {
                return static_cast<RRTC<Robot, rake, resolution> *>(nullptr);
            }
            else if constexpr (P == Planner::PRM)
            {
                return static_cast<PRM<Robot, rake, resolution> *>(nullptr);
            }
            else if constexpr (P == Planner::FCIT)
            {
                return static_cast<FCIT<Robot, rake, resolution> *>(nullptr);
            }
            else if constexpr (P == Planner::AORRTC)
            {
                return static_cast<AORRTC<Robot, rake, resolution> *>(nullptr);
            }
            else if constexpr (P == Planner::GRRTSTAR)
            {
                return static_cast<GRRTStar<Robot, rake, resolution> *>(nullptr);
            }
        }
    }  // namespace detail

    template <typename Robot, std::size_t rake, std::size_t resolution, Planner P>
    using PlannerClass = std::remove_pointer_t<decltype(detail::pick_planner<Robot, rake, resolution, P>())>;
}  // namespace vamp::planning
