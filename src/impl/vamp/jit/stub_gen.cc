#include <vamp/jit/stub_gen.hh>

#include <vamp/jit/embedded_stubs.hh>

#include <inja/inja.hpp>
#include <nlohmann/json.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace vamp::jit
{
    namespace
    {
        struct PlannerDescriptor
        {
            std::string_view name;
            std::string_view class_name;
            std::string_view settings_class;
            std::string_view header;
            std::string_view settings_header;
        };

        constexpr PlannerDescriptor planner_table[] = {
            {"rrtc", "RRTC", "RRTCSettings", "rrtc.hh", "rrtc_settings.hh"},
            {"prm", "PRM", "RoadmapSettings<vamp::planning::PRMStarNeighborParams>", "prm.hh", "roadmap.hh"},
            {"fcit",
             "FCIT",
             "RoadmapSettings<vamp::planning::FCITStarNeighborParams>",
             "fcit.hh",
             "roadmap.hh"},
            {"aorrtc", "AORRTC", "AORRTCSettings", "aorrtc.hh", "aorrtc_settings.hh"},
            {"grrtstar", "GRRTStar", "GRRTStarSettings", "grrtstar.hh", "grrtstar_settings.hh"},
        };

        constexpr auto descriptor(vamp::planning::Planner p) -> const PlannerDescriptor &
        {
            return planner_table[static_cast<std::size_t>(p)];
        }
    }  // namespace

    auto planner_symbol(vamp::planning::Planner p, std::string_view suffix) -> std::string
    {
        return std::string("vamp_jit_") + std::string(descriptor(p).name) + "_" + std::string(suffix);
    }

    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string
    {
        return std::string("vamp_jit_") + robot_name + "_" + std::string(suffix);
    }

    auto generate_stub_source(const StubOptions &opts) -> std::string
    {
        if (opts.robot_source.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: empty robot_source");
        }

        if (opts.robot_name.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: empty robot_name");
        }

        if (opts.planners.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: no planners requested");
        }

        std::ostringstream out;
        out << embedded::preamble << "\n" << opts.robot_source << "\n";

        inja::Environment env;

        nlohmann::json base = {
            {"robot_name", opts.robot_name},
            {"rake", opts.rake},
            {"resolution", opts.resolution},
        };

        out << env.render(std::string(embedded::sampler_stub), base);
        out << env.render(std::string(embedded::simplify_stub), base);
        out << env.render(std::string(embedded::debug_stub), base);
        out << env.render(std::string(embedded::phs_stub), base);

        for (auto p : opts.planners)
        {
            const auto &d = descriptor(p);
            nlohmann::json data = base;
            data["planner_name"] = std::string(d.name);
            data["planner_class"] = std::string(d.class_name);
            data["settings_class"] = std::string(d.settings_class);
            data["planner_header"] = std::string(d.header);
            data["settings_header"] = std::string(d.settings_header);
            out << env.render(std::string(embedded::planner_stub), data);
        }

        return out.str();
    }
}  // namespace vamp::jit
