#include <vamp/jit/stub_gen.hh>

#include <vamp/jit/embedded_stubs.hh>

#include <inja/inja.hpp>
#include <nlohmann/json.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace vamp::jit
{
    auto planner_symbol(vamp::planning::Planner p, std::string_view suffix) -> std::string
    {
        return std::string("vamp_jit_") + std::string(vamp::planning::planner_name(p)) + "_" +
               std::string(suffix);
    }

    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string
    {
        return std::string("vamp_jit_") + robot_name + "_" + std::string(suffix);
    }

    auto generate_stub_source(
        const std::string &robot_source,
        const std::string &robot_name,
        std::size_t rake,
        std::size_t resolution,
        const std::vector<vamp::planning::Planner> &planners) -> std::string
    {
        if (robot_source.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: empty robot_source");
        }

        if (robot_name.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: empty robot_name");
        }

        if (planners.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: no planners requested");
        }

        std::ostringstream out;
        out << embedded::preamble << "\n" << robot_source << "\n";

        inja::Environment env;

        nlohmann::json base = {
            {"robot_name", robot_name},
            {"rake", rake},
            {"resolution", resolution},
        };

        out << env.render(std::string(embedded::robot_stub), base);

        for (auto p : planners)
        {
            const auto &d = vamp::planning::planner_descriptor(p);
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
