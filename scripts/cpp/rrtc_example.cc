#include <vector>
#include <array>
#include <utility>
#include <iostream>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr std::array<float, dimension> start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr std::array<float, dimension> goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    auto rng = std::make_shared<vamp::rng::Halton<Robot::dimension>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 1.0;
    auto result = RRTC::solve(Configuration(start), Configuration(goal), env_v, rrtc_settings, rng);

    if (result.path.size() > 0)
    {
        vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
            result.path, EnvironmentVector(environment), simplify_settings, rng);

        for (const auto &configuration : simplify_result.path)
        {
            std::cout << configuration << std::endl;
        }
    }

    return 0;
}
