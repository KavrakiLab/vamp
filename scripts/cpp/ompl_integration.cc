#include <vamp/planning/validate.hh>
#include <vector>
#include <array>

#include <vamp/collision/factory.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/robots/panda.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Exception.h>

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
using Halton = vamp::rng::Halton<dimension>;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using RRTC = vamp::planning::RRTC<Robot, Halton, rake, Robot::resolution>;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

const std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
const std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

const std::array<float, dimension> start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
const std::array<float, dimension> goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
const std::vector<std::array<float, 3>> problem = {
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
static constexpr const float radius = 0.2;

namespace ob = ompl::base;
namespace og = ompl::geometric;

inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(as->values[i]);
    }

    Configuration c(aligned_buffer.data());
    return c;
}

class VAMPMotionValidator : public ob::MotionValidator
{
public:
    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
            ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
        -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }

    const EnvironmentVector &env_v;
};

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

    // Create OMPL state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);

    // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    auto zero_v = Configuration(zeros);
    auto one_v = Configuration(ones);

    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);

    ob::RealVectorBounds bounds(dimension);
    for (auto i = 0U; i < dimension; ++i)
    {
        bounds.setLow(i, zero_v[{i, 0}]);
        bounds.setHigh(i, one_v[{i, 0}]);
    }

    space->setBounds(bounds);

    // Create space information and set state validator and custom VAMP motion validator
    auto si = std::make_shared<ob::SpaceInformation>(space);

    si->setStateValidityChecker(
        [&env_v](const ob::State *state)
        {
            auto configuration = ompl_to_vamp(state);
            return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
        });

    si->setMotionValidator(std::make_shared<VAMPMotionValidator>(si, env_v));
    si->setup();

    // Set start and goal
    ob::ScopedState<> start_ompl(space), goal_ompl(space);
    for (auto i = 0U; i < dimension; ++i)
    {
        start_ompl[i] = start[i];
        goal_ompl[i] = goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start_ompl, goal_ompl);

    // Set optimization objective
    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    pdef->setOptimizationObjective(obj);

    // Create planner
    auto planner = std::make_shared<og::RRTstar>(si);
    // planner->setRange(1.0);

    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution! Simplfying..." << std::endl;
        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric&>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(si, pdef->getGoal(), obj);
        if (not simplifier.simplify(path_geometric, 1.0))
        {
            std::cout << "Path not valid!" << std::endl;
        }

        auto simplified_cost = path_geometric.cost(obj);

        std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
        std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}
