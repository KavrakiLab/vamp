#include <vector>
#include <array>
#include <utility>
#include <iostream>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>

#include <vamp/robots/panda.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/util/Exception.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

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

// Maximum planning time
static constexpr float planning_time = 1.0;

// Maximum simplification time
static constexpr float simplification_time = 1.0;

// Convert an OMPL state into a VAMP vector
inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy OMPL data into aligned buffer
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(as->values[i]);
    }

    // Create configuration from aligned buffer data
    return Configuration(aligned_buffer.data());
}

// Convert a VAMP vector to an OMPL state
inline static auto vamp_to_ompl(const Configuration &c, ob::State *state)
{
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        as->values[i] = static_cast<double>(c[{i, 0}]);
    }
}

// State validator using VAMP
struct VAMPStateValidator : public ob::StateValidityChecker
{
    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        // Convert OMPL to VAMP vector and validate
        auto configuration = ompl_to_vamp(state);
        return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
    }

    const EnvironmentVector &env_v;
};

struct VAMPMotionValidator : public ob::MotionValidator
{
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
        // Convert OMPL states to VAMP vectors and check motion between states
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

auto main(int argc, char **) -> int
{
    bool optimize = false;  // Flag - if true, will spend entire planning budget optimizing, otherwise exit on
                            // first solution

    // Set optimize flag if another argument is provided
    if (argc == 2)
    {
        optimize = true;
    }

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
    static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

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

    si->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si, env_v));
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

    if (not optimize)
    {
        // Set planner to terminate as soon as a solution is found.
        obj->setCostThreshold(obj->infiniteCost());
    }

    // Create planner - BITstar by default, but you can change this to use other geometric planners instead
    auto planner = std::make_shared<og::BITstar>(si);

    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve the problem
    auto start_time = std::chrono::steady_clock::now();
    ob::PlannerStatus solved = planner->ob::Planner::solve(planning_time);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    // Only accept exact solutions
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;

        // Simplify the path using OMPL's path simplification
        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(si, pdef->getGoal(), obj);
        if (not simplifier.simplify(path_geometric, simplification_time))
        {
            std::cout << "Path not valid!" << std::endl;
        }

        auto simplified_cost = path_geometric.cost(obj);

        // Output statistics
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
