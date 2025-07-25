#pragma once
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

#include <utility>
#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/utils.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vector>

namespace vamp::planning
{

    template <
        typename Robot,
        std::size_t rake,
        std::size_t resolution,
        typename NeighborParamsT = PRMStarNeighborParams>
    struct PRM
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<dimension> roadmap;

            auto start_time = std::chrono::steady_clock::now();

            // Check if the straight-line solution is valid
            for (const auto &goal : goals)
            {
                if (validate_motion<Robot, rake, resolution>(start, goal, environment))
                {
                    result.path.emplace_back(start);
                    result.path.emplace_back(goal);
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = 0;
                    result.size.emplace_back(1);
                    result.size.emplace_back(1);

                    return result;
                }
            }

            std::size_t iter = 0;
            std::vector<std::pair<NNNode<dimension>, float>> neighbors;
            typename Robot::template ConfigurationBlock<rake> temp_block;
            auto states = std::unique_ptr<float>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded));
            // TODO: Is it better to just use arrays for these since we're reserving full capacity
            // anyway? Test it!
            std::vector<RoadmapNode> nodes;
            nodes.reserve(settings.max_samples);
            std::vector<utils::ConnectedComponent> components;
            components.reserve(settings.max_samples);

            const auto state_index = [&states](unsigned int index) -> float *
            { return states.get() + index * Configuration::num_scalars_rounded; };

            // Add start and goal to structures
            constexpr const unsigned int start_index = 0;

            auto *start_state = state_index(start_index);
            start.to_array(start_state);
            nodes.emplace_back(start_index, start_index, 0.0);
            roadmap.insert(NNNode<dimension>{start_index, {start_state}});
            components.emplace_back(utils::ConnectedComponent{start_index, 1});

            for (const auto &goal : goals)
            {
                uint32_t index = nodes.size();
                auto *goal_state = state_index(index);
                goal.to_array(goal_state);
                nodes.emplace_back(index, index);
                roadmap.insert(NNNode<dimension>{index, {goal_state}});
                components.emplace_back(utils::ConnectedComponent{index, 1});
            }

            const std::size_t goal_max_index = nodes.size();

            while (iter++ < settings.max_iterations and nodes.size() < settings.max_samples)
            {
                auto temp = rng->next();
                // TODO: This is a gross hack to get around the instruction cache issue...I realized
                // that edge sampling, while valid, wastes too much effort with our current
                // validation API

                // Check sample validity
                for (auto i = 0U; i < dimension; ++i)
                {
                    temp_block[i] = temp.broadcast(i);
                }

                if (not Robot::template fkcc<rake>(environment, temp_block))
                {
                    continue;
                }

                // Insert valid state into graph structures
                auto *state = state_index(nodes.size());
                temp.to_array(state);
                auto &node = nodes.emplace_back(nodes.size(), std::numeric_limits<unsigned int>::max());

                // Add valid edges
                const auto k = settings.neighbor_params.max_neighbors(roadmap.size());
                const auto r = settings.neighbor_params.neighbor_radius(roadmap.size());
                roadmap.nearest(neighbors, NNFloatArray<dimension>{state}, k, r);
                for (const auto &[neighbor, distance] : neighbors)
                {
                    if (validate_motion<Robot, rake, resolution>(neighbor.as_vector(), temp, environment))
                    {
                        node.neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{
                                static_cast<unsigned int>(neighbor.index), distance});
                        nodes[neighbor.index].neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{node.index, distance});
                    }
                }

                // Insert valid state into roadmap - after query to prevent returning self as
                // neighbor
                roadmap.insert(NNNode<dimension>{node.index, {state}});

                // Unify connected components
                if (node.neighbors.empty())
                {
                    node.component = components.size();
                    components.emplace_back(
                        utils::ConnectedComponent{static_cast<unsigned int>(components.size()), 1});
                }
                else
                {
                    node.component = nodes[node.neighbors.front().index].component;
                    for (const auto &neighbor : node.neighbors)
                    {
                        utils::merge_components(components, node.component, nodes[neighbor.index].component);
                    }
                }

                for (auto i = 1U; i < goal_max_index; ++i)
                {
                    // If the start and goal are in the same connected component, run A* to find the
                    // solution
                    if (utils::find_root(components, start_index) != utils::find_root(components, i))
                    {
                        continue;
                    }

                    const auto &goal = goals[i - 1];
                    auto parents = utils::astar(nodes, start, goal, state_index);
                    // NOTE: If the connected component check is correct, we can assume that a solution
                    // was found by A* when we've reached this point
                    utils::recover_path<Robot>(std::move(parents), state_index, result.path);
                    result.cost = nodes[i].g;
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = iter;
                    result.size.emplace_back(roadmap.size());
                    result.size.emplace_back(0);
                    return result;
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(roadmap.size());
            result.size.emplace_back(0);
            return result;
        }

        inline static auto build_roadmap(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings,
            typename RNG::Ptr rng) noexcept -> Roadmap<Robot>
        {
            NN<dimension> roadmap;

            constexpr const unsigned int start_index = 0;
            constexpr const unsigned int goal_index = 1;

            auto start_time = std::chrono::steady_clock::now();

            std::size_t iter = 0;
            std::vector<std::pair<NNNode<dimension>, float>> neighbors;
            typename Robot::template ConfigurationBlock<rake> temp_block;
            auto states = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded),
                &free);
            // TODO: Is it better to just use arrays for these since we're reserving full capacity
            // anyway? Test it!
            std::vector<RoadmapNode> nodes;
            nodes.reserve(settings.max_samples);

            const auto state_index = [&states](unsigned int index) -> float *
            { return states.get() + index * Configuration::num_scalars_rounded; };

            // Add start and goal to structures
            start.to_array(state_index(start_index));
            auto *goal_state = state_index(goal_index);
            goal.to_array(goal_state);
            nodes.emplace_back(start_index, start_index, 0.0);
            nodes.emplace_back(goal_index, goal_index);
            roadmap.insert(NNNode<dimension>{start_index, {state_index(start_index)}});
            roadmap.insert(NNNode<dimension>{goal_index, {goal_state}});

            while (iter++ < settings.max_iterations and nodes.size() < settings.max_samples)
            {
                auto temp = rng->next();

                // TODO: This is a gross hack to get around the instruction cache issue...I realized
                // that edge sampling, while valid, wastes too much effort with our current
                // validation API

                // Check sample validity
                for (auto i = 0U; i < dimension; ++i)
                {
                    temp_block[i] = temp.broadcast(i);
                }

                if (not Robot::template fkcc<rake>(environment, temp_block))
                {
                    continue;
                }

                // Insert valid state into graph structures
                auto *state = state_index(nodes.size());
                temp.to_array(state);
                auto &node = nodes.emplace_back(nodes.size(), std::numeric_limits<unsigned int>::max());

                // Add valid edges
                const auto k = settings.neighbor_params.max_neighbors(roadmap.size());
                const auto r = settings.neighbor_params.neighbor_radius(roadmap.size());
                roadmap.nearest(neighbors, NNFloatArray<dimension>{state}, k, r);
                for (const auto &[neighbor, distance] : neighbors)
                {
                    if (validate_motion<Robot, rake, resolution>(neighbor.as_vector(), temp, environment))
                    {
                        node.neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{
                                static_cast<unsigned int>(neighbor.index), distance});
                        nodes[neighbor.index].neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{node.index, distance});
                    }
                }

                // Insert valid state into roadmap - after query to prevent returning self as
                // neighbor
                roadmap.insert(NNNode<dimension>{node.index, {state}});
            }

            Roadmap<Robot> result;
            result.vertices.reserve(nodes.size());

            for (const auto &node : nodes)
            {
                result.vertices.emplace_back(state_index(node.index));
                result.edges.emplace_back(std::vector<std::size_t>());

                for (const auto &nbr : node.neighbors)
                {
                    result.edges.back().emplace_back(nbr.index);
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;

            return result;
        }
    };
}  // namespace vamp::planning
