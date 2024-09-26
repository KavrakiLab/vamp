#pragma once
#include <algorithm>
#include <array>
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
#include <vamp/random/halton.hh>
#include <vamp/random/uniform.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vector>

#include <iostream>
#include <fstream>
#include <string.h>

namespace vamp::planning
{

    template <
        typename Robot,
        typename RNG,
        std::size_t rake,
        std::size_t resolution,
        typename NeighborParamsT = FCITStarNeighborParams,
        std::size_t batch = 128>
    struct FCIT
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings) noexcept -> PlanningResult<dimension>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings) noexcept -> PlanningResult<dimension>
        {
            auto start_time = std::chrono::steady_clock::now();

            PlanningResult<dimension> result;
            NN<dimension> roadmap;

            int batch_size = 100;
            // Whether to use the standard halton sequence, or one offset by a random vector
            bool offset_halton = false;

            // Generate offest vector to offset halton sequence by a random (constant) value in each dimension
            RNG rng;
            // Each value in the offset vector will be in the range [0, 1/rng_offset]
            int rng_offset = 5;
            // Seed rng
            srand(std::clock()); 
            std::vector<float> offset_vec;
            for (int i = 0; i < 7; i++)
            {
                auto x = ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) / (rng_offset));
                offset_vec.push_back(x);
            }
            offset_vec.push_back(0);

            std::size_t iter = 0;
            typename Robot::template ConfigurationBlock<rake> temp_block;
            auto states = std::unique_ptr<float>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded));
            // TODO: Is it better to just use arrays for these since we're reserving full capacity
            // anyway? Test it!
            std::vector<FCITRoadmapNode> nodes;
            nodes.reserve(settings.max_samples);
            std::vector<unsigned int> parents;
            parents.reserve(goals.size() + 1);

            const auto state_index = [&states](unsigned int index) -> float *
            { return states.get() + index * Configuration::num_scalars_rounded; };

            // Add start and goal to structures
            constexpr const unsigned int start_index = 0;

            float *start_state = state_index(start_index);
            start.to_array(start_state);
            parents.emplace_back(std::numeric_limits<unsigned int>::max());
            nodes.emplace_back(start_index, 0.0);
            roadmap.insert(NNNode<dimension>{start_index, {start_state}});
            auto &start_node = nodes[start_index];
            start_node.neighbor_iterator = start_node.neighbors.begin();

            for (const auto &goal : goals)
            {
                std::size_t index = nodes.size();
                auto *goal_state = state_index(index);
                goal.to_array(goal_state);
                parents.emplace_back(std::numeric_limits<unsigned int>::max());
                nodes.emplace_back(index);
                roadmap.insert(NNNode<dimension>{index, {goal_state}});
            }

            float path_cost = std::numeric_limits<float>::infinity();
            int stop_time = settings.max_time * 1000000000;  // Convert s to ns

            Configuration temp_config;
            Configuration temp_config_self;
            std::vector<utils::QueueEdge> open_set;

            // Search until Initial Solution
            while (parents[1] == std::numeric_limits<unsigned int>::max())
            // ASAO Search
            //while (vamp::utils::get_elapsed_nanoseconds(start_time) < stop_time)
            {
                for (auto i = 0; i < goals.size(); ++i)
                {
                    const auto &goal = goals[i];
                    const auto &goal_node = nodes[i+1];

                    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                    // +++++++++++++++++++++++++++++++++++++++++++
                    // Iterate through neighbors and add all outgoing neighbors
                    for (auto it = nodes.begin() + start_node.sampleIdx; it != nodes.end(); it++)
                    {
                        if ((*it).index == start_index)
                        {
                            start_node.sampleIdx++;
                            continue;
                        }

                        const auto neighbor_index = (*it).index;
                        temp_config = state_index(neighbor_index);
                        const auto neighbor_distance = start.distance(temp_config);
                        start_node.sampleIdx = std::max(neighbor_index, start_node.sampleIdx);
                        
                        // g(p) + c^(p,c) < g(c)
                        if (neighbor_distance < (*it).g)
                        {
                            // f^(c) = g(p) + c^(p,c) + h^(c)
                            auto cost = neighbor_distance + goal.distance(temp_config);

                            start_node.neighbors.emplace_back(typename FCITRoadmapNode::Neighbor{
                                neighbor_index, cost});
                        }
                        start_node.sampleIdx++;
                    }
                    // ===========================================

                    pdqsort(
                        start_node.neighbors.begin(),
                        start_node.neighbors.end(),
                        [&goal, &state_index](const auto &a, const auto &b) { return a.distance < b.distance; });
                    start_node.neighbor_iterator = start_node.neighbors.begin();

                    open_set.emplace_back(
                        utils::QueueEdge{(*start_node.neighbor_iterator).index, start_index, (*start_node.neighbor_iterator).distance});
                    start_node.neighbor_iterator++;

                    while (not open_set.empty())
                    {
                        // +++++++++++++++++++++++++++++++++++++++++++
                        pdqsort(
                            open_set.begin(),
                            open_set.end(),
                            [](const auto &a, const auto &b) { return a.cost > b.cost; });
                        // ===========================================

                        const auto current = open_set.back();
                        open_set.pop_back();
            
                        const int current_index = current.index;
                        auto &current_node = nodes[current_index];
                        auto current_g = current_node.g;
                        int current_p = current.parent;

                        auto &parent_node = nodes[current_p];
                        // +++++++++++++++++++++++++++++++++++++++++++
                        while (parent_node.neighbor_iterator != parent_node.neighbors.end())
                        {
                            auto &node = nodes[(*parent_node.neighbor_iterator).index];

                            if ((*parent_node.neighbor_iterator).distance < node.g + goal.distance(state_index(node.index)))
                            {
                                open_set.emplace_back(
                                    utils::QueueEdge{(*parent_node.neighbor_iterator).index, current_p, (*parent_node.neighbor_iterator).distance});
                                parent_node.neighbor_iterator++;

                                break;
                            } 
                            else { parent_node.neighbor_iterator++; }
                        }
                        // ===========================================

                        bool skip_to_neighbors = false;

                        // +++++++++++++++++++++++++++++++++++++++++++
                        if (parents[current_index] == current_p)
                        {
                            skip_to_neighbors = true;
                        }
                        // ===========================================

                        if (!skip_to_neighbors)
                        {
                            temp_config_self = state_index(current_index);
                            auto dist_to_goal = goal.distance(temp_config_self);

                            if (current.cost <= goal_node.g)
                            {
                                // If this edge could improve the path through this node
                                if (current.cost < current_g + dist_to_goal)
                                {
                                    bool valid = !current_node.invalidList.count(current_p);

                                    // If this edge hasn't already been found as invalid
                                    if (valid)
                                    {
                                        if (current_index != current_p)
                                        {
                                            temp_config = state_index(current_p);
                                            valid = validate_motion<Robot, rake, resolution>(temp_config, temp_config_self, environment);
                                        }

                                        // If the edge is valid
                                        if (valid)
                                        {
                                            // Update the node's parent and g value
                                            parents[current_index] = current_p;
                                            current_g = parent_node.g + temp_config.distance(temp_config_self);
                                            current_node.g = current_g;
                                        } else {
                                            // Found to be invalid, add to necessary sets
                                            parent_node.invalidList.insert(current_index);
                                            current_node.invalidList.insert(current_p);

                                            (*(parent_node.neighbor_iterator - 1)).distance = std::numeric_limits<float>::max();
                                            continue;
                                        }
                                    } else {
                                        continue;
                                    }
                                } else {
                                    continue;
                                }
                            } else {
                                break;
                            }
                        }

                        int counter = 0;

                        // +++++++++++++++++++++++++++++++++++++++++++
                        current_node.neighbors.reserve(nodes.size());
                        // Iterate through neighbors and add all outgoing neighbors
                        for (auto it = nodes.begin() + current_node.sampleIdx; it != nodes.end(); it++)
                        {
                            if ((*it).index == current_index)
                            {
                                current_node.sampleIdx++;
                                continue;
                            }

                            const auto neighbor_index = (*it).index;
                            temp_config = state_index(neighbor_index);
                            const auto neighbor_distance = temp_config_self.distance(temp_config);
                            current_node.sampleIdx = std::max(neighbor_index, current_node.sampleIdx);
                            
                            // f^(c) = g(p) + c^(p,c) + h^(c)
                            auto cost = current_g + neighbor_distance + goal.distance(temp_config);

                            current_node.neighbors.emplace_back(typename FCITRoadmapNode::Neighbor{
                                neighbor_index, cost});
                            counter++;
                            current_node.sampleIdx++;
                        }
                        // ===========================================

                        // If I added any new neighbours
                        if (counter > 0)
                        {
                            pdqsort(
                                current_node.neighbors.begin(),
                                current_node.neighbors.end(),
                                [&goal, &state_index](const auto &a, const auto &b) { return a.distance < b.distance; });
                            current_node.neighbor_iterator = current_node.neighbors.begin();

                            open_set.emplace_back(
                                utils::QueueEdge{(*current_node.neighbor_iterator).index, current_index, (*current_node.neighbor_iterator).distance});
                            current_node.neighbor_iterator++;
                        }
                    }
                }
                
                parents.reserve(parents.size() + batch_size);

                iter++;
                int new_samples = 0;
                for (int i = 0; new_samples < batch_size && nodes.size() < settings.max_samples; i++)
                {
                    auto rng_temp = rng.next();

                    // If offsetting the halton sequence, add a random (constant) offset in each dimension, then wrap to fit within [0,1] 
                    if (settings.neighbor_params.offset_halton)
                    {
                        rng_temp = rng_temp * (1 + (1 / rng_offset));
                        rng_temp[0] = rng_temp[0] - offset_vec;

                        for (int i = 0; i < 7; i++)
                        {
                            if (rng_temp.data[0][i] < 0) { rng_temp.data[0][i] = rng_temp.data[0][i] + 1; }
                        }
                    }

                    Robot::scale_configuration(rng_temp);

                    // TODO: This is a gross hack to get around the instruction cache issue...I realized
                    // that edge sampling, while valid, wastes too much effort with our current
                    // validation API

                    // Check sample validity
                    for (auto i = 0U; i < dimension; ++i)
                    {
                        temp_block[i] = rng_temp.broadcast(i);
                    }

                    if (not Robot::template fkcc<rake>(environment, temp_block))
                    {
                        continue;
                    }
                    
                    // Insert valid state into graph structures
                    auto *state = state_index(nodes.size());
                    rng_temp.to_array(state);

                    parents.emplace_back(std::numeric_limits<unsigned int>::max());
                    auto &node = nodes.emplace_back(nodes.size());

                    auto road_node = NNNode<dimension>{node.index, {state}};
                    roadmap.insert(road_node);
                    new_samples++;
                }
            }

            utils::recover_path<Configuration>(parents, state_index, result.path);
            result.cost = nodes[1].g;
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
            const RoadmapSettings<NeighborParamsT> &settings) noexcept -> Roadmap<dimension>
        {
            NN<dimension> roadmap;

            constexpr const unsigned int start_index = 0;
            constexpr const unsigned int goal_index = 1;

            auto start_time = std::chrono::steady_clock::now();

            RNG rng;
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
                auto temp = rng.next();
                Robot::scale_configuration(temp);

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
                        node.neighbors.emplace_back(typename RoadmapNode::Neighbor{
                            static_cast<unsigned int>(neighbor.index), distance});
                        nodes[neighbor.index].neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{node.index, distance});
                    }
                }

                // Insert valid state into roadmap - after query to prevent returning self as
                // neighbor
                roadmap.insert(NNNode<dimension>{node.index, {state}});
            }

            Roadmap<dimension> result;
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