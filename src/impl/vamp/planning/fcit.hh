#pragma once
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>

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

#include <pdqsort.h>

namespace vamp::planning
{

    struct QueueEdge
    {
        unsigned int index;
        unsigned int parent;
        float cost;

        inline constexpr auto operator==(const QueueEdge &o) const noexcept -> bool
        {
            return index == o.index;
        }
    };

    struct FCITRoadmapNode
    {
    public:
        FCITRoadmapNode(
            unsigned int index,
            float g = std::numeric_limits<float>::infinity(),
            unsigned int sampleIdx = 0)
          : index(index), g(g), sampleIdx(sampleIdx)
        {
        }

        unsigned int index;
        float g;
        unsigned int sampleIdx;

        struct Neighbor
        {
            unsigned int index;
            float distance;
        };

        std::vector<Neighbor> neighbors;
        std::vector<Neighbor>::iterator neighbor_iterator;
        std::unordered_set<int> invalidList;
    };

    template <
        typename Robot,
        std::size_t rake,
        std::size_t resolution,
        typename NeighborParamsT = FCITStarNeighborParams>
    struct FCIT
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings,
            typename RNG::Ptr &rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings,
            typename RNG::Ptr &rng) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();

            PlanningResult<Robot> result;
            NN<dimension> roadmap;

            std::size_t iter = 0;
            typename Robot::template ConfigurationBlock<rake> temp_block;
            auto states = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded),
                &free);

            // TODO: Is it better to just use arrays for these since we're reserving full capacity
            // anyway? Test it!
            std::vector<FCITRoadmapNode> nodes;
            nodes.reserve(settings.max_samples);
            std::vector<unsigned int> parents;
            parents.reserve(settings.max_samples);

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

            Configuration temp_config;
            Configuration temp_config_self;
            std::vector<QueueEdge> open_set;

            // Search until Initial Solution
            while (nodes.size() < settings.max_samples and iter++ < settings.max_iterations)
            {
                for (auto i = 0U; i < goals.size(); ++i)
                {
                    const auto &goal = goals[i];
                    const auto &goal_node = nodes[i + 1];

                    // Iterate through neighbors and add all outgoing neighbors
                    for (auto it = nodes.begin() + start_node.sampleIdx; it != nodes.end(); it++)
                    {
                        if (it->index == start_index)
                        {
                            start_node.sampleIdx++;
                            continue;
                        }

                        const auto neighbor_index = it->index;
                        temp_config = state_index(neighbor_index);
                        const auto neighbor_distance = start.distance(temp_config);
                        start_node.sampleIdx = std::max(neighbor_index, start_node.sampleIdx);

                        // g(p) + c^(p,c) < g(c)
                        if (neighbor_distance < it->g)
                        {
                            // f^(c) = g(p) + c^(p,c) + h^(c)
                            const auto cost = neighbor_distance + goal.distance(temp_config);
                            start_node.neighbors.emplace_back(
                                typename FCITRoadmapNode::Neighbor{neighbor_index, cost});
                        }
                        start_node.sampleIdx++;
                    }
                    // ===========================================

                    pdqsort_branchless(
                        start_node.neighbors.begin(),
                        start_node.neighbors.end(),
                        [](const auto &a, const auto &b) { return a.distance < b.distance; });
                    start_node.neighbor_iterator = start_node.neighbors.begin();

                    open_set.emplace_back(
                        QueueEdge{
                            (*start_node.neighbor_iterator).index,
                            start_index,
                            (*start_node.neighbor_iterator).distance});
                    start_node.neighbor_iterator++;

                    while (not open_set.empty())
                    {
                        pdqsort_branchless(
                            open_set.begin(),
                            open_set.end(),
                            [](const auto &a, const auto &b) { return a.cost > b.cost; });

                        const auto current = open_set.back();
                        open_set.pop_back();

                        const auto current_index = current.index;
                        auto &current_node = nodes[current_index];
                        auto current_g = current_node.g;
                        auto current_p = current.parent;

                        auto &parent_node = nodes[current_p];
                        // +++++++++++++++++++++++++++++++++++++++++++
                        while (parent_node.neighbor_iterator != parent_node.neighbors.end())
                        {
                            const auto &node = nodes[(*parent_node.neighbor_iterator).index];

                            if ((*parent_node.neighbor_iterator).distance <
                                node.g + goal.distance(state_index(node.index)))
                            {
                                open_set.emplace_back(
                                    QueueEdge{
                                        (*parent_node.neighbor_iterator).index,
                                        current_p,
                                        (*parent_node.neighbor_iterator).distance});
                                parent_node.neighbor_iterator++;
                                break;
                            }

                            parent_node.neighbor_iterator++;
                        }
                        // ===========================================

                        if (parents[current_index] != current_p)
                        {
                            temp_config_self = state_index(current_index);
                            const auto dist_to_goal = goal.distance(temp_config_self);

                            if (current.cost <= goal_node.g)
                            {
                                // If this edge could improve the path through this node
                                if (current.cost < current_g + dist_to_goal)
                                {
                                    bool valid = not current_node.invalidList.count(current_p);

                                    // If this edge hasn't already been found as invalid
                                    if (valid)
                                    {
                                        if (current_index != current_p)
                                        {
                                            temp_config = state_index(current_p);
                                            valid = validate_motion<Robot, rake, resolution>(
                                                temp_config, temp_config_self, environment);
                                        }

                                        // If the edge is valid
                                        if (valid)
                                        {
                                            // Update the node's parent and g value
                                            parents[current_index] = current_p;
                                            current_g =
                                                parent_node.g + temp_config.distance(temp_config_self);
                                            current_node.g = current_g;
                                        }
                                        else
                                        {
                                            // Found to be invalid, add to necessary sets
                                            parent_node.invalidList.insert(current_index);
                                            current_node.invalidList.insert(current_p);

                                            (*(parent_node.neighbor_iterator - 1)).distance =
                                                std::numeric_limits<float>::max();
                                            continue;
                                        }
                                    }
                                }
                            }
                            else
                            {
                                break;
                            }
                        }

                        current_node.neighbors.reserve(nodes.size());

                        bool added_neighbors = false;
                        // Iterate through neighbors and add all outgoing neighbors
                        for (auto it = nodes.begin() + current_node.sampleIdx; it != nodes.end(); it++)
                        {
                            if (it->index == current_index)
                            {
                                current_node.sampleIdx++;
                                continue;
                            }

                            const auto neighbor_index = it->index;
                            temp_config = state_index(neighbor_index);
                            const auto neighbor_distance = temp_config_self.distance(temp_config);
                            current_node.sampleIdx = std::max(neighbor_index, current_node.sampleIdx);

                            // f^(c) = g(p) + c^(p,c) + h^(c)
                            auto cost = current_g + neighbor_distance + goal.distance(temp_config);

                            current_node.neighbors.emplace_back(
                                typename FCITRoadmapNode::Neighbor{neighbor_index, cost});
                            current_node.sampleIdx++;

                            added_neighbors = true;
                        }

                        // If I added any new neighbours
                        if (added_neighbors)
                        {
                            pdqsort_branchless(
                                current_node.neighbors.begin(),
                                current_node.neighbors.end(),
                                [](const auto &a, const auto &b) { return a.distance < b.distance; });
                            current_node.neighbor_iterator = current_node.neighbors.begin();

                            open_set.emplace_back(
                                QueueEdge{
                                    (*current_node.neighbor_iterator).index,
                                    current_index,
                                    (*current_node.neighbor_iterator).distance});
                            current_node.neighbor_iterator++;
                        }
                    }
                }

                // If we have a solution and just want an initial solution, break
                if (not settings.optimize and parents[1] != std::numeric_limits<unsigned int>::max())
                {
                    break;
                }

                for (auto new_samples = 0U;
                     new_samples < settings.batch_size and nodes.size() < settings.max_samples;)
                {
                    auto rng_temp = rng->next();

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

            utils::recover_path<Robot>(parents, state_index, result.path);
            result.cost = nodes[1].g;
            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(roadmap.size());
            result.size.emplace_back(0);

            return result;
        }
    };
}  // namespace vamp::planning
