#pragma once

#include <memory>
#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/rrt_star_settings.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <cmath>

namespace vamp::planning
{
    template <typename Robot, typename RNG, std::size_t rake, std::size_t resolution>
    struct RRT_star
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRT_star_settings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<dimension>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRT_star_settings &settings,
            typename RNG::Ptr rng) -> PlanningResult<dimension>
        {
            // algorithm specific constants
            const auto unit_ball_volume =
                std::pow(std::sqrt(M_PI), dimension) / std::tgamma(dimension / 2.0 + 1.0);
            const auto free_volume = Robot::space_measure();
            const auto dim_recip = 1.0 / dimension;
            const auto gamma_rrt =
                std::pow(2 * (1.0 + 1.0 / dimension) * (free_volume / unit_ball_volume), dim_recip);

            PlanningResult<dimension> result;
            NN<dimension> tree;
            constexpr const std::size_t start_index = 0;

            auto buffer = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded),
                &free);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            std::vector<std::size_t> parent(settings.max_samples);
            std::vector<std::vector<std::size_t>> children(settings.max_samples, std::vector<std::size_t>());
            std::vector<float> cost(settings.max_samples);
            std::vector<bool> collision_free(settings.max_samples);

            const auto add_edge = [&parent, &children](std::size_t parent_node, std::size_t child_node)
            {
                parent[child_node] = parent_node;
                children[parent_node].emplace_back(child_node);
            };

            const auto remove_edge = [&parent, &children](std::size_t parent_node, std::size_t child_node)
            {
                parent[child_node] = -1;
                auto it = std::find(children[parent_node].begin(), children[parent_node].end(), child_node);
                if (it != children[parent_node].end())
                {
                    children[parent_node].erase(it);
                }
            };

            const std::function<void(std::size_t)> update_children =
                [&cost, &children, &buffer_index, &update_children](std::size_t idx)
            {
                Configuration cur(buffer_index(idx));
                for (const auto &child : children[idx])
                {
                    cost[child] = cost[idx] + cur.distance(buffer_index(child));
                }

                for (const auto &child : children[idx])
                {
                    update_children(child);
                }
            };

            const auto build_path = [&result, &buffer_index, &parent](
                                        const Configuration &goal, const std::size_t last_config_idx)
            {
                result.path.emplace_back(goal);
                result.path.emplace_back(buffer_index(last_config_idx));
                auto current = last_config_idx;
                while (parent[current] != current)
                {
                    auto parent_idx = parent[current];
                    result.path.emplace_back(buffer_index(parent_idx));
                    current = parent_idx;
                }
                std::reverse(result.path.begin(), result.path.end());
            };

            auto start_time = std::chrono::steady_clock::now();

            for (const auto &goal : goals)
            {
                if (validate_motion<Robot, rake, resolution>(start, goal, environment))
                {
                    result.path.emplace_back(start);
                    result.path.emplace_back(goal);
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = 0;
                    result.size.emplace_back(1);
                    return result;
                }
            }

            std::size_t free_index = start_index + 1;

            // add start to tree
            start.to_array(buffer_index(start_index));
            tree.insert(NNNode<dimension>{start_index, {buffer_index(start_index)}});
            parent[start_index] = start_index;
            cost[start_index] = 0.0;

            std::vector<std::pair<std::size_t, Configuration>> goal_motions;
            float best_path_cost = std::numeric_limits<float>::max();
            std::pair<std::size_t, Configuration> best_goal_motion;

            // main loop
            std::size_t iter = 0;
            std::vector<std::pair<NNNode<dimension>, float>> near;
            near.reserve(settings.max_samples);

            while (iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                // sample random config
                auto sample_config = rng->next();
                Robot::scale_configuration(sample_config);
                typename Robot::ConfigurationBuffer sample_config_arr;
                sample_config.to_array(sample_config_arr.data());

                const auto nearest = tree.nearest(NNFloatArray<dimension>{sample_config_arr.data()});

                if (not nearest)
                {
                    continue;
                }

                // steer (calculate next state within range)
                const auto &[nearest_node, nearest_distance] = *nearest;
                auto nearest_configuration = nearest_node.as_vector();
                auto nearest_vector = sample_config - nearest_configuration;

                bool reach = nearest_distance < settings.range;
                auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (settings.range / nearest_distance);

                if (validate_vector<Robot, rake, resolution>(
                        nearest_configuration,
                        extension_vector,
                        (reach) ? nearest_distance : settings.range,
                        environment))
                {
                    // insert new config into vertex set
                    float *new_configuration_ptr = buffer_index(free_index);
                    auto new_configuration = nearest_configuration + extension_vector;
                    new_configuration.to_array(new_configuration_ptr);
                    NNNode<dimension> new_node({free_index, {new_configuration_ptr}});

                    // sample points in graph within radius r of new config
                    const std::size_t card =
                        tree.size() + 1;  // plus 1 to account for the new configuration not inserted yet
                    const float rrt_radius = std::fmin(
                        settings.rewire_factor * gamma_rrt * std::pow((std::log(card) / card), dim_recip),
                        settings.range);
                    tree.nearest(near, NNFloatArray<dimension>{new_configuration_ptr}, card, rrt_radius);

                    // initialize variables to keep track of the min cost neighbor and the cost of that
                    // neighbor
                    auto min_neighbor = nearest_node;
                    float min_cost = cost[nearest_node.index] + nearest_distance;

                    // loop through near neighbors, find the one that gives us min cost from root
                    for (auto i = 0U; i < near.size(); ++i)
                    {
                        const auto &[node, distance] = near[i];
                        auto nbr_configuration = node.as_vector();
                        const float nbr_cost = cost[node.index] + distance;
                        if (nbr_cost < min_cost)
                        {
                            if ((collision_free[i] = validate_motion<Robot, rake, resolution>(
                                     nbr_configuration, new_configuration, environment)))
                            {
                                min_cost = nbr_cost;
                                min_neighbor = node;
                            }
                        }
                        else
                        {
                            collision_free[i] = false;
                        }
                    }

                    // add an edge from min cost neighbor to new config
                    tree.insert(new_node);
                    add_edge(min_neighbor.index, free_index);
                    cost[free_index] = min_cost;

                    // rewire with our newly added node to if we can use it to get a shorter path to any of
                    // its neighbors
                    bool check_new_best = false;
                    for (auto i = 0U; i < near.size(); ++i)
                    {
                        const auto &[node, distance] = near[i];
                        if (collision_free[i] and (min_cost + distance < cost[node.index]))
                        {
                            remove_edge(parent[node.index], node.index);
                            add_edge(free_index, node.index);

                            // propgate saved cost to children
                            cost[node.index] = min_cost + distance;
                            update_children(node.index);
                            check_new_best = true;
                        }
                    }

                    free_index++;

                    // check if we can reach a goal
                    for (const auto &goal : goals)
                    {
                        if (validate_motion<Robot, rake, resolution>(new_configuration, goal, environment))
                        {
                            if (not settings.optimize)
                            {  // found a solution, exit the algorithm
                                auto current = free_index - 1;
                                build_path(goal, current);
                                result.cost = cost[current] + goal.distance(buffer_index(current));
                                break;
                            }
                            else
                            {
                                goal_motions.emplace_back(free_index - 1, goal);
                                check_new_best = true;
                            }
                        }
                    }

                    if (settings.optimize and check_new_best)
                    {
                        // loop through goal nodes to find the best solution
                        bool updated = false;
                        for (const auto &[end_node_idx, goal] : goal_motions)
                        {
                            const float cur_cost =
                                cost[end_node_idx] + goal.distance(buffer_index(end_node_idx));
                            if (cur_cost < best_path_cost)
                            {
                                best_path_cost = cur_cost;
                                best_goal_motion = {end_node_idx, goal};
                                updated = true;
                            }
                        }

                        if (updated)
                        {
                            result.intermediate.emplace_back(Intermediate{
                                best_path_cost,
                                iter,
                                vamp::utils::get_elapsed_nanoseconds(start_time),
                                tree.size()});
                        }
                    }
                }
            }

            if (not settings.optimize)
            {
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                result.iterations = iter;
                result.size.emplace_back(tree.size());
                return result;
            }

            if (goal_motions.size() > 0)
            {
                result.cost = best_path_cost;
                const auto &[best_end_node_idx, best_goal] = best_goal_motion;
                build_path(best_goal, best_end_node_idx);
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(tree.size());
            result.intermediate.emplace_back(
                Intermediate{best_path_cost, iter, result.nanoseconds, tree.size()});
            return result;
        }
    };
}  // namespace vamp::planning
