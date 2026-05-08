/**
 * @brief Greedy-RRT* (G-RRT*) is a greedy version of the anytime Rapidly-exploring Random Trees algorithm.
 * It finds initial solutions as quickly as RRT-Connect by maintaining two balanced trees
 * rooted at the start and goal, advancing toward each other using greedy connection heuristics.
 * The algorithm then performs incremental rewiring of the growing trees to asymptotically find
 * better solutions, similar to RRT*. Both approximation and search in G-RRT* use a greedy version of
 * the direct informed sampling heuristic to focus exploration on promising regions of the state space.
 *
 * @author Phone Thiha Kyaw (University of Toronto)
 *
 * @par Associated publications:
 *
 * Kyaw, P. T., Le, A. V., Elara, M. R., & Kelly, J. (2025).
 * Greedy heuristics for sampling-based motion planning in high-dimensional state spaces.
 * arXiv preprint arXiv:2405.03411.
 * DOI: https://doi.org/10.48550/arXiv.2405.03411
 * arXiv: https://arxiv.org/abs/2405.03411 [cs.RO]
 *
 * Kyaw, P. T., et al. (2022).
 * Energy-efficient path planning of reconfigurable robots in complex environments.
 * IEEE Transactions on Robotics, 38(4), 2481-2494.
 * DOI: https://doi.org/10.1109/TRO.2022.3147408
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <pdqsort.h>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/grrtstar_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct GRRTStar
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
        using NNNodeType = NNNode<dimension>;
        using NNTree = NN<dimension>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const GRRTStarSettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const GRRTStarSettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            auto start_time = std::chrono::steady_clock::now();

            NNTree start_tree;
            NNTree goal_tree;

            constexpr const std::size_t start_index = 0;

            auto buffer = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded),
                &free);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            std::vector<std::size_t> parents(settings.max_samples);
            std::vector<float> costs(settings.max_samples, std::numeric_limits<float>::max());
            std::vector<float> radii(settings.max_samples);
            std::vector<std::vector<std::size_t>> children(settings.max_samples);
            std::vector<char> in_start_tree(settings.max_samples, 0);
            std::vector<char> pruned(settings.max_samples, 0);

            // Best solution tracking
            float best_cost = std::numeric_limits<float>::max();
            float greedy_best_cost = std::numeric_limits<float>::max();
            Path<Robot> best_path;

            // Heuristic cost lower bound for a node
            const auto solution_heuristic = [&](std::size_t idx) -> float
            {
                Configuration cfg(buffer_index(idx));
                float g_hat = cfg.distance(start);
                float h_hat = std::numeric_limits<float>::max();
                for (const auto &goal : goals)
                {
                    h_hat = std::min(h_hat, cfg.distance(goal));
                }
                return g_hat + h_hat;
            };

            // Check if a node and ALL its descendants can be pruned
            const auto can_prune_branch = [&](const auto &self, std::size_t idx, float threshold) -> bool
            {
                if (solution_heuristic(idx) < threshold)
                {
                    return false;
                }
                for (const auto child_idx : children[idx])
                {
                    if (not pruned[child_idx] and not self(self, child_idx, threshold))
                    {
                        return false;
                    }
                }
                return true;
            };

            // Mark a node and all its descendants as pruned
            const auto mark_pruned = [&](const auto &self, std::size_t idx) -> void
            {
                pruned[idx] = 1;
                for (const auto child_idx : children[idx])
                {
                    self(self, child_idx);
                }
            };

            // Initialize start node
            start.to_array(buffer_index(start_index));
            start_tree.insert(NNNodeType{start_index, {buffer_index(start_index)}});
            parents[start_index] = start_index;
            costs[start_index] = 0.F;
            radii[start_index] = std::numeric_limits<float>::max();
            in_start_tree[start_index] = 1;

            std::size_t free_index = start_index + 1;

            // Initialize goal nodes
            std::vector<std::size_t> goal_indices;
            goal_indices.reserve(goals.size());
            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                goal_tree.insert(NNNodeType{free_index, {buffer_index(free_index)}});
                parents[free_index] = free_index;
                costs[free_index] = 0.F;
                radii[free_index] = std::numeric_limits<float>::max();
                in_start_tree[free_index] = 0;
                goal_indices.push_back(free_index);
                free_index++;
            }

            // Rebuild a tree from scratch, keeping only unpruned nodes.
            const auto rebuild_tree = [&](NNTree &tree, bool is_start)
            {
                tree.clear();
                for (std::size_t i = 0; i < free_index; ++i)
                {
                    if (not pruned[i] and static_cast<bool>(in_start_tree[i]) == is_start)
                    {
                        tree.insert(NNNodeType{i, {buffer_index(i)}});
                    }
                }
            };

            // Prune both trees and rebuild
            const auto prune_trees = [&](float threshold)
            {
                // Mark prunable branches from start tree root
                for (const auto child_idx : children[start_index])
                {
                    if (not pruned[child_idx] and can_prune_branch(can_prune_branch, child_idx, threshold))
                    {
                        mark_pruned(mark_pruned, child_idx);
                    }
                }

                // Mark prunable branches from each goal tree root
                for (const auto goal_idx : goal_indices)
                {
                    for (const auto child_idx : children[goal_idx])
                    {
                        if (not pruned[child_idx] and
                            can_prune_branch(can_prune_branch, child_idx, threshold))
                        {
                            mark_pruned(mark_pruned, child_idx);
                        }
                    }
                }

                // Rebuild both trees without pruned nodes
                rebuild_tree(start_tree, true);
                rebuild_tree(goal_tree, false);
            };

            // PHS for informed sampling (single goal only)
            bool has_solution = false;
            std::unique_ptr<ProlateHyperspheroid<Robot>> phs_ptr;
            std::shared_ptr<ProlateHyperspheroidRNG<Robot>> phs_rng_ptr;
            if (settings.use_phs and goals.size() == 1)
            {
                phs_ptr = std::make_unique<ProlateHyperspheroid<Robot>>(start, goals[0]);
                phs_rng_ptr = std::make_shared<ProlateHyperspheroidRNG<Robot>>(*phs_ptr, rng);
            }

            // Rewiring constants
            // k_rrt > 2^(d+1) * e * (1 + 1/d)
            const double dim_d = static_cast<double>(dimension);
            const double k_rrt_star = settings.rewire_factor * std::pow(2.0, dim_d + 1.0) *
                                      vamp::utils::constants::e * (1.0 + 1.0 / dim_d);

            // r_rrt > (2*(1+1/d))^(1/d) * (measure/ballvolume)^(1/d)
            const double space_measure = Robot::space_measure();
            const double inverse_dim = 1.0 / dim_d;
            const double r_rrt_star =
                settings.rewire_factor *
                std::pow(
                    2.0 * (1.0 + 1.0 / dim_d) * (space_measure / utils::unit_ball_measure(dimension)),
                    inverse_dim);

            // Update descendant costs recursively after rewiring
            const auto update_child_costs =
                [&](const auto &self, std::size_t node_idx, float cost_delta) -> void
            {
                for (const auto child_idx : children[node_idx])
                {
                    costs[child_idx] -= cost_delta;
                    self(self, child_idx, cost_delta);
                }
            };

            // Extract path through a start-side and goal-side connection point
            const auto extract_path = [&](std::size_t start_side_node,
                                          std::size_t goal_side_node) -> Path<Robot>
            {
                Path<Robot> path;

                std::vector<std::size_t> start_chain;
                auto current = start_side_node;
                while (parents[current] != current)
                {
                    start_chain.push_back(current);
                    current = parents[current];
                }
                start_chain.push_back(current);

                for (auto it = start_chain.rbegin(); it != start_chain.rend(); ++it)
                {
                    path.emplace_back(buffer_index(*it));
                }

                current = goal_side_node;
                while (parents[current] != current)
                {
                    auto parent = parents[current];
                    path.emplace_back(buffer_index(parent));
                    current = parent;
                }

                return path;
            };

            // Sample a configuration (uniform or PHS-informed)
            const auto sample = [&]() -> Configuration
            {
                if (has_solution and settings.use_phs and phs_rng_ptr and goals.size() == 1)
                {
                    float u = rng->dist.uniform_01();
                    if (u < settings.greedy_biasing_ratio and greedy_best_cost < best_cost)
                    {
                        phs_rng_ptr->phs.set_transverse_diameter(greedy_best_cost);
                    }
                    else
                    {
                        phs_rng_ptr->phs.set_transverse_diameter(best_cost);
                    }

                    return phs_rng_ptr->next();
                }

                return rng->next();
            };

            // Trees for balanced bidirectional growth
            bool tree_a_is_start = false;
            auto *tree_a = &goal_tree;
            auto *tree_b = &start_tree;

            std::size_t iter = 0;
            std::vector<std::pair<NNNodeType, float>> neighbors;
            neighbors.reserve(settings.max_samples);

            while (iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                // Balanced bidirectional tree swapping
                float asize = static_cast<float>(tree_a->size());
                float bsize = static_cast<float>(tree_b->size());
                float ratio = std::abs(asize - bsize) / std::max(asize, 1.F);

                if ((not settings.balanced) or ratio < settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                // Sample
                auto temp = sample();

                // Cost gating
                if (has_solution)
                {
                    float g_hat = temp.distance(start);
                    float h_hat = std::numeric_limits<float>::max();
                    for (const auto &goal : goals)
                    {
                        h_hat = std::min(h_hat, temp.distance(goal));
                    }
                    float f_hat = g_hat + h_hat;
                    if (f_hat >= best_cost)
                    {
                        continue;
                    }
                }

                // Find nearest in active tree
                if (tree_a->size() == 0)
                {
                    continue;
                }

                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                auto nearest_result = tree_a->nearest(NNFloatArray<dimension>{temp_array.data()});
                if (not nearest_result)
                {
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest_result;

                // Dynamic domain check
                if (settings.dynamic_domain and radii[nearest_node.index] < nearest_distance)
                {
                    continue;
                }

                const auto nearest_configuration = nearest_node.as_vector();
                auto nearest_vector = temp - nearest_configuration;

                bool reach = nearest_distance < settings.range;
                auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (settings.range / nearest_distance);
                float extension_distance = reach ? nearest_distance : settings.range;

                // Validate extension edge
                if (not validate_vector<Robot, rake, resolution>(
                        nearest_configuration, extension_vector, extension_distance, environment))
                {
                    if (settings.dynamic_domain)
                    {
                        if (radii[nearest_node.index] == std::numeric_limits<float>::max())
                        {
                            radii[nearest_node.index] = settings.dd_radius;
                        }
                        else
                        {
                            radii[nearest_node.index] = std::max(
                                radii[nearest_node.index] * (1.F - settings.dd_alpha),
                                settings.dd_min_radius);
                        }
                    }
                    continue;
                }

                auto new_configuration = nearest_configuration + extension_vector;
                float new_cost = costs[nearest_node.index] + extension_distance;
                std::size_t best_parent = nearest_node.index;

                // Write config to buffer once (before both parent selection and node addition)
                float *new_config_ptr = buffer_index(free_index);
                new_configuration.to_array(new_config_ptr);

                // RRT* parent selection and rewiring (skip until first solution if delay_rewiring)
                bool do_rewire = not settings.delay_rewiring or has_solution;

                if (do_rewire)
                {
                    // Combined tree cardinality
                    // cardDbl = tStart_->size() + tGoal_->size() + 1
                    const auto card = static_cast<double>(start_tree.size() + goal_tree.size() + 1);

                    NNFloatArray<dimension> new_key{new_config_ptr};

                    // Query neighbors using nigh KD-tree (returns sorted by distance)
                    neighbors.clear();
                    if (settings.use_k_nearest)
                    {
                        std::size_t k = std::min(
                            static_cast<std::size_t>(std::ceil(k_rrt_star * std::log(card))),
                            settings.max_k_neighbors);
                        tree_a->nearest(neighbors, new_key, k);
                    }
                    else
                    {
                        float r = static_cast<float>(std::min(
                            static_cast<double>(settings.range),
                            r_rrt_star * std::pow(std::log(card) / card, inverse_dim)));
                        tree_a->nearest(neighbors, new_key, std::numeric_limits<std::size_t>::max(), r);
                    }

                    if (settings.delay_cc)
                    {
                        pdqsort(
                            neighbors.begin(),
                            neighbors.end(),
                            [&](const auto &a, const auto &b)
                            { return costs[a.first.index] + a.second < costs[b.first.index] + b.second; });

                        for (const auto &[nbr_node, nbr_dist] : neighbors)
                        {
                            float candidate_cost = costs[nbr_node.index] + nbr_dist;
                            if (candidate_cost >= new_cost)
                            {
                                break;
                            }

                            if (settings.use_k_nearest or nbr_dist < settings.range)
                            {
                                const auto nbr_config = nbr_node.as_vector();
                                if (validate_motion<Robot, rake, resolution>(
                                        nbr_config, new_configuration, environment))
                                {
                                    new_cost = candidate_cost;
                                    best_parent = nbr_node.index;
                                    break;
                                }
                            }
                        }
                    }
                    else
                    {
                        for (const auto &[nbr_node, nbr_dist] : neighbors)
                        {
                            float candidate_cost = costs[nbr_node.index] + nbr_dist;
                            if (candidate_cost < new_cost)
                            {
                                const auto nbr_config = nbr_node.as_vector();
                                if (validate_motion<Robot, rake, resolution>(
                                        nbr_config, new_configuration, environment))
                                {
                                    new_cost = candidate_cost;
                                    best_parent = nbr_node.index;
                                }
                            }
                        }
                    }
                }

                // Tight cost gating: use actual tree cost (after parent selection) + heuristic.
                // Placed after parent selection so new_cost reflects the best parent, not just nearest.
                if (has_solution)
                {
                    float h_hat;
                    if (tree_a_is_start)
                    {
                        h_hat = std::numeric_limits<float>::max();
                        for (const auto &g : goals)
                        {
                            h_hat = std::min(h_hat, new_configuration.distance(g));
                        }
                    }
                    else
                    {
                        h_hat = new_configuration.distance(start);
                    }

                    float tight_f = new_cost + h_hat;
                    if (tight_f >= best_cost)
                    {
                        continue;
                    }
                }

                // Add new node (config already written to buffer)
                tree_a->insert(NNNodeType{free_index, {new_config_ptr}});

                parents[free_index] = best_parent;
                costs[free_index] = new_cost;
                radii[free_index] = std::numeric_limits<float>::max();
                in_start_tree[free_index] = tree_a_is_start ? 1 : 0;
                children[best_parent].push_back(free_index);

                const std::size_t new_node_index = free_index;
                free_index++;

                // Dynamic domain grow
                if (settings.dynamic_domain and
                    radii[nearest_node.index] != std::numeric_limits<float>::max())
                {
                    radii[nearest_node.index] *= (1 + settings.dd_alpha);
                }

                // Rewiring
                if (do_rewire)
                {
                    for (const auto &[nbr_node, nbr_dist] : neighbors)
                    {
                        if (nbr_node.index == best_parent)
                        {
                            continue;
                        }

                        float nbr_new_cost = new_cost + nbr_dist;
                        if (nbr_new_cost < costs[nbr_node.index])
                        {
                            bool motion_valid;
                            if (settings.use_k_nearest or nbr_dist < settings.range)
                            {
                                motion_valid = validate_motion<Robot, rake, resolution>(
                                    new_configuration, nbr_node.as_vector(), environment);
                            }
                            else
                            {
                                motion_valid = false;
                            }

                            if (motion_valid)
                            {
                                float cost_delta = costs[nbr_node.index] - nbr_new_cost;

                                auto &old_parent_children = children[parents[nbr_node.index]];
                                auto it = std::find(
                                    old_parent_children.begin(), old_parent_children.end(), nbr_node.index);
                                if (it != old_parent_children.end())
                                {
                                    *it = old_parent_children.back();
                                    old_parent_children.pop_back();
                                }

                                parents[nbr_node.index] = new_node_index;
                                costs[nbr_node.index] = nbr_new_cost;
                                children[new_node_index].push_back(nbr_node.index);

                                update_child_costs(update_child_costs, nbr_node.index, cost_delta);
                            }
                        }
                    }
                }

                // Connect to other tree
                if (tree_b->size() == 0)
                {
                    continue;
                }

                auto other_result = tree_b->nearest(NNFloatArray<dimension>{new_config_ptr});
                if (not other_result)
                {
                    continue;
                }

                const auto &[other_nearest_node, other_nearest_distance] = *other_result;

                // Check if connection would improve best solution
                float connection_cost = new_cost + other_nearest_distance + costs[other_nearest_node.index];
                if (has_solution and connection_cost >= best_cost)
                {
                    continue;
                }

                const auto other_nearest_configuration = other_nearest_node.as_vector();
                auto other_nearest_vector = other_nearest_configuration - new_configuration;

                // Extend incrementally toward other tree
                const std::size_t n_extensions = std::max(
                    static_cast<std::size_t>(std::ceil(other_nearest_distance / settings.range)),
                    static_cast<std::size_t>(1));
                const float increment_length = other_nearest_distance / static_cast<float>(n_extensions);
                auto increment = other_nearest_vector * (1.0F / static_cast<float>(n_extensions));

                std::size_t i_extension = 0;
                auto prior = new_configuration;
                for (; i_extension < n_extensions and
                       validate_vector<Robot, rake, resolution>(
                           prior, increment, increment_length, environment) and
                       free_index < settings.max_samples;
                     ++i_extension)
                {
                    auto next = prior + increment;
                    float *next_ptr = buffer_index(free_index);
                    next.to_array(next_ptr);
                    tree_a->insert(NNNodeType{free_index, {next_ptr}});
                    parents[free_index] = free_index - 1;
                    costs[free_index] = costs[free_index - 1] + increment_length;
                    radii[free_index] = std::numeric_limits<float>::max();
                    in_start_tree[free_index] = tree_a_is_start ? 1 : 0;
                    children[free_index - 1].push_back(free_index);

                    free_index++;
                    prior = next;
                }

                if (i_extension == n_extensions)  // connected
                {
                    std::size_t connect_a_node = free_index - 1;
                    std::size_t connect_b_node = other_nearest_node.index;

                    float total_cost = costs[connect_a_node] + costs[connect_b_node];

                    if (total_cost < best_cost)
                    {
                        std::size_t start_side_node, goal_side_node;
                        if (tree_a_is_start)
                        {
                            start_side_node = connect_a_node;
                            goal_side_node = connect_b_node;
                        }
                        else
                        {
                            start_side_node = connect_b_node;
                            goal_side_node = connect_a_node;
                        }

                        auto candidate_path = extract_path(start_side_node, goal_side_node);

                        if (not candidate_path.empty())
                        {
                            float path_cost = candidate_path.cost();
                            if (path_cost < best_cost)
                            {
                                bool should_prune = false;

                                if (settings.tree_pruning and has_solution)
                                {
                                    float improvement = (best_cost - path_cost) / best_cost;
                                    if (improvement > settings.prune_threshold)
                                    {
                                        should_prune = true;
                                    }
                                }

                                best_cost = path_cost;
                                best_path = std::move(candidate_path);
                                has_solution = true;

                                if (not settings.optimize)
                                {
                                    break;
                                }

                                // Compute greedy best cost (max f^ along solution path)
                                if (settings.use_phs and phs_ptr and goals.size() == 1)
                                {
                                    greedy_best_cost = 0.F;
                                    for (std::size_t i = 0; i < best_path.size(); ++i)
                                    {
                                        float g_hat = best_path[i].distance(start);
                                        float h_hat = best_path[i].distance(goals[0]);
                                        float f_hat = g_hat + h_hat;
                                        greedy_best_cost = std::max(greedy_best_cost, f_hat);
                                    }
                                }

                                if (should_prune)
                                {
                                    float prune_cost_threshold =
                                        (settings.greedy_biasing_ratio > 0.5F) ? greedy_best_cost : best_cost;
                                    prune_trees(prune_cost_threshold);
                                }
                            }
                        }
                    }
                }
            }

            if (not best_path.empty())
            {
                result.path = std::move(best_path);
                result.cost = best_cost;
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            result.size.emplace_back(goal_tree.size());
            return result;
        }
    };
}  // namespace vamp::planning
