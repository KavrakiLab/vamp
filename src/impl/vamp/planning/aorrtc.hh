#pragma once

#include <limits>
#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/aox_nn.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/aorrtc_settings.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AOX_RRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        using NNNode = GNATNode<dimension>;
        using NN = NearestNeighborsGNAT<NNNode>;

        std::unique_ptr<float, decltype(&free)> buffer;
        std::vector<std::size_t> parents;
        std::vector<float> radii;
        std::vector<float> costs;

        inline auto buffer_index(std::size_t index) -> float *
        {
            return buffer.get() + index * Configuration::num_scalars_rounded;
        };

        inline auto
        add_to_tree(NN *nn, const Configuration &c, std::size_t index, std::size_t parent_index, float cost)
            -> NNNode
        {
            c.to_array(buffer_index(index));

            radii[index] = std::numeric_limits<float>::max();
            parents[index] = parent_index;
            costs[index] = cost;

            auto node = NNNode{index, cost, c};
            nn->add(node);

            return node;
        };

        // Get r-disc neighbours, then iterate through list until a valid connection is found
        // Necessary workaround given asymmetric cost function
        //* ------------------ ------ -------------------
        // Only need to check nodes that are closer than the root of the tree, since connecting to the
        // root will always be valid
        inline auto find_nearest(NN *nn, const NNNode &root, const Configuration &c, float cost)
            -> std::pair<NNNode, float>
        {
            std::vector<NNNode> near_list;

            // Almost always just pulls in the entire graph, but good to be principled.
            near_list.reserve(nn->size());

            auto temp_node = NNNode{0, cost, c};
            nn->nearestR(temp_node, NNNode::distance(temp_node, root), near_list);

            const auto *new_nearest_node = &near_list[0];
            float new_nearest_distance = c.distance(new_nearest_node->array);

            for (auto idx = 1U; new_nearest_node->cost > 0                                //
                                and cost < new_nearest_node->cost + new_nearest_distance  //
                                and idx < near_list.size();
                 ++idx)
            {
                new_nearest_node = &near_list[idx];
                new_nearest_distance = c.distance(new_nearest_node->array);
            }

            return {*new_nearest_node, new_nearest_distance};
        }

        AOX_RRTC(std::size_t max_samples)
          : buffer(
                std::unique_ptr<float, decltype(&free)>(
                    vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                        max_samples * Configuration::num_scalars_rounded),
                    &free))
        {
            parents.resize(max_samples);
            radii.resize(max_samples);
            costs.resize(max_samples);
        }

        inline auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings,
            const float max_cost,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            static constexpr std::size_t start_index = 0;
            const RRTCSettings &rrtc_settings = settings.rrtc;
            PlanningResult<Robot> result;

            NN start_tree;
            NN goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            auto start_vert = add_to_tree(&start_tree, start, start_index, start_index, 0);

            // Add goals to tree
            std::vector<NNNode> goal_verts;
            goal_verts.reserve(goals.size());

            for (const auto &goal : goals)
            {
                goal_verts.emplace_back(add_to_tree(&goal_tree, goal, free_index, free_index, 0));
                free_index++;
            }

            // trees
            bool tree_a_is_start = not rrtc_settings.start_tree_first;
            auto *tree_a = (rrtc_settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (rrtc_settings.start_tree_first) ? &start_tree : &goal_tree;

            // Search loop
            while (iter++ < rrtc_settings.max_iterations and free_index < rrtc_settings.max_samples)
            {
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                // Balanced RRTC
                if ((not rrtc_settings.balance) or ratio < rrtc_settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                const auto temp = rng->next();

                NNNode goal_vert = *std::min_element(
                    goal_verts.begin(),
                    goal_verts.end(),
                    [&temp](const auto &a, const auto &b)
                    { return temp.distance(a.array) < temp.distance(b.array); });

                const auto &root_vert = tree_a_is_start ? start_vert : goal_vert;
                const auto &target_vert = tree_a_is_start ? goal_vert : start_vert;

                const float g_hat = temp.distance(root_vert.array);
                const float h_hat = temp.distance(target_vert.array);
                const float f_hat = g_hat + h_hat;

                // The range between the minimum possible cost and maximum allowable cost
                // - Floating point error can result in a (barely) negative range
                // - If c_range is 0, only valid connection is to root of tree
                //   (sampled upper cost bound == g^)
                const float c_range = std::max(max_cost - f_hat, 0.0F);

                // Sampled upper cost bound
                const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                // Find nearest with asymmetric cost function
                auto [nearest_node, nearest_distance] = find_nearest(tree_a, root_vert, temp, c_rand);
                if (rrtc_settings.dynamic_domain and radii[nearest_node.index] < nearest_distance)
                {
                    continue;
                }

                const auto nearest_vector = temp - nearest_node.array;
                bool reach = nearest_distance < rrtc_settings.range;
                const auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (rrtc_settings.range / nearest_distance);

                // Evaluate edge reaching towards sample
                if (validate_vector<Robot, rake, resolution>(
                        nearest_node.array,
                        extension_vector,
                        (reach) ? nearest_distance : rrtc_settings.range,
                        environment))
                {
                    const auto new_configuration = nearest_node.array + extension_vector;

                    // Calculate and store actual node cost
                    auto new_cost = nearest_node.cost + new_configuration.distance(nearest_node.array);

                    // If resampling costs to try and find a better parent...
                    if (settings.cost_bound_resample)
                    {
                        const float g_hat = new_configuration.distance(root_vert.array);

                        // Continuously resample cost until an invalid connection is found
                        for (auto i = 0U; i < settings.max_cost_bound_resamples; ++i)
                        {
                            const float c_range = std::max(new_cost - g_hat, 0.0F);
                            const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                            auto [new_nearest_node, new_nearest_distance] =
                                find_nearest(tree_a, root_vert, new_configuration, c_rand);

                            // If we have connected:
                            //      to the same parent
                            //      with a worse cost
                            //      to the best possible parent before this (crange == 0 \equiv cost == g^)
                            // ...then stop spending effort resampling costs
                            if (new_nearest_node.index == nearest_node.index or
                                new_nearest_node.cost + new_nearest_distance >= new_cost or c_range == 0)
                            {
                                break;
                            }
                            // Validate edge to newly found parent
                            else if (validate_vector<Robot, rake, resolution>(
                                         new_nearest_node.array,
                                         new_configuration - new_nearest_node.array,
                                         new_nearest_distance,
                                         environment))
                            {
                                // Congratulations to the new parent
                                nearest_node = new_nearest_node;
                                new_cost = new_nearest_node.cost + new_nearest_distance;
                            }
                            // The edge is invalid, we have failed a connection. Stop resampling!
                            else
                            {
                                break;
                            }
                        }
                    }

                    add_to_tree(tree_a, new_configuration, free_index, nearest_node.index, new_cost);
                    free_index++;

                    if (rrtc_settings.dynamic_domain and
                        radii[nearest_node.index] != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + rrtc_settings.alpha);
                    }

                    // Extend to goal tree

                    // Because we are extending to the other tree, we need to change our upper cost bound
                    // We need to find a connection that improves upon our current best solution cost
                    // The cost from the root of the other tree to our new vertex, + our new vertex's cost
                    // through the current tree, must be lesser than our maximum path cost
                    // Therefore, our maximum allowable cost for a connection through the other tree is
                    // max_cost - vertex_cost
                    const auto [other_nearest_node, other_nearest_distance] =
                        find_nearest(tree_b, target_vert, new_configuration, max_cost - new_cost);
                    const auto other_nearest_vector = other_nearest_node.array - new_configuration;

                    // Just to be safe, make sure we've improved upon our best solution
                    if (new_cost + other_nearest_distance + other_nearest_node.cost >= max_cost)
                    {
                        continue;
                    }

                    // Extend incrementally towards other tree
                    const std::size_t n_extensions = std::ceil(other_nearest_distance / rrtc_settings.range);
                    const float increment_length = other_nearest_distance / static_cast<float>(n_extensions);
                    const auto increment = other_nearest_vector * (1.0F / static_cast<float>(n_extensions));

                    std::size_t i_extension = 0;
                    auto prior = new_configuration;
                    for (; i_extension < n_extensions and
                           validate_vector<Robot, rake, resolution>(
                               prior, increment, increment_length, environment) and
                           free_index < rrtc_settings.max_samples;
                         ++i_extension)
                    {
                        const auto next = prior + increment;
                        add_to_tree(
                            tree_a,
                            next,
                            free_index,
                            free_index - 1,
                            increment_length + costs[free_index - 1]);

                        free_index++;
                        prior = next;
                    }

                    if (i_extension == n_extensions)  // connected
                    {
                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.cost += result.path[result.path.size() - 1].distance(
                                result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        current = other_nearest_node.index;

                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.cost += result.path[result.path.size() - 1].distance(
                                result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        if (not tree_a_is_start)
                        {
                            std::reverse(result.path.begin(), result.path.end());
                        }

                        break;
                    }
                }
                else if (rrtc_settings.dynamic_domain)
                {
                    if (radii[nearest_node.index] == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = rrtc_settings.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] = std::max(
                            radii[nearest_node.index] * (1.F - rrtc_settings.alpha),
                            rrtc_settings.min_radius);
                    }
                }
            }

            result.iterations = iter;
            return result;
        }
    };

    // --------------------------------------------- AOX RRTC Algorithm
    // ---------------------------------------------
    // ==============================================================================================================
    // --------------------------------------------- AOX Meta Algorithm
    // ---------------------------------------------

    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AORRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
        using AOX_RRTC = typename vamp::planning::AOX_RRTC<Robot, rake, resolution>;
        using RRTC = typename vamp::planning::RRTC<Robot, rake, resolution>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings_in,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();

            // Update the settings for internal searches
            AORRTCSettings settings = settings_in;  // make a mutable copy
            const std::size_t &max_samples = settings.max_samples;
            const std::size_t &max_iterations = settings.max_iterations;

            // Configure internal RRTC settings
            RRTCSettings &rrtc_settings = settings.rrtc;
            rrtc_settings.max_iterations = max_iterations;
            rrtc_settings.max_samples = max_samples;

            PlanningResult<Robot> result;
            float best_path_cost = std::numeric_limits<float>::max();
            std::size_t iters = 0;

            do
            {
                // Find an initial solution
                result = RRTC::solve(start, goals, environment, rrtc_settings, rng);
                iters += result.iterations;
            } while (result.path.empty() and iters < settings.max_iterations);

            // Simplify solution if enabled
            if (settings.simplify_intermediate and not result.path.empty())
            {
                result = simplify<Robot, rake, resolution>(result.path, environment, settings.simplify, rng);
            }

            // Exit early if trivial, unsolved, or not optimizing
            if (not settings.optimize or result.path.empty() or result.path.size() == 2)
            {
                return result;
            }

            // We have a new best solution
            PlanningResult<Robot> final_result;
            final_result.path = result.path;
            best_path_cost = result.path.cost();

            float best_possible_cost = std::numeric_limits<float>::max();
            for (const auto &goal : goals)
            {
                best_possible_cost = std::min(best_possible_cost, start.distance(goal));
            }

            ProlateHyperspheroid<Robot> phs(start, goals[0]);
            phs.set_transverse_diameter(best_path_cost);

            auto phs_rng = std::make_shared<ProlateHyperspheroidRNG<Robot>>(phs, rng);

            AOX_RRTC instance(max_samples);

            // If we get close to straight line, just call it.
            // Also handles numerical issues with PHS when too close to straight line...
            while (iters < max_iterations and (best_path_cost - best_possible_cost) > 1e-8)
            {
                // Update internal maximum iterations
                rrtc_settings.max_iterations =
                    std::min(settings.max_iterations - iters, settings.max_internal_iterations);

                // By default, use AORRTC
                if (not settings.anytime)
                {
                    // If there is a single goal, sample with PHS
                    if (settings.use_phs and goals.size() == 1)
                    {
                        result = instance.solve(start, goals, environment, settings, best_path_cost, phs_rng);
                    }
                    else
                    {
                        result = instance.solve(start, goals, environment, settings, best_path_cost, rng);
                    }
                }
                // If anytime, use Anytime RRTC
                else
                {
                    if (settings.use_phs and goals.size() == 1)
                    {
                        result = RRTC::solve(start, goals, environment, rrtc_settings, phs_rng);
                    }
                    else
                    {
                        result = RRTC::solve(start, goals, environment, rrtc_settings, rng);
                    }
                }

                iters += result.iterations;

                // If last search found a solution
                if (not result.path.empty())
                {
                    // Simplify
                    if (settings.simplify_intermediate)
                    {
                        result = simplify<Robot, rake, resolution>(
                            result.path, environment, settings.simplify, rng);
                    }

                    // To be safe, ensure new path is actually a better solution
                    if (result.path.cost() < best_path_cost)
                    {
                        // Update best solution
                        final_result.path = result.path;
                        best_path_cost = result.path.cost();

                        phs_rng->phs.set_transverse_diameter(best_path_cost);
                    }
                }
            }

            final_result.iterations = iters;
            final_result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

            return final_result;
        }
    };
}  // namespace vamp::planning
