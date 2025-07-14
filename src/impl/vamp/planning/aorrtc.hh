#pragma once

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

        std::unique_ptr<float, decltype(&free)> buffer;
        std::vector<std::size_t> parents;
        std::vector<float> radii;
        std::vector<float> costs;
        std::vector<GNATNode<dimension>> near_list;
        NearestNeighborsGNAT<GNATNode<dimension>> start_tree;
        NearestNeighborsGNAT<GNATNode<dimension>> goal_tree;

        inline auto buffer_index(std::size_t index) -> float *
        {
            return buffer.get() + index * Configuration::num_scalars_rounded;
        };

        AOX_RRTC(std::size_t max_samples)
          : buffer(
                std::unique_ptr<float, decltype(&free)>(
                    vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                        max_samples * Configuration::num_scalars_rounded),
                    &free))
          , start_tree(max_samples)
          , goal_tree(max_samples)
        {
            parents.resize(max_samples);
            radii.resize(max_samples);
            costs.resize(max_samples);
            start_tree.setDistanceFunction(aox_dist_fun);
            goal_tree.setDistanceFunction(aox_dist_fun);
        }

        inline auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings,
            const float max_cost,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, max_cost, rng);
        }

        inline static auto standard_dist_fun(const GNATNode<dimension> &a, const GNATNode<dimension> &b)
            -> float
        {
            return a.array.distance(b.array);
        }

        inline static auto aox_dist_fun(const GNATNode<dimension> &a, const GNATNode<dimension> &b) -> float
        {
            //               Configuration space distance             Cost space distance
            return std::sqrt(std::pow(a.array.distance(b.array), 2) + std::pow(a.cost - b.cost, 2));
        }

        inline auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings,
            const float max_cost,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            start_tree.clear();
            goal_tree.clear();

            const RRTCSettings &rrtc_settings = settings.rrtc;

            constexpr const std::size_t start_index = 0;

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

            // trees
            bool tree_a_is_start = not rrtc_settings.start_tree_first;
            auto *tree_a = (rrtc_settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (rrtc_settings.start_tree_first) ? &start_tree : &goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            // Add start to tree
            start.to_array(buffer_index(start_index));
            GNATNode<dimension> start_vert = GNATNode<dimension>{start_index, 0, {buffer_index(start_index)}};
            start_tree.add(start_vert);
            parents[start_index] = start_index;
            radii[start_index] = std::numeric_limits<float>::max();
            costs[start_index] = 0;

            // Add goals to tree
            std::vector<GNATNode<dimension>> goal_verts;
            goal_verts.reserve(goals.size());

            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                GNATNode<dimension> temp_goal =
                    GNATNode<dimension>{free_index, 0, {buffer_index(free_index)}};
                goal_verts.emplace_back(temp_goal);
                goal_tree.add(temp_goal);
                parents[free_index] = free_index;
                radii[free_index] = std::numeric_limits<float>::max();
                costs[free_index] = 0;
                free_index++;
            }

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

                auto temp = rng->next();
                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                float cost_sample = rng->dist.uniform_01();

                // Find closest goal for optimistic f^ in case of multi-goals
                GNATNode<dimension> *goal_vert = nullptr;
                auto min_goal_dist = std::numeric_limits<double>::infinity();
                for (GNATNode<dimension> &v : goal_verts)
                {
                    if (temp.distance(v.array) < min_goal_dist)
                    {
                        goal_vert = &v;
                        min_goal_dist = temp.distance(v.array);
                    }
                }

                if (not goal_vert)
                {
                    continue;
                }

                const auto root_vert = tree_a_is_start ? start_vert : *goal_vert;
                const auto target_vert = tree_a_is_start ? *goal_vert : start_vert;

                float g_hat = temp.distance(root_vert.array);
                float h_hat = temp.distance(target_vert.array);
                float f_hat = g_hat + h_hat;

                // The range between the minimum possible cost and maximum allowable cost
                // - Floating point error can result in a (barely) negative range
                // - If c_range is 0, only valid connection is to root of tree (sampled upper cost bound ==
                // g^)
                float c_range = std::max(max_cost - f_hat, 0.0F);

                // Sampled upper cost bound
                float c_rand = (cost_sample * c_range) + g_hat;

                GNATNode<dimension> temp_node{free_index, c_rand, temp_array.data()};
                GNATNode<dimension> *nearest_node_ptr;
                GNATNode<dimension> nearest_node;

                // Get r-disc neighbours, then iterate through list until a valid connection is found
                // Necessary workaround given asymmetric cost function
                //* ------------------ ------ -------------------
                // Only need to check nodes that are closer than the root of the tree, since connecting to the
                // root will always be valid
                auto root_dist = tree_a->getDistanceFunction()(temp_node, root_vert);
                tree_a->nearestR(temp_node, root_dist, near_list);
                std::size_t idx = 0;
                nearest_node_ptr = &near_list[idx];
                auto nearest_distance = temp.distance(buffer_index(nearest_node_ptr->index));
                // Loop over nearest nodes until one is found that satisfies asymmetric distance function
                // i.e., look for nearest neighbour in cost-augmented space that doesn't violate the sampled
                // upper cost bound
                while (nearest_node_ptr->cost > 0 and                          //
                       c_rand < nearest_node_ptr->cost + nearest_distance and  //
                       idx < near_list.size())
                {
                    idx++;
                    nearest_node_ptr = &near_list[idx];
                    nearest_distance = temp.distance(buffer_index(nearest_node_ptr->index));
                }
                // =============================================*/

                const auto nearest_radius = radii[nearest_node_ptr->index];
                if (rrtc_settings.dynamic_domain and nearest_radius < nearest_distance)
                {
                    continue;
                }

                auto nearest_configuration = nearest_node_ptr->array;
                auto nearest_vector = temp - nearest_configuration;

                bool reach = nearest_distance < rrtc_settings.range;
                auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (rrtc_settings.range / nearest_distance);

                // Evaluate edge reaching towards sample
                if (validate_vector<Robot, rake, resolution>(
                        nearest_configuration,
                        extension_vector,
                        (reach) ? nearest_distance : rrtc_settings.range,
                        environment))
                {
                    float *new_configuration_index = buffer_index(free_index);
                    const auto new_configuration = nearest_configuration + extension_vector;
                    new_configuration.to_array(new_configuration_index);

                    // Calculate and store actual node cost
                    const auto dist = new_configuration.distance(buffer_index(nearest_node_ptr->index));
                    auto new_cost = nearest_node_ptr->cost + dist;

                    nearest_node = *nearest_node_ptr;

                    // If resampling costs to try and find a better parent...
                    if (settings.cost_bound_resample)
                    {
                        g_hat = new_configuration.distance(root_vert.array);

                        GNATNode<dimension> *new_nearest_node;

                        temp_node.index = free_index;
                        temp_node.array = {new_configuration};

                        // Continuously resample cost until an invalid connection is found
                        while (true)
                        {
                            cost_sample = rng->dist.uniform_01();
                            c_range = std::max(new_cost - g_hat, 0.0F);
                            c_rand = (cost_sample * c_range) + g_hat;
                            temp_node.cost = c_rand;

                            //* ------------------ ------ -------------------
                            root_dist = tree_a->getDistanceFunction()(temp_node, root_vert);
                            tree_a->nearestR(temp_node, root_dist, near_list);
                            idx = 0;
                            new_nearest_node = &near_list[idx];
                            auto new_nearest_distance =
                                new_configuration.distance(buffer_index(new_nearest_node->index));
                            while (new_nearest_node->cost > 0 and
                                   c_rand < new_nearest_node->cost + new_nearest_distance)
                            {
                                idx++;
                                new_nearest_node = &near_list[idx];
                                new_nearest_distance =
                                    new_configuration.distance(buffer_index(new_nearest_node->index));
                            }
                            // =============================================*/

                            const auto new_nearest_configuration = new_nearest_node->array;
                            const auto new_nearest_vector = new_configuration - new_nearest_configuration;

                            // If we have connected:
                            //      to the same parent
                            //      with a worse cost
                            //      to the best possible parent before this (crange == 0 \equiv cost == g^)
                            // ...then stop spending effort resampling costs
                            if (new_nearest_node->index == nearest_node.index or
                                new_nearest_node->cost + new_nearest_distance >= new_cost or c_range == 0)
                            {
                                break;
                            }

                            // Validate edge to newly found parent
                            else if (validate_vector<Robot, rake, resolution>(
                                         new_nearest_configuration,
                                         new_nearest_vector,
                                         new_nearest_distance,
                                         environment))
                            {
                                // Congratulations to the new parent
                                nearest_node = *new_nearest_node;
                                new_cost = new_nearest_node->cost + new_nearest_distance;
                            }

                            // The edge is invalid, we have failed a connection. Stop resampling!
                            else
                            {
                                break;
                            }
                        }
                    }

                    // Add the new vertex with the appropriate cost to the tree
                    tree_a->add(GNATNode<dimension>{free_index, new_cost, {new_configuration}});

                    parents[free_index] = nearest_node.index;
                    radii[free_index] = std::numeric_limits<float>::max();
                    costs[free_index] = new_cost;

                    free_index++;

                    if (rrtc_settings.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + rrtc_settings.alpha);
                    }

                    // Extend to goal tree
                    GNATNode<dimension> *other_nearest_node;
                    temp_node.index = free_index - 1;

                    // Because we are extending to the other tree, we need to change our upper cost bound
                    // We need to find a connection that improves upon our current best solution cost

                    // The cost from the root of the other tree to our new vertex, + our new vertex's cost
                    // through the current tree, must be lesser than our maximum path cost

                    // Therefore, our maximum allowable cost for a connection through the other tree is
                    // max_cost - vertex_cost
                    temp_node.cost = max_cost - new_cost;
                    temp_node.array = {new_configuration};

                    // Once again we must loop over the nearest R neighbours - this time for the other tree
                    //* ------------------ ------ -------------------
                    root_dist = tree_b->getDistanceFunction()(temp_node, target_vert);
                    tree_b->nearestR(temp_node, root_dist, near_list);
                    idx = 0;
                    other_nearest_node = &near_list[idx];
                    auto other_nearest_distance =
                        new_configuration.distance(buffer_index(other_nearest_node->index));
                    // Loop until we try connecting:
                    //      to the root of the other tree (which will have cost=0), since this will always be
                    //      valid w.r.t. cost with a cost that would satisfy our upper cost bound
                    while (other_nearest_node->cost > 0 and
                           temp_node.cost < other_nearest_node->cost + other_nearest_distance and
                           idx < near_list.size())
                    {
                        idx++;
                        other_nearest_node = &near_list[idx];
                        other_nearest_distance =
                            new_configuration.distance(buffer_index(other_nearest_node->index));
                    }
                    // =============================================*/

                    const auto other_nearest_configuration = other_nearest_node->array;
                    const auto other_nearest_vector = other_nearest_configuration - new_configuration;

                    // Just to be safe, make sure we've improved upon our best solution
                    if (new_cost + other_nearest_distance + other_nearest_node->cost >= max_cost)
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
                        auto next = prior + increment;
                        float *next_index = buffer_index(free_index);
                        next.to_array(next_index);

                        tree_a->add(
                            GNATNode<dimension>{
                                free_index, increment_length + costs[free_index - 1], {next_index}});
                        parents[free_index] = free_index - 1;
                        radii[free_index] = std::numeric_limits<float>::max();
                        costs[free_index] = increment_length + costs[free_index - 1];

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
                        current = other_nearest_node->index;

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
                    if (nearest_radius == std::numeric_limits<float>::max())
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

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            result.size.emplace_back(goal_tree.size());
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
            const AORRTCSettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();

            // Update the settings for internal searches
            RRTCSettings rrtc_settings = settings.rrtc;
            rrtc_settings.max_iterations = settings.max_iterations;
            rrtc_settings.max_samples = settings.max_samples;

            PlanningResult<Robot> result;
            float best_path_cost = std::numeric_limits<float>::infinity();
            std::size_t iters = 0;

            do
            {
                // Find an initial solution
                result = RRTC::solve(start, goals, environment, rrtc_settings, rng);
                iters += result.iterations;
            } while (result.path.empty() and iters < settings.max_iterations);

            // Simplify solution
            if (not result.path.empty())
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

            ProlateHyperspheroid<Robot> phs(start, goals[0]);
            phs.set_transverse_diameter(best_path_cost);
            auto phs_rng = std::make_shared<ProlateHyperspheroidRNG<Robot>>(phs, rng);

            AOX_RRTC instance(rrtc_settings.max_samples);

            while (iters < settings.max_iterations)
            {
                // Update internal maximum iterations
                rrtc_settings.max_iterations = settings.max_iterations - iters;

                // If there is a single goal, sample with PHS
                if (goals.size() == 1)
                {
                    result = instance.solve(start, goals, environment, settings, best_path_cost, phs_rng);
                }
                else
                {
                    result = instance.solve(start, goals, environment, settings, best_path_cost, rng);
                }

                iters += result.iterations;

                // If last search found a solution
                if (not result.path.empty())
                {
                    // Simplify
                    result =
                        simplify<Robot, rake, resolution>(result.path, environment, settings.simplify, rng);

                    // To be safe, ensure new path is actually a better solution
                    if (result.path.cost() < best_path_cost)
                    {
                        // Update best solution
                        final_result.path = result.path;
                        best_path_cost = result.path.cost();
                    }
                }

                phs_rng->phs.set_transverse_diameter(best_path_cost);
            }

            final_result.iterations = iters;
            final_result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

            return final_result;
        }
    };
}  // namespace vamp::planning
