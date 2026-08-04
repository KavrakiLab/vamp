#pragma once

namespace vamp::planning
{
    struct GRRTStarSettings
    {
        float range = 2.;
        float rewire_factor = 1.1;
        float greedy_biasing_ratio = 0.9;
        bool use_k_nearest = true;
        std::size_t max_k_neighbors = 512;
        bool delay_cc = true;
        bool balanced = true;
        float tree_ratio = 1.;
        bool use_phs = true;
        bool delay_rewiring = true;
        bool dynamic_domain = true;
        float dd_radius = 4.;
        float dd_alpha = 0.0001;
        float dd_min_radius = 1.;
        bool tree_pruning = true;
        float prune_threshold = 0.05;
        bool optimize = true;
        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
    };
}  // namespace vamp::planning
