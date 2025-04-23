#pragma once

namespace vamp::planning
{
    struct RRT_star_settings
    {
        float range = 2.;
        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
        std::size_t rng_skip_iterations = 0;
        float rewire_factor = 1.0;
        bool force_max_iters = false;
    };
}