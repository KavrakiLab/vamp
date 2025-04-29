#pragma once

namespace vamp::planning
{
    struct RRTCSettings
    {
        float range = 2.;

        bool dynamic_domain = true;
        float radius = 4.;
        float alpha = 0.0001;
        float min_radius = 1.;

        bool balance = true;
        float tree_ratio = 1.;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
        bool start_tree_first = true;

        // time in nanoseconds (10s)
        std::size_t max_time = 10000000000;
    };
}  // namespace vamp::planning
