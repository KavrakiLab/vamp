#pragma once

#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

namespace vamp::planning
{
    struct AORRTCSettings
    {
        RRTCSettings rrtc;
        SimplifySettings simplify;

        bool optimize = true;
        bool cost_bound_resample = true;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;

        // nanoseconds
        std::size_t max_time = 10000000000;
    };

}  // namespace vamp::planning
