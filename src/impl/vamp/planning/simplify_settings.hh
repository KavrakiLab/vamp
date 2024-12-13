#pragma once

#include <vector>

namespace vamp::planning
{
    enum SimplifyRoutine
    {
        BSPLINE,
        REDUCE,
        SHORTCUT,
        PERTURB,
    };

    struct BSplineSettings
    {
        std::size_t max_steps{1};
        float min_change{0.1};
        float midpoint_interpolation{0.5};
    };

    struct ReduceSettings
    {
        std::size_t max_steps{10};
        std::size_t max_empty_steps{5};
        float range_ratio{1. / 2.};
    };

    struct ShortcutSettings
    {
    };

    struct PerturbSettings
    {
        std::size_t max_steps{10};
        std::size_t max_empty_steps{5};
        std::size_t perturbation_attempts{5};
        float range{0.1};
    };

    struct SimplifySettings
    {
        std::size_t max_iterations{5};
        std::size_t interpolate{0};
        std::vector<SimplifyRoutine> operations{{SHORTCUT, BSPLINE}};

        ReduceSettings reduce;
        ShortcutSettings shortcut;
        BSplineSettings bspline;
        PerturbSettings perturb;
    };
}  // namespace vamp::planning
