#pragma once

#include <vamp/planning/kdtree.hh>

namespace vamp::planning
{
    template <typename Robot>
    using NN = KDTree<Robot>;
}  // namespace vamp::planning
