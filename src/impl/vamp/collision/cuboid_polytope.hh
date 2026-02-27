#pragma once

#include <limits>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/cuboid_cuboid.hh>

namespace vamp::collision
{

    template <typename DataT>
    inline auto cuboid_polytope(const Cuboid<DataT> &a, const ConvexPolytope<DataT> &b) noexcept -> DataT
    {
        // Broadphase: check collision between cuboid and the polytope's cuboid bounding box
        auto obb_dist = cuboid_cuboid(a, b.obb);
        if (obb_dist.test_zero())
        {
            return obb_dist;
        }

        // Narrowphase: test each halfspace of the H-representation of polytope B.
        DataT max_sep = DataT::fill(-std::numeric_limits<float>::max());

        for (auto i = 0U; i < b.num_planes; ++i)
        {
            const auto center_proj = dot_3(b.nx[i], b.ny[i], b.nz[i], a.x, a.y, a.z);
            const auto extent_proj =
                dot_3(b.nx[i], b.ny[i], b.nz[i], a.axis_1_x, a.axis_1_y, a.axis_1_z).abs() * a.axis_1_r +
                dot_3(b.nx[i], b.ny[i], b.nz[i], a.axis_2_x, a.axis_2_y, a.axis_2_z).abs() * a.axis_2_r +
                dot_3(b.nx[i], b.ny[i], b.nz[i], a.axis_3_x, a.axis_3_y, a.axis_3_z).abs() * a.axis_3_r;

            const auto sep = (center_proj - extent_proj) - b.d[i];
            max_sep = max_sep.max(sep);

            if (max_sep.test_zero())
            {
                break;
            }
        }

        return max_sep;
    }
}  // namespace vamp::collision
