#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/sphere_cuboid.hh>

#include <limits>

namespace vamp::collision
{
    template <typename DataT>
    inline auto sphere_polytope(
        const ConvexPolytope<DataT> &p,
        const DataT &cx,
        const DataT &cy,
        const DataT &cz,
        const DataT &r) noexcept -> DataT
    {
        const auto rsq = r * r;
        auto obb_dist = sphere_cuboid(p.obb, cx, cy, cz, rsq);

        if (obb_dist.test_zero())
        {
            return obb_dist;
        }

        DataT max_dist = DataT::fill(-std::numeric_limits<float>::max());

        // TODO: Proper GJK? Seems OK for now.
        for (auto i = 0U; i < p.num_planes; ++i)
        {
            // dist = n.C - d - r (positive = separated)
            auto dist = dot_3(p.nx[i], p.ny[i], p.nz[i], cx, cy, cz) - p.d[i] - r;
            max_dist = max_dist.max(dist);

            if (max_dist.test_zero())
            {
                break;
            }
        }

        return max_dist;
    }

    template <typename DataT>
    inline auto sphere_polytope(
        const ConvexPolytope<DataT> &p,
        const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_polytope(p, s.x, s.y, s.z, s.r);
    }
}  // namespace vamp::collision
