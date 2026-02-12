#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

#include <limits>

namespace vamp::collision
{
    // Sphere-polytope collision: returns negative if collision
    // Algorithm: find max signed distance across all planes
    // If max > 0, separating plane exists (no collision)
    template <typename DataT>
    inline auto sphere_polytope(
        const ConvexPolytope<DataT> &p,
        const DataT &cx,
        const DataT &cy,
        const DataT &cz,
        const DataT &r) noexcept -> DataT
    {
        DataT max_dist = DataT::fill(-std::numeric_limits<float>::max());

        for (auto i = 0U; i < p.num_planes; ++i)
        {
            auto nx_i = DataT::fill(p.nx[i]);
            auto ny_i = DataT::fill(p.ny[i]);
            auto nz_i = DataT::fill(p.nz[i]);
            auto d_i = DataT::fill(p.d[i]);

            // dist = n.C - d - r (positive = separated)
            auto dist = dot_3(nx_i, ny_i, nz_i, cx, cy, cz) - d_i - r;
            max_dist = max_dist.max(dist);

            // Early out: if all spheres are already separated, skip remaining planes
            if (max_dist.test_zero())
            {
                break;
            }
        }

        // Convention: negative = collision, non-negative = no collision
        // max_dist > 0 means separating plane exists (no collision)
        // max_dist <= 0 means all planes satisfied (collision)
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
