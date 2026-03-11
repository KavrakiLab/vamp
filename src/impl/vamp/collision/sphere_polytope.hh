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

        // iteratively project (cx, cy, cz) onto the polytope and test distance between original centers and
        // projected centers

        DataT lower_bound_dist = DataT::fill(-std::numeric_limits<float>::max());
        DataT cx_proj = cx;
        DataT cy_proj = cy;
        DataT cz_proj = cz;

        for (auto i = 0U; i < p.num_planes; ++i)
        {
            const DataT dot_product = dot_3(p.nx[i], p.ny[i], p.nz[i], cx_proj, cy_proj, cz_proj);
            const DataT n_scale = (dot_product - p.d[i]).max(0.0);

            // project c onto the halfspace by subtracting out the part parallel to n that lies beyond the
            // plane

            cx_proj = cx_proj - n_scale * p.nx[i];
            cy_proj = cy_proj - n_scale * p.ny[i];
            cz_proj = cz_proj - n_scale * p.nz[i];

            lower_bound_dist = sql2_3(cx, cy, cz, cx_proj, cy_proj, cz_proj) - rsq;

            // if projected centers are all sufficiently far away from the original centers, we have proven
            // the sphere is not in collision
            if (lower_bound_dist.test_all_greater_equal(0.0))
            {
                break;
            }
        }

        return lower_bound_dist;
    }

    template <typename DataT>
    inline auto sphere_polytope(const ConvexPolytope<DataT> &p, const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_polytope(p, s.x, s.y, s.z, s.r);
    }
}  // namespace vamp::collision
