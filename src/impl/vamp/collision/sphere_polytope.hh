#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/gjk.hh>
#include <vamp/collision/sphere_cuboid.hh>

namespace vamp::collision
{
    // Test a block of spheres against a convex polytope. The polytope's oriented bounding box acts as a
    // SIMD broadphase, and each sphere that survives it is checked exactly with a scalar GJK distance
    // query against the polytope's vertices, seeded from the sphere's closest point on the bounding box.
    //
    // The result has a negative lane if and only if some sphere collides with the polytope.
    // As soon as one colliding sphere is found, the remaining spheres are skipped and keep their broadphase
    // value, so the result tells whether any sphere collides, but not necessarily which ones.
    template <typename DataT>
    inline auto sphere_polytope(
        const ConvexPolytope<DataT> &p,
        const DataT &cx,
        const DataT &cy,
        const DataT &cz,
        const DataT &r) noexcept -> DataT
    {
        const auto rsq = r * r;
        const auto obb_dist = sphere_cuboid(p.obb, cx, cy, cz, rsq);

        if (obb_dist.test_zero())
        {
            return obb_dist;
        }

        // Closest point on the bounding box to each sphere center, used to seed the GJK search direction
        const auto &b = p.obb;
        const auto xs = cx - b.x;
        const auto ys = cy - b.y;
        const auto zs = cz - b.z;
        const auto t1 = clamp(dot_3(b.axis_1_x, b.axis_1_y, b.axis_1_z, xs, ys, zs), -b.axis_1_r, b.axis_1_r);
        const auto t2 = clamp(dot_3(b.axis_2_x, b.axis_2_y, b.axis_2_z, xs, ys, zs), -b.axis_2_r, b.axis_2_r);
        const auto t3 = clamp(dot_3(b.axis_3_x, b.axis_3_y, b.axis_3_z, xs, ys, zs), -b.axis_3_r, b.axis_3_r);
        const auto seed_x = b.x + b.axis_1_x * t1 + b.axis_2_x * t2 + b.axis_3_x * t3;
        const auto seed_y = b.y + b.axis_1_y * t1 + b.axis_2_y * t2 + b.axis_3_y * t3;
        const auto seed_z = b.z + b.axis_1_z * t1 + b.axis_2_z * t2 + b.axis_3_z * t3;

        auto result = obb_dist.to_array();
        const auto x_arr = cx.to_array();
        const auto y_arr = cy.to_array();
        const auto z_arr = cz.to_array();
        const auto rsq_arr = rsq.to_array();
        const auto sx_arr = seed_x.to_array();
        const auto sy_arr = seed_y.to_array();
        const auto sz_arr = seed_z.to_array();

        // do each lane sequentially.
        // I tried many times to figure out a good block-parallel way to do this, but it was always faster to
        // do simple sequential GJK.
        for (auto lane = 0U; lane < result.size(); ++lane)
        {
            if (not(result[lane] < 0.F))
            {
                continue;
            }

            const float r_sq = rsq_arr[lane];
            const float dist_sq = gjk::sql2(
                p.vx.data(),
                p.vy.data(),
                p.vz.data(),
                p.num_vertices,
                Eigen::Vector3f(x_arr[lane], y_arr[lane], z_arr[lane]),
                Eigen::Vector3f(sx_arr[lane], sy_arr[lane], sz_arr[lane]),
                r_sq);

            result[lane] = dist_sq - r_sq;
            if (result[lane] < 0.F)
            {
                break;
            }
        }

        return DataT(result.data(), false);
    }

    template <typename DataT>
    inline auto sphere_polytope(const ConvexPolytope<DataT> &p, const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_polytope(p, s.x, s.y, s.z, s.r);
    }
}  // namespace vamp::collision
