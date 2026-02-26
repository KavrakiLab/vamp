#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/sphere_capsule.hh>
#include <vamp/collision/sphere_cuboid.hh>
#include <vamp/collision/sphere_heightfield.hh>
#include <vamp/collision/math.hh>

namespace vamp
{
    template <
        typename VectorDataT,
        typename ArgT1,
        typename ArgT2,
        typename ArgT3,
        typename ArgT4,
        typename ArgT5,
        typename ArgT6,
        typename ArgT7,
        typename ArgT8>
    inline constexpr auto sphere_sphere_self_collision(
        ArgT1 ax_,
        ArgT2 ay_,
        ArgT3 az_,
        ArgT4 ar_,
        ArgT5 bx_,
        ArgT6 by_,
        ArgT7 bz_,
        ArgT8 br_) noexcept -> bool
    {
        auto ax = static_cast<VectorDataT>(ax_);
        auto ay = static_cast<VectorDataT>(ay_);
        auto az = static_cast<VectorDataT>(az_);
        auto ar = static_cast<VectorDataT>(ar_);
        auto bx = static_cast<VectorDataT>(bx_);
        auto by = static_cast<VectorDataT>(by_);
        auto bz = static_cast<VectorDataT>(bz_);
        auto br = static_cast<VectorDataT>(br_);

        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        return not collision::sphere_sphere_sql2(ax, ay, az, ar, bx, by, bz, br).test_zero();
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto sphere_environment_in_collision(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

        for (const auto &es : e.spheres)
        {
            const auto diff = es.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_sphere_sql2(es, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        const auto rsq = sr * sr;
        for (const auto &ec : e.cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                return true;
            }
        }

        for (const auto &eh : e.heightfields)
        {
            if (not collision::sphere_heightfield(eh, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        const std::array<DataT, 3> positions = {sx, sy, sz};
        for (const auto &pc : e.pointclouds)
        {
            if (pc.collides_simd(positions, sr))
            {
                return true;
            }
        }

        return false;
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto sphere_environment_signed_distance(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> DataT
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        
        const auto s = collision::Sphere<DataT>(sx, sy, sz, sr);

        auto min_dist = DataT(1000.0f); // Large value

        for (const auto &es : e.spheres)
        {
            auto dist = collision::sphere_sphere_l2(es, s);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        for (const auto &ec : e.capsules)
        {
            auto dist = collision::sphere_capsule_l2(ec, s);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            auto dist = collision::sphere_z_aligned_capsule_l2(ec, s);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        for (const auto &ec : e.cuboids)
        {
            auto dist = collision::sphere_cuboid_l2(ec, s);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            auto dist = collision::sphere_z_aligned_cuboid_l2(ec, s);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        for (const auto &eh : e.heightfields)
        {
            auto dist = collision::sphere_heightfield_l2(eh, sx, sy, sz, sr);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        const std::array<DataT, 3> positions = {sx, sy, sz};
        for (const auto &pc : e.pointclouds)
        {
            auto dist = pc.min_clearance_simd(positions, sr);
            min_dist = min_dist.blend(dist, dist < min_dist);
        }

        return min_dist;
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto sphere_environment_signed_distance_and_gradient(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> std::pair<DataT, std::array<DataT, 3>>
    {
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        
        const auto s = collision::Sphere<DataT>(sx, sy, sz, sr);

        auto min_dist = DataT(1000.0f); 
        auto gx = DataT(0.0f);
        auto gy = DataT(0.0f);
        auto gz = DataT(0.0f);

        auto update = [&](const DataT& dist, const DataT& nx, const DataT& ny, const DataT& nz) {
            auto mask = dist < min_dist;
            min_dist = min_dist.blend(dist, mask);
            gx = gx.blend(nx, mask);
            gy = gy.blend(ny, mask);
            gz = gz.blend(nz, mask);
        };

        auto normalize_and_update = [&](const DataT& dist, const DataT& dx, const DataT& dy, const DataT& dz) {
             // Normalized gradient vector pointing FROM obstacle TO sphere center
             // dist = ||d|| - (r1 + r2)
             // grad = d / ||d||
             auto len = collision::sqrt(collision::dot_3(dx, dy, dz, dx, dy, dz));
             // Handle zero length case to avoid NaN? Usually implies penetration or center overlap. 
             // For now assume non-zero.
             auto inv_len = DataT(1.0f) / len; 
             update(dist, dx * inv_len, dy * inv_len, dz * inv_len);
        };
        
        // Spheres
        for (const auto &es : e.spheres)
        {
             // Vectors: s (query) - es (obstacle)
             auto dx = s.x - es.x;
             auto dy = s.y - es.y;
             auto dz = s.z - es.z;
             auto dist_sq = collision::dot_3(dx, dy, dz, dx, dy, dz);

             // Pruning: if dist_sq >= (min_dist + s.r + es.r)^2, we can skip.
             // We use a conservative check for all lanes.
             // Note: min_dist decreases, so we start eagerly.
             auto combined_r = s.r + es.r;
             auto threshold = min_dist + combined_r;
             
             // Check if we can safely skip this sphere for all lanes
             // Skip if all: dist_sq >= threshold^2 AND threshold > 0
             // We use blend/mask logic or just operate. 
             // Since "all()" might be expensive or not available, we can just proceed to calculate dist
             // but defer gradient.
             
             // However, strictly calculating sqrt is what we want to avoid.
             // Let's rely on the mask after distance calc to skip gradient, 
             // which is the main cost besides sqrt.
             
             auto len = collision::sqrt(dist_sq);
             auto dist = len - combined_r;
             
             // Check if ANY lane improves min_dist
             auto mask = dist < min_dist;
             if (mask.any())
             {
                 auto inv_len = DataT(1.0f) / len;
                 update(dist, dx * inv_len, dy * inv_len, dz * inv_len);
             }
        }

        // Capsules
        for (const auto &ec : e.capsules)
        {
            // From sphere_capsule_l2
            auto dot = collision::dot_3(s.x - ec.x1, s.y - ec.y1, s.z - ec.z1, ec.xv, ec.yv, ec.zv);
            auto cdf = (dot * ec.rdv).clamp(0.F, 1.F);
            
            // Closest point on capsule segment
            auto cx = ec.x1 + ec.xv * cdf;
            auto cy = ec.y1 + ec.yv * cdf;
            auto cz = ec.z1 + ec.zv * cdf;
            
            auto dx = s.x - cx;
            auto dy = s.y - cy;
            auto dz = s.z - cz;
            
            auto dist_sq = collision::dot_3(dx, dy, dz, dx, dy, dz);
            
            auto combined_r = s.r + ec.r;
            // Pruning could happen here based on dist_sq but we already did some math. 
            // The segment projection is cheap enough. Sqrt is the cost.
            
            auto len = collision::sqrt(dist_sq);
            auto dist = len - combined_r;

            auto mask = dist < min_dist;
            if (mask.any())
            {
                auto inv_len = DataT(1.0f) / len;
                update(dist, dx * inv_len, dy * inv_len, dz * inv_len);
            }
        }

        // Z-Aligned Capsules
        for (const auto &ec : e.z_aligned_capsules)
        {
            auto dot = (s.z - ec.z1) * ec.zv;
            auto cdf = (dot * ec.rdv).clamp(0.F, 1.F);
            
            auto cx = ec.x1;
            auto cy = ec.y1;
            auto cz = ec.z1 + ec.zv * cdf;
            
            auto dx = s.x - cx;
            auto dy = s.y - cy;
            auto dz = s.z - cz;

            auto dist_sq = collision::dot_3(dx, dy, dz, dx, dy, dz);
            auto combined_r = s.r + ec.r;
            
            auto len = collision::sqrt(dist_sq);
            auto dist = len - combined_r;

            auto mask = dist < min_dist;
            if (mask.any())
            {
                 auto inv_len = DataT(1.0f) / len;
                 update(dist, dx * inv_len, dy * inv_len, dz * inv_len);
            }
        }

        // Cuboids
        for (const auto &ec : e.cuboids)
        {
            auto xs = s.x - ec.x;
            auto ys = s.y - ec.y;
            auto zs = s.z - ec.z;

            // Bounding Sphere Pruning
            // Conservative radius of cuboid = length of half-diagonal
            auto diag_sq = ec.axis_1_r * ec.axis_1_r + ec.axis_2_r * ec.axis_2_r + ec.axis_3_r * ec.axis_3_r;
            auto R = collision::sqrt(diag_sq);
            auto center_dist_sq = collision::dot_3(xs, ys, zs, xs, ys, zs);
            auto thresh = min_dist + s.r + R;
            
            // If strictly outside bounding sphere for all lanes, skip.
            // Condition: center_dist > thresh. (Using squared check)
            // Safety: thresh must be positive. min_dist usually > 0.
            auto skip_mask = (center_dist_sq >= thresh * thresh) & (thresh > 0.0f);
            if (skip_mask.all())
            {
                continue;
            }

            // Project query point onto local axes
            auto p1 = collision::dot_3(ec.axis_1_x, ec.axis_1_y, ec.axis_1_z, xs, ys, zs);
            auto p2 = collision::dot_3(ec.axis_2_x, ec.axis_2_y, ec.axis_2_z, xs, ys, zs);
            auto p3 = collision::dot_3(ec.axis_3_x, ec.axis_3_y, ec.axis_3_z, xs, ys, zs);
            
            auto a1 = (p1.abs() - ec.axis_1_r).max(0.);
            auto a2 = (p2.abs() - ec.axis_2_r).max(0.);
            auto a3 = (p3.abs() - ec.axis_3_r).max(0.);
            
            auto sum = collision::dot_3(a1, a2, a3, a1, a2, a3).sqrt();
            auto dist = sum - s.r;

            auto mask = dist < min_dist;
            if (mask.any())
            {
                // Gradient in local frame (unnormalized)
                // grad_v = sum( 2 * ai * grad(ai) )
                // grad(ai) = sign(pi) * axis_i  if ai > 0 else 0
                // The gradient direction of sqrt(v) is same as v.
                // So we want vector: sum( ai * sign(pi) * axis_i )
                
                // Sign of projections
                auto s1 = DataT(-1.0f).blend(DataT(1.0f), p1 >= 0.0f);
                auto s2 = DataT(-1.0f).blend(DataT(1.0f), p2 >= 0.0f);
                auto s3 = DataT(-1.0f).blend(DataT(1.0f), p3 >= 0.0f);

                // Components. Only needed if mask is true? 
                // We compute for vector width, masked writes happen in update.
                // We can save some ops if we really wanted to blend 0, but scalar broadcast is cheap.

                auto c1 = a1 * s1;
                auto c2 = a2 * s2;
                auto c3 = a3 * s3;

                // Transform back to world
                auto gx_local = c1 * ec.axis_1_x + c2 * ec.axis_2_x + c3 * ec.axis_3_x;
                auto gy_local = c1 * ec.axis_1_y + c2 * ec.axis_2_y + c3 * ec.axis_3_y;
                auto gz_local = c1 * ec.axis_1_z + c2 * ec.axis_2_z + c3 * ec.axis_3_z;
                
                // Normalize
                auto g_len = collision::sqrt(collision::dot_3(gx_local, gy_local, gz_local, gx_local, gy_local, gz_local));
                
                // Beware divide by zero.
                auto non_zero = g_len > 0.0001f;
                auto inv_g_len = DataT(1.0f) / g_len;
                
                auto nx = gx_local * inv_g_len;
                auto ny = gy_local * inv_g_len;
                auto nz = gz_local * inv_g_len;
                
                // Blend zero if length is small
                nx = DataT(0.0f).blend(nx, non_zero);
                ny = DataT(0.0f).blend(ny, non_zero);
                nz = DataT(0.0f).blend(nz, non_zero);

                update(dist, nx, ny, nz);
            }
        }
        
         for (const auto &ec : e.z_aligned_cuboids)
        {
            auto xs = s.x - ec.x;
            auto ys = s.y - ec.y;
            auto zs = s.z - ec.z; // axis 3 is Z

            // Bounding Sphere Pruning
            auto diag_sq = ec.axis_1_r * ec.axis_1_r + ec.axis_2_r * ec.axis_2_r + ec.axis_3_r * ec.axis_3_r;
            auto R = collision::sqrt(diag_sq);
            auto center_dist_sq = collision::dot_3(xs, ys, zs, xs, ys, zs);
            auto thresh = min_dist + s.r + R;
            
            auto skip_mask = (center_dist_sq >= thresh * thresh) & (thresh > 0.0f);
            if (skip_mask.all())
            {
                continue;
            }

            // Project
            auto p1 = collision::dot_2(ec.axis_1_x, ec.axis_1_y, xs, ys);
            auto p2 = collision::dot_2(ec.axis_2_x, ec.axis_2_y, xs, ys);
            auto p3 = zs;
            
            auto a1 = (p1.abs() - ec.axis_1_r).max(0.);
            auto a2 = (p2.abs() - ec.axis_2_r).max(0.);
            auto a3 = (p3.abs() - ec.axis_3_r).max(0.);

            auto sum = collision::dot_3(a1, a2, a3, a1, a2, a3).sqrt();
            auto dist = sum - s.r;
            
            auto mask = dist < min_dist;
            if (mask.any())
            {
                auto s1 = DataT(-1.0f).blend(DataT(1.0f), p1 >= 0.0f);
                auto s2 = DataT(-1.0f).blend(DataT(1.0f), p2 >= 0.0f);
                auto s3 = DataT(-1.0f).blend(DataT(1.0f), p3 >= 0.0f);

                auto c1 = a1 * s1;
                auto c2 = a2 * s2;
                auto c3 = a3 * s3;
                
                // box axes: axis1, axis2, (0,0,1)
                auto gx_local = c1 * ec.axis_1_x + c2 * ec.axis_2_x;
                auto gy_local = c1 * ec.axis_1_y + c2 * ec.axis_2_y;
                auto gz_local = c3;

                 auto g_len = collision::sqrt(collision::dot_3(gx_local, gy_local, gz_local, gx_local, gy_local, gz_local));
                 auto non_zero = g_len > 0.0001f;
                 auto inv_g_len = DataT(1.0f) / g_len;

                 auto nx = DataT(0.0f).blend(gx_local * inv_g_len, non_zero);
                 auto ny = DataT(0.0f).blend(gy_local * inv_g_len, non_zero);
                 auto nz = DataT(0.0f).blend(gz_local * inv_g_len, non_zero);
                 
                 update(dist, nx, ny, nz);
            }
        }

        // Heightfields - gradient approx up (0,0,1) for now? 
        // Or analytical? Heightfield is specialized.
        // For now use (0,0,1) as a placeholder if active, or just skip grad update?
        // Let's implement basic gradient logic later if needed. 
        // For now just update distance (gradient 0)
        for (const auto &eh : e.heightfields)
        {
            auto dist = collision::sphere_heightfield_l2(eh, sx, sy, sz, sr);
            auto mask = dist < min_dist;
            if (mask.any())
            {
                update(dist, DataT(0.0f), DataT(0.0f), DataT(1.0f)); // dummy gradient up
            }
        }

        const std::array<DataT, 3> positions = {sx, sy, sz};
        for (const auto &pc : e.pointclouds)
        {
            auto dist = pc.min_clearance_simd(positions, sr);
            auto mask = dist < min_dist;
            if (mask.any())
            {
                update(dist, DataT(0.0f), DataT(0.0f), DataT(0.0f)); // dummy gradient
            }
        }

        return {min_dist, {gx, gy, gz}};
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline auto sphere_environment_get_collisions(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> std::vector<std::string>
    {
        std::vector<std::string> objects;

        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

        for (const auto &es : e.spheres)
        {
            const auto diff = es.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_sphere_sql2(es, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(es.name);
            }
        }

        for (const auto &ec : e.capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        const auto rsq = sr * sr;
        for (const auto &ec : e.cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &eh : e.heightfields)
        {
            if (not collision::sphere_heightfield(eh, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(eh.name);
            }
        }

        return objects;
    }

    template <typename DataT>
    inline constexpr auto attachment_environment_collision(const collision::Environment<DataT> &e) noexcept
        -> bool
    {
        for (const auto &s : e.attachments->posed_spheres)
        {
            // HACK: The radius needs to be a float, and unfortunately the spheres assume homogeneous
            // DataT for storage
            // TODO: Fix the sphere representation to allow to store float radii even with vector
            // centers
            if (sphere_environment_in_collision(e, s.x, s.y, s.z, s.r[{0, 0}]))
            {
                return true;
            }
        }

        return false;
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto attachment_sphere_collision(
        const collision::Environment<DataT> &e,
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        for (const auto &att_s : e.attachments->posed_spheres)
        {
            if (not collision::sphere_sphere_sql2(sx, sy, sz, sr, att_s.x, att_s.y, att_s.z, att_s.r)
                        .test_zero())
            {
                return true;
            }
        }

        return false;
    }
}  // namespace vamp
