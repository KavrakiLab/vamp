#pragma once

#include <array>
#include <limits>
#include <type_traits>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/cuboid_cuboid.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{

    template <typename DataT>
    inline constexpr auto cuboid_capsule(const Cuboid<DataT> &box, const Capsule<DataT> &capsule) noexcept
        -> DataT
    {
        const DataT zero = 0.;
        const DataT one = 1.;
        const DataT eps = 1e-6F;
        const DataT inf = std::numeric_limits<float>::infinity();

        const auto axes = vamp::collision::cuboid_helpers::cuboid_axes(box);
        const auto extents = vamp::collision::cuboid_helpers::cuboid_extents(box);

        const auto sx = capsule.x1 - box.x;
        const auto sy = capsule.y1 - box.y;
        const auto sz = capsule.z1 - box.z;

        std::array<DataT, 3> p0 = {
            vamp::collision::cuboid_helpers::project_onto_axis(axes, 0, sx, sy, sz),
            vamp::collision::cuboid_helpers::project_onto_axis(axes, 1, sx, sy, sz),
            vamp::collision::cuboid_helpers::project_onto_axis(axes, 2, sx, sy, sz)};

        std::array<DataT, 3> dir = {
            vamp::collision::cuboid_helpers::project_onto_axis(axes, 0, capsule.xv, capsule.yv, capsule.zv),
            vamp::collision::cuboid_helpers::project_onto_axis(axes, 1, capsule.xv, capsule.yv, capsule.zv),
            vamp::collision::cuboid_helpers::project_onto_axis(axes, 2, capsule.xv, capsule.yv, capsule.zv)};

        const auto eval_at = [&](const DataT &t) {
            const auto px = p0[0] + dir[0] * t;
            const auto py = p0[1] + dir[1] * t;
            const auto pz = p0[2] + dir[2] * t;

            const auto dx = (px.abs() - extents[0]).max(zero);
            const auto dy = (py.abs() - extents[1]).max(zero);
            const auto dz = (pz.abs() - extents[2]).max(zero);

            return dot_3(dx, dy, dz, dx, dy, dz);
        };

        auto best = eval_at(zero);
        best = vamp::collision::cuboid_helpers::min_value(best, eval_at(one));

        const std::array<DataT, 2> signs = {one, DataT(-1.)};

        for (auto axis = 0U; axis < 3; ++axis)
        {
            const auto non_zero_mask = dir[axis].abs() > eps;
            const auto denom = vamp::collision::cuboid_helpers::mask_select(non_zero_mask, dir[axis], one);

            for (const auto &sign : signs)
            {
                const auto numer = sign * extents[axis] - p0[axis];
                const auto t = (numer / denom).clamp(0.F, 1.F);
                const auto candidate = vamp::collision::cuboid_helpers::mask_select(non_zero_mask, eval_at(t), inf);
                best = vamp::collision::cuboid_helpers::min_value(best, candidate);
            }
        }

        return best - (capsule.r * capsule.r);
    }
}  // namespace vamp::collision
