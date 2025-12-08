#pragma once

#include <array>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision::cuboid_helpers
{
    template <typename DataT>
    inline constexpr auto cuboid_axes(const Cuboid<DataT> &c) noexcept -> std::array<std::array<DataT, 3>, 3>
    {
        return {{
            {c.axis_1_x, c.axis_1_y, c.axis_1_z},
            {c.axis_2_x, c.axis_2_y, c.axis_2_z},
            {c.axis_3_x, c.axis_3_y, c.axis_3_z},
        }};
    }

    template <typename DataT>
    inline constexpr auto cuboid_radii(const Cuboid<DataT> &c) noexcept -> std::array<DataT, 3>
    {
        return {c.axis_1_r, c.axis_2_r, c.axis_3_r};
    }

    template <typename DataT>
    inline constexpr auto axis_dot(
        const std::array<std::array<DataT, 3>, 3> &axes,
        std::size_t idx,
        const DataT &x,
        const DataT &y,
        const DataT &z) noexcept -> DataT
    {
        return dot_3(axes[idx][0], axes[idx][1], axes[idx][2], x, y, z);
    }

    template <typename MaskT, typename ValueT>
    inline constexpr auto mask_select(const MaskT &mask, const ValueT &truthy, const ValueT &falsy) -> ValueT
    {
        if constexpr (std::is_same_v<MaskT, bool>)
        {
            return mask ? truthy : falsy;
        }
        else
        {
            return falsy.blend(truthy, mask);
        }
    }

    template <typename DataT>
    inline constexpr auto min_value(const DataT &a, const DataT &b) -> DataT
    {
        const auto mask = (a <= b);
        return mask_select(mask, a, b);
    }

    template <typename DataT>
    inline constexpr auto cuboid_extents(const Cuboid<DataT> &c) noexcept -> std::array<DataT, 3>
    {
        return {c.axis_1_r, c.axis_2_r, c.axis_3_r};
    }

    template <typename DataT>
    inline constexpr auto project_onto_axis(
        const std::array<std::array<DataT, 3>, 3> &axes,
        std::size_t idx,
        const DataT &x,
        const DataT &y,
        const DataT &z) noexcept -> DataT
    {
        return dot_3(axes[idx][0], axes[idx][1], axes[idx][2], x, y, z);
    }
}  // namespace vamp::collision::cuboid_helpers

namespace vamp::collision
{

    /**
     * Separating axis theorem test between two arbitrarily oriented cuboids.
     *
     * The returned value is the maximum separation measured across the 15 candidate axes
     * (three face normals from each cuboid and the nine cross products between them). A
     * strictly positive result indicates that the cuboids are separated by that margin,
     * while a non-positive result indicates an overlap (zero meaning they are exactly
     * touching).
     */
    template <typename DataT>
    inline constexpr auto cuboid_cuboid(const Cuboid<DataT> &a, const Cuboid<DataT> &b) noexcept -> DataT
    {
        const auto axes_a = vamp::collision::cuboid_helpers::cuboid_axes(a);
        const auto axes_b = vamp::collision::cuboid_helpers::cuboid_axes(b);
        const auto radii_a = vamp::collision::cuboid_helpers::cuboid_radii(a);
        const auto radii_b = vamp::collision::cuboid_helpers::cuboid_radii(b);

        // Translation from A to B, expressed in A's frame.
        const auto tx = b.x - a.x;
        const auto ty = b.y - a.y;
        const auto tz = b.z - a.z;

        std::array<DataT, 3> t = {
            vamp::collision::cuboid_helpers::axis_dot(axes_a, 0, tx, ty, tz),
            vamp::collision::cuboid_helpers::axis_dot(axes_a, 1, tx, ty, tz),
            vamp::collision::cuboid_helpers::axis_dot(axes_a, 2, tx, ty, tz)};

        std::array<std::array<DataT, 3>, 3> r{};
        std::array<std::array<DataT, 3>, 3> abs_r{};
        const DataT eps = DataT(1e-6F);

        for (auto i = 0U; i < 3; ++i)
        {
            for (auto j = 0U; j < 3; ++j)
            {
                const auto dot = dot_3(
                    axes_a[i][0],
                    axes_a[i][1],
                    axes_a[i][2],
                    axes_b[j][0],
                    axes_b[j][1],
                    axes_b[j][2]);
                r[i][j] = dot;
                abs_r[i][j] = dot.abs() + eps;
            }
        }

        const auto axis_test_a = [&](std::size_t i) {
            const auto proj = t[i].abs();
            const auto rb =
                radii_b[0] * abs_r[i][0] + radii_b[1] * abs_r[i][1] + radii_b[2] * abs_r[i][2];
            return proj - (radii_a[i] + rb);
        };

        const auto axis_test_b = [&](std::size_t j) {
            const auto proj = (t[0] * r[0][j] + t[1] * r[1][j] + t[2] * r[2][j]).abs();
            const auto ra =
                radii_a[0] * abs_r[0][j] + radii_a[1] * abs_r[1][j] + radii_a[2] * abs_r[2][j];
            return proj - (ra + radii_b[j]);
        };

        const auto axis_test_cross = [&](std::size_t i, std::size_t j) {
            const auto i1 = (i + 1) % 3;
            const auto i2 = (i + 2) % 3;
            const auto j1 = (j + 1) % 3;
            const auto j2 = (j + 2) % 3;

            const auto ra = radii_a[i1] * abs_r[i2][j] + radii_a[i2] * abs_r[i1][j];
            const auto rb = radii_b[j1] * abs_r[i][j2] + radii_b[j2] * abs_r[i][j1];
            const auto proj = (t[i2] * r[i1][j] - t[i1] * r[i2][j]).abs();
            return proj - (ra + rb);
        };

        auto max_sep = axis_test_a(0);
        max_sep = max_sep.max(axis_test_a(1));
        max_sep = max_sep.max(axis_test_a(2));

        max_sep = max_sep.max(axis_test_b(0));
        max_sep = max_sep.max(axis_test_b(1));
        max_sep = max_sep.max(axis_test_b(2));

        for (auto i = 0U; i < 3; ++i)
        {
            for (auto j = 0U; j < 3; ++j)
            {
                max_sep = max_sep.max(axis_test_cross(i, j));
            }
        }

        return max_sep;
    }
}  // namespace vamp::collision
