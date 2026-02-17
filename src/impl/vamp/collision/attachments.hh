#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <type_traits>
#include <vamp/collision/shapes.hh>
#include <vector>

namespace vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(const Eigen::Transform<DataT, 3, Eigen::Isometry> &tf) noexcept : tf(std::move(tf))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Eigen::Transform<float, 3, Eigen::Isometry> &tf) noexcept
          : Attachment(tf.cast<DataT>())
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept : Attachment(o.tf)
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }

            cuboids.reserve(o.cuboids.size());
            for (const auto &cuboid : o.cuboids)
            {
                cuboids.emplace_back(cuboid);
            }
        }

        std::vector<Sphere<DataT>> spheres;
        std::vector<Cuboid<DataT>> cuboids;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        mutable std::vector<Cuboid<DataT>> posed_cuboids;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        inline void pose(const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) const noexcept
        {
            const auto &n_tf = p_tf * tf;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                Eigen::Matrix<DataT, 3, 1> sp(s.x, s.y, s.z);
                auto tfs = n_tf * sp;
                posed_spheres[i] = Sphere<DataT>(tfs[0], tfs[1], tfs[2], s.r);
            }

            posed_cuboids.resize(cuboids.size());
            for (auto i = 0U; i < cuboids.size(); ++i)
            {
                const auto &c = cuboids[i];
                Eigen::Matrix<DataT, 3, 1> cp(c.x, c.y, c.z);
                Eigen::Matrix<DataT, 3, 1> ax1(c.axis_1_x, c.axis_1_y, c.axis_1_z);
                Eigen::Matrix<DataT, 3, 1> ax2(c.axis_2_x, c.axis_2_y, c.axis_2_z);
                Eigen::Matrix<DataT, 3, 1> ax3(c.axis_3_x, c.axis_3_y, c.axis_3_z);

                const auto t_center = n_tf * cp;
                const auto rot = n_tf.linear();
                const auto t_axis_1 = rot * ax1;
                const auto t_axis_2 = rot * ax2;
                const auto t_axis_3 = rot * ax3;

                posed_cuboids[i] = Cuboid<DataT>(
                    t_center[0],
                    t_center[1],
                    t_center[2],
                    t_axis_1[0],
                    t_axis_1[1],
                    t_axis_1[2],
                    t_axis_2[0],
                    t_axis_2[1],
                    t_axis_2[2],
                    t_axis_3[0],
                    t_axis_3[1],
                    t_axis_3[2],
                    c.axis_1_r,
                    c.axis_2_r,
                    c.axis_3_r);
            }
        }
    };
}  // namespace vamp::collision
