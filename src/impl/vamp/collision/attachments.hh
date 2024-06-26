#pragma once

#include <type_traits>
#include <vamp/collision/shapes.hh>
#include <vector>

namespace vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(
            DataT tf_tx,
            DataT tf_ty,
            DataT tf_tz,
            DataT tf_rx,
            DataT tf_ry,
            DataT tf_rz,
            DataT tf_rw) noexcept
          : tf_tx(std::move(tf_tx))
          , tf_ty(std::move(tf_ty))
          , tf_tz(std::move(tf_tz))
          , tf_rx(std::move(tf_rx))
          , tf_ry(std::move(tf_ry))
          , tf_rz(std::move(tf_rz))
          , tf_rw(std::move(tf_rw))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(
            float tf_tx,
            float tf_ty,
            float tf_tz,
            float tf_rx,
            float tf_ry,
            float tf_rz,
            float tf_rw) noexcept
          : Attachment(
                static_cast<DataT>(tf_tx),
                static_cast<DataT>(tf_ty),
                static_cast<DataT>(tf_tz),
                static_cast<DataT>(tf_rx),
                static_cast<DataT>(tf_ry),
                static_cast<DataT>(tf_rz),
                static_cast<DataT>(tf_rw))
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept
          : Attachment(o.tf_tx, o.tf_ty, o.tf_tz, o.tf_rx, o.tf_ry, o.tf_rz, o.tf_rw)
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }
        }

        std::vector<Sphere<DataT>> spheres;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        DataT tf_tx;
        DataT tf_ty;
        DataT tf_tz;
        DataT tf_rx;
        DataT tf_ry;
        DataT tf_rz;
        DataT tf_rw;

        inline void pose(DataT p_tx, DataT p_ty, DataT p_tz, DataT p_rx, DataT p_ry, DataT p_rz, DataT p_rw)
            const noexcept
        {
            // Multiply parent orientation quaternion and attachment orientation quaternion
            const auto rx = p_rw * tf_rx + p_rx * tf_rw + p_ry * tf_rz - p_rz * tf_ry;
            const auto ry = p_rw * tf_ry - p_rx * tf_rz + p_ry * tf_rw + p_rz * tf_rx;
            const auto rz = p_rw * tf_rz + p_rx * tf_ry - p_ry * tf_rx + p_rz * tf_rw;
            const auto rw = p_rw * tf_rw - p_rx * tf_rx - p_ry * tf_ry - p_rz * tf_rz;

            // Rotate and compose parent translation and attachment translation
            const auto x0 = p_ry * tf_tz - p_rz * tf_ty;
            const auto x1 = p_rx * tf_ty - p_ry * tf_tx;
            const auto x2 = p_rx * tf_tz - p_rz * tf_tx;
            const auto tx = p_tx + 2.0 * (p_rw * x0 + p_ry * x1 + p_rz * x2) + tf_tx;
            const auto ty = p_ty + 2.0 * (-p_rw * x2 - p_rx * x1 + p_rz * x0) + tf_ty;
            const auto tz = p_tz + 2.0 * (p_rw * x1 - p_rx * x2 - p_ry * x0) + tf_tz;

            // Rotate basis by composed orientation
            const auto bx0 = ry * ry;
            const auto bx1 = rz * rz;
            const auto bx2 = rw * rz;
            const auto bx3 = rw * ry;
            const auto bx4 = rx * rx;
            const auto bx5 = rw * rx;
            const auto bx6 = rx * ry;
            const auto bx7 = rx * rz;
            const auto bx8 = ry * rz;
            const auto b_xx = -2.0 * (bx0 + bx1) + 1.0;
            const auto b_xy = 2.0 * (bx6 + bx2);
            const auto b_xz = 2.0 * (bx7 - bx3);
            const auto b_yx = 2.0 * (bx6 - bx2);
            const auto b_yy = -2.0 * (bx1 + bx4) + 1.0;
            const auto b_yz = 2.0 * (bx8 + bx5);
            const auto b_zx = 2.0 * (bx7 + bx3);
            const auto b_zy = 2.0 * (bx8 - bx5);
            const auto b_zz = -2.0 * (bx0 + bx4) + 1.0;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                // Pose each center
                const auto s_x = s.x * b_xx + s.x * b_yx + s.x * b_zx + tx;
                const auto s_y = s.y * b_xy + s.y * b_yy + s.y * b_zy + ty;
                const auto s_z = s.z * b_xz + s.z * b_yz + s.z * b_zz + tz;
                posed_spheres[i] = Sphere<DataT>(s_x, s_y, s_z, s.r);
            }
        }
    };
}  // namespace vamp::collision
