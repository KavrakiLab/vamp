#pragma once

#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <optional>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/capt.hh>
#include <vamp/collision/attachments.hh>

namespace vamp::collision
{
    template <typename DataT>
    struct Environment
    {
        std::vector<Sphere<DataT>> spheres;
        std::vector<Capsule<DataT>> capsules;
        std::vector<Capsule<DataT>> z_aligned_capsules;
        std::vector<Cylinder<DataT>> cylinders;
        std::vector<Cuboid<DataT>> cuboids;
        std::vector<Cuboid<DataT>> z_aligned_cuboids;
        std::vector<HeightField<DataT>> heightfields;
        std::vector<CAPT> pointclouds;
        std::map<size_t, Attachment<DataT>> attachments; // eef_id to attachment

        Environment() = default;

        template <typename OtherDataT>
        explicit Environment(const Environment<OtherDataT> &other)
          : spheres(other.spheres.begin(), other.spheres.end())
          , capsules(other.capsules.begin(), other.capsules.end())
          , z_aligned_capsules(other.z_aligned_capsules.begin(), other.z_aligned_capsules.end())
          , cylinders(other.cylinders.begin(), other.cylinders.end())
          , cuboids(other.cuboids.begin(), other.cuboids.end())
          , z_aligned_cuboids(other.z_aligned_cuboids.begin(), other.z_aligned_cuboids.end())
          , heightfields(other.heightfields.begin(), other.heightfields.end())
          , pointclouds(other.pointclouds.begin(), other.pointclouds.end())
          , attachments(other.attachments.begin(), other.attachments.end())
        {
        }

        inline auto sort()
        {
            std::sort(
                spheres.begin(),
                spheres.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                capsules.begin(),
                capsules.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                z_aligned_capsules.begin(),
                z_aligned_capsules.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                cylinders.begin(),
                cylinders.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                cuboids.begin(),
                cuboids.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                z_aligned_cuboids.begin(),
                z_aligned_cuboids.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
        }

    private:
        template <typename OtherDataT>
        friend struct Environment;
    };

    template <size_t N, typename DataT>
    inline auto set_attachment_pose(
        const Environment<DataT> &e,
        const std::array<Eigen::Transform<DataT, 3, Eigen::Isometry>, N> &p_tfs
    ) noexcept
    {
        for(auto i=0U; i < N; i++){
            if(e.attachments.find(i) != e.attachments.end())
                e.attachments.at(i).pose(p_tfs[i]);
        }
    }

}  // namespace vamp::collision
