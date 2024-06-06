#pragma once

#include <vector>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/capt.hh>

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
    };

}  // namespace vamp::collision
