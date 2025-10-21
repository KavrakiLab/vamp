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
    struct EEFAttachment {
        size_t eef_idx;
        Attachment<DataT> attachment;

        EEFAttachment(size_t eef_idx, const Attachment<DataT>& _a) : eef_idx(eef_idx), attachment(_a) {}

        template <typename OtherDataT>
        EEFAttachment(const EEFAttachment<OtherDataT> &other)
        : eef_idx(other.eef_idx), attachment(other.attachment)
        {
        }

    };

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
        std::vector<EEFAttachment<DataT>> eef_attachments; // eef_id to attachment

        void attach(const Attachment<DataT> &a, size_t eef_idx = 0){
            for(auto &eef_attachment: eef_attachments)
                if (eef_attachment.eef_idx == eef_idx){
                    eef_attachment.attachment = a;
                    return ;
                }
            eef_attachments.emplace_back(eef_idx, a);
        }
        void detach(size_t eef_idx = 0){
            for(size_t i = 0; i < eef_attachments.size(); i++)
                if (eef_attachments[i].eef_idx == eef_idx){
                    eef_attachments.erase(eef_attachments.begin() + i);
                    return ;
                }
        }

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
          , eef_attachments(other.eef_attachments.begin(), other.eef_attachments.end())
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
        // small enough N that brute forcing is the best option.
        for(auto i=0U; i < N; i++){
            for(auto &eef_attachment : e.eef_attachments) {
                if (eef_attachment.eef_idx == i)
                    eef_attachment.attachment.pose(p_tfs[i]);
            }
        }
    }

}  // namespace vamp::collision
