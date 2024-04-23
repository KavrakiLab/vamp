#pragma once

#include <vamp/vector.hh>
#include <Eigen/Dense>

namespace vamp
{
    template <std::size_t alignment>
    inline constexpr auto get_eigen_alignment() -> Eigen::AlignmentType;

    template <>
    inline constexpr auto get_eigen_alignment<16>() -> Eigen::AlignmentType
    {
        return Eigen::Aligned16;
    }

    template <>
    inline constexpr auto get_eigen_alignment<32>() -> Eigen::AlignmentType
    {
        return Eigen::Aligned32;
    }

    template <std::size_t dim>
    using EigenFloatVectorMap =
        Eigen::Map<Eigen::Matrix<float, dim, 1>, get_eigen_alignment<FloatVectorAlignment>()>;
}  // namespace vamp
