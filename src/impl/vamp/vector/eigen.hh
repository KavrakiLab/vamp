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

    template <std::size_t dim>
    using EigenConstFloatVectorMap =
        Eigen::Map<const Eigen::Matrix<float, dim, 1>, get_eigen_alignment<FloatVectorAlignment>()>;

    template <std::size_t dim>
    inline constexpr auto eigen_to_vector(const Eigen::Vector<float, dim> &vector) noexcept
        -> vamp::FloatVector<dim>
    {
        alignas(FloatVectorAlignment) std::array<float, dim> array;
        EigenFloatVectorMap<dim>(array.data()) = vector;
        return vamp::FloatVector<dim>(array.data());
    }

    template <std::size_t dim>
    inline constexpr auto vector_to_eigen(const vamp::FloatVector<dim> &vector) noexcept
        -> Eigen::Matrix<float, dim, 1>
    {
        alignas(FloatVectorAlignment) std::array<float, vamp::FloatVector<dim>::num_scalars_rounded> array =
            {};
        vector.to_array(array);
        return Eigen::Matrix<float, dim, 1>(array.data());
    }

}  // namespace vamp
