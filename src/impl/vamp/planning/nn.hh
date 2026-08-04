#pragma once

#include <vamp/vector.hh>
#include <vamp/vector/eigen.hh>

#include <nigh/nigh_forward.hpp>
#include <nigh/lp_space.hpp>
#include <nigh/kdtree_batch.hpp>

namespace vamp::planning
{
    template <std::size_t dim_>
    struct NNFloatArray
    {
        inline static constexpr auto dim = dim_;
        float *v;

        inline operator Eigen::Matrix<float, dim_, 1>() const
        {
            return Eigen::Map<const Eigen::Matrix<float, dim_, 1>, Eigen::Unaligned>(v);
        }
    };
}  // namespace vamp::planning

namespace unc::robotics::nigh::metric
{
    template <std::size_t dim>
    struct Space<vamp::planning::NNFloatArray<dim>, LP<2>>
    {
        using Type = vamp::planning::NNFloatArray<dim>;
        using Metric = LP<2>;
        using Distance = float;
        static constexpr int kDimensions = dim;

        static auto isValid(const Type & /*v*/) -> bool
        {
            return true;
        }

        static auto coeff(const Type &v, std::size_t i) -> float
        {
            return *(v.v + i);
        }

        [[nodiscard]] static constexpr auto dimensions() -> unsigned
        {
            return kDimensions;
        }

        static auto distance(const Type &a, const Type &b) -> float
        {
            const auto diff = vamp::FloatVector<kDimensions>(a.v, false)
                            - vamp::FloatVector<kDimensions>(b.v, false);
            return diff.l2_norm();
        }
    };
}  // namespace unc::robotics::nigh::metric

namespace vamp::planning
{
    namespace nigh = unc::robotics::nigh;

    template <typename Robot>
    struct NNNode
    {
        std::size_t index;
        typename Robot::NNKey key;
    };

    template <typename Robot>
    struct NNNodeKey
    {
        inline auto operator()(const NNNode<Robot> &node) const noexcept
            -> const typename Robot::NNKey &
        {
            return node.key;
        }
    };

    template <typename Robot, std::size_t batch = 128>
    using NN = nigh::Nigh<
        NNNode<Robot>,           //
        typename Robot::NNSpace, //
        NNNodeKey<Robot>,        //
        nigh::NoThreadSafety,    //
        nigh::KDTreeBatch<batch>>;
}  // namespace vamp::planning
