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

        // NOTE: This lint (google-explicit-constructor) is incorrect in this case; we want implicit
        // conversion
        inline operator Eigen::Matrix<float, dim_, 1>() const
        {
            return EigenFloatVectorMap<dim_>(v);
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
            const auto diff = vamp::FloatVector<kDimensions>(a.v) - vamp::FloatVector<kDimensions>(b.v);
            return diff.l2_norm();
        }
    };
}  // namespace unc::robotics::nigh::metric

namespace vamp::planning
{
    namespace nigh = unc::robotics::nigh;

    template <std::size_t dim>
    using Space = nigh::metric::Space<NNFloatArray<dim>, nigh::metric::LP<2>>;

    template <std::size_t dimension>
    struct NNNode
    {
        std::size_t index;
        NNFloatArray<dimension> array;

        [[nodiscard]] inline auto as_vector() const noexcept -> FloatVector<dimension>
        {
            return FloatVector<dimension>(array.v);
        }
    };

    template <std::size_t dimension>
    struct NNNodeKey
    {
        inline auto operator()(const NNNode<dimension> &node) const noexcept
            -> const NNFloatArray<dimension> &
        {
            return node.array;
        }
    };

    template <std::size_t dimension, std::size_t batch = 128>
    using NN = nigh::Nigh<
        NNNode<dimension>,     //
        Space<dimension>,      //
        NNNodeKey<dimension>,  //
        nigh::NoThreadSafety,  //
        nigh::KDTreeBatch<batch>>;
}  // namespace vamp::planning
