#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <vamp/random/distribution.hh>
#include <vamp/random/rng.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/vector/eigen.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    namespace utils
    {
        template <std::size_t dimension>
        inline auto phs_measure(float d_foci, float d_transverse) noexcept -> float
        {
            const float conjugate_diameter = std::sqrt(d_transverse * d_transverse - d_foci * d_foci);

            float lmeas = d_transverse / 2.0;
            for (auto i = 1U; i < dimension; ++i)
            {
                lmeas = lmeas * conjugate_diameter / 2.0;
            }

            return lmeas * unit_ball_measure(dimension);
        }

    }  // namespace utils

    template <typename Robot>
    class ProlateHyperspheroid
    {
        static constexpr auto dimension = Robot::dimension;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        ProlateHyperspheroid(
            const vamp::FloatVector<dimension> &focus_1_in,
            const vamp::FloatVector<dimension> &focus_2_in)
          : focus_1(focus_1_in), focus_2(focus_2_in)
        {
            min_transverse_diameter = (focus_1 - focus_2).l2_norm();
            center = 0.5 * (focus_1 + focus_2);
            update_rotation();
        }

        inline void set_transverse_diameter(float transverse_diameter_in) noexcept
        {
            transverse_diameter = transverse_diameter_in;
            update_transformation();
        }

        inline auto transform(const vamp::FloatVector<dimension> &sphere) const noexcept
            -> vamp::FloatVector<dimension>
        {
            alignas(FloatVectorAlignment) auto sphere_data = sphere.to_array();
            vamp::EigenFloatVectorMap<dimension>(sphere_data.data()) =
                tf_world_from_ellipse * vamp::EigenConstFloatVectorMap<dimension>(sphere_data.data());

            return vamp::FloatVector<dimension>(sphere_data.data()) + center;
        }

        inline auto inside(const vamp::FloatVector<dimension> &point) const noexcept -> bool
        {
            return path_length(point) < transverse_diameter;
        }

        inline auto measure() const noexcept -> float
        {
            return phs_measure;
        }

        inline auto measure(float transverse_diameter_in) const noexcept -> float
        {
            return phs_measure<dimension>(min_transverse_diameter, transverse_diameter_in);
        }

        inline auto get_min_transverse_diameter() const noexcept -> float
        {
            return min_transverse_diameter;
        }

        inline auto path_length(const vamp::FloatVector<dimension> point) const -> float
        {
            return (focus_1 - point).l2_norm() + (point - focus_2).l2_norm();
        }

    private:
        vamp::FloatVector<dimension> focus_1;
        vamp::FloatVector<dimension> focus_2;
        vamp::FloatVector<dimension> center;

        float transverse_diameter{0.};
        float min_transverse_diameter;
        float phs_measure;

        using EigenVector = Eigen::Vector<float, dimension>;
        using EigenMatrix = Eigen::Matrix<float, dimension, dimension>;

        EigenMatrix rot_world_from_ellipse;
        EigenMatrix tf_world_from_ellipse;

        inline void update_rotation() noexcept
        {
            // If the transverse diameter is too close to 0, we treat this as a circle.
            static constexpr float circle_tolerance = 1e-6;
            if (min_transverse_diameter < circle_tolerance)
            {
                rot_world_from_ellipse.setIdentity(dimension, dimension);
            }
            else
            {
                const EigenVector f1 = vamp::vector_to_eigen(focus_1);
                const EigenVector f2 = vamp::vector_to_eigen(focus_2);
                const EigenVector transverse_axis = (f2 - f1) / min_transverse_diameter;
                const EigenMatrix wahba_prob = transverse_axis * EigenMatrix::Identity().col(0).transpose();

                Eigen::JacobiSVD<EigenMatrix, Eigen::NoQRPreconditioner> svd(
                    wahba_prob, Eigen::ComputeFullV | Eigen::ComputeFullU);

                EigenVector middle_m = EigenVector::Ones();
                middle_m(dimension - 1) = svd.matrixU().determinant() * svd.matrixV().determinant();
                rot_world_from_ellipse = svd.matrixU() * middle_m.asDiagonal() * svd.matrixV().transpose();
            }
        }

        void update_transformation()

        {
            const float conjugate_diamater = std::sqrt(
                transverse_diameter * transverse_diameter -
                min_transverse_diameter * min_transverse_diameter);

            EigenVector diag = EigenVector::Constant(0.5 * conjugate_diamater);
            diag(0) = 0.5 * transverse_diameter;

            tf_world_from_ellipse = rot_world_from_ellipse * diag.asDiagonal();
            phs_measure = utils::phs_measure<dimension>(min_transverse_diameter, transverse_diameter);
        }
    };

    template <typename Robot>
    struct ProlateHyperspheroidRNG : public rng::RNG<Robot>
    {
        ProlateHyperspheroidRNG(ProlateHyperspheroid<Robot> phs, typename vamp::rng::RNG<Robot>::Ptr &rng)
          : phs(phs), rng(rng)
        {
        }

        inline void reset() noexcept override
        {
            rng->reset();
            rng->dist.reset();
        }

        inline auto next() noexcept -> FloatVector<Robot::dimension> override
        {
            auto x = phs.transform(uniform_in_ball());

            // Clamp values
            Robot::descale_configuration(x);
            x = x.clamp(0.F, 1.F);
            Robot::scale_configuration(x);

            return x;
        }

        inline auto logit() noexcept -> vamp::FloatVector<Robot::dimension>
        {
            auto U1 = rng->next();
            Robot::descale_configuration(U1);
            return (U1 * (1 - U1).rcp()).log() * std::sqrt(vamp::utils::constants::pi / 8.F);
        }

        inline auto uniform_on_ball() noexcept -> vamp::FloatVector<Robot::dimension>
        {
            const auto unv = logit().trim();
            return unv / unv.l2_norm();
        }

        inline auto uniform_in_ball() noexcept -> vamp::FloatVector<Robot::dimension>
        {
            return std::pow(rng->dist.uniform_01(), 1.0 / static_cast<float>(Robot::dimension)) *
                   uniform_on_ball();
        }

        ProlateHyperspheroid<Robot> phs;
        typename vamp::rng::RNG<Robot>::Ptr rng;
    };

}  // namespace vamp::planning
