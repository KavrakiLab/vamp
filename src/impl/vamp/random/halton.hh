#pragma once

#include <algorithm>
#include <vamp/random/rng.hh>

namespace vamp::rng
{
    template <typename Robot>
    struct Halton : public RNG<Robot>
    {
        // Numerical precision degrades around 1.4M iterations, this value can be increased up to that point.
        static constexpr const std::size_t max_iterations = 1000000U;

        using Configuration = typename Robot::Configuration;
        using Sample = FloatVector<Robot::sample_dimension>;

        // First 32 primes (excluding 2 — Halton avoids it because the base-2
        // sequence has poor stratification at coarse subdivisions). 32 entries
        // is enough for the largest non-Euclidean robots we ship (PR2 has
        // sample_dimension=18 with its planar base); extend if you need more.
        static constexpr const std::array<float, 32> primes{
            3.F,   5.F,   7.F,   11.F,  13.F,  17.F,  19.F,  23.F,
            29.F,  31.F,  37.F,  41.F,  43.F,  47.F,  53.F,  59.F,
            61.F,  67.F,  71.F,  73.F,  79.F,  83.F,  89.F,  97.F,
            101.F, 103.F, 107.F, 109.F, 113.F, 127.F, 131.F, 137.F};

        explicit Halton(Sample b_in) noexcept : b_init(b_in), b(b_in)
        {
        }

        Halton(std::initializer_list<FloatT> v) noexcept : Halton(Sample::pack_and_pad(v))
        {
        }

        explicit Halton() : Halton(bases())
        {
        }

        virtual ~Halton() = default;

        inline constexpr auto bases() noexcept -> Sample
        {
            alignas(FloatVectorAlignment) std::array<float, Robot::sample_dimension> a;
            std::copy_n(primes.cbegin(), Robot::sample_dimension, a.begin());
            return Sample(a);
        }

        auto rotate_bases() noexcept
        {
            alignas(FloatVectorAlignment) std::array<float, Sample::num_scalars_rounded> a;
            b.to_array(a.data());
            std::rotate(a.begin(), a.begin() + 1, a.begin() + Robot::sample_dimension);
            b = Sample(a.data());
        }

        const Sample b_init;
        Sample b;
        Sample n = Sample::fill(0);
        Sample d = Sample::fill(1);
        std::size_t iterations = 0;

        inline void reset() noexcept override final
        {
            iterations = 0;
            b = b_init;
            n = Sample::fill(0);
            d = Sample::fill(1);
        }

        inline auto next() noexcept -> Configuration override final
        {
            iterations++;
            if (iterations > max_iterations)
            {
                n = Sample::fill(0);
                d = Sample::fill(1);
                iterations = 0;
                rotate_bases();
            }

            auto xf = d - n;
            auto x_eq_1 = xf == 1.;
            auto x_neq_1 = ~x_eq_1;

            // if x == 1
            d = d.blend((d * b).floor(), x_eq_1);

            // if x != 1 (zero out) ignore if x == 1
            auto y = x_neq_1 & (d / b).floor();
            auto x_le_y = x_neq_1 & (xf <= y);

            while (x_le_y.any())
            {
                y = y.blend((y / b).floor(), x_le_y);
                x_le_y = x_le_y & (xf <= y);
            }

            n = (((b + 1.F) * y).floor() - xf).blend(Sample::fill(1), x_eq_1);

            return Robot::sample((n / d).trim());
        }
    };
}  // namespace vamp::rng
