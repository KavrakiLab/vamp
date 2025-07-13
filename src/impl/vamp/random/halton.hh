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

        static constexpr const std::array<float, 16> primes{
            3.F,
            5.F,
            7.F,
            11.F,
            13.F,
            17.F,
            19.F,
            23.F,
            29.F,
            31.F,
            37.F,
            41.F,
            43.F,
            47.F,
            53.F,
            59.F};

        explicit Halton(Configuration b_in) noexcept : b_init(b_in), b(b_in)
        {
        }

        Halton(std::initializer_list<FloatT> v) noexcept : Halton(Configuration::pack_and_pad(v))
        {
        }

        explicit Halton() : Halton(bases())
        {
        }

        inline constexpr auto bases() noexcept -> Configuration
        {
            alignas(FloatVectorAlignment) std::array<float, Robot::dimension> a;
            std::copy_n(primes.cbegin(), Robot::dimension, a.begin());
            return Configuration(a);
        }

        auto rotate_bases() noexcept
        {
            alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded> a;
            b.to_array(a.data());
            std::rotate(a.begin(), a.begin() + 1, a.begin() + Robot::dimension);
            b = Configuration(a.data());
        }

        const Configuration b_init;
        Configuration b;
        Configuration n = Configuration::fill(0);
        Configuration d = Configuration::fill(1);
        std::size_t iterations = 0;

        inline void reset() noexcept override final
        {
            iterations = 0;
            b = b_init;
            n = Configuration::fill(0);
            d = Configuration::fill(1);
        }

        inline auto next() noexcept -> Configuration override final
        {
            iterations++;
            if (iterations > max_iterations)
            {
                n = Configuration::fill(0);
                d = Configuration::fill(1);
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

            n = (((b + 1.F) * y).floor() - xf).blend(Configuration::fill(1), x_eq_1);

            auto result = (n / d).trim();
            Robot::scale_configuration(result);
            return result;
        }
    };
}  // namespace vamp::rng
