#pragma once

#include <algorithm>
#include <vamp/random/rng.hh>

namespace vamp::rng
{
    template <std::size_t dim>
    struct Halton : public RNG<dim>
    {
        static constexpr const std::size_t max_iterations = 100000U;

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

        explicit Halton(FloatVector<dim> b_in, std::size_t skip_iterations = 0) noexcept : b(b_in)
        {
            initialize(skip_iterations);
        }

        Halton(std::initializer_list<FloatT> v, std::size_t skip_iterations = 0) noexcept
          : Halton(FloatVector<dim>::pack_and_pad(v), skip_iterations)
        {
        }

        explicit Halton(std::size_t skip_iterations = 0)
          : Halton(
                []()
                {
                    alignas(FloatVectorAlignment) std::array<float, dim> a;
                    std::copy_n(primes.cbegin(), dim, a.begin());
                    return a;
                }(),
                skip_iterations)
        {
        }

        inline auto initialize(std::size_t n) noexcept
        {
            for (auto i = 0U; i < n; ++i)
            {
                next();
            }
        }

        inline auto rotate_bases() noexcept
        {
            alignas(FloatVectorAlignment) std::array<float, dim> a;
            b.to_array(a.data());
            std::rotate(a.begin(), a.begin() + 1, a.end());
            b = FloatVector<dim>(a);
        }

        std::size_t iterations = 0;
        FloatVector<dim> b;
        FloatVector<dim> n = FloatVector<dim>::fill(0);
        FloatVector<dim> d = FloatVector<dim>::fill(1);

        inline auto next() noexcept -> FloatVector<dim> override
        {
            iterations++;
            if (iterations > max_iterations)
            {
                n = FloatVector<dim>::fill(0);
                d = FloatVector<dim>::fill(1);
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

            n = (((b + 1.F) * y).floor() - xf).blend(FloatVector<dim>::fill(1), x_eq_1);
            return (n / d).trim();
        }
    };
}  // namespace vamp::rng
