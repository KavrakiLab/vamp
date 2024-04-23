#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::sphere
{
    using Configuration = FloatVector<3>;

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, 3>;

    // Pad and align vectors for easy loading.
    alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> lows{-10, -10, 0};
    alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> highs{10, 10, 5};
    static float radius = 0.2;

    inline void set_radius(float new_radius) noexcept
    {
        radius = new_radius;
    }

    inline void set_lows(std::array<float, 3> new_lows) noexcept
    {
        std::copy_n(new_lows.cbegin(), 3, lows.begin());
    }

    inline void set_highs(std::array<float, 3> new_highs) noexcept
    {
        std::copy_n(new_highs.cbegin(), 3, highs.begin());
    }

    inline void scale_configuration(Configuration &q) noexcept
    {
        Configuration clow(lows.data());
        Configuration chigh(highs.data());

        q = q * (chigh - clow) + clow;
    }

    inline void descale_configuration(Configuration &q) noexcept
    {
        Configuration clow(lows.data());
        Configuration chigh(highs.data());

        q = (q - clow) / (chigh - clow);
    }

    template <std::size_t rake>
    inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = lows[0] + (q[0] * (highs[0] - lows[0]));
        q[1] = lows[1] + (q[1] * (highs[1] - lows[1]));
        q[2] = lows[2] + (q[2] * (highs[2] - lows[2]));
    }

    template <std::size_t rake>
    inline void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = (q[0] - lows[0]) / (highs[0] - lows[0]);
        q[1] = (q[1] - lows[1]) / (highs[1] - lows[1]);
        q[2] = (q[2] - lows[2]) / (highs[2] - lows[2]);
    }

    inline static auto space_measure() noexcept -> float
    {
        Configuration clow(lows.data());
        Configuration chigh(highs.data());
        return (chigh - clow).l2_norm();
    }

    constexpr auto n_spheres = 1;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, 1> x;
        FloatVector<rake, 1> y;
        FloatVector<rake, 1> z;
        FloatVector<rake, 1> r;
    };

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
        out.x[0] = q[0];
        out.y[0] = q[1];
        out.z[0] = q[2];
        out.r[0] = radius;
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        return not sphere_environment_in_collision(environment, q[0], q[1], q[2], radius);
    }
}  // namespace vamp::robots::sphere

// NOLINTEND(*-magic-numbers)
