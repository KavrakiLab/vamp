#pragma once

#include <vamp/vector.hh>

namespace vamp::robots
{
    namespace
    {
        // Pad and align vectors for easy loading.
        alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> lows{-10, -10, 0};
        alignas(FloatVectorAlignment) static std::array<float, FloatVectorWidth> highs{10, 10, 5};
        static float radius = 0.2;
    }  // namespace

    struct Sphere
    {
        static constexpr auto name = "sphere";
        static constexpr auto dimension = 3;
        static constexpr auto n_spheres = 1;
        static constexpr auto resolution = 32;

        static constexpr float &min_radius = radius;
        static constexpr float &max_radius = radius;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        template <std::size_t rake>
        using ConfigurationBlock = FloatVector<rake, 3>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        static constexpr std::array<std::string_view, dimension> joint_names = {"x", "y", "z"};
        static constexpr char *end_effector = "";

        template <std::size_t rake>
        struct Spheres
        {
            FloatVector<rake, 1> x;
            FloatVector<rake, 1> y;
            FloatVector<rake, 1> z;
            FloatVector<rake, 1> r;
        };

        inline static void set_radius(float new_radius) noexcept
        {
            radius = new_radius;
        }

        inline static void set_lows(std::array<float, 3> new_lows) noexcept
        {
            std::copy_n(new_lows.cbegin(), 3, lows.begin());
        }

        inline static void set_highs(std::array<float, 3> new_highs) noexcept
        {
            std::copy_n(new_highs.cbegin(), 3, highs.begin());
        }

        inline static void scale_configuration(Configuration &q) noexcept
        {
            Configuration clow(lows.data());
            Configuration chigh(highs.data());

            q = q * (chigh - clow) + clow;
        }

        inline static void descale_configuration(Configuration &q) noexcept
        {
            Configuration clow(lows.data());
            Configuration chigh(highs.data());

            q = (q - clow) / (chigh - clow);
        }

        template <std::size_t rake>
        inline static void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
        {
            q[0] = lows[0] + (q[0] * (highs[0] - lows[0]));
            q[1] = lows[1] + (q[1] * (highs[1] - lows[1]));
            q[2] = lows[2] + (q[2] * (highs[2] - lows[2]));
        }

        template <std::size_t rake>
        inline static void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept
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

        template <std::size_t rake>
        static auto fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept
        {
            return not sphere_environment_in_collision(environment, q[0], q[1], q[2], radius);
        }

        using Debug = std::
            pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

        template <std::size_t rake>
        static auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept -> Debug
        {
            Debug output;

            output.first.emplace_back(
                sphere_environment_get_collisions<decltype(q[0])>(environment, q[0], q[1], q[2], radius));

            return output;
        }

        template <std::size_t rake>
        static auto fkcc_attach(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &q) noexcept
        {
            return not sphere_environment_in_collision(environment, q[0], q[1], q[2], radius);
        }

        template <std::size_t rake>
        inline static void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
        {
            out.x[0] = q[0];
            out.y[0] = q[1];
            out.z[0] = q[2];
            out.r[0] = radius;
        }

        static auto eefk(const std::array<float, 3> &q) noexcept -> Eigen::Isometry3f
        {
            auto tf = Eigen::Isometry3f::Identity();
            tf.translation() = Eigen::Vector3f(q[0], q[1], q[2]);
            return tf;
        }
    };
}  // namespace vamp::robots
