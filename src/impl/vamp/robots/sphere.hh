#pragma once

#include <vamp/robots/sphere/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Sphere
    {
        static constexpr auto name = "sphere";
        static constexpr auto dimension = 3;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = sphere::n_spheres;
        static constexpr auto space_measure = sphere::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = sphere::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = sphere::Spheres<rake>;

        static constexpr auto scale_configuration = sphere::scale_configuration;
        static constexpr auto descale_configuration = sphere::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = sphere::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = sphere::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = sphere::interleaved_sphere_fk<rake>;

        // Currently not implemented
        template <std::size_t rake>
        static constexpr auto fkcc_attach = sphere::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = sphere::sphere_fk<rake>;

        // Currently not implemented
        static constexpr auto eefk = sphere::eefk;
    };
}  // namespace vamp::robots
