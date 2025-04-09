#pragma once

#include <vamp/robots/stretch/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Stretch
    {
        static constexpr auto name = "stretch";
        static constexpr auto dimension = 13;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = stretch::n_spheres;
        static constexpr auto space_measure = stretch::space_measure;
        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = stretch::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = stretch::Spheres<rake>;

        static constexpr auto scale_configuration = stretch::scale_configuration;
        static constexpr auto descale_configuration = stretch::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = stretch::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = stretch::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = stretch::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = stretch::interleaved_sphere_fk_attachment<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = stretch::sphere_fk<rake>;

        static constexpr auto eefk = stretch::eefk;
    };
}  // namespace vamp::robots
