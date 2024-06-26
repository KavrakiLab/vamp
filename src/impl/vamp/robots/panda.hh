#pragma once

#include <vamp/robots/panda/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Panda
    {
        static constexpr auto name = "panda";
        static constexpr auto dimension = 7;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = panda::n_spheres;
        static constexpr auto space_measure = panda::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = panda::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = panda::Spheres<rake>;

        static constexpr auto scale_configuration = panda::scale_configuration;
        static constexpr auto descale_configuration = panda::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = panda::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = panda::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = panda::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = panda::interleaved_sphere_fk_attachment<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = panda::sphere_fk<rake>;

        static constexpr auto eefk = panda::eefk;
    };
}  // namespace vamp::robots
