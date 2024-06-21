#pragma once

#include <vamp/robots/ur5/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct UR5
    {
        static constexpr auto name = "ur5";
        static constexpr auto dimension = 6;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = ur5::n_spheres;
        static constexpr auto space_measure = ur5::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = ur5::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = ur5::Spheres<rake>;

        static constexpr auto scale_configuration = ur5::scale_configuration;
        static constexpr auto descale_configuration = ur5::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = ur5::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = ur5::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = ur5::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = ur5::interleaved_sphere_fk_attachment<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = ur5::sphere_fk<rake>;

        static constexpr auto eefk = ur5::eefk;
    };
}  // namespace vamp::robots
