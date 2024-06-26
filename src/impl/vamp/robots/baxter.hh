#pragma once

#include <vamp/robots/baxter/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Baxter
    {
        static constexpr auto name = "baxter";
        static constexpr auto dimension = 14;
        static constexpr auto resolution = 64;
        static constexpr auto n_spheres = baxter::n_spheres;
        static constexpr auto space_measure = baxter::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = baxter::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = baxter::Spheres<rake>;

        static constexpr auto scale_configuration = baxter::scale_configuration;
        static constexpr auto descale_configuration = baxter::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = baxter::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = baxter::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = baxter::interleaved_sphere_fk<rake>;

        // TODO: Add attachment support for one of Baxter's end-effectors
        template <std::size_t rake>
        static constexpr auto fkcc_attach = baxter::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = baxter::sphere_fk<rake>;

        // Currently not implemented
        static constexpr auto eefk = baxter::eefk;
    };
}  // namespace vamp::robots
