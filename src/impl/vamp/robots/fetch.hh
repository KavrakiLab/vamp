#pragma once

#include <vamp/robots/fetch/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Fetch
    {
        static constexpr auto name = "fetch";
        static constexpr auto dimension = 8;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = fetch::n_spheres;
        static constexpr auto space_measure = fetch::space_measure;
        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = fetch::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = fetch::Spheres<rake>;

        static constexpr auto scale_configuration = fetch::scale_configuration;
        static constexpr auto descale_configuration = fetch::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = fetch::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = fetch::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = fetch::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = fetch::interleaved_sphere_fk_attachment<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = fetch::sphere_fk<rake>;

        static constexpr auto eefk = fetch::eefk;
    };
}  // namespace vamp::robots
