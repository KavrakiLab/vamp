#pragma once

#include <vamp/robots/panda/fk_attachment.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct PandaAttachment
    {
        static constexpr auto name = "panda_attachment";
        static constexpr auto dimension = 7;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = panda_attachment::n_spheres;
        static constexpr auto space_measure = panda_attachment::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = panda_attachment::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = panda_attachment::Spheres<rake>;

        static constexpr auto scale_configuration = panda_attachment::scale_configuration;
        static constexpr auto descale_configuration = panda_attachment::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = panda_attachment::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block =
            panda_attachment::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = panda_attachment::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = panda_attachment::sphere_fk<rake>;
    };
}  // namespace vamp::robots
