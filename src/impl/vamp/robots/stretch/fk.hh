
#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

namespace vamp::robots::stretch
{
    using Configuration = FloatVector<12>;
    template <std::size_t block_width> using ConfigurationBlock = FloatVector<block_width, 12>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, 12> s_m_a{1.1, 0.13, 0.13, 0.13, 0.13, 5.75, 5.4, 2.3200000000000003, 2.13, 6.28, 0.3, 0.3};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 12> s_a_a{0.0, 0.0, 0.0, 0.0, 0.0, -1.75, -3.9, -1.53, -1.57, -3.14, 0.0, 0.0};
    static const Configuration s_m(s_m_a);
    static const Configuration s_a(s_a_a);inline void scale_configuration(Configuration& q) noexcept { q = q * s_m + s_a; }template <std::size_t block_width> inline void scale_configuration_block(ConfigurationBlock<block_width>& q) noexcept {q[0] = 0.0 + (q[0] * 1.1);
    q[1] = 0.0 + (q[1] * 0.13);
    q[2] = 0.0 + (q[2] * 0.13);
    q[3] = 0.0 + (q[3] * 0.13);
    q[4] = 0.0 + (q[4] * 0.13);
    q[5] = -1.75 + (q[5] * 5.75);
    q[6] = -3.9 + (q[6] * 5.4);
    q[7] = -1.53 + (q[7] * 2.3200000000000003);
    q[8] = -1.57 + (q[8] * 2.13);
    q[9] = -3.14 + (q[9] * 6.28);
    q[10] = 0.0 + (q[10] * 0.3);
    q[11] = 0.0 + (q[11] * 0.3);}

    alignas(Configuration::S::Alignment) constexpr std::array<float, 12> d_m_a{0.9090909090909091, 7.692307692307692, 7.692307692307692, 7.692307692307692, 7.692307692307692, 0.17391304347826086, 0.18518518518518517, 0.43103448275862066, 0.4694835680751174, 0.1592356687898089, 3.3333333333333335, 3.3333333333333335};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 12> d_a_a{0.0, 0.0, 0.0, 0.0, 0.0, -1.75, -3.9, -1.53, -1.57, -3.14, 0.0, 0.0};
    static const Configuration d_m(d_m_a);
    static const Configuration d_a(d_a_a);inline void descale_configuration(Configuration& q) noexcept { q = (q - d_a) * d_m; }template <std::size_t block_width> inline void descale_configuration_block(ConfigurationBlock<block_width>& q) noexcept {q[0] = 0.9090909090909091 * (q[0] - 0.0);
    q[1] = 7.692307692307692 * (q[1] - 0.0);
    q[2] = 7.692307692307692 * (q[2] - 0.0);
    q[3] = 7.692307692307692 * (q[3] - 0.0);
    q[4] = 7.692307692307692 * (q[4] - 0.0);
    q[5] = 0.17391304347826086 * (q[5] - -1.75);
    q[6] = 0.18518518518518517 * (q[6] - -3.9);
    q[7] = 0.43103448275862066 * (q[7] - -1.53);
    q[8] = 0.4694835680751174 * (q[8] - -1.57);
    q[9] = 0.1592356687898089 * (q[9] - -3.14);
    q[10] = 3.3333333333333335 * (q[10] - 0.0);
    q[11] = 3.3333333333333335 * (q[11] - 0.0);}

    //constexpr auto space_measure = 0.2646465106032249;
    inline static auto space_measure() noexcept -> float
    {
        return  0.2646465106032249;
    }

    constexpr auto n_spheres = 139;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, 139> x;
        FloatVector<rake, 139> y;
        FloatVector<rake, 139> z;
        FloatVector<rake, 139> r;
    };

    inline auto eefk(const std::array<float, 12> &q) noexcept -> std::array<float, 12>
    {
    }

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk_attachment(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        return false;
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
    if(/*base_link*/ sphere_environment_in_collision(environment, -0.07, 0.0, 0.092, 0.23)){ if(sphere_environment_in_collision(environment, 0.009, 0.0, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, 0.009, 0.105, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, 0.009, -0.105, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, 0.009, 0.0525, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, 0.009, -0.0525, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, 0.004, -0.13, 0.0944, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, 0.004, 0.13, 0.0944, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.048, -0.13, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.048, 0.13, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.048, -0.07875, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.048, -0.02625, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.048, 0.07875, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.048, 0.02625, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, 0.0, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, -0.0525, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, 0.0525, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, -0.105, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, 0.105, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, -0.13, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.096, 0.13, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.144, -0.13, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.144, 0.13, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.144, -0.07875, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.144, -0.02625, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.144, 0.07875, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.144, 0.02625, 0.092, 0.079)){ return false; }
    if(sphere_environment_in_collision(environment, -0.213, -0.059, 0.095, 0.086)){ return false; }
    if(sphere_environment_in_collision(environment, -0.213, 0.059, 0.095, 0.086)){ return false; }
    if(sphere_environment_in_collision(environment, -0.192, -0.1, 0.095, 0.086)){ return false; }
    if(sphere_environment_in_collision(environment, -0.192, 0.1, 0.095, 0.086)){ return false; }
    if(sphere_environment_in_collision(environment, -0.23, 0.0, 0.095, 0.086)){ return false; } } // (0, 0)
    if(/*caster_link vs. link_head*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.0270003, 0.081995, 1.3984001, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.0384372, 0.129995, 1.3839643, 0.074)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.0021212, 0.0450668, 1.3839642, 0.074)){ return false; } } // (0, 0)
    if(/*caster_link vs. link_mast*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349972, 0.7884, 0.55)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349994, 0.1884, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349992, 0.2584, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349989, 0.3284, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349986, 0.3984, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349984, 0.4684, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349981, 0.5384, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349979, 0.6084, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349976, 0.6784, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349974, 0.7484, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349971, 0.8184, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349968, 0.8884, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349966, 0.9584, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349963, 1.0284, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349961, 1.0984, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349958, 1.1684, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349956, 1.2384, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, -0.067, 0.1349953, 1.3084, 0.045)){ return false; } } // (0, 0)
    if(/*caster_link*/ sphere_environment_in_collision(environment, -0.2449332, 0.0001548, 0.031821, 0.0315)){ return false; } // (0, 0)
    if(/*laser*/ sphere_environment_in_collision(environment, 0.004, 0.0, 0.1584, 0.04)){ return false; } // (0, 0)
    if(/*link_head*/ sphere_environment_in_collision(environment, -0.0270003, 0.081995, 1.3984001, 0.12)){ if(sphere_environment_in_collision(environment, -0.0384372, 0.129995, 1.3839643, 0.074)){ return false; }
    if(sphere_environment_in_collision(environment, -0.0021212, 0.0450668, 1.3839642, 0.074)){ return false; } } // (0, 0)
    if(/*link_left_wheel vs. caster_link*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.2449332, 0.0001548, 0.031821, 0.0315)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.2449332, 0.0001548, 0.031821, 0.0315)){ return false; } } // (0, 0)
    if(/*link_left_wheel vs. link_head*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.0270003, 0.081995, 1.3984001, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.0384372, 0.129995, 1.3839643, 0.074)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.0021212, 0.0450668, 1.3839642, 0.074)){ return false; } } // (0, 0)
    if(/*link_left_wheel vs. link_mast*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349972, 0.7884, 0.55)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349994, 0.1884, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349992, 0.2584, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349989, 0.3284, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349986, 0.3984, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349984, 0.4684, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349981, 0.5384, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349979, 0.6084, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349976, 0.6784, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349974, 0.7484, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349971, 0.8184, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349968, 0.8884, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349966, 0.9584, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349963, 1.0284, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349961, 1.0984, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349958, 1.1684, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349956, 1.2384, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, -0.067, 0.1349953, 1.3084, 0.045)){ return false; } } // (0, 0)
    if(/*link_left_wheel*/ sphere_environment_in_collision(environment, -0.0, 0.15635, 0.0508, 0.052)){ return false; } // (0, 0)
    if(/*link_mast*/ sphere_environment_in_collision(environment, -0.067, 0.1349972, 0.7884, 0.55)){ if(sphere_environment_in_collision(environment, -0.067, 0.1349994, 0.1884, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349992, 0.2584, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349989, 0.3284, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349986, 0.3984, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349984, 0.4684, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349981, 0.5384, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349979, 0.6084, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349976, 0.6784, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349974, 0.7484, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349971, 0.8184, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349968, 0.8884, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349966, 0.9584, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349963, 1.0284, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349961, 1.0984, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349958, 1.1684, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349956, 1.2384, 0.045)){ return false; }
    if(sphere_environment_in_collision(environment, -0.067, 0.1349953, 1.3084, 0.045)){ return false; } } // (0, 0)
    if(/*link_right_wheel vs. caster_link*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.2449332, 0.0001548, 0.031821, 0.0315)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.2449332, 0.0001548, 0.031821, 0.0315)){ return false; } } // (0, 0)
    if(/*link_right_wheel vs. link_head*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.0270003, 0.081995, 1.3984001, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.0384372, 0.129995, 1.3839643, 0.074)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.0021212, 0.0450668, 1.3839642, 0.074)){ return false; } } // (0, 0)
    if(/*link_right_wheel vs. link_left_wheel*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.0, 0.15635, 0.0508, 0.052)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.0, 0.15635, 0.0508, 0.052)){ return false; } } // (0, 0)
    if(/*link_right_wheel vs. link_mast*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349972, 0.7884, 0.55)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349994, 0.1884, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349992, 0.2584, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349989, 0.3284, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349986, 0.3984, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349984, 0.4684, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349981, 0.5384, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349979, 0.6084, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349976, 0.6784, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349974, 0.7484, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349971, 0.8184, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349968, 0.8884, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349966, 0.9584, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349963, 1.0284, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349961, 1.0984, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349958, 1.1684, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349956, 1.2384, 0.045)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, -0.067, 0.1349953, 1.3084, 0.045)){ return false; } } // (0, 0)
    if(/*link_right_wheel*/ sphere_environment_in_collision(environment, 0.0, -0.15635, 0.0507999, 0.052)){ return false; } // (0, 0)
    auto INPUT_0 = q[0];
    auto MUL_21 = INPUT_0 * 2.6e-06;
    auto MUL_35 = MUL_21 * 0.7071055;
    auto MUL_43 = MUL_35 * 2.0;
    auto ADD_70 = MUL_43 + 0.104385;
    auto ADD_6834 = ADD_70 + 0.0329999;
    auto NEGATE_6835 = -ADD_6834;
    auto MUL_51 = MUL_21 * 0.7071081;
    auto MUL_54 = MUL_51 * 2.0;
    auto SUB_73 = 0.1349994 - MUL_54;
    auto SUB_6838 = SUB_73 - 0.0410002;
    auto MUL_62 = MUL_21 * 2.6e-06;
    auto MUL_65 = MUL_62 * 2.0;
    auto SUB_67 = INPUT_0 - MUL_65;
    auto ADD_75 = SUB_67 + 0.195;
    auto ADD_6840 = ADD_75 + 0.0199997;
    auto SUB_6842 = 0.0199999 - ADD_70;
    auto ADD_6845 = ADD_75 + 0.0150001;
    auto ADD_6848 = ADD_70 + 0.0799999;
    auto NEGATE_6849 = -ADD_6848;
    auto SUB_6852 = SUB_73 - 0.0410004;
    auto ADD_6854 = ADD_75 + 0.0199996;
    if(/*caster_link vs. link_lift*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, NEGATE_6835, SUB_6838, ADD_6840, 0.14)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6842, SUB_73, ADD_6845, 0.08)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, NEGATE_6849, SUB_6852, ADD_6854, 0.105)){ return false; } } // (0, 22)
    if(/*link_left_wheel vs. link_lift*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, NEGATE_6835, SUB_6838, ADD_6840, 0.14)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6842, SUB_73, ADD_6845, 0.08)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, NEGATE_6849, SUB_6852, ADD_6854, 0.105)){ return false; } } // (22, 22)
    if(/*link_lift*/ sphere_environment_in_collision(environment, NEGATE_6835, SUB_6838, ADD_6840, 0.14)){ if(sphere_environment_in_collision(environment, SUB_6842, SUB_73, ADD_6845, 0.08)){ return false; }
    if(sphere_environment_in_collision(environment, NEGATE_6849, SUB_6852, ADD_6854, 0.105)){ return false; } } // (22, 22)
    if(/*link_right_wheel vs. link_lift*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, NEGATE_6835, SUB_6838, ADD_6840, 0.14)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6842, SUB_73, ADD_6845, 0.08)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, NEGATE_6849, SUB_6852, ADD_6854, 0.105)){ return false; } } // (22, 22)
    auto SUB_77 = 9e-07 - ADD_70;
    auto SUB_6863 = SUB_77 - 0.0;
    auto SUB_80 = SUB_73 - 0.2547;
    auto ADD_6865 = SUB_80 + 0.11;
    auto SUB_83 = ADD_75 - 9e-07;
    auto ADD_6867 = SUB_83 + 8e-07;
    auto ADD_6872 = SUB_80 + 0.01;
    auto ADD_6874 = SUB_83 + 1e-07;
    auto ADD_6886 = SUB_80 + 0.16;
    auto ADD_6888 = SUB_83 + 1.2e-06;
    auto ADD_6893 = SUB_80 + 0.06;
    auto ADD_6895 = SUB_83 + 4e-07;
    if(/*base_link vs. link_arm_l4*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, SUB_6863, ADD_6865, ADD_6867, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; } } // (22, 34)
    if(/*caster_link vs. link_arm_l4*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6863, ADD_6865, ADD_6867, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; } } // (34, 34)
    if(/*link_arm_l4*/ sphere_environment_in_collision(environment, SUB_6863, ADD_6865, ADD_6867, 0.12)){ if(sphere_environment_in_collision(environment, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; } } // (34, 34)
    if(/*link_left_wheel vs. link_arm_l4*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6863, ADD_6865, ADD_6867, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; } } // (34, 34)
    if(/*link_right_wheel vs. link_arm_l4*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6863, ADD_6865, ADD_6867, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6863, ADD_6872, ADD_6874, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6863, ADD_6865, ADD_6867, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6863, ADD_6886, ADD_6888, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6863, ADD_6893, ADD_6895, 0.04)){ return false; } } // (34, 34)
    auto INPUT_1 = q[1];
    auto MUL_113 = INPUT_1 * 0.7071094;
    auto MUL_103 = INPUT_1 * 1.3e-06;
    auto MUL_123 = MUL_113 * 1.3e-06;
    auto MUL_117 = MUL_103 * 0.7071042;
    auto SUB_124 = MUL_123 - MUL_117;
    auto MUL_126 = SUB_124 * 2.0;
    auto ADD_155 = SUB_77 + MUL_126;
    auto SUB_6908 = ADD_155 - 0.0;
    auto MUL_129 = MUL_113 * 0.7071042;
    auto MUL_134 = MUL_103 * 1.3e-06;
    auto ADD_136 = MUL_129 + MUL_134;
    auto MUL_139 = ADD_136 * 2.0;
    auto SUB_90 = SUB_80 - 0.013;
    auto SUB_156 = SUB_90 - MUL_139;
    auto ADD_6910 = SUB_156 + 0.08;
    auto SUB_93 = SUB_83 - 1e-07;
    auto MUL_144 = MUL_113 * 0.7071094;
    auto ADD_149 = MUL_144 + MUL_134;
    auto MUL_152 = ADD_149 * 2.0;
    auto SUB_154 = INPUT_1 - MUL_152;
    auto ADD_157 = SUB_93 + SUB_154;
    auto ADD_6912 = ADD_157 + 6e-07;
    auto ADD_6917 = SUB_156 + 0.01;
    auto ADD_6919 = ADD_157 + 1e-07;
    auto ADD_6924 = SUB_156 + 0.06;
    auto ADD_6926 = ADD_157 + 4e-07;
    auto ADD_6931 = SUB_156 + 0.11;
    auto ADD_6933 = ADD_157 + 8e-07;
    if(/*base_link vs. link_arm_l3*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, SUB_6908, ADD_6910, ADD_6912, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; } } // (34, 63)
    if(/*caster_link vs. link_arm_l3*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6908, ADD_6910, ADD_6912, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; } } // (63, 63)
    if(/*link_arm_l3*/ sphere_environment_in_collision(environment, SUB_6908, ADD_6910, ADD_6912, 0.087)){ if(sphere_environment_in_collision(environment, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; } } // (63, 63)
    if(/*link_left_wheel vs. link_arm_l3*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6908, ADD_6910, ADD_6912, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; } } // (63, 63)
    if(/*link_right_wheel vs. link_arm_l3*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6908, ADD_6910, ADD_6912, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6908, ADD_6917, ADD_6919, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6908, ADD_6924, ADD_6926, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6908, ADD_6931, ADD_6933, 0.04)){ return false; } } // (63, 63)
    auto INPUT_2 = q[2];
    auto MUL_184 = INPUT_2 * 0.7071094;
    auto MUL_174 = INPUT_2 * 1.3e-06;
    auto MUL_194 = MUL_184 * 1.3e-06;
    auto MUL_188 = MUL_174 * 0.7071042;
    auto SUB_195 = MUL_194 - MUL_188;
    auto MUL_197 = SUB_195 * 2.0;
    auto ADD_226 = ADD_155 + MUL_197;
    auto SUB_6944 = ADD_226 - 0.0;
    auto SUB_161 = SUB_156 - 0.013;
    auto MUL_200 = MUL_184 * 0.7071042;
    auto MUL_205 = MUL_174 * 1.3e-06;
    auto ADD_207 = MUL_200 + MUL_205;
    auto MUL_210 = ADD_207 * 2.0;
    auto SUB_227 = SUB_161 - MUL_210;
    auto ADD_6946 = SUB_227 + 0.08;
    auto SUB_164 = ADD_157 - 1e-07;
    auto MUL_215 = MUL_184 * 0.7071094;
    auto ADD_220 = MUL_215 + MUL_205;
    auto MUL_223 = ADD_220 * 2.0;
    auto SUB_225 = INPUT_2 - MUL_223;
    auto ADD_228 = SUB_164 + SUB_225;
    auto ADD_6948 = ADD_228 + 6e-07;
    auto ADD_6953 = SUB_227 + 0.01;
    auto ADD_6955 = ADD_228 + 1e-07;
    auto ADD_6960 = SUB_227 + 0.06;
    auto ADD_6962 = ADD_228 + 4e-07;
    auto ADD_6967 = SUB_227 + 0.11;
    auto ADD_6969 = ADD_228 + 8e-07;
    if(/*base_link vs. link_arm_l2*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, SUB_6944, ADD_6946, ADD_6948, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; } } // (63, 92)
    if(/*caster_link vs. link_arm_l2*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6944, ADD_6946, ADD_6948, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; } } // (92, 92)
    if(/*link_arm_l2*/ sphere_environment_in_collision(environment, SUB_6944, ADD_6946, ADD_6948, 0.087)){ if(sphere_environment_in_collision(environment, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; } } // (92, 92)
    if(/*link_left_wheel vs. link_arm_l2*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6944, ADD_6946, ADD_6948, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; } } // (92, 92)
    if(/*link_right_wheel vs. link_arm_l2*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6944, ADD_6946, ADD_6948, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6944, ADD_6953, ADD_6955, 0.04)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6944, ADD_6960, ADD_6962, 0.036)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6944, ADD_6967, ADD_6969, 0.036)){ return false; } } // (92, 92)
    auto INPUT_3 = q[3];
    auto MUL_255 = INPUT_3 * 0.7071094;
    auto MUL_245 = INPUT_3 * 1.3e-06;
    auto MUL_259 = MUL_245 * 0.7071042;
    auto MUL_265 = MUL_255 * 1.3e-06;
    auto SUB_266 = MUL_265 - MUL_259;
    auto MUL_268 = SUB_266 * 2.0;
    auto ADD_297 = ADD_226 + MUL_268;
    auto SUB_6980 = ADD_297 - 0.0;
    auto SUB_232 = SUB_227 - 0.013;
    auto MUL_276 = MUL_245 * 1.3e-06;
    auto MUL_271 = MUL_255 * 0.7071042;
    auto ADD_278 = MUL_271 + MUL_276;
    auto MUL_281 = ADD_278 * 2.0;
    auto SUB_298 = SUB_232 - MUL_281;
    auto ADD_6982 = SUB_298 + 0.08;
    auto SUB_235 = ADD_228 - 1e-07;
    auto MUL_286 = MUL_255 * 0.7071094;
    auto ADD_291 = MUL_286 + MUL_276;
    auto MUL_294 = ADD_291 * 2.0;
    auto SUB_296 = INPUT_3 - MUL_294;
    auto ADD_299 = SUB_235 + SUB_296;
    auto ADD_6984 = ADD_299 + 6e-07;
    auto ADD_6989 = SUB_298 + 0.01;
    auto ADD_6991 = ADD_299 + 1e-07;
    auto ADD_6996 = SUB_298 + 0.06;
    auto ADD_6998 = ADD_299 + 4e-07;
    auto ADD_7003 = SUB_298 + 0.11;
    auto ADD_7005 = ADD_299 + 8e-07;
    if(/*base_link vs. link_arm_l1*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, SUB_6980, ADD_6982, ADD_6984, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; } } // (92, 121)
    if(/*caster_link vs. link_arm_l1*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6980, ADD_6982, ADD_6984, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; } } // (121, 121)
    if(/*link_arm_l1*/ sphere_environment_in_collision(environment, SUB_6980, ADD_6982, ADD_6984, 0.087)){ if(sphere_environment_in_collision(environment, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; } } // (121, 121)
    if(/*link_left_wheel vs. link_arm_l1*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6980, ADD_6982, ADD_6984, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; } } // (121, 121)
    if(/*link_right_wheel vs. link_arm_l1*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6980, ADD_6982, ADD_6984, 0.087)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6980, ADD_6989, ADD_6991, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6980, ADD_6996, ADD_6998, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_6980, ADD_7003, ADD_7005, 0.034)){ return false; } } // (121, 121)
    auto INPUT_4 = q[4];
    auto MUL_326 = INPUT_4 * 0.7071094;
    auto MUL_316 = INPUT_4 * 1.3e-06;
    auto MUL_330 = MUL_316 * 0.7071042;
    auto MUL_336 = MUL_326 * 1.3e-06;
    auto SUB_337 = MUL_336 - MUL_330;
    auto MUL_339 = SUB_337 * 2.0;
    auto ADD_368 = SUB_6980 + MUL_339;
    auto ADD_7015 = ADD_368 + 0.03;
    auto MUL_347 = MUL_316 * 1.3e-06;
    auto MUL_342 = MUL_326 * 0.7071042;
    auto ADD_349 = MUL_342 + MUL_347;
    auto MUL_352 = ADD_349 * 2.0;
    auto ADD_304 = SUB_298 + 0.01375;
    auto SUB_369 = ADD_304 - MUL_352;
    auto ADD_7017 = SUB_369 + 0.05;
    auto MUL_357 = MUL_326 * 0.7071094;
    auto ADD_362 = MUL_357 + MUL_347;
    auto MUL_365 = ADD_362 * 2.0;
    auto SUB_367 = INPUT_4 - MUL_365;
    auto ADD_370 = ADD_6991 + SUB_367;
    auto ADD_7019 = ADD_370 + 5e-07;
    auto ADD_7021 = ADD_368 + 0.003;
    auto SUB_7024 = SUB_369 - 0.0;
    auto SUB_7028 = ADD_368 - 0.0;
    auto ADD_7030 = SUB_369 + 0.035;
    auto ADD_7032 = ADD_370 + 3e-07;
    auto ADD_7037 = SUB_369 + 0.075;
    auto ADD_7039 = ADD_370 + 6e-07;
    auto ADD_7041 = ADD_368 + 0.047;
    auto ADD_7043 = SUB_369 + 0.0025;
    auto ADD_7045 = ADD_370 + 2e-07;
    auto ADD_7047 = ADD_368 + 0.07;
    if(/*base_link vs. link_arm_l0*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, ADD_7015, ADD_7017, ADD_7019, 0.09)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; } } // (121, 154)
    if(/*caster_link vs. link_arm_l0*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7015, ADD_7017, ADD_7019, 0.09)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; } } // (154, 154)
    if(/*link_arm_l0*/ sphere_environment_in_collision(environment, ADD_7015, ADD_7017, ADD_7019, 0.09)){ if(sphere_environment_in_collision(environment, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; } } // (154, 154)
    if(/*link_left_wheel vs. link_arm_l0*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7015, ADD_7017, ADD_7019, 0.09)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; } } // (154, 154)
    if(/*link_right_wheel vs. link_arm_l0*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7015, ADD_7017, ADD_7019, 0.09)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7021, SUB_7024, ADD_370, 0.0345)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_7028, ADD_7030, ADD_7032, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, SUB_7028, ADD_7037, ADD_7039, 0.034)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7041, ADD_7043, ADD_7045, 0.042)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7047, SUB_7024, ADD_7032, 0.034)){ return false; } } // (154, 154)
    auto ADD_372 = ADD_368 + 0.0830001;
    auto INPUT_5 = q[5];
    auto DIV_379 = INPUT_5 * 0.5;
    auto SIN_380 = DIV_379.sin();
    auto COS_389 = DIV_379.cos();
    auto MUL_413 = COS_389 * 1.0;
    auto MUL_431 = COS_389 * 3.7e-06;
    auto MUL_436 = COS_389 * 1.8e-06;
    auto MUL_395 = COS_389 * 0.0;
    auto MUL_387 = SIN_380 * 1.0;
    auto MUL_399 = MUL_387 * 1.0;
    auto SUB_400 = MUL_399 - MUL_395;
    auto MUL_446 = MUL_387 * 3.7e-06;
    auto ADD_447 = MUL_436 + MUL_446;
    auto MUL_422 = MUL_387 * 1.8e-06;
    auto SUB_433 = MUL_422 - MUL_431;
    auto MUL_7067 = ADD_447 * SUB_433;
    auto MUL_7066 = SUB_433 * SUB_433;
    auto MUL_7075 = SUB_400 * SUB_433;
    auto MUL_409 = MUL_387 * 0.0;
    auto ADD_415 = MUL_409 + MUL_413;
    auto MUL_7069 = ADD_447 * ADD_415;
    auto ADD_7108 = MUL_7075 + MUL_7069;
    auto MUL_7110 = ADD_7108 * 2.0;
    auto MUL_7182 = MUL_7110 * 0.019001;
    auto MUL_7065 = ADD_415 * ADD_415;
    auto ADD_7078 = MUL_7065 + MUL_7066;
    auto MUL_7081 = ADD_7078 * 2.0;
    auto SUB_7084 = 1.0 - MUL_7081;
    auto MUL_7163 = SUB_7084 * 1e-06;
    auto MUL_7073 = SUB_400 * ADD_415;
    auto SUB_7093 = MUL_7067 - MUL_7073;
    auto MUL_7095 = SUB_7093 * 2.0;
    auto MUL_7171 = MUL_7095 * 1e-06;
    auto SUB_7192 = MUL_7163 - MUL_7171;
    auto SUB_7196 = SUB_7192 - MUL_7182;
    auto ADD_7200 = ADD_372 + SUB_7196;
    auto ADD_7085 = MUL_7073 + MUL_7067;
    auto ADD_374 = SUB_369 + 2e-07;
    auto MUL_7088 = ADD_7085 * 2.0;
    auto MUL_7165 = MUL_7088 * 1e-06;
    auto MUL_7071 = ADD_447 * SUB_400;
    auto MUL_7076 = ADD_415 * SUB_433;
    auto SUB_7111 = MUL_7071 - MUL_7076;
    auto MUL_7113 = SUB_7111 * 2.0;
    auto MUL_7186 = MUL_7113 * 0.019001;
    auto MUL_7070 = SUB_400 * SUB_400;
    auto ADD_7096 = MUL_7066 + MUL_7070;
    auto MUL_7099 = ADD_7096 * 2.0;
    auto SUB_7102 = 1.0 - MUL_7099;
    auto MUL_7175 = SUB_7102 * 1e-06;
    auto ADD_7193 = MUL_7165 + MUL_7175;
    auto ADD_7197 = ADD_7193 + MUL_7186;
    auto SUB_7201 = ADD_374 - ADD_7197;
    auto SUB_7090 = MUL_7075 - MUL_7069;
    auto ADD_7103 = MUL_7076 + MUL_7071;
    auto ADD_7114 = MUL_7065 + MUL_7070;
    auto MUL_7117 = ADD_7114 * 2.0;
    auto SUB_7120 = 1.0 - MUL_7117;
    auto MUL_7190 = SUB_7120 * 0.019001;
    auto MUL_7106 = ADD_7103 * 2.0;
    auto MUL_7179 = MUL_7106 * 1e-06;
    auto MUL_7092 = SUB_7090 * 2.0;
    auto MUL_7168 = MUL_7092 * 1e-06;
    auto ADD_7195 = MUL_7168 + MUL_7179;
    auto SUB_7199 = ADD_7195 - MUL_7190;
    auto SUB_377 = ADD_370 - 0.0307497;
    auto ADD_7202 = SUB_377 + SUB_7199;
    if(/*link_wrist_yaw*/ sphere_environment_in_collision(environment, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } // (154, 222)
    auto INPUT_7 = q[7];
    auto DIV_619 = INPUT_7 * 0.5;
    auto SIN_620 = DIV_619.sin();
    auto COS_626 = DIV_619.cos();
    auto INPUT_6 = q[6];
    auto DIV_453 = INPUT_6 * 0.5;
    auto SIN_454 = DIV_453.sin();
    auto COS_460 = DIV_453.cos();
    auto MUL_498 = COS_460 * 1.0;
    auto MUL_540 = MUL_498 * 0.7071068;
    auto MUL_481 = COS_460 * 3.7e-06;
    auto MUL_466 = COS_460 * 1.8e-06;
    auto MUL_487 = SIN_454 * 1.0;
    auto MUL_535 = MUL_487 * 0.7071068;
    auto MUL_555 = MUL_487 * 0.0277625;
    auto MUL_571 = MUL_487 * 0.0013;
    auto MUL_469 = SIN_454 * 3.7e-06;
    auto SUB_470 = MUL_469 - MUL_466;
    auto MUL_543 = SUB_470 * 0.7071068;
    auto SUB_512 = MUL_543 - MUL_540;
    auto ADD_544 = MUL_540 + MUL_543;
    auto MUL_646 = ADD_544 * COS_626;
    auto MUL_638 = ADD_544 * SIN_620;
    auto MUL_628 = SUB_512 * COS_626;
    auto MUL_633 = SUB_512 * SIN_620;
    auto MUL_567 = SUB_470 * 0.0533108;
    auto ADD_572 = MUL_567 + MUL_571;
    auto MUL_578 = MUL_487 * ADD_572;
    auto MUL_559 = SUB_470 * 0.0277625;
    auto MUL_478 = SIN_454 * 1.8e-06;
    auto ADD_482 = MUL_478 + MUL_481;
    auto MUL_532 = ADD_482 * 0.7071068;
    auto SUB_526 = MUL_532 - MUL_535;
    auto ADD_537 = MUL_532 + MUL_535;
    auto MUL_642 = ADD_537 * COS_626;
    auto ADD_644 = MUL_638 + MUL_642;
    auto MUL_7280 = ADD_644 * ADD_644;
    auto MUL_650 = ADD_537 * SIN_620;
    auto SUB_652 = MUL_650 - MUL_646;
    auto MUL_7281 = SUB_652 * ADD_644;
    auto MUL_635 = SUB_526 * COS_626;
    auto SUB_636 = MUL_635 - MUL_633;
    auto MUL_7283 = SUB_652 * SUB_636;
    auto MUL_7279 = SUB_636 * SUB_636;
    auto ADD_7291 = MUL_7279 + MUL_7280;
    auto MUL_7294 = ADD_7291 * 2.0;
    auto SUB_7297 = 1.0 - MUL_7294;
    auto MUL_7335 = SUB_7297 * 0.0105151;
    auto MUL_629 = SUB_526 * SIN_620;
    auto ADD_630 = MUL_628 + MUL_629;
    auto MUL_7287 = ADD_630 * ADD_644;
    auto SUB_7319 = MUL_7283 - MUL_7287;
    auto MUL_7321 = SUB_7319 * 2.0;
    auto MUL_7354 = MUL_7321 * 0.0318223;
    auto MUL_7286 = ADD_630 * SUB_636;
    auto ADD_7306 = MUL_7286 + MUL_7281;
    auto MUL_7308 = ADD_7306 * 2.0;
    auto MUL_7343 = MUL_7308 * 0.02;
    auto SUB_7360 = MUL_7335 - MUL_7343;
    auto ADD_7364 = SUB_7360 + MUL_7354;
    auto MUL_552 = ADD_482 * 0.0533108;
    auto SUB_557 = MUL_555 - MUL_552;
    auto MUL_574 = MUL_498 * SUB_557;
    auto MUL_562 = ADD_482 * 0.0013;
    auto ADD_564 = MUL_559 + MUL_562;
    auto MUL_576 = ADD_482 * ADD_564;
    auto SUB_577 = MUL_576 - MUL_574;
    auto ADD_579 = SUB_577 + MUL_578;
    auto MUL_581 = ADD_579 * 2.0;
    auto SUB_584 = MUL_581 - 0.0013;
    auto ADD_612 = SUB_584 + 0.0061;
    auto ADD_7367 = ADD_612 + ADD_7364;
    auto SUB_7298 = MUL_7286 - MUL_7281;
    auto MUL_7300 = SUB_7298 * 2.0;
    auto MUL_7337 = MUL_7300 * 0.0105151;
    auto MUL_7285 = SUB_652 * ADD_630;
    auto MUL_7289 = SUB_636 * ADD_644;
    auto ADD_7322 = MUL_7289 + MUL_7285;
    auto MUL_7325 = ADD_7322 * 2.0;
    auto MUL_7356 = MUL_7325 * 0.0318223;
    auto MUL_7284 = ADD_630 * ADD_630;
    auto ADD_7309 = MUL_7280 + MUL_7284;
    auto MUL_7312 = ADD_7309 * 2.0;
    auto SUB_7315 = 1.0 - MUL_7312;
    auto MUL_7347 = SUB_7315 * 0.02;
    auto SUB_7361 = MUL_7337 - MUL_7347;
    auto SUB_7365 = SUB_7361 - MUL_7356;
    auto MUL_585 = MUL_498 * ADD_572;
    auto MUL_590 = MUL_487 * SUB_557;
    auto MUL_587 = SUB_470 * ADD_564;
    auto ADD_588 = MUL_585 + MUL_587;
    auto ADD_592 = ADD_588 + MUL_590;
    auto MUL_595 = ADD_592 * 2.0;
    auto SUB_598 = 0.0277625 - MUL_595;
    auto SUB_615 = SUB_598 - 4.9e-06;
    auto ADD_7368 = SUB_615 + SUB_7365;
    auto SUB_7316 = MUL_7285 - MUL_7289;
    auto ADD_7301 = MUL_7287 + MUL_7283;
    auto ADD_7327 = MUL_7279 + MUL_7284;
    auto MUL_7330 = ADD_7327 * 2.0;
    auto SUB_7333 = 1.0 - MUL_7330;
    auto MUL_7359 = SUB_7333 * 0.0318223;
    auto MUL_7318 = SUB_7316 * 2.0;
    auto MUL_7351 = MUL_7318 * 0.02;
    auto MUL_7304 = ADD_7301 * 2.0;
    auto MUL_7339 = MUL_7304 * 0.0105151;
    auto ADD_7362 = MUL_7339 + MUL_7351;
    auto SUB_7366 = MUL_7359 - ADD_7362;
    auto MUL_599 = MUL_498 * ADD_564;
    auto MUL_604 = ADD_482 * SUB_557;
    auto MUL_601 = SUB_470 * ADD_572;
    auto SUB_603 = MUL_601 - MUL_599;
    auto SUB_605 = SUB_603 - MUL_604;
    auto MUL_607 = SUB_605 * 2.0;
    auto SUB_610 = MUL_607 - 0.0533108;
    auto ADD_617 = SUB_610 + 1.3552038;
    auto ADD_7369 = ADD_617 + SUB_7366;
    auto MUL_7378 = MUL_7308 * 0.0053853;
    auto ADD_7390 = MUL_7335 + MUL_7378;
    auto ADD_7393 = ADD_7390 + MUL_7354;
    auto ADD_7396 = ADD_612 + ADD_7393;
    auto MUL_7380 = SUB_7315 * 0.0053853;
    auto ADD_7391 = MUL_7337 + MUL_7380;
    auto SUB_7394 = ADD_7391 - MUL_7356;
    auto ADD_7397 = SUB_615 + SUB_7394;
    auto MUL_7382 = MUL_7318 * 0.0053853;
    auto SUB_7392 = MUL_7382 - MUL_7339;
    auto ADD_7395 = SUB_7392 + MUL_7359;
    auto ADD_7398 = ADD_617 + ADD_7395;
    auto MUL_7408 = MUL_7308 * 0.0619847;
    auto MUL_7400 = SUB_7297 * 0.0105141;
    auto SUB_7425 = MUL_7400 - MUL_7408;
    auto ADD_7429 = SUB_7425 + MUL_7354;
    auto ADD_7432 = ADD_612 + ADD_7429;
    auto MUL_7412 = SUB_7315 * 0.0619847;
    auto MUL_7402 = MUL_7300 * 0.0105141;
    auto SUB_7426 = MUL_7402 - MUL_7412;
    auto SUB_7430 = SUB_7426 - MUL_7356;
    auto ADD_7433 = SUB_615 + SUB_7430;
    auto MUL_7416 = MUL_7318 * 0.0619847;
    auto MUL_7404 = MUL_7304 * 0.0105141;
    auto ADD_7427 = MUL_7404 + MUL_7416;
    auto SUB_7431 = MUL_7359 - ADD_7427;
    auto ADD_7434 = ADD_617 + SUB_7431;
    if(/*base_link vs. link_head_tilt*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (222, 366)
    if(/*caster_link vs. link_head_tilt*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (366, 366)
    if(/*link_head vs. link_head_tilt*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0270003, 0.081995, 1.3984001, 0.12, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0384372, 0.129995, 1.3839643, 0.074, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0384372, 0.129995, 1.3839643, 0.074, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0021212, 0.0450668, 1.3839642, 0.074, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0021212, 0.0450668, 1.3839642, 0.074, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (366, 366)
    if(/*link_head_tilt*/ sphere_environment_in_collision(environment, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_environment_in_collision(environment, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (366, 366)
    if(/*link_left_wheel vs. link_head_tilt*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (366, 366)
    if(/*link_right_wheel vs. link_head_tilt*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (366, 366)
    auto MUL_2714 = ADD_447 * 0.7071068;
    auto MUL_2722 = SUB_433 * 0.7071068;
    auto SUB_2712 = MUL_2714 - MUL_2722;
    auto ADD_2724 = MUL_2714 + MUL_2722;
    auto MUL_8174 = ADD_2724 * ADD_2724;
    auto MUL_8181 = SUB_2712 * ADD_2724;
    auto MUL_2743 = ADD_415 * 0.7071068;
    auto MUL_2740 = SUB_400 * 0.7071068;
    auto SUB_2733 = MUL_2740 - MUL_2743;
    auto ADD_2745 = MUL_2740 + MUL_2743;
    auto MUL_8177 = ADD_2745 * ADD_2724;
    auto MUL_8176 = ADD_2745 * SUB_2733;
    auto ADD_8199 = MUL_8181 + MUL_8176;
    auto MUL_8202 = ADD_8199 * 2.0;
    auto MUL_8183 = SUB_2712 * SUB_2733;
    auto SUB_8214 = MUL_8183 - MUL_8177;
    auto MUL_8216 = SUB_8214 * 2.0;
    auto MUL_8175 = SUB_2733 * SUB_2733;
    auto ADD_8186 = MUL_8174 + MUL_8175;
    auto MUL_8189 = ADD_8186 * 2.0;
    auto SUB_8192 = 1.0 - MUL_8189;
    auto MUL_8243 = MUL_8202 * 0.048456;
    auto MUL_8231 = SUB_8192 * 0.030127;
    auto SUB_8263 = MUL_8243 - MUL_8231;
    auto MUL_8254 = MUL_8216 * 0.029375;
    auto SUB_8268 = SUB_8263 - MUL_8254;
    auto ADD_8272 = ADD_372 + SUB_8268;
    auto SUB_8193 = MUL_8176 - MUL_8181;
    auto MUL_8195 = SUB_8193 * 2.0;
    auto MUL_8235 = MUL_8195 * 0.030127;
    auto MUL_8180 = ADD_2745 * SUB_2712;
    auto MUL_8184 = ADD_2724 * SUB_2733;
    auto ADD_8217 = MUL_8184 + MUL_8180;
    auto MUL_8220 = ADD_8217 * 2.0;
    auto MUL_8258 = MUL_8220 * 0.029375;
    auto MUL_8179 = SUB_2712 * SUB_2712;
    auto ADD_8204 = MUL_8175 + MUL_8179;
    auto MUL_8207 = ADD_8204 * 2.0;
    auto SUB_8210 = 1.0 - MUL_8207;
    auto MUL_8246 = SUB_8210 * 0.048456;
    auto ADD_8264 = MUL_8235 + MUL_8246;
    auto SUB_8269 = MUL_8258 - ADD_8264;
    auto ADD_8273 = ADD_374 + SUB_8269;
    auto SUB_8211 = MUL_8180 - MUL_8184;
    auto ADD_8196 = MUL_8183 + MUL_8177;
    auto ADD_8222 = MUL_8174 + MUL_8179;
    auto MUL_8225 = ADD_8222 * 2.0;
    auto SUB_8228 = 1.0 - MUL_8225;
    auto MUL_8213 = SUB_8211 * 2.0;
    auto MUL_8250 = MUL_8213 * 0.048456;
    auto MUL_8261 = SUB_8228 * 0.029375;
    auto MUL_8198 = ADD_8196 * 2.0;
    auto MUL_8239 = MUL_8198 * 0.030127;
    auto ADD_8266 = MUL_8239 + MUL_8250;
    auto ADD_8270 = ADD_8266 + MUL_8261;
    auto SUB_8274 = SUB_377 - ADD_8270;
    auto SUB_8314 = SUB_8263 - MUL_8254;
    auto ADD_8318 = ADD_372 + SUB_8314;
    auto SUB_8315 = MUL_8258 - ADD_8264;
    auto ADD_8319 = ADD_374 + SUB_8315;
    auto ADD_8316 = ADD_8266 + MUL_8261;
    auto SUB_8320 = SUB_377 - ADD_8316;
    auto MUL_8335 = MUL_8202 * 0.048455;
    auto MUL_8346 = MUL_8216 * 0.029374;
    auto MUL_8323 = SUB_8192 * 0.004374;
    auto SUB_8355 = MUL_8335 - MUL_8323;
    auto SUB_8360 = SUB_8355 - MUL_8346;
    auto ADD_8364 = ADD_372 + SUB_8360;
    auto MUL_8338 = SUB_8210 * 0.048455;
    auto MUL_8327 = MUL_8195 * 0.004374;
    auto ADD_8356 = MUL_8327 + MUL_8338;
    auto MUL_8350 = MUL_8220 * 0.029374;
    auto SUB_8361 = MUL_8350 - ADD_8356;
    auto ADD_8365 = ADD_374 + SUB_8361;
    auto MUL_8331 = MUL_8198 * 0.004374;
    auto MUL_8342 = MUL_8213 * 0.048455;
    auto ADD_8358 = MUL_8331 + MUL_8342;
    auto MUL_8353 = SUB_8228 * 0.029374;
    auto ADD_8362 = ADD_8358 + MUL_8353;
    auto SUB_8366 = SUB_377 - ADD_8362;
    if(/*link_wrist_yaw_bottom*/ sphere_environment_in_collision(environment, ADD_8272, ADD_8273, SUB_8274, 0.06)){ if(sphere_environment_in_collision(environment, ADD_8318, ADD_8319, SUB_8320, 0.046341)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_8364, ADD_8365, SUB_8366, 0.046342)){ return false; } } // (366, 446)
    auto MUL_2857 = ADD_2745 * 0.7071081;
    auto MUL_2871 = ADD_2745 * 0.7071055;
    auto MUL_2886 = ADD_2745 * 2.6e-06;
    auto MUL_2894 = ADD_2724 * 0.7071081;
    auto MUL_2848 = ADD_2724 * 0.7071055;
    auto MUL_2877 = ADD_2724 * 2.6e-06;
    auto MUL_2873 = SUB_2712 * 0.7071081;
    auto ADD_2874 = MUL_2871 + MUL_2873;
    auto SUB_2878 = ADD_2874 - MUL_2877;
    auto MUL_2859 = SUB_2712 * 0.7071055;
    auto SUB_2860 = MUL_2857 - MUL_2859;
    auto ADD_2864 = SUB_2860 + MUL_2877;
    auto MUL_2890 = SUB_2712 * 2.6e-06;
    auto SUB_2892 = MUL_2890 - MUL_2886;
    auto ADD_2845 = MUL_2886 + MUL_2890;
    auto ADD_2896 = SUB_2892 + MUL_2894;
    auto ADD_2850 = ADD_2845 + MUL_2848;
    auto MUL_2853 = SUB_2733 * 0.7071081;
    auto ADD_2854 = ADD_2850 + MUL_2853;
    auto MUL_2898 = SUB_2733 * 0.7071055;
    auto SUB_2899 = ADD_2896 - MUL_2898;
    auto MUL_2881 = SUB_2733 * 2.6e-06;
    auto SUB_2883 = SUB_2878 - MUL_2881;
    auto SUB_2869 = ADD_2864 - MUL_2881;
    auto MUL_2902 = ADD_2724 * 0.0305;
    auto MUL_2916 = SUB_2712 * 0.0305;
    auto MUL_2923 = SUB_2733 * MUL_2916;
    auto MUL_2910 = SUB_2712 * 0.019455;
    auto MUL_2921 = ADD_2724 * MUL_2910;
    auto MUL_2905 = SUB_2733 * 0.019455;
    auto ADD_2907 = MUL_2902 + MUL_2905;
    auto MUL_2920 = ADD_2745 * ADD_2907;
    auto ADD_2922 = MUL_2920 + MUL_2921;
    auto SUB_2925 = ADD_2922 - MUL_2923;
    auto MUL_2927 = SUB_2925 * 2.0;
    auto ADD_2954 = ADD_372 + MUL_2927;
    auto INPUT_8 = q[8];
    auto DIV_2958 = INPUT_8 * 0.5;
    auto SIN_2959 = DIV_2958.sin();
    auto COS_2968 = DIV_2958.cos();
    auto MUL_2966 = SIN_2959 * 1.0;
    auto MUL_2983 = SUB_2899 * MUL_2966;
    auto MUL_2989 = SUB_2899 * COS_2968;
    auto MUL_2992 = SUB_2883 * MUL_2966;
    auto ADD_2994 = MUL_2989 + MUL_2992;
    auto MUL_2987 = SUB_2883 * COS_2968;
    auto SUB_2988 = MUL_2987 - MUL_2983;
    auto MUL_8375 = ADD_2994 * SUB_2988;
    auto MUL_8374 = SUB_2988 * SUB_2988;
    auto MUL_2972 = SUB_2869 * MUL_2966;
    auto MUL_2980 = SUB_2869 * COS_2968;
    auto MUL_2978 = ADD_2854 * MUL_2966;
    auto SUB_2981 = MUL_2980 - MUL_2978;
    auto MUL_8376 = ADD_2994 * SUB_2981;
    auto MUL_8373 = SUB_2981 * SUB_2981;
    auto ADD_8385 = MUL_8373 + MUL_8374;
    auto MUL_8388 = ADD_8385 * 2.0;
    auto SUB_8391 = 1.0 - MUL_8388;
    auto MUL_8429 = SUB_8391 * 0.0012589;
    auto MUL_2970 = ADD_2854 * COS_2968;
    auto ADD_2974 = MUL_2970 + MUL_2972;
    auto MUL_8382 = ADD_2974 * SUB_2988;
    auto SUB_8415 = MUL_8376 - MUL_8382;
    auto MUL_8417 = SUB_8415 * 2.0;
    auto MUL_8447 = MUL_8417 * 0.018175;
    auto MUL_8380 = ADD_2974 * SUB_2981;
    auto ADD_8400 = MUL_8380 + MUL_8375;
    auto MUL_8403 = ADD_8400 * 2.0;
    auto MUL_8437 = MUL_8403 * 0.015;
    auto ADD_8452 = MUL_8429 + MUL_8437;
    auto ADD_8456 = ADD_8452 + MUL_8447;
    auto ADD_8459 = ADD_2954 + ADD_8456;
    auto SUB_8392 = MUL_8375 - MUL_8380;
    auto MUL_8394 = SUB_8392 * 2.0;
    auto MUL_8431 = MUL_8394 * 0.0012589;
    auto MUL_8378 = ADD_2994 * ADD_2974;
    auto MUL_8384 = SUB_2981 * SUB_2988;
    auto ADD_8418 = MUL_8384 + MUL_8378;
    auto MUL_8420 = ADD_8418 * 2.0;
    auto MUL_8449 = MUL_8420 * 0.018175;
    auto MUL_8377 = ADD_2974 * ADD_2974;
    auto ADD_8405 = MUL_8374 + MUL_8377;
    auto MUL_8408 = ADD_8405 * 2.0;
    auto SUB_8411 = 1.0 - MUL_8408;
    auto MUL_8440 = SUB_8411 * 0.015;
    auto SUB_8453 = MUL_8431 - MUL_8440;
    auto ADD_8457 = SUB_8453 + MUL_8449;
    auto MUL_2930 = ADD_2745 * MUL_2916;
    auto MUL_2931 = SUB_2712 * MUL_2910;
    auto ADD_2933 = MUL_2930 + MUL_2931;
    auto MUL_2934 = SUB_2733 * ADD_2907;
    auto ADD_2935 = ADD_2933 + MUL_2934;
    auto MUL_2937 = ADD_2935 * 2.0;
    auto SUB_2940 = MUL_2937 - 0.019455;
    auto ADD_2955 = ADD_374 + SUB_2940;
    auto ADD_8460 = ADD_2955 + ADD_8457;
    auto SUB_8412 = MUL_8384 - MUL_8378;
    auto ADD_8395 = MUL_8382 + MUL_8376;
    auto ADD_8421 = MUL_8373 + MUL_8377;
    auto MUL_8424 = ADD_8421 * 2.0;
    auto SUB_8427 = 1.0 - MUL_8424;
    auto MUL_8451 = SUB_8427 * 0.018175;
    auto MUL_8414 = SUB_8412 * 2.0;
    auto MUL_8444 = MUL_8414 * 0.015;
    auto MUL_8398 = ADD_8395 * 2.0;
    auto MUL_8433 = MUL_8398 * 0.0012589;
    auto ADD_8454 = MUL_8433 + MUL_8444;
    auto SUB_8458 = MUL_8451 - ADD_8454;
    auto MUL_2941 = ADD_2745 * MUL_2910;
    auto MUL_2946 = ADD_2724 * ADD_2907;
    auto MUL_2943 = SUB_2712 * MUL_2916;
    auto SUB_2945 = MUL_2943 - MUL_2941;
    auto ADD_2948 = SUB_2945 + MUL_2946;
    auto MUL_2950 = ADD_2948 * 2.0;
    auto SUB_2953 = MUL_2950 - 0.0305;
    auto ADD_2956 = SUB_377 + SUB_2953;
    auto ADD_8461 = ADD_2956 + SUB_8458;
    auto MUL_8471 = MUL_8403 * 0.0373123;
    auto ADD_8486 = MUL_8429 + MUL_8471;
    auto ADD_8490 = ADD_8486 + MUL_8447;
    auto ADD_8493 = ADD_2954 + ADD_8490;
    auto MUL_8474 = SUB_8411 * 0.0373123;
    auto SUB_8487 = MUL_8431 - MUL_8474;
    auto ADD_8491 = SUB_8487 + MUL_8449;
    auto ADD_8494 = ADD_2955 + ADD_8491;
    auto MUL_8478 = MUL_8414 * 0.0373123;
    auto ADD_8488 = MUL_8433 + MUL_8478;
    auto SUB_8492 = MUL_8451 - ADD_8488;
    auto ADD_8495 = ADD_2956 + SUB_8492;
    auto MUL_8504 = MUL_8403 * 0.0028977;
    auto MUL_8497 = SUB_8391 * 0.0014159;
    auto SUB_8516 = MUL_8497 - MUL_8504;
    auto ADD_8519 = SUB_8516 + MUL_8447;
    auto ADD_8522 = ADD_2954 + ADD_8519;
    auto MUL_8507 = SUB_8411 * 0.0028977;
    auto MUL_8499 = MUL_8394 * 0.0014159;
    auto ADD_8517 = MUL_8499 + MUL_8507;
    auto ADD_8520 = ADD_8517 + MUL_8449;
    auto ADD_8523 = ADD_2955 + ADD_8520;
    auto MUL_8509 = MUL_8414 * 0.0028977;
    auto MUL_8501 = MUL_8398 * 0.0014159;
    auto SUB_8518 = MUL_8509 - MUL_8501;
    auto ADD_8521 = SUB_8518 + MUL_8451;
    auto ADD_8524 = ADD_2956 + ADD_8521;
    if(/*link_wrist_pitch*/ sphere_environment_in_collision(environment, ADD_8459, ADD_8460, ADD_8461, 0.05)){ if(sphere_environment_in_collision(environment, ADD_8493, ADD_8494, ADD_8495, 0.038801)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_8522, ADD_8523, ADD_8524, 0.039007)){ return false; } } // (446, 590)
    auto MUL_3062 = ADD_2994 * 0.7071081;
    auto MUL_3027 = ADD_2994 * 0.7071055;
    auto MUL_3044 = ADD_2994 * 2.6e-06;
    auto MUL_3091 = SUB_2988 * 0.7071081;
    auto MUL_3058 = SUB_2988 * 0.7071055;
    auto MUL_3039 = SUB_2988 * 2.6e-06;
    auto MUL_3034 = SUB_2981 * 0.7071081;
    auto MUL_3069 = SUB_2981 * 0.7071055;
    auto MUL_3053 = SUB_2981 * 2.6e-06;
    auto MUL_3048 = ADD_2974 * 0.7071081;
    auto ADD_3049 = MUL_3044 + MUL_3048;
    auto ADD_3055 = ADD_3049 + MUL_3053;
    auto SUB_3059 = MUL_3058 - ADD_3055;
    auto MUL_3081 = ADD_2974 * 0.7071055;
    auto SUB_3083 = MUL_3081 - MUL_3044;
    auto ADD_3088 = SUB_3083 + MUL_3053;
    auto ADD_3093 = ADD_3088 + MUL_3091;
    auto MUL_3030 = ADD_2974 * 2.6e-06;
    auto SUB_3067 = MUL_3030 - MUL_3062;
    auto SUB_3070 = SUB_3067 - MUL_3069;
    auto SUB_3075 = SUB_3070 - MUL_3039;
    auto ADD_3031 = MUL_3027 + MUL_3030;
    auto SUB_3036 = ADD_3031 - MUL_3034;
    auto ADD_3041 = SUB_3036 + MUL_3039;
    auto MUL_3098 = SUB_2988 * 0.024;
    auto MUL_3103 = ADD_2974 * 0.024;
    auto MUL_3095 = SUB_2981 * 0.01955;
    auto ADD_3100 = MUL_3095 + MUL_3098;
    auto MUL_3117 = ADD_2994 * ADD_3100;
    auto MUL_3110 = ADD_2974 * 0.01955;
    auto MUL_3114 = SUB_2988 * 0.018859;
    auto SUB_3116 = MUL_3114 - MUL_3110;
    auto MUL_3120 = SUB_2988 * SUB_3116;
    auto MUL_3106 = SUB_2981 * 0.018859;
    auto ADD_3108 = MUL_3103 + MUL_3106;
    auto MUL_3118 = SUB_2981 * ADD_3108;
    auto ADD_3119 = MUL_3117 + MUL_3118;
    auto ADD_3121 = ADD_3119 + MUL_3120;
    auto MUL_3123 = ADD_3121 * 2.0;
    auto SUB_3126 = MUL_3123 - 0.018859;
    auto ADD_3150 = ADD_2954 + SUB_3126;
    auto INPUT_9 = q[9];
    auto DIV_3154 = INPUT_9 * 0.5;
    auto SIN_3155 = DIV_3154.sin();
    auto COS_3161 = DIV_3154.cos();
    auto MUL_3178 = ADD_3093 * COS_3161;
    auto MUL_3173 = ADD_3093 * SIN_3155;
    auto MUL_3176 = SUB_3075 * COS_3161;
    auto ADD_3177 = MUL_3173 + MUL_3176;
    auto MUL_3415 = ADD_3177 * 1.0;
    auto MUL_3181 = SUB_3075 * SIN_3155;
    auto SUB_3182 = MUL_3178 - MUL_3181;
    auto MUL_3399 = SUB_3182 * 1.0;
    auto MUL_8629 = MUL_3415 * MUL_3399;
    auto MUL_8628 = MUL_3399 * MUL_3399;
    auto MUL_3170 = SUB_3059 * COS_3161;
    auto MUL_3164 = SUB_3059 * SIN_3155;
    auto MUL_3163 = ADD_3041 * COS_3161;
    auto ADD_3165 = MUL_3163 + MUL_3164;
    auto MUL_3391 = ADD_3165 * 1.0;
    auto MUL_8631 = MUL_3415 * MUL_3391;
    auto MUL_8627 = MUL_3391 * MUL_3391;
    auto ADD_8640 = MUL_8627 + MUL_8628;
    auto MUL_8643 = ADD_8640 * 2.0;
    auto SUB_8646 = 1.0 - MUL_8643;
    auto MUL_8685 = SUB_8646 * 0.0004704;
    auto MUL_3426 = ADD_3165 * 0.021;
    auto MUL_3431 = ADD_3177 * MUL_3426;
    auto MUL_3168 = ADD_3041 * SIN_3155;
    auto SUB_3171 = MUL_3170 - MUL_3168;
    auto MUL_3383 = SUB_3171 * 1.0;
    auto MUL_8637 = MUL_3383 * MUL_3399;
    auto ADD_8670 = MUL_8637 + MUL_8631;
    auto MUL_8672 = ADD_8670 * 2.0;
    auto MUL_8702 = MUL_8672 * 0.0372672;
    auto MUL_8635 = MUL_3383 * MUL_3391;
    auto SUB_8655 = MUL_8629 - MUL_8635;
    auto MUL_8657 = SUB_8655 * 2.0;
    auto MUL_8695 = MUL_8657 * 0.02;
    auto SUB_8707 = MUL_8695 - MUL_8685;
    auto ADD_8711 = SUB_8707 + MUL_8702;
    auto MUL_3418 = SUB_3171 * 0.021;
    auto MUL_3429 = SUB_3182 * MUL_3418;
    auto ADD_3432 = MUL_3429 + MUL_3431;
    auto MUL_3434 = ADD_3432 * 2.0;
    auto ADD_3456 = ADD_3150 + MUL_3434;
    auto ADD_8714 = ADD_3456 + ADD_8711;
    auto ADD_8647 = MUL_8635 + MUL_8629;
    auto MUL_8650 = ADD_8647 * 2.0;
    auto MUL_8689 = MUL_8650 * 0.0004704;
    auto MUL_8633 = MUL_3415 * MUL_3383;
    auto MUL_8638 = MUL_3391 * MUL_3399;
    auto SUB_8673 = MUL_8633 - MUL_8638;
    auto MUL_8675 = SUB_8673 * 2.0;
    auto MUL_8704 = MUL_8675 * 0.0372672;
    auto MUL_8632 = MUL_3383 * MUL_3383;
    auto ADD_8658 = MUL_8628 + MUL_8632;
    auto MUL_8661 = ADD_8658 * 2.0;
    auto SUB_8664 = 1.0 - MUL_8661;
    auto MUL_8697 = SUB_8664 * 0.02;
    auto ADD_8708 = MUL_8689 + MUL_8697;
    auto ADD_8712 = ADD_8708 + MUL_8704;
    auto MUL_3437 = SUB_3182 * MUL_3426;
    auto MUL_3440 = ADD_3177 * MUL_3418;
    auto SUB_3441 = MUL_3440 - MUL_3437;
    auto MUL_3443 = SUB_3441 * 2.0;
    auto MUL_3128 = ADD_2994 * SUB_3116;
    auto MUL_3133 = SUB_2988 * ADD_3100;
    auto MUL_3130 = ADD_2974 * ADD_3108;
    auto SUB_3132 = MUL_3130 - MUL_3128;
    auto ADD_3134 = SUB_3132 + MUL_3133;
    auto MUL_3136 = ADD_3134 * 2.0;
    auto SUB_3139 = MUL_3136 - 0.024;
    auto ADD_3151 = ADD_2955 + SUB_3139;
    auto ADD_3457 = ADD_3151 + MUL_3443;
    auto ADD_8715 = ADD_3457 + ADD_8712;
    auto SUB_8652 = MUL_8637 - MUL_8631;
    auto ADD_8676 = MUL_8627 + MUL_8632;
    auto ADD_8665 = MUL_8638 + MUL_8633;
    auto MUL_8679 = ADD_8676 * 2.0;
    auto SUB_8682 = 1.0 - MUL_8679;
    auto MUL_8706 = SUB_8682 * 0.0372672;
    auto MUL_8654 = SUB_8652 * 2.0;
    auto MUL_8668 = ADD_8665 * 2.0;
    auto MUL_8699 = MUL_8668 * 0.02;
    auto MUL_8692 = MUL_8654 * 0.0004704;
    auto ADD_8709 = MUL_8692 + MUL_8699;
    auto SUB_8713 = MUL_8706 - ADD_8709;
    auto MUL_3448 = SUB_3171 * MUL_3418;
    auto MUL_3446 = ADD_3165 * MUL_3426;
    auto ADD_3449 = MUL_3446 + MUL_3448;
    auto MUL_3452 = ADD_3449 * 2.0;
    auto SUB_3455 = 0.021 - MUL_3452;
    auto MUL_3140 = ADD_2994 * ADD_3108;
    auto MUL_3144 = SUB_2981 * ADD_3100;
    auto MUL_3141 = ADD_2974 * SUB_3116;
    auto ADD_3143 = MUL_3140 + MUL_3141;
    auto SUB_3145 = ADD_3143 - MUL_3144;
    auto MUL_3147 = SUB_3145 * 2.0;
    auto ADD_3149 = MUL_3147 + 0.01955;
    auto ADD_3152 = ADD_2956 + ADD_3149;
    auto ADD_3458 = ADD_3152 + SUB_3455;
    auto ADD_8716 = ADD_3458 + SUB_8713;
    auto MUL_8725 = MUL_8657 * 0.0568983;
    auto MUL_8732 = MUL_8672 * 0.0401042;
    auto MUL_8718 = SUB_8646 * 0.001;
    auto ADD_8737 = MUL_8718 + MUL_8725;
    auto ADD_8740 = ADD_8737 + MUL_8732;
    auto ADD_8743 = ADD_3456 + ADD_8740;
    auto MUL_8734 = MUL_8675 * 0.0401042;
    auto MUL_8720 = MUL_8650 * 0.001;
    auto MUL_8727 = SUB_8664 * 0.0568983;
    auto SUB_8738 = MUL_8727 - MUL_8720;
    auto ADD_8741 = SUB_8738 + MUL_8734;
    auto ADD_8744 = ADD_3457 + ADD_8741;
    auto MUL_8736 = SUB_8682 * 0.0401042;
    auto MUL_8729 = MUL_8668 * 0.0568983;
    auto MUL_8723 = MUL_8654 * 0.001;
    auto SUB_8739 = MUL_8723 - MUL_8729;
    auto ADD_8742 = SUB_8739 + MUL_8736;
    auto ADD_8745 = ADD_3458 + ADD_8742;
    auto MUL_8759 = MUL_8672 * 0.03;
    auto ADD_8764 = ADD_3456 + MUL_8759;
    auto MUL_8761 = MUL_8675 * 0.03;
    auto ADD_8765 = ADD_3457 + MUL_8761;
    auto MUL_8763 = SUB_8682 * 0.03;
    auto ADD_8766 = ADD_3458 + MUL_8763;
    auto MUL_8780 = MUL_8672 * 0.0699512;
    auto ADD_8785 = ADD_3456 + MUL_8780;
    auto MUL_8782 = MUL_8675 * 0.0699512;
    auto ADD_8786 = ADD_3457 + MUL_8782;
    auto MUL_8784 = SUB_8682 * 0.0699512;
    auto ADD_8787 = ADD_3458 + MUL_8784;
    if(/*link_gripper_s3_body*/ sphere_environment_in_collision(environment, ADD_8714, ADD_8715, ADD_8716, 0.08)){ if(sphere_environment_in_collision(environment, ADD_8743, ADD_8744, ADD_8745, 0.04)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_8764, ADD_8765, ADD_8766, 0.056)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_8785, ADD_8786, ADD_8787, 0.05)){ return false; } } // (590, 763)
    auto MUL_3551 = MUL_3415 * 0.5202904;
    auto MUL_3580 = MUL_3399 * 0.5202904;
    auto MUL_3576 = MUL_3391 * 0.5202904;
    auto MUL_3554 = MUL_3383 * 0.5202904;
    auto SUB_3540 = MUL_3554 - MUL_3551;
    auto ADD_3555 = MUL_3551 + MUL_3554;
    auto MUL_3567 = MUL_3415 * 0.4788507;
    auto MUL_3563 = MUL_3399 * 0.4788507;
    auto MUL_3558 = MUL_3391 * 0.4788507;
    auto SUB_3545 = SUB_3540 - MUL_3558;
    auto ADD_3560 = ADD_3555 + MUL_3558;
    auto ADD_3564 = ADD_3560 + MUL_3563;
    auto ADD_3549 = SUB_3545 + MUL_3563;
    auto MUL_3571 = MUL_3383 * 0.4788507;
    auto SUB_3525 = MUL_3571 - MUL_3567;
    auto ADD_3572 = MUL_3567 + MUL_3571;
    auto SUB_3578 = MUL_3576 - ADD_3572;
    auto ADD_3582 = SUB_3578 + MUL_3580;
    auto ADD_3528 = SUB_3525 + MUL_3576;
    auto SUB_3532 = ADD_3528 - MUL_3580;
    auto MUL_3584 = MUL_3391 * 0.0371909;
    auto MUL_3601 = MUL_3415 * MUL_3584;
    auto MUL_3594 = MUL_3383 * 0.0371909;
    auto MUL_3598 = MUL_3399 * 0.0246929;
    auto ADD_3599 = MUL_3594 + MUL_3598;
    auto MUL_3604 = MUL_3399 * ADD_3599;
    auto MUL_3591 = MUL_3391 * 0.0246929;
    auto MUL_3602 = MUL_3391 * MUL_3591;
    auto ADD_3603 = MUL_3601 + MUL_3602;
    auto ADD_3605 = ADD_3603 + MUL_3604;
    auto MUL_3607 = ADD_3605 * 2.0;
    auto SUB_3610 = MUL_3607 - 0.0246929;
    auto ADD_3631 = ADD_3456 + SUB_3610;
    auto INPUT_10 = q[10];
    auto DIV_3635 = INPUT_10 * 0.5;
    auto SIN_3636 = DIV_3635.sin();
    auto COS_3642 = DIV_3635.cos();
    auto MUL_3654 = ADD_3582 * SIN_3636;
    auto MUL_3657 = ADD_3564 * COS_3642;
    auto ADD_3658 = MUL_3654 + MUL_3657;
    auto MUL_8797 = ADD_3658 * ADD_3658;
    auto MUL_3651 = ADD_3549 * COS_3642;
    auto MUL_3649 = SUB_3532 * SIN_3636;
    auto SUB_3652 = MUL_3651 - MUL_3649;
    auto MUL_8796 = SUB_3652 * SUB_3652;
    auto ADD_8805 = MUL_8796 + MUL_8797;
    auto MUL_8808 = ADD_8805 * 2.0;
    auto SUB_8811 = 1.0 - MUL_8808;
    auto MUL_8846 = SUB_8811 * 0.1;
    auto SUB_8868 = ADD_3631 - MUL_8846;
    auto MUL_3659 = ADD_3582 * COS_3642;
    auto MUL_3662 = ADD_3564 * SIN_3636;
    auto SUB_3663 = MUL_3659 - MUL_3662;
    auto MUL_8798 = SUB_3663 * ADD_3658;
    auto MUL_3645 = ADD_3549 * SIN_3636;
    auto MUL_3644 = SUB_3532 * COS_3642;
    auto ADD_3646 = MUL_3644 + MUL_3645;
    auto MUL_8802 = ADD_3646 * SUB_3652;
    auto ADD_8812 = MUL_8802 + MUL_8798;
    auto MUL_8814 = ADD_8812 * 2.0;
    auto MUL_8850 = MUL_8814 * 0.1;
    auto MUL_3612 = MUL_3415 * ADD_3599;
    auto MUL_3616 = MUL_3399 * MUL_3584;
    auto MUL_3613 = MUL_3383 * MUL_3591;
    auto ADD_3615 = MUL_3612 + MUL_3613;
    auto SUB_3618 = ADD_3615 - MUL_3616;
    auto MUL_3620 = SUB_3618 * 2.0;
    auto ADD_3632 = ADD_3457 + MUL_3620;
    auto SUB_8869 = ADD_3632 - MUL_8850;
    auto MUL_8799 = SUB_3663 * SUB_3652;
    auto MUL_8803 = ADD_3646 * ADD_3658;
    auto SUB_8815 = MUL_8803 - MUL_8799;
    auto MUL_8817 = SUB_8815 * 2.0;
    auto MUL_8854 = MUL_8817 * 0.1;
    auto MUL_3622 = MUL_3415 * MUL_3591;
    auto MUL_3625 = MUL_3391 * MUL_3584;
    auto MUL_3623 = MUL_3383 * ADD_3599;
    auto SUB_3624 = MUL_3622 - MUL_3623;
    auto SUB_3626 = SUB_3624 - MUL_3625;
    auto MUL_3628 = SUB_3626 * 2.0;
    auto ADD_3630 = MUL_3628 + 0.0371909;
    auto ADD_3633 = ADD_3458 + ADD_3630;
    auto SUB_8870 = ADD_3633 - MUL_8854;
    auto SUB_8818 = MUL_8802 - MUL_8798;
    auto MUL_8820 = SUB_8818 * 2.0;
    auto MUL_8885 = MUL_8820 * 0.003;
    auto MUL_8873 = SUB_8811 * 0.067;
    auto ADD_8901 = MUL_8873 + MUL_8885;
    auto SUB_8907 = ADD_3631 - ADD_8901;
    auto MUL_8877 = MUL_8814 * 0.067;
    auto MUL_8800 = ADD_3646 * ADD_3646;
    auto ADD_8821 = MUL_8797 + MUL_8800;
    auto MUL_8824 = ADD_8821 * 2.0;
    auto SUB_8827 = 1.0 - MUL_8824;
    auto MUL_8889 = SUB_8827 * 0.003;
    auto ADD_8903 = MUL_8877 + MUL_8889;
    auto SUB_8908 = ADD_3632 - ADD_8903;
    auto MUL_8881 = MUL_8817 * 0.067;
    auto MUL_8801 = SUB_3663 * ADD_3646;
    auto MUL_8804 = SUB_3652 * ADD_3658;
    auto ADD_8828 = MUL_8804 + MUL_8801;
    auto MUL_8830 = ADD_8828 * 2.0;
    auto MUL_8893 = MUL_8830 * 0.003;
    auto ADD_8905 = MUL_8881 + MUL_8893;
    auto SUB_8909 = ADD_3633 - ADD_8905;
    auto MUL_8912 = SUB_8811 * 0.08;
    auto ADD_8940 = MUL_8912 + MUL_8885;
    auto SUB_8946 = ADD_3631 - ADD_8940;
    auto MUL_8916 = MUL_8814 * 0.08;
    auto ADD_8942 = MUL_8916 + MUL_8889;
    auto SUB_8947 = ADD_3632 - ADD_8942;
    auto MUL_8920 = MUL_8817 * 0.08;
    auto ADD_8944 = MUL_8920 + MUL_8893;
    auto SUB_8948 = ADD_3633 - ADD_8944;
    auto MUL_8951 = SUB_8811 * 0.093;
    auto ADD_8979 = MUL_8951 + MUL_8885;
    auto SUB_8985 = ADD_3631 - ADD_8979;
    auto MUL_8955 = MUL_8814 * 0.093;
    auto ADD_8981 = MUL_8955 + MUL_8889;
    auto SUB_8986 = ADD_3632 - ADD_8981;
    auto MUL_8959 = MUL_8817 * 0.093;
    auto ADD_8983 = MUL_8959 + MUL_8893;
    auto SUB_8987 = ADD_3633 - ADD_8983;
    auto MUL_8990 = SUB_8811 * 0.106;
    auto ADD_9018 = MUL_8990 + MUL_8885;
    auto SUB_9024 = ADD_3631 - ADD_9018;
    auto MUL_8994 = MUL_8814 * 0.106;
    auto ADD_9020 = MUL_8994 + MUL_8889;
    auto SUB_9025 = ADD_3632 - ADD_9020;
    auto MUL_8998 = MUL_8817 * 0.106;
    auto ADD_9022 = MUL_8998 + MUL_8893;
    auto SUB_9026 = ADD_3633 - ADD_9022;
    auto MUL_9029 = SUB_8811 * 0.119;
    auto MUL_9041 = MUL_8820 * 0.002;
    auto ADD_9057 = MUL_9029 + MUL_9041;
    auto SUB_9063 = ADD_3631 - ADD_9057;
    auto MUL_9045 = SUB_8827 * 0.002;
    auto MUL_9033 = MUL_8814 * 0.119;
    auto ADD_9059 = MUL_9033 + MUL_9045;
    auto SUB_9064 = ADD_3632 - ADD_9059;
    auto MUL_9049 = MUL_8830 * 0.002;
    auto MUL_9037 = MUL_8817 * 0.119;
    auto ADD_9061 = MUL_9037 + MUL_9049;
    auto SUB_9065 = ADD_3633 - ADD_9061;
    auto MUL_9068 = SUB_8811 * 0.131;
    auto MUL_9080 = MUL_8820 * 0.0017;
    auto ADD_9096 = MUL_9068 + MUL_9080;
    auto SUB_9102 = ADD_3631 - ADD_9096;
    auto MUL_9084 = SUB_8827 * 0.0017;
    auto MUL_9072 = MUL_8814 * 0.131;
    auto ADD_9098 = MUL_9072 + MUL_9084;
    auto SUB_9103 = ADD_3632 - ADD_9098;
    auto MUL_9088 = MUL_8830 * 0.0017;
    auto MUL_9076 = MUL_8817 * 0.131;
    auto ADD_9100 = MUL_9076 + MUL_9088;
    auto SUB_9104 = ADD_3633 - ADD_9100;
    auto MUL_9107 = SUB_8811 * 0.144;
    auto MUL_9119 = MUL_8820 * 0.0032;
    auto ADD_9135 = MUL_9107 + MUL_9119;
    auto SUB_9141 = ADD_3631 - ADD_9135;
    auto MUL_9123 = SUB_8827 * 0.0032;
    auto MUL_9111 = MUL_8814 * 0.144;
    auto ADD_9137 = MUL_9111 + MUL_9123;
    auto SUB_9142 = ADD_3632 - ADD_9137;
    auto MUL_9127 = MUL_8830 * 0.0032;
    auto MUL_9115 = MUL_8817 * 0.144;
    auto ADD_9139 = MUL_9115 + MUL_9127;
    auto SUB_9143 = ADD_3633 - ADD_9139;
    auto MUL_9146 = SUB_8811 * 0.16;
    auto MUL_9158 = MUL_8820 * 0.009;
    auto ADD_9174 = MUL_9146 + MUL_9158;
    auto SUB_9180 = ADD_3631 - ADD_9174;
    auto MUL_9162 = SUB_8827 * 0.009;
    auto MUL_9150 = MUL_8814 * 0.16;
    auto ADD_9176 = MUL_9150 + MUL_9162;
    auto SUB_9181 = ADD_3632 - ADD_9176;
    auto MUL_9166 = MUL_8830 * 0.009;
    auto MUL_9154 = MUL_8817 * 0.16;
    auto ADD_9178 = MUL_9154 + MUL_9166;
    auto SUB_9182 = ADD_3633 - ADD_9178;
    auto MUL_9185 = SUB_8811 * 0.17;
    auto MUL_9197 = MUL_8820 * 0.011;
    auto ADD_9213 = MUL_9185 + MUL_9197;
    auto SUB_9219 = ADD_3631 - ADD_9213;
    auto MUL_9201 = SUB_8827 * 0.011;
    auto MUL_9189 = MUL_8814 * 0.17;
    auto ADD_9215 = MUL_9189 + MUL_9201;
    auto SUB_9220 = ADD_3632 - ADD_9215;
    auto MUL_9205 = MUL_8830 * 0.011;
    auto MUL_9193 = MUL_8817 * 0.17;
    auto ADD_9217 = MUL_9193 + MUL_9205;
    auto SUB_9221 = ADD_3633 - ADD_9217;
    auto MUL_9236 = MUL_8820 * 0.013;
    auto MUL_9224 = SUB_8811 * 0.178;
    auto ADD_9252 = MUL_9224 + MUL_9236;
    auto SUB_9258 = ADD_3631 - ADD_9252;
    auto MUL_9240 = SUB_8827 * 0.013;
    auto MUL_9228 = MUL_8814 * 0.178;
    auto ADD_9254 = MUL_9228 + MUL_9240;
    auto SUB_9259 = ADD_3632 - ADD_9254;
    auto MUL_9244 = MUL_8830 * 0.013;
    auto MUL_9232 = MUL_8817 * 0.178;
    auto ADD_9256 = MUL_9232 + MUL_9244;
    auto SUB_9260 = ADD_3633 - ADD_9256;
    if(/*link_gripper_finger_right*/ sphere_environment_in_collision(environment, SUB_8868, SUB_8869, SUB_8870, 0.062)){ if(sphere_environment_in_collision(environment, SUB_8907, SUB_8908, SUB_8909, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_8946, SUB_8947, SUB_8948, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_8985, SUB_8986, SUB_8987, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9024, SUB_9025, SUB_9026, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9063, SUB_9064, SUB_9065, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9102, SUB_9103, SUB_9104, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9141, SUB_9142, SUB_9143, 0.016)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9180, SUB_9181, SUB_9182, 0.014)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9219, SUB_9220, SUB_9221, 0.013)){ return false; }
    if(sphere_environment_in_collision(environment, SUB_9258, SUB_9259, SUB_9260, 0.012)){ return false; } } // (763, 967)
    auto MUL_3742 = SUB_3663 * 0.7010574;
    auto MUL_3723 = ADD_3658 * 0.7010574;
    auto MUL_3719 = SUB_3652 * 0.7010574;
    auto MUL_3745 = ADD_3646 * 0.7010574;
    auto SUB_3701 = MUL_3745 - MUL_3742;
    auto ADD_3747 = MUL_3742 + MUL_3745;
    auto MUL_3775 = ADD_3658 * 0.1777845;
    auto MUL_3780 = ADD_3658 * MUL_3775;
    auto MUL_3768 = SUB_3652 * 0.1777845;
    auto MUL_3712 = SUB_3663 * 0.092296;
    auto MUL_3754 = ADD_3658 * 0.092296;
    auto MUL_3750 = SUB_3652 * 0.092296;
    auto ADD_3752 = ADD_3747 + MUL_3750;
    auto SUB_3755 = ADD_3752 - MUL_3754;
    auto ADD_3704 = SUB_3701 + MUL_3750;
    auto ADD_3709 = ADD_3704 + MUL_3754;
    auto MUL_3715 = ADD_3646 * 0.092296;
    auto SUB_3732 = MUL_3712 - MUL_3715;
    auto ADD_3737 = SUB_3732 + MUL_3719;
    auto ADD_3740 = ADD_3737 + MUL_3723;
    auto ADD_3716 = MUL_3712 + MUL_3715;
    auto SUB_3720 = MUL_3719 - ADD_3716;
    auto SUB_3725 = SUB_3720 - MUL_3723;
    auto MUL_9286 = SUB_3755 * SUB_3725;
    auto MUL_9284 = ADD_3740 * ADD_3740;
    auto MUL_9283 = SUB_3725 * SUB_3725;
    auto ADD_9292 = MUL_9283 + MUL_9284;
    auto MUL_9295 = ADD_9292 * 2.0;
    auto SUB_9298 = 1.0 - MUL_9295;
    auto MUL_9332 = SUB_9298 * 1e-06;
    auto MUL_9290 = ADD_3709 * ADD_3740;
    auto ADD_9318 = MUL_9290 + MUL_9286;
    auto MUL_9320 = ADD_9318 * 2.0;
    auto MUL_3760 = ADD_3658 * 0.0106722;
    auto MUL_3777 = SUB_3663 * MUL_3760;
    auto MUL_3764 = ADD_3646 * 0.0106722;
    auto SUB_3770 = MUL_3768 - MUL_3764;
    auto MUL_3778 = SUB_3652 * SUB_3770;
    auto ADD_3779 = MUL_3777 + MUL_3778;
    auto ADD_3781 = ADD_3779 + MUL_3780;
    auto MUL_3783 = ADD_3781 * 2.0;
    auto SUB_3786 = MUL_3783 - 0.1777845;
    auto ADD_3808 = ADD_3631 + SUB_3786;
    auto MUL_9344 = MUL_9320 * 0.0085;
    auto ADD_9349 = MUL_9332 + MUL_9344;
    auto ADD_9352 = ADD_3808 + ADD_9349;
    auto MUL_9285 = SUB_3755 * ADD_3740;
    auto MUL_9288 = SUB_3755 * ADD_3709;
    auto MUL_9291 = SUB_3725 * ADD_3740;
    auto SUB_9321 = MUL_9291 - MUL_9288;
    auto MUL_9323 = SUB_9321 * 2.0;
    auto MUL_9346 = MUL_9323 * 0.0085;
    auto MUL_9289 = ADD_3709 * SUB_3725;
    auto ADD_9299 = MUL_9289 + MUL_9285;
    auto MUL_9301 = ADD_9299 * 2.0;
    auto MUL_9334 = MUL_9301 * 1e-06;
    auto ADD_9350 = MUL_9334 + MUL_9346;
    auto MUL_3788 = SUB_3663 * MUL_3775;
    auto MUL_3793 = ADD_3658 * MUL_3760;
    auto MUL_3790 = ADD_3646 * SUB_3770;
    auto ADD_3791 = MUL_3788 + MUL_3790;
    auto SUB_3794 = MUL_3793 - ADD_3791;
    auto MUL_3796 = SUB_3794 * 2.0;
    auto SUB_3799 = MUL_3796 - 0.0106722;
    auto ADD_3809 = ADD_3632 + SUB_3799;
    auto ADD_9353 = ADD_3809 + ADD_9350;
    auto SUB_9302 = MUL_9290 - MUL_9286;
    auto MUL_9304 = SUB_9302 * 2.0;
    auto MUL_9336 = MUL_9304 * 1e-06;
    auto MUL_9287 = ADD_3709 * ADD_3709;
    auto ADD_9324 = MUL_9283 + MUL_9287;
    auto MUL_9327 = ADD_9324 * 2.0;
    auto SUB_9330 = 1.0 - MUL_9327;
    auto MUL_9348 = SUB_9330 * 0.0085;
    auto ADD_9351 = MUL_9336 + MUL_9348;
    auto MUL_3800 = SUB_3663 * SUB_3770;
    auto MUL_3803 = SUB_3652 * MUL_3760;
    auto MUL_3801 = ADD_3646 * MUL_3775;
    auto SUB_3802 = MUL_3800 - MUL_3801;
    auto SUB_3804 = SUB_3802 - MUL_3803;
    auto MUL_3806 = SUB_3804 * 2.0;
    auto ADD_3810 = ADD_3633 + MUL_3806;
    auto ADD_9354 = ADD_3810 + ADD_9351;
    auto MUL_9368 = MUL_9320 * 0.004;
    auto ADD_9373 = ADD_3808 + MUL_9368;
    auto MUL_9370 = MUL_9323 * 0.004;
    auto ADD_9374 = ADD_3809 + MUL_9370;
    auto MUL_9372 = SUB_9330 * 0.004;
    auto ADD_9375 = ADD_3810 + MUL_9372;
    auto SUB_9305 = MUL_9289 - MUL_9285;
    auto MUL_9307 = SUB_9305 * 2.0;
    auto MUL_9384 = MUL_9307 * 0.017;
    auto MUL_9395 = MUL_9320 * 0.0115;
    auto SUB_9400 = MUL_9395 - MUL_9384;
    auto ADD_9403 = ADD_3808 + SUB_9400;
    auto ADD_9308 = MUL_9284 + MUL_9287;
    auto MUL_9397 = MUL_9323 * 0.0115;
    auto MUL_9311 = ADD_9308 * 2.0;
    auto SUB_9314 = 1.0 - MUL_9311;
    auto MUL_9388 = SUB_9314 * 0.017;
    auto SUB_9401 = MUL_9397 - MUL_9388;
    auto ADD_9404 = ADD_3809 + SUB_9401;
    auto ADD_9315 = MUL_9291 + MUL_9288;
    auto MUL_9399 = SUB_9330 * 0.0115;
    auto MUL_9317 = ADD_9315 * 2.0;
    auto MUL_9392 = MUL_9317 * 0.017;
    auto SUB_9402 = MUL_9399 - MUL_9392;
    auto ADD_9405 = ADD_3810 + SUB_9402;
    auto ADD_9424 = MUL_9384 + MUL_9395;
    auto ADD_9427 = ADD_3808 + ADD_9424;
    auto ADD_9425 = MUL_9388 + MUL_9397;
    auto ADD_9428 = ADD_3809 + ADD_9425;
    auto ADD_9426 = MUL_9392 + MUL_9399;
    auto ADD_9429 = ADD_3810 + ADD_9426;
    auto MUL_9431 = SUB_9298 * 0.017;
    auto ADD_9448 = MUL_9431 + MUL_9395;
    auto ADD_9451 = ADD_3808 + ADD_9448;
    auto MUL_9433 = MUL_9301 * 0.017;
    auto ADD_9449 = MUL_9433 + MUL_9397;
    auto ADD_9452 = ADD_3809 + ADD_9449;
    auto MUL_9435 = MUL_9304 * 0.017;
    auto ADD_9450 = MUL_9435 + MUL_9399;
    auto ADD_9453 = ADD_3810 + ADD_9450;
    auto SUB_9478 = MUL_9395 - MUL_9431;
    auto ADD_9481 = ADD_3808 + SUB_9478;
    auto SUB_9479 = MUL_9397 - MUL_9433;
    auto ADD_9482 = ADD_3809 + SUB_9479;
    auto SUB_9480 = MUL_9399 - MUL_9435;
    auto ADD_9483 = ADD_3810 + SUB_9480;
    auto MUL_9492 = MUL_9307 * 0.012;
    auto MUL_9485 = SUB_9298 * 0.012;
    auto SUB_9508 = MUL_9485 - MUL_9492;
    auto ADD_9511 = SUB_9508 + MUL_9395;
    auto ADD_9514 = ADD_3808 + ADD_9511;
    auto MUL_9496 = SUB_9314 * 0.012;
    auto MUL_9487 = MUL_9301 * 0.012;
    auto SUB_9509 = MUL_9487 - MUL_9496;
    auto ADD_9512 = SUB_9509 + MUL_9397;
    auto ADD_9515 = ADD_3809 + ADD_9512;
    auto MUL_9500 = MUL_9317 * 0.012;
    auto MUL_9489 = MUL_9304 * 0.012;
    auto SUB_9510 = MUL_9489 - MUL_9500;
    auto ADD_9513 = SUB_9510 + MUL_9399;
    auto ADD_9516 = ADD_3810 + ADD_9513;
    auto ADD_9535 = MUL_9485 + MUL_9492;
    auto ADD_9538 = ADD_9535 + MUL_9395;
    auto ADD_9541 = ADD_3808 + ADD_9538;
    auto ADD_9536 = MUL_9487 + MUL_9496;
    auto ADD_9539 = ADD_9536 + MUL_9397;
    auto ADD_9542 = ADD_3809 + ADD_9539;
    auto ADD_9537 = MUL_9489 + MUL_9500;
    auto ADD_9540 = ADD_9537 + MUL_9399;
    auto ADD_9543 = ADD_3810 + ADD_9540;
    auto SUB_9568 = MUL_9492 - MUL_9485;
    auto ADD_9571 = SUB_9568 + MUL_9395;
    auto ADD_9574 = ADD_3808 + ADD_9571;
    auto SUB_9569 = MUL_9496 - MUL_9487;
    auto ADD_9572 = SUB_9569 + MUL_9397;
    auto ADD_9575 = ADD_3809 + ADD_9572;
    auto SUB_9570 = MUL_9500 - MUL_9489;
    auto ADD_9573 = SUB_9570 + MUL_9399;
    auto ADD_9576 = ADD_3810 + ADD_9573;
    auto SUB_9613 = MUL_9395 - ADD_9535;
    auto ADD_9616 = ADD_3808 + SUB_9613;
    auto SUB_9614 = MUL_9397 - ADD_9536;
    auto ADD_9617 = ADD_3809 + SUB_9614;
    auto SUB_9615 = MUL_9399 - ADD_9537;
    auto ADD_9618 = ADD_3810 + SUB_9615;
    auto MUL_9627 = MUL_9307 * 0.0157;
    auto MUL_9620 = SUB_9298 * 0.0065;
    auto SUB_9643 = MUL_9620 - MUL_9627;
    auto ADD_9646 = SUB_9643 + MUL_9395;
    auto ADD_9649 = ADD_3808 + ADD_9646;
    auto MUL_9631 = SUB_9314 * 0.0157;
    auto MUL_9622 = MUL_9301 * 0.0065;
    auto SUB_9644 = MUL_9622 - MUL_9631;
    auto ADD_9647 = SUB_9644 + MUL_9397;
    auto ADD_9650 = ADD_3809 + ADD_9647;
    auto MUL_9635 = MUL_9317 * 0.0157;
    auto MUL_9624 = MUL_9304 * 0.0065;
    auto SUB_9645 = MUL_9624 - MUL_9635;
    auto ADD_9648 = SUB_9645 + MUL_9399;
    auto ADD_9651 = ADD_3810 + ADD_9648;
    auto ADD_9670 = MUL_9620 + MUL_9627;
    auto ADD_9673 = ADD_9670 + MUL_9395;
    auto ADD_9676 = ADD_3808 + ADD_9673;
    auto ADD_9671 = MUL_9622 + MUL_9631;
    auto ADD_9674 = ADD_9671 + MUL_9397;
    auto ADD_9677 = ADD_3809 + ADD_9674;
    auto ADD_9672 = MUL_9624 + MUL_9635;
    auto ADD_9675 = ADD_9672 + MUL_9399;
    auto ADD_9678 = ADD_3810 + ADD_9675;
    auto SUB_9703 = MUL_9627 - MUL_9620;
    auto ADD_9706 = SUB_9703 + MUL_9395;
    auto ADD_9709 = ADD_3808 + ADD_9706;
    auto SUB_9704 = MUL_9631 - MUL_9622;
    auto ADD_9707 = SUB_9704 + MUL_9397;
    auto ADD_9710 = ADD_3809 + ADD_9707;
    auto SUB_9705 = MUL_9635 - MUL_9624;
    auto ADD_9708 = SUB_9705 + MUL_9399;
    auto ADD_9711 = ADD_3810 + ADD_9708;
    auto SUB_9748 = MUL_9395 - ADD_9670;
    auto ADD_9751 = ADD_3808 + SUB_9748;
    auto SUB_9749 = MUL_9397 - ADD_9671;
    auto ADD_9752 = ADD_3809 + SUB_9749;
    auto SUB_9750 = MUL_9399 - ADD_9672;
    auto ADD_9753 = ADD_3810 + SUB_9750;
    auto MUL_9762 = MUL_9307 * 0.0065;
    auto MUL_9755 = SUB_9298 * 0.0157;
    auto SUB_9778 = MUL_9755 - MUL_9762;
    auto ADD_9781 = SUB_9778 + MUL_9395;
    auto ADD_9784 = ADD_3808 + ADD_9781;
    auto MUL_9766 = SUB_9314 * 0.0065;
    auto MUL_9757 = MUL_9301 * 0.0157;
    auto SUB_9779 = MUL_9757 - MUL_9766;
    auto ADD_9782 = SUB_9779 + MUL_9397;
    auto ADD_9785 = ADD_3809 + ADD_9782;
    auto MUL_9770 = MUL_9317 * 0.0065;
    auto MUL_9759 = MUL_9304 * 0.0157;
    auto SUB_9780 = MUL_9759 - MUL_9770;
    auto ADD_9783 = SUB_9780 + MUL_9399;
    auto ADD_9786 = ADD_3810 + ADD_9783;
    auto ADD_9805 = MUL_9755 + MUL_9762;
    auto ADD_9808 = ADD_9805 + MUL_9395;
    auto ADD_9811 = ADD_3808 + ADD_9808;
    auto ADD_9806 = MUL_9757 + MUL_9766;
    auto ADD_9809 = ADD_9806 + MUL_9397;
    auto ADD_9812 = ADD_3809 + ADD_9809;
    auto ADD_9807 = MUL_9759 + MUL_9770;
    auto ADD_9810 = ADD_9807 + MUL_9399;
    auto ADD_9813 = ADD_3810 + ADD_9810;
    auto SUB_9838 = MUL_9762 - MUL_9755;
    auto ADD_9841 = SUB_9838 + MUL_9395;
    auto ADD_9844 = ADD_3808 + ADD_9841;
    auto SUB_9839 = MUL_9766 - MUL_9757;
    auto ADD_9842 = SUB_9839 + MUL_9397;
    auto ADD_9845 = ADD_3809 + ADD_9842;
    auto SUB_9840 = MUL_9770 - MUL_9759;
    auto ADD_9843 = SUB_9840 + MUL_9399;
    auto ADD_9846 = ADD_3810 + ADD_9843;
    auto SUB_9883 = MUL_9395 - ADD_9805;
    auto ADD_9886 = ADD_3808 + SUB_9883;
    auto SUB_9884 = MUL_9397 - ADD_9806;
    auto ADD_9887 = ADD_3809 + SUB_9884;
    auto SUB_9885 = MUL_9399 - ADD_9807;
    auto ADD_9888 = ADD_3810 + SUB_9885;
    if(/*link_gripper_fingertip_right*/ sphere_environment_in_collision(environment, ADD_9352, ADD_9353, ADD_9354, 0.025)){ if(sphere_environment_in_collision(environment, ADD_9373, ADD_9374, ADD_9375, 0.014)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9403, ADD_9404, ADD_9405, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9427, ADD_9428, ADD_9429, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9451, ADD_9452, ADD_9453, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9481, ADD_9482, ADD_9483, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9514, ADD_9515, ADD_9516, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9541, ADD_9542, ADD_9543, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9574, ADD_9575, ADD_9576, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9616, ADD_9617, ADD_9618, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9649, ADD_9650, ADD_9651, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9676, ADD_9677, ADD_9678, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9709, ADD_9710, ADD_9711, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9751, ADD_9752, ADD_9753, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9784, ADD_9785, ADD_9786, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9811, ADD_9812, ADD_9813, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9844, ADD_9845, ADD_9846, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_9886, ADD_9887, ADD_9888, 0.006)){ return false; } } // (967, 1213)
    auto SUB_4064 = SUB_3525 - MUL_3576;
    auto SUB_4097 = MUL_3558 - ADD_3555;
    auto SUB_4074 = MUL_3551 - MUL_3554;
    auto SUB_4079 = SUB_4074 - MUL_3558;
    auto ADD_4113 = ADD_3572 + MUL_3576;
    auto ADD_4118 = ADD_4113 + MUL_3580;
    auto ADD_4101 = SUB_4097 + MUL_3563;
    auto ADD_4083 = SUB_4079 + MUL_3563;
    auto ADD_4068 = SUB_4064 + MUL_3580;
    auto MUL_4133 = MUL_3399 * 0.0245029;
    auto SUB_4135 = MUL_4133 - MUL_3594;
    auto MUL_4140 = MUL_3399 * SUB_4135;
    auto MUL_4127 = MUL_3391 * 0.0245029;
    auto MUL_4137 = MUL_3391 * MUL_4127;
    auto SUB_4139 = MUL_3601 - MUL_4137;
    auto SUB_4142 = SUB_4139 - MUL_4140;
    auto MUL_4144 = SUB_4142 * 2.0;
    auto ADD_4146 = MUL_4144 + 0.0245029;
    auto ADD_4172 = ADD_3456 + ADD_4146;
    auto INPUT_11 = q[11];
    auto DIV_4176 = INPUT_11 * 0.5;
    auto SIN_4177 = DIV_4176.sin();
    auto COS_4186 = DIV_4176.cos();
    auto MUL_4184 = SIN_4177 * 1.0;
    auto MUL_4199 = ADD_4118 * MUL_4184;
    auto MUL_4202 = ADD_4101 * COS_4186;
    auto ADD_4203 = MUL_4199 + MUL_4202;
    auto MUL_9974 = ADD_4203 * ADD_4203;
    auto MUL_4196 = ADD_4083 * COS_4186;
    auto MUL_4194 = ADD_4068 * MUL_4184;
    auto ADD_4197 = MUL_4194 + MUL_4196;
    auto MUL_9973 = ADD_4197 * ADD_4197;
    auto ADD_9982 = MUL_9973 + MUL_9974;
    auto MUL_9985 = ADD_9982 * 2.0;
    auto SUB_9988 = 1.0 - MUL_9985;
    auto MUL_10022 = SUB_9988 * 0.1;
    auto ADD_10039 = ADD_4172 + MUL_10022;
    auto MUL_4204 = ADD_4118 * COS_4186;
    auto MUL_4208 = ADD_4101 * MUL_4184;
    auto SUB_4210 = MUL_4208 - MUL_4204;
    auto MUL_9975 = SUB_4210 * ADD_4203;
    auto MUL_4189 = ADD_4083 * MUL_4184;
    auto MUL_4188 = ADD_4068 * COS_4186;
    auto SUB_4191 = MUL_4188 - MUL_4189;
    auto MUL_9979 = SUB_4191 * ADD_4197;
    auto ADD_9989 = MUL_9979 + MUL_9975;
    auto MUL_9991 = ADD_9989 * 2.0;
    auto MUL_10024 = MUL_9991 * 0.1;
    auto MUL_4148 = MUL_3415 * SUB_4135;
    auto MUL_4150 = MUL_3383 * MUL_4127;
    auto ADD_4151 = MUL_4148 + MUL_4150;
    auto ADD_4155 = ADD_4151 + MUL_3616;
    auto MUL_4158 = ADD_4155 * 2.0;
    auto SUB_4173 = ADD_3457 - MUL_4158;
    auto ADD_10040 = SUB_4173 + MUL_10024;
    auto MUL_9976 = SUB_4210 * ADD_4197;
    auto MUL_9980 = SUB_4191 * ADD_4203;
    auto SUB_9992 = MUL_9980 - MUL_9976;
    auto MUL_9994 = SUB_9992 * 2.0;
    auto MUL_10026 = MUL_9994 * 0.1;
    auto MUL_4161 = MUL_3415 * MUL_4127;
    auto MUL_4163 = MUL_3383 * SUB_4135;
    auto SUB_4165 = MUL_4163 - MUL_4161;
    auto SUB_4167 = SUB_4165 - MUL_3625;
    auto MUL_4169 = SUB_4167 * 2.0;
    auto ADD_4171 = MUL_4169 + 0.0371909;
    auto ADD_4174 = ADD_3458 + ADD_4171;
    auto ADD_10041 = ADD_4174 + MUL_10026;
    auto SUB_9995 = MUL_9979 - MUL_9975;
    auto MUL_9997 = SUB_9995 * 2.0;
    auto MUL_10050 = MUL_9997 * 0.003;
    auto MUL_10043 = SUB_9988 * 0.067;
    auto SUB_10066 = MUL_10043 - MUL_10050;
    auto ADD_10069 = ADD_4172 + SUB_10066;
    auto MUL_10045 = MUL_9991 * 0.067;
    auto MUL_9977 = SUB_4191 * SUB_4191;
    auto ADD_9998 = MUL_9974 + MUL_9977;
    auto MUL_10001 = ADD_9998 * 2.0;
    auto SUB_10004 = 1.0 - MUL_10001;
    auto MUL_10054 = SUB_10004 * 0.003;
    auto SUB_10067 = MUL_10045 - MUL_10054;
    auto ADD_10070 = SUB_4173 + SUB_10067;
    auto MUL_10047 = MUL_9994 * 0.067;
    auto MUL_9978 = SUB_4210 * SUB_4191;
    auto MUL_9981 = ADD_4197 * ADD_4203;
    auto ADD_10005 = MUL_9981 + MUL_9978;
    auto MUL_10007 = ADD_10005 * 2.0;
    auto MUL_10058 = MUL_10007 * 0.003;
    auto SUB_10068 = MUL_10047 - MUL_10058;
    auto ADD_10071 = ADD_4174 + SUB_10068;
    auto MUL_10073 = SUB_9988 * 0.08;
    auto SUB_10096 = MUL_10073 - MUL_10050;
    auto ADD_10099 = ADD_4172 + SUB_10096;
    auto MUL_10075 = MUL_9991 * 0.08;
    auto SUB_10097 = MUL_10075 - MUL_10054;
    auto ADD_10100 = SUB_4173 + SUB_10097;
    auto MUL_10077 = MUL_9994 * 0.08;
    auto SUB_10098 = MUL_10077 - MUL_10058;
    auto ADD_10101 = ADD_4174 + SUB_10098;
    auto MUL_10103 = SUB_9988 * 0.093;
    auto SUB_10126 = MUL_10103 - MUL_10050;
    auto ADD_10129 = ADD_4172 + SUB_10126;
    auto MUL_10105 = MUL_9991 * 0.093;
    auto SUB_10127 = MUL_10105 - MUL_10054;
    auto ADD_10130 = SUB_4173 + SUB_10127;
    auto MUL_10107 = MUL_9994 * 0.093;
    auto SUB_10128 = MUL_10107 - MUL_10058;
    auto ADD_10131 = ADD_4174 + SUB_10128;
    auto MUL_10133 = SUB_9988 * 0.106;
    auto SUB_10156 = MUL_10133 - MUL_10050;
    auto ADD_10159 = ADD_4172 + SUB_10156;
    auto MUL_10135 = MUL_9991 * 0.106;
    auto SUB_10157 = MUL_10135 - MUL_10054;
    auto ADD_10160 = SUB_4173 + SUB_10157;
    auto MUL_10137 = MUL_9994 * 0.106;
    auto SUB_10158 = MUL_10137 - MUL_10058;
    auto ADD_10161 = ADD_4174 + SUB_10158;
    auto MUL_10170 = MUL_9997 * 0.002;
    auto MUL_10163 = SUB_9988 * 0.119;
    auto SUB_10186 = MUL_10163 - MUL_10170;
    auto ADD_10189 = ADD_4172 + SUB_10186;
    auto MUL_10174 = SUB_10004 * 0.002;
    auto MUL_10165 = MUL_9991 * 0.119;
    auto SUB_10187 = MUL_10165 - MUL_10174;
    auto ADD_10190 = SUB_4173 + SUB_10187;
    auto MUL_10178 = MUL_10007 * 0.002;
    auto MUL_10167 = MUL_9994 * 0.119;
    auto SUB_10188 = MUL_10167 - MUL_10178;
    auto ADD_10191 = ADD_4174 + SUB_10188;
    auto MUL_10200 = MUL_9997 * 0.0017;
    auto MUL_10193 = SUB_9988 * 0.131;
    auto SUB_10216 = MUL_10193 - MUL_10200;
    auto ADD_10219 = ADD_4172 + SUB_10216;
    auto MUL_10204 = SUB_10004 * 0.0017;
    auto MUL_10195 = MUL_9991 * 0.131;
    auto SUB_10217 = MUL_10195 - MUL_10204;
    auto ADD_10220 = SUB_4173 + SUB_10217;
    auto MUL_10208 = MUL_10007 * 0.0017;
    auto MUL_10197 = MUL_9994 * 0.131;
    auto SUB_10218 = MUL_10197 - MUL_10208;
    auto ADD_10221 = ADD_4174 + SUB_10218;
    auto MUL_10230 = MUL_9997 * 0.0032;
    auto MUL_10223 = SUB_9988 * 0.144;
    auto SUB_10246 = MUL_10223 - MUL_10230;
    auto ADD_10249 = ADD_4172 + SUB_10246;
    auto MUL_10234 = SUB_10004 * 0.0032;
    auto MUL_10225 = MUL_9991 * 0.144;
    auto SUB_10247 = MUL_10225 - MUL_10234;
    auto ADD_10250 = SUB_4173 + SUB_10247;
    auto MUL_10238 = MUL_10007 * 0.0032;
    auto MUL_10227 = MUL_9994 * 0.144;
    auto SUB_10248 = MUL_10227 - MUL_10238;
    auto ADD_10251 = ADD_4174 + SUB_10248;
    auto MUL_10260 = MUL_9997 * 0.009;
    auto MUL_10253 = SUB_9988 * 0.16;
    auto SUB_10276 = MUL_10253 - MUL_10260;
    auto ADD_10279 = ADD_4172 + SUB_10276;
    auto MUL_10264 = SUB_10004 * 0.009;
    auto MUL_10255 = MUL_9991 * 0.16;
    auto SUB_10277 = MUL_10255 - MUL_10264;
    auto ADD_10280 = SUB_4173 + SUB_10277;
    auto MUL_10268 = MUL_10007 * 0.009;
    auto MUL_10257 = MUL_9994 * 0.16;
    auto SUB_10278 = MUL_10257 - MUL_10268;
    auto ADD_10281 = ADD_4174 + SUB_10278;
    auto MUL_10290 = MUL_9997 * 0.011;
    auto MUL_10283 = SUB_9988 * 0.17;
    auto SUB_10306 = MUL_10283 - MUL_10290;
    auto ADD_10309 = ADD_4172 + SUB_10306;
    auto MUL_10294 = SUB_10004 * 0.011;
    auto MUL_10285 = MUL_9991 * 0.17;
    auto SUB_10307 = MUL_10285 - MUL_10294;
    auto ADD_10310 = SUB_4173 + SUB_10307;
    auto MUL_10298 = MUL_10007 * 0.011;
    auto MUL_10287 = MUL_9994 * 0.17;
    auto SUB_10308 = MUL_10287 - MUL_10298;
    auto ADD_10311 = ADD_4174 + SUB_10308;
    auto MUL_10320 = MUL_9997 * 0.013;
    auto MUL_10313 = SUB_9988 * 0.178;
    auto SUB_10336 = MUL_10313 - MUL_10320;
    auto ADD_10339 = ADD_4172 + SUB_10336;
    auto MUL_10324 = SUB_10004 * 0.013;
    auto MUL_10315 = MUL_9991 * 0.178;
    auto SUB_10337 = MUL_10315 - MUL_10324;
    auto ADD_10340 = SUB_4173 + SUB_10337;
    auto MUL_10328 = MUL_10007 * 0.013;
    auto MUL_10317 = MUL_9994 * 0.178;
    auto SUB_10338 = MUL_10317 - MUL_10328;
    auto ADD_10341 = ADD_4174 + SUB_10338;
    if(/*link_gripper_finger_left*/ sphere_environment_in_collision(environment, ADD_10039, ADD_10040, ADD_10041, 0.062)){ if(sphere_environment_in_collision(environment, ADD_10069, ADD_10070, ADD_10071, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10099, ADD_10100, ADD_10101, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10129, ADD_10130, ADD_10131, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10159, ADD_10160, ADD_10161, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10189, ADD_10190, ADD_10191, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10219, ADD_10220, ADD_10221, 0.02)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10249, ADD_10250, ADD_10251, 0.016)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10279, ADD_10280, ADD_10281, 0.014)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10309, ADD_10310, ADD_10311, 0.013)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10339, ADD_10340, ADD_10341, 0.012)){ return false; } } // (1213, 1402)
    auto MUL_4253 = SUB_4210 * 0.7010574;
    auto MUL_4275 = SUB_4210 * 0.092296;
    auto MUL_4283 = ADD_4203 * 0.7010574;
    auto MUL_4302 = ADD_4203 * 0.1777845;
    auto MUL_4308 = ADD_4203 * MUL_4302;
    auto MUL_4261 = ADD_4203 * 0.092296;
    auto MUL_4289 = ADD_4203 * 0.0106722;
    auto MUL_4304 = SUB_4210 * MUL_4289;
    auto MUL_4255 = SUB_4191 * 0.7010574;
    auto SUB_4256 = MUL_4253 - MUL_4255;
    auto ADD_4267 = MUL_4253 + MUL_4255;
    auto MUL_4277 = SUB_4191 * 0.092296;
    auto SUB_4278 = MUL_4275 - MUL_4277;
    auto ADD_4245 = MUL_4275 + MUL_4277;
    auto MUL_4293 = SUB_4191 * 0.0106722;
    auto MUL_4280 = ADD_4197 * 0.7010574;
    auto SUB_4281 = SUB_4278 - MUL_4280;
    auto SUB_4284 = SUB_4281 - MUL_4283;
    auto ADD_4248 = ADD_4245 + MUL_4280;
    auto SUB_4251 = ADD_4248 - MUL_4283;
    auto MUL_4296 = ADD_4197 * 0.1777845;
    auto ADD_4297 = MUL_4293 + MUL_4296;
    auto MUL_4305 = ADD_4197 * ADD_4297;
    auto SUB_4307 = MUL_4304 - MUL_4305;
    auto SUB_4310 = SUB_4307 - MUL_4308;
    auto MUL_4312 = SUB_4310 * 2.0;
    auto ADD_4314 = MUL_4312 + 0.1777845;
    auto ADD_4337 = ADD_4172 + ADD_4314;
    auto MUL_4258 = ADD_4197 * 0.092296;
    auto SUB_4270 = ADD_4267 - MUL_4258;
    auto ADD_4273 = SUB_4270 + MUL_4261;
    auto ADD_4259 = SUB_4256 + MUL_4258;
    auto ADD_4262 = ADD_4259 + MUL_4261;
    auto MUL_10367 = SUB_4284 * ADD_4262;
    auto MUL_10365 = ADD_4273 * ADD_4273;
    auto MUL_10364 = ADD_4262 * ADD_4262;
    auto ADD_10373 = MUL_10364 + MUL_10365;
    auto MUL_10376 = ADD_10373 * 2.0;
    auto SUB_10379 = 1.0 - MUL_10376;
    auto MUL_10413 = SUB_10379 * 1e-06;
    auto MUL_10371 = SUB_4251 * ADD_4273;
    auto ADD_10399 = MUL_10371 + MUL_10367;
    auto MUL_10401 = ADD_10399 * 2.0;
    auto MUL_10425 = MUL_10401 * 0.0085;
    auto ADD_10430 = MUL_10413 + MUL_10425;
    auto ADD_10433 = ADD_4337 + ADD_10430;
    auto MUL_10366 = SUB_4284 * ADD_4273;
    auto MUL_10369 = SUB_4284 * SUB_4251;
    auto MUL_10372 = ADD_4262 * ADD_4273;
    auto SUB_10402 = MUL_10372 - MUL_10369;
    auto MUL_10404 = SUB_10402 * 2.0;
    auto MUL_10427 = MUL_10404 * 0.0085;
    auto MUL_10370 = SUB_4251 * ADD_4262;
    auto ADD_10380 = MUL_10370 + MUL_10366;
    auto MUL_10382 = ADD_10380 * 2.0;
    auto MUL_10415 = MUL_10382 * 1e-06;
    auto ADD_10431 = MUL_10415 + MUL_10427;
    auto MUL_4316 = SUB_4210 * MUL_4302;
    auto MUL_4320 = ADD_4203 * MUL_4289;
    auto MUL_4317 = SUB_4191 * ADD_4297;
    auto ADD_4319 = MUL_4316 + MUL_4317;
    auto ADD_4321 = ADD_4319 + MUL_4320;
    auto MUL_4323 = ADD_4321 * 2.0;
    auto SUB_4326 = MUL_4323 - 0.0106722;
    auto ADD_4338 = SUB_4173 + SUB_4326;
    auto ADD_10434 = ADD_4338 + ADD_10431;
    auto SUB_10383 = MUL_10371 - MUL_10367;
    auto MUL_10385 = SUB_10383 * 2.0;
    auto MUL_10417 = MUL_10385 * 1e-06;
    auto MUL_10368 = SUB_4251 * SUB_4251;
    auto ADD_10405 = MUL_10364 + MUL_10368;
    auto MUL_10408 = ADD_10405 * 2.0;
    auto SUB_10411 = 1.0 - MUL_10408;
    auto MUL_10429 = SUB_10411 * 0.0085;
    auto ADD_10432 = MUL_10417 + MUL_10429;
    auto MUL_4327 = SUB_4210 * ADD_4297;
    auto MUL_4329 = SUB_4191 * MUL_4302;
    auto SUB_4331 = MUL_4329 - MUL_4327;
    auto MUL_4332 = ADD_4197 * MUL_4289;
    auto SUB_4333 = SUB_4331 - MUL_4332;
    auto MUL_4335 = SUB_4333 * 2.0;
    auto ADD_4339 = ADD_4174 + MUL_4335;
    auto ADD_10435 = ADD_4339 + ADD_10432;
    auto MUL_10449 = MUL_10401 * 0.004;
    auto ADD_10454 = ADD_4337 + MUL_10449;
    auto MUL_10451 = MUL_10404 * 0.004;
    auto ADD_10455 = ADD_4338 + MUL_10451;
    auto MUL_10453 = SUB_10411 * 0.004;
    auto ADD_10456 = ADD_4339 + MUL_10453;
    auto SUB_10386 = MUL_10370 - MUL_10366;
    auto MUL_10476 = MUL_10401 * 0.0115;
    auto MUL_10388 = SUB_10386 * 2.0;
    auto MUL_10465 = MUL_10388 * 0.017;
    auto SUB_10481 = MUL_10476 - MUL_10465;
    auto ADD_10484 = ADD_4337 + SUB_10481;
    auto ADD_10389 = MUL_10365 + MUL_10368;
    auto MUL_10478 = MUL_10404 * 0.0115;
    auto MUL_10392 = ADD_10389 * 2.0;
    auto SUB_10395 = 1.0 - MUL_10392;
    auto MUL_10469 = SUB_10395 * 0.017;
    auto SUB_10482 = MUL_10478 - MUL_10469;
    auto ADD_10485 = ADD_4338 + SUB_10482;
    auto ADD_10396 = MUL_10372 + MUL_10369;
    auto MUL_10480 = SUB_10411 * 0.0115;
    auto MUL_10398 = ADD_10396 * 2.0;
    auto MUL_10473 = MUL_10398 * 0.017;
    auto SUB_10483 = MUL_10480 - MUL_10473;
    auto ADD_10486 = ADD_4339 + SUB_10483;
    auto ADD_10505 = MUL_10465 + MUL_10476;
    auto ADD_10508 = ADD_4337 + ADD_10505;
    auto ADD_10506 = MUL_10469 + MUL_10478;
    auto ADD_10509 = ADD_4338 + ADD_10506;
    auto ADD_10507 = MUL_10473 + MUL_10480;
    auto ADD_10510 = ADD_4339 + ADD_10507;
    auto MUL_10512 = SUB_10379 * 0.017;
    auto ADD_10529 = MUL_10512 + MUL_10476;
    auto ADD_10532 = ADD_4337 + ADD_10529;
    auto MUL_10514 = MUL_10382 * 0.017;
    auto ADD_10530 = MUL_10514 + MUL_10478;
    auto ADD_10533 = ADD_4338 + ADD_10530;
    auto MUL_10516 = MUL_10385 * 0.017;
    auto ADD_10531 = MUL_10516 + MUL_10480;
    auto ADD_10534 = ADD_4339 + ADD_10531;
    auto SUB_10559 = MUL_10476 - MUL_10512;
    auto ADD_10562 = ADD_4337 + SUB_10559;
    auto SUB_10560 = MUL_10478 - MUL_10514;
    auto ADD_10563 = ADD_4338 + SUB_10560;
    auto SUB_10561 = MUL_10480 - MUL_10516;
    auto ADD_10564 = ADD_4339 + SUB_10561;
    auto MUL_10573 = MUL_10388 * 0.012;
    auto MUL_10566 = SUB_10379 * 0.012;
    auto SUB_10589 = MUL_10566 - MUL_10573;
    auto ADD_10592 = SUB_10589 + MUL_10476;
    auto ADD_10595 = ADD_4337 + ADD_10592;
    auto MUL_10577 = SUB_10395 * 0.012;
    auto MUL_10568 = MUL_10382 * 0.012;
    auto SUB_10590 = MUL_10568 - MUL_10577;
    auto ADD_10593 = SUB_10590 + MUL_10478;
    auto ADD_10596 = ADD_4338 + ADD_10593;
    auto MUL_10581 = MUL_10398 * 0.012;
    auto MUL_10570 = MUL_10385 * 0.012;
    auto SUB_10591 = MUL_10570 - MUL_10581;
    auto ADD_10594 = SUB_10591 + MUL_10480;
    auto ADD_10597 = ADD_4339 + ADD_10594;
    auto ADD_10616 = MUL_10566 + MUL_10573;
    auto ADD_10619 = ADD_10616 + MUL_10476;
    auto ADD_10622 = ADD_4337 + ADD_10619;
    auto ADD_10617 = MUL_10568 + MUL_10577;
    auto ADD_10620 = ADD_10617 + MUL_10478;
    auto ADD_10623 = ADD_4338 + ADD_10620;
    auto ADD_10618 = MUL_10570 + MUL_10581;
    auto ADD_10621 = ADD_10618 + MUL_10480;
    auto ADD_10624 = ADD_4339 + ADD_10621;
    auto SUB_10649 = MUL_10573 - MUL_10566;
    auto ADD_10652 = SUB_10649 + MUL_10476;
    auto ADD_10655 = ADD_4337 + ADD_10652;
    auto SUB_10650 = MUL_10577 - MUL_10568;
    auto ADD_10653 = SUB_10650 + MUL_10478;
    auto ADD_10656 = ADD_4338 + ADD_10653;
    auto SUB_10651 = MUL_10581 - MUL_10570;
    auto ADD_10654 = SUB_10651 + MUL_10480;
    auto ADD_10657 = ADD_4339 + ADD_10654;
    auto SUB_10694 = MUL_10476 - ADD_10616;
    auto ADD_10697 = ADD_4337 + SUB_10694;
    auto SUB_10695 = MUL_10478 - ADD_10617;
    auto ADD_10698 = ADD_4338 + SUB_10695;
    auto SUB_10696 = MUL_10480 - ADD_10618;
    auto ADD_10699 = ADD_4339 + SUB_10696;
    auto MUL_10708 = MUL_10388 * 0.0157;
    auto MUL_10701 = SUB_10379 * 0.0065;
    auto SUB_10724 = MUL_10701 - MUL_10708;
    auto ADD_10727 = SUB_10724 + MUL_10476;
    auto ADD_10730 = ADD_4337 + ADD_10727;
    auto MUL_10712 = SUB_10395 * 0.0157;
    auto MUL_10703 = MUL_10382 * 0.0065;
    auto SUB_10725 = MUL_10703 - MUL_10712;
    auto ADD_10728 = SUB_10725 + MUL_10478;
    auto ADD_10731 = ADD_4338 + ADD_10728;
    auto MUL_10716 = MUL_10398 * 0.0157;
    auto MUL_10705 = MUL_10385 * 0.0065;
    auto SUB_10726 = MUL_10705 - MUL_10716;
    auto ADD_10729 = SUB_10726 + MUL_10480;
    auto ADD_10732 = ADD_4339 + ADD_10729;
    auto ADD_10751 = MUL_10701 + MUL_10708;
    auto ADD_10754 = ADD_10751 + MUL_10476;
    auto ADD_10757 = ADD_4337 + ADD_10754;
    auto ADD_10752 = MUL_10703 + MUL_10712;
    auto ADD_10755 = ADD_10752 + MUL_10478;
    auto ADD_10758 = ADD_4338 + ADD_10755;
    auto ADD_10753 = MUL_10705 + MUL_10716;
    auto ADD_10756 = ADD_10753 + MUL_10480;
    auto ADD_10759 = ADD_4339 + ADD_10756;
    auto SUB_10784 = MUL_10708 - MUL_10701;
    auto ADD_10787 = SUB_10784 + MUL_10476;
    auto ADD_10790 = ADD_4337 + ADD_10787;
    auto SUB_10785 = MUL_10712 - MUL_10703;
    auto ADD_10788 = SUB_10785 + MUL_10478;
    auto ADD_10791 = ADD_4338 + ADD_10788;
    auto SUB_10786 = MUL_10716 - MUL_10705;
    auto ADD_10789 = SUB_10786 + MUL_10480;
    auto ADD_10792 = ADD_4339 + ADD_10789;
    auto SUB_10829 = MUL_10476 - ADD_10751;
    auto ADD_10832 = ADD_4337 + SUB_10829;
    auto SUB_10830 = MUL_10478 - ADD_10752;
    auto ADD_10833 = ADD_4338 + SUB_10830;
    auto SUB_10831 = MUL_10480 - ADD_10753;
    auto ADD_10834 = ADD_4339 + SUB_10831;
    auto MUL_10843 = MUL_10388 * 0.0065;
    auto MUL_10836 = SUB_10379 * 0.0157;
    auto SUB_10859 = MUL_10836 - MUL_10843;
    auto ADD_10862 = SUB_10859 + MUL_10476;
    auto ADD_10865 = ADD_4337 + ADD_10862;
    auto MUL_10847 = SUB_10395 * 0.0065;
    auto MUL_10838 = MUL_10382 * 0.0157;
    auto SUB_10860 = MUL_10838 - MUL_10847;
    auto ADD_10863 = SUB_10860 + MUL_10478;
    auto ADD_10866 = ADD_4338 + ADD_10863;
    auto MUL_10851 = MUL_10398 * 0.0065;
    auto MUL_10840 = MUL_10385 * 0.0157;
    auto SUB_10861 = MUL_10840 - MUL_10851;
    auto ADD_10864 = SUB_10861 + MUL_10480;
    auto ADD_10867 = ADD_4339 + ADD_10864;
    auto ADD_10886 = MUL_10836 + MUL_10843;
    auto ADD_10889 = ADD_10886 + MUL_10476;
    auto ADD_10892 = ADD_4337 + ADD_10889;
    auto ADD_10887 = MUL_10838 + MUL_10847;
    auto ADD_10890 = ADD_10887 + MUL_10478;
    auto ADD_10893 = ADD_4338 + ADD_10890;
    auto ADD_10888 = MUL_10840 + MUL_10851;
    auto ADD_10891 = ADD_10888 + MUL_10480;
    auto ADD_10894 = ADD_4339 + ADD_10891;
    auto SUB_10919 = MUL_10843 - MUL_10836;
    auto ADD_10922 = SUB_10919 + MUL_10476;
    auto ADD_10925 = ADD_4337 + ADD_10922;
    auto SUB_10920 = MUL_10847 - MUL_10838;
    auto ADD_10923 = SUB_10920 + MUL_10478;
    auto ADD_10926 = ADD_4338 + ADD_10923;
    auto SUB_10921 = MUL_10851 - MUL_10840;
    auto ADD_10924 = SUB_10921 + MUL_10480;
    auto ADD_10927 = ADD_4339 + ADD_10924;
    auto SUB_10964 = MUL_10476 - ADD_10886;
    auto ADD_10967 = ADD_4337 + SUB_10964;
    auto SUB_10965 = MUL_10478 - ADD_10887;
    auto ADD_10968 = ADD_4338 + SUB_10965;
    auto SUB_10966 = MUL_10480 - ADD_10888;
    auto ADD_10969 = ADD_4339 + SUB_10966;
    if(/*link_gripper_fingertip_left*/ sphere_environment_in_collision(environment, ADD_10433, ADD_10434, ADD_10435, 0.025)){ if(sphere_environment_in_collision(environment, ADD_10454, ADD_10455, ADD_10456, 0.014)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10484, ADD_10485, ADD_10486, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10508, ADD_10509, ADD_10510, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10532, ADD_10533, ADD_10534, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10562, ADD_10563, ADD_10564, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10595, ADD_10596, ADD_10597, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10622, ADD_10623, ADD_10624, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10655, ADD_10656, ADD_10657, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10697, ADD_10698, ADD_10699, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10730, ADD_10731, ADD_10732, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10757, ADD_10758, ADD_10759, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10790, ADD_10791, ADD_10792, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10832, ADD_10833, ADD_10834, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10865, ADD_10866, ADD_10867, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10892, ADD_10893, ADD_10894, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10925, ADD_10926, ADD_10927, 0.006)){ return false; }
    if(sphere_environment_in_collision(environment, ADD_10967, ADD_10968, ADD_10969, 0.006)){ return false; } } // (1402, 1648)
    auto MUL_4930 = SUB_652 * 0.7071068;
    auto MUL_4945 = ADD_644 * 0.7071068;
    auto MUL_4934 = SUB_636 * 0.7071068;
    auto SUB_4954 = MUL_4930 - MUL_4934;
    auto ADD_4935 = MUL_4930 + MUL_4934;
    auto MUL_11160 = SUB_4954 * ADD_4935;
    auto MUL_11157 = ADD_4935 * ADD_4935;
    auto MUL_4941 = ADD_630 * 0.7071068;
    auto SUB_4947 = MUL_4941 - MUL_4945;
    auto ADD_4928 = MUL_4941 + MUL_4945;
    auto MUL_11159 = SUB_4954 * SUB_4947;
    auto MUL_11158 = SUB_4947 * SUB_4947;
    auto ADD_11166 = MUL_11157 + MUL_11158;
    auto MUL_11169 = ADD_11166 * 2.0;
    auto SUB_11172 = 1.0 - MUL_11169;
    auto MUL_11164 = ADD_4928 * SUB_4947;
    auto ADD_11192 = MUL_11164 + MUL_11160;
    auto MUL_11194 = ADD_11192 * 2.0;
    auto MUL_11163 = ADD_4928 * ADD_4935;
    auto SUB_11179 = MUL_11163 - MUL_11159;
    auto MUL_11181 = SUB_11179 * 2.0;
    auto MUL_4963 = ADD_644 * 0.041173;
    auto MUL_4968 = ADD_630 * 0.041173;
    auto MUL_4979 = ADD_644 * 0.0402;
    auto MUL_4971 = SUB_636 * 0.0402;
    auto ADD_4972 = MUL_4968 + MUL_4971;
    auto MUL_4984 = SUB_636 * ADD_4972;
    auto MUL_4959 = SUB_636 * 0.0245786;
    auto ADD_4964 = MUL_4959 + MUL_4963;
    auto MUL_4982 = SUB_652 * ADD_4964;
    auto ADD_4986 = MUL_4982 + MUL_4984;
    auto MUL_4976 = ADD_630 * 0.0245786;
    auto SUB_4981 = MUL_4979 - MUL_4976;
    auto MUL_4988 = ADD_644 * SUB_4981;
    auto ADD_4990 = ADD_4986 + MUL_4988;
    auto MUL_4993 = ADD_4990 * 2.0;
    auto SUB_4996 = 0.0402 - MUL_4993;
    auto ADD_5023 = ADD_612 + SUB_4996;
    auto MUL_11252 = MUL_11194 * 0.0171596;
    auto MUL_11245 = MUL_11181 * 0.0021271;
    auto MUL_11239 = SUB_11172 * 0.0003056;
    auto ADD_11262 = MUL_11239 + MUL_11245;
    auto SUB_11265 = ADD_11262 - MUL_11252;
    auto ADD_11268 = ADD_5023 + SUB_11265;
    auto ADD_11173 = MUL_11163 + MUL_11159;
    auto MUL_11175 = ADD_11173 * 2.0;
    auto MUL_11241 = MUL_11175 * 0.0003056;
    auto MUL_11162 = SUB_4954 * ADD_4928;
    auto MUL_11165 = ADD_4935 * SUB_4947;
    auto SUB_11195 = MUL_11165 - MUL_11162;
    auto MUL_11197 = SUB_11195 * 2.0;
    auto MUL_11256 = MUL_11197 * 0.0171596;
    auto MUL_11161 = ADD_4928 * ADD_4928;
    auto ADD_11182 = MUL_11158 + MUL_11161;
    auto MUL_11185 = ADD_11182 * 2.0;
    auto SUB_11188 = 1.0 - MUL_11185;
    auto MUL_11247 = SUB_11188 * 0.0021271;
    auto ADD_11263 = MUL_11241 + MUL_11247;
    auto SUB_11266 = ADD_11263 - MUL_11256;
    auto MUL_4998 = SUB_652 * SUB_4981;
    auto MUL_5003 = ADD_644 * ADD_4964;
    auto MUL_5000 = ADD_630 * ADD_4972;
    auto SUB_5002 = MUL_5000 - MUL_4998;
    auto ADD_5004 = SUB_5002 + MUL_5003;
    auto MUL_5006 = ADD_5004 * 2.0;
    auto SUB_5009 = MUL_5006 - 0.041173;
    auto ADD_5024 = SUB_615 + SUB_5009;
    auto ADD_11269 = ADD_5024 + SUB_11266;
    auto SUB_11176 = MUL_11164 - MUL_11160;
    auto ADD_11198 = MUL_11157 + MUL_11161;
    auto ADD_11189 = MUL_11165 + MUL_11162;
    auto MUL_11178 = SUB_11176 * 2.0;
    auto MUL_11243 = MUL_11178 * 0.0003056;
    auto MUL_11191 = ADD_11189 * 2.0;
    auto MUL_11249 = MUL_11191 * 0.0021271;
    auto ADD_11264 = MUL_11243 + MUL_11249;
    auto MUL_11201 = ADD_11198 * 2.0;
    auto SUB_11204 = 1.0 - MUL_11201;
    auto MUL_5010 = SUB_652 * ADD_4972;
    auto MUL_5015 = SUB_636 * ADD_4964;
    auto MUL_5012 = ADD_630 * SUB_4981;
    auto ADD_5013 = MUL_5010 + MUL_5012;
    auto SUB_5017 = MUL_5015 - ADD_5013;
    auto MUL_5019 = SUB_5017 * 2.0;
    auto SUB_5022 = MUL_5019 - 0.0245786;
    auto ADD_5025 = ADD_617 + SUB_5022;
    auto MUL_11260 = SUB_11204 * 0.0171596;
    auto SUB_11267 = ADD_11264 - MUL_11260;
    auto ADD_11270 = ADD_5025 + SUB_11267;
    if(/*link_head_nav_cam*/ sphere_environment_in_collision(environment, ADD_11268, ADD_11269, ADD_11270, 0.050857)){ return false; } // (1648, 1737)
    auto SUB_7155 = SUB_7192 - MUL_7182;
    auto ADD_7159 = ADD_372 + SUB_7155;
    auto ADD_7156 = ADD_7193 + MUL_7186;
    auto SUB_7160 = ADD_374 - ADD_7156;
    auto SUB_7158 = ADD_7195 - MUL_7190;
    auto ADD_7161 = SUB_377 + SUB_7158;
    if(/*base_link vs. link_wrist_yaw*/ sphere_sphere_self_collision<decltype(q[0])>(-0.07, 0.0, 0.092, 0.23, ADD_7159, SUB_7160, ADD_7161, 0.02848)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.105, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.105, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, 0.0525, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.009, -0.0525, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, -0.13, 0.0944, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(0.004, 0.13, 0.0944, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.13, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.13, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.07875, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, -0.02625, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.07875, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.048, 0.02625, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.0525, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.0525, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.105, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.105, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, -0.13, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.096, 0.13, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.13, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.13, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.07875, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, -0.02625, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.07875, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.144, 0.02625, 0.092, 0.079, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, -0.059, 0.095, 0.086, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.213, 0.059, 0.095, 0.086, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, -0.1, 0.095, 0.086, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.192, 0.1, 0.095, 0.086, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.23, 0.0, 0.095, 0.086, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } } // (1737, 1743)
    if(/*caster_link vs. link_wrist_yaw*/ sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7159, SUB_7160, ADD_7161, 0.02848)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.2449332, 0.0001548, 0.031821, 0.0315, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } } // (1743, 1743)
    if(/*link_left_wheel vs. link_wrist_yaw*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7159, SUB_7160, ADD_7161, 0.02848)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0, 0.15635, 0.0508, 0.052, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } } // (1743, 1743)
    if(/*link_lift vs. link_wrist_yaw*/ sphere_sphere_self_collision<decltype(q[0])>(NEGATE_6835, SUB_6838, ADD_6840, 0.14, ADD_7159, SUB_7160, ADD_7161, 0.02848)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_6842, SUB_73, ADD_6845, 0.08, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(NEGATE_6849, SUB_6852, ADD_6854, 0.105, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } } // (1743, 1743)
    if(/*link_mast vs. link_wrist_yaw*/ sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349972, 0.7884, 0.55, ADD_7159, SUB_7160, ADD_7161, 0.02848)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349994, 0.1884, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349992, 0.2584, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349989, 0.3284, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349986, 0.3984, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349984, 0.4684, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349981, 0.5384, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349979, 0.6084, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349976, 0.6784, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349974, 0.7484, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349971, 0.8184, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349968, 0.8884, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349966, 0.9584, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349963, 1.0284, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349961, 1.0984, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349958, 1.1684, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349956, 1.2384, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(-0.067, 0.1349953, 1.3084, 0.045, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } } // (1743, 1743)
    if(/*link_right_wheel vs. link_wrist_yaw*/ sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7159, SUB_7160, ADD_7161, 0.02848)){ if(sphere_sphere_self_collision<decltype(q[0])>(0.0, -0.15635, 0.0507999, 0.052, ADD_7200, SUB_7201, ADD_7202, 0.02848)){ return false; } } // (1743, 1743)
    if(/*link_wrist_yaw vs. link_head*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_7159, SUB_7160, ADD_7161, 0.02848, -0.0270003, 0.081995, 1.3984001, 0.12)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_7200, SUB_7201, ADD_7202, 0.02848, -0.0384372, 0.129995, 1.3839643, 0.074)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(ADD_7200, SUB_7201, ADD_7202, 0.02848, -0.0021212, 0.0450668, 1.3839642, 0.074)){ return false; } } // (1743, 1743)
    if(/*link_wrist_yaw vs. link_head_tilt*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_7159, SUB_7160, ADD_7161, 0.02848, ADD_7367, ADD_7368, ADD_7369, 0.07)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_7200, SUB_7201, ADD_7202, 0.02848, ADD_7396, ADD_7397, ADD_7398, 0.057304)){ return false; }
    if(sphere_sphere_self_collision<decltype(q[0])>(ADD_7200, SUB_7201, ADD_7202, 0.02848, ADD_7432, ADD_7433, ADD_7434, 0.056933)){ return false; } } // (1743, 1743)
    }
}