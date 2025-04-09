
#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

namespace vamp::robots::stretch
{
    using Configuration = FloatVector<13>;
    template <std::size_t block_width> using ConfigurationBlock = FloatVector<block_width, 13>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, 13> s_m_a{10.0, 10.0, 6.28318, 1.1, 0.13, 0.13, 0.13, 0.13, 5.75, 5.4, 2.3200000000000003, 2.13, 6.28};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 13> s_a_a{-5.0, -5.0, -3.14159, 0.0, 0.0, 0.0, 0.0, 0.0, -1.75, -3.9, -1.53, -1.57, -3.14};
    static const Configuration s_m(s_m_a);
    static const Configuration s_a(s_a_a);inline void scale_configuration(Configuration& q) noexcept { q = q * s_m + s_a; }template <std::size_t block_width> inline void scale_configuration_block(ConfigurationBlock<block_width>& q) noexcept {q[0] = -5.0 + (q[0] * 10.0);
    q[1] = -5.0 + (q[1] * 10.0);
    q[2] = -3.14159 + (q[2] * 6.28318);
    q[3] = 0.0 + (q[3] * 1.1);
    q[4] = 0.0 + (q[4] * 0.13);
    q[5] = 0.0 + (q[5] * 0.13);
    q[6] = 0.0 + (q[6] * 0.13);
    q[7] = 0.0 + (q[7] * 0.13);
    q[8] = -1.75 + (q[8] * 5.75);
    q[9] = -3.9 + (q[9] * 5.4);
    q[10] = -1.53 + (q[10] * 2.3200000000000003);
    q[11] = -1.57 + (q[11] * 2.13);
    q[12] = -3.14 + (q[12] * 6.28);}

    alignas(Configuration::S::Alignment) constexpr std::array<float, 13> d_m_a{0.1, 0.1, 0.15915507752443828, 0.9090909090909091, 7.692307692307692, 7.692307692307692, 7.692307692307692, 7.692307692307692, 0.17391304347826086, 0.18518518518518517, 0.43103448275862066, 0.4694835680751174, 0.1592356687898089};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 13> d_a_a{-5.0, -5.0, -3.14159, 0.0, 0.0, 0.0, 0.0, 0.0, -1.75, -3.9, -1.53, -1.57, -3.14};
    static const Configuration d_m(d_m_a);
    static const Configuration d_a(d_a_a);inline void descale_configuration(Configuration& q) noexcept { q = (q - d_a) * d_m; }template <std::size_t block_width> inline void descale_configuration_block(ConfigurationBlock<block_width>& q) noexcept {q[0] = 0.1 * (q[0] - -5.0);
    q[1] = 0.1 * (q[1] - -5.0);
    q[2] = 0.15915507752443828 * (q[2] - -3.14159);
    q[3] = 0.9090909090909091 * (q[3] - 0.0);
    q[4] = 7.692307692307692 * (q[4] - 0.0);
    q[5] = 7.692307692307692 * (q[5] - 0.0);
    q[6] = 7.692307692307692 * (q[6] - 0.0);
    q[7] = 7.692307692307692 * (q[7] - 0.0);
    q[8] = 0.17391304347826086 * (q[8] - -1.75);
    q[9] = 0.18518518518518517 * (q[9] - -3.9);
    q[10] = 0.43103448275862066 * (q[10] - -1.53);
    q[11] = 0.4694835680751174 * (q[11] - -1.57);
    q[12] = 0.1592356687898089 * (q[12] - -3.14);}

    //constexpr auto space_measure = 6235.581234344888;
    inline static auto space_measure() noexcept -> float
    {
        return  6235.581234344888;
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

    inline auto eefk(const std::array<float, 13> &q) noexcept -> std::array<float, 13>
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
        auto INPUT_2 = q[2];
        auto DIV_107 = INPUT_2 * 0.5;
        auto SIN_108 = DIV_107.sin();
        auto MUL_9198 = SIN_108 * SIN_108;
        auto MUL_9208 = MUL_9198 * 2.0;
        auto SUB_9211 = 1.0 - MUL_9208;
        auto MUL_9238 = SUB_9211 * 0.07;
        auto INPUT_0 = q[0];
        auto SUB_9258 = INPUT_0 - MUL_9238;
        auto COS_114 = DIV_107.cos();
        auto MUL_9199 = COS_114 * SIN_108;
        auto MUL_9213 = MUL_9199 * 2.0;
        auto MUL_9242 = MUL_9213 * 0.07;
        auto INPUT_1 = q[1];
        auto SUB_9259 = INPUT_1 - MUL_9242;
        auto MUL_9261 = SUB_9211 * 0.009;
        auto ADD_9277 = INPUT_0 + MUL_9261;
        auto MUL_9263 = MUL_9213 * 0.009;
        auto ADD_9278 = INPUT_1 + MUL_9263;
        auto MUL_9286 = MUL_9213 * 0.105;
        auto SUB_9297 = MUL_9261 - MUL_9286;
        auto ADD_9299 = INPUT_0 + SUB_9297;
        auto MUL_9289 = SUB_9211 * 0.105;
        auto ADD_9298 = MUL_9263 + MUL_9289;
        auto ADD_9300 = INPUT_1 + ADD_9298;
        auto ADD_9322 = MUL_9261 + MUL_9286;
        auto ADD_9324 = INPUT_0 + ADD_9322;
        auto SUB_9323 = MUL_9263 - MUL_9289;
        auto ADD_9325 = INPUT_1 + SUB_9323;
        auto MUL_9333 = MUL_9213 * 0.0525;
        auto SUB_9344 = MUL_9261 - MUL_9333;
        auto ADD_9346 = INPUT_0 + SUB_9344;
        auto MUL_9336 = SUB_9211 * 0.0525;
        auto ADD_9345 = MUL_9263 + MUL_9336;
        auto ADD_9347 = INPUT_1 + ADD_9345;
        auto ADD_9369 = MUL_9261 + MUL_9333;
        auto ADD_9371 = INPUT_0 + ADD_9369;
        auto SUB_9370 = MUL_9263 - MUL_9336;
        auto ADD_9372 = INPUT_1 + SUB_9370;
        auto MUL_9381 = MUL_9213 * 0.13;
        auto MUL_9374 = SUB_9211 * 0.004;
        auto ADD_9394 = MUL_9374 + MUL_9381;
        auto ADD_9396 = INPUT_0 + ADD_9394;
        auto MUL_9376 = MUL_9213 * 0.004;
        auto MUL_9384 = SUB_9211 * 0.13;
        auto SUB_9395 = MUL_9376 - MUL_9384;
        auto ADD_9397 = INPUT_1 + SUB_9395;
        auto SUB_9416 = MUL_9374 - MUL_9381;
        auto ADD_9418 = INPUT_0 + SUB_9416;
        auto ADD_9417 = MUL_9376 + MUL_9384;
        auto ADD_9419 = INPUT_1 + ADD_9417;
        auto MUL_9422 = SUB_9211 * 0.048;
        auto SUB_9446 = MUL_9381 - MUL_9422;
        auto ADD_9449 = INPUT_0 + SUB_9446;
        auto MUL_9426 = MUL_9213 * 0.048;
        auto ADD_9447 = MUL_9426 + MUL_9384;
        auto SUB_9450 = INPUT_1 - ADD_9447;
        auto ADD_9474 = MUL_9422 + MUL_9381;
        auto SUB_9477 = INPUT_0 - ADD_9474;
        auto SUB_9476 = MUL_9384 - MUL_9426;
        auto ADD_9478 = INPUT_1 + SUB_9476;
        auto MUL_9492 = MUL_9213 * 0.07875;
        auto SUB_9505 = MUL_9492 - MUL_9422;
        auto ADD_9508 = INPUT_0 + SUB_9505;
        auto MUL_9495 = SUB_9211 * 0.07875;
        auto ADD_9506 = MUL_9426 + MUL_9495;
        auto SUB_9509 = INPUT_1 - ADD_9506;
        auto MUL_9523 = MUL_9213 * 0.02625;
        auto SUB_9536 = MUL_9523 - MUL_9422;
        auto ADD_9539 = INPUT_0 + SUB_9536;
        auto MUL_9526 = SUB_9211 * 0.02625;
        auto ADD_9537 = MUL_9426 + MUL_9526;
        auto SUB_9540 = INPUT_1 - ADD_9537;
        auto ADD_9564 = MUL_9422 + MUL_9492;
        auto SUB_9567 = INPUT_0 - ADD_9564;
        auto SUB_9566 = MUL_9495 - MUL_9426;
        auto ADD_9568 = INPUT_1 + SUB_9566;
        auto ADD_9592 = MUL_9422 + MUL_9523;
        auto SUB_9595 = INPUT_0 - ADD_9592;
        auto SUB_9594 = MUL_9526 - MUL_9426;
        auto ADD_9596 = INPUT_1 + SUB_9594;
        auto MUL_9599 = SUB_9211 * 0.096;
        auto SUB_9619 = INPUT_0 - MUL_9599;
        auto MUL_9603 = MUL_9213 * 0.096;
        auto SUB_9620 = INPUT_1 - MUL_9603;
        auto SUB_9647 = MUL_9333 - MUL_9599;
        auto ADD_9650 = INPUT_0 + SUB_9647;
        auto ADD_9648 = MUL_9603 + MUL_9336;
        auto SUB_9651 = INPUT_1 - ADD_9648;
        auto ADD_9675 = MUL_9599 + MUL_9333;
        auto SUB_9678 = INPUT_0 - ADD_9675;
        auto SUB_9677 = MUL_9336 - MUL_9603;
        auto ADD_9679 = INPUT_1 + SUB_9677;
        auto SUB_9706 = MUL_9286 - MUL_9599;
        auto ADD_9709 = INPUT_0 + SUB_9706;
        auto ADD_9707 = MUL_9603 + MUL_9289;
        auto SUB_9710 = INPUT_1 - ADD_9707;
        auto ADD_9734 = MUL_9599 + MUL_9286;
        auto SUB_9737 = INPUT_0 - ADD_9734;
        auto SUB_9736 = MUL_9289 - MUL_9603;
        auto ADD_9738 = INPUT_1 + SUB_9736;
        auto SUB_9765 = MUL_9381 - MUL_9599;
        auto ADD_9768 = INPUT_0 + SUB_9765;
        auto ADD_9766 = MUL_9603 + MUL_9384;
        auto SUB_9769 = INPUT_1 - ADD_9766;
        auto ADD_9793 = MUL_9599 + MUL_9381;
        auto SUB_9796 = INPUT_0 - ADD_9793;
        auto SUB_9795 = MUL_9384 - MUL_9603;
        auto ADD_9797 = INPUT_1 + SUB_9795;
        auto MUL_9800 = SUB_9211 * 0.144;
        auto SUB_9824 = MUL_9381 - MUL_9800;
        auto ADD_9827 = INPUT_0 + SUB_9824;
        auto MUL_9804 = MUL_9213 * 0.144;
        auto ADD_9825 = MUL_9804 + MUL_9384;
        auto SUB_9828 = INPUT_1 - ADD_9825;
        auto ADD_9852 = MUL_9800 + MUL_9381;
        auto SUB_9855 = INPUT_0 - ADD_9852;
        auto SUB_9854 = MUL_9384 - MUL_9804;
        auto ADD_9856 = INPUT_1 + SUB_9854;
        auto SUB_9883 = MUL_9492 - MUL_9800;
        auto ADD_9886 = INPUT_0 + SUB_9883;
        auto ADD_9884 = MUL_9804 + MUL_9495;
        auto SUB_9887 = INPUT_1 - ADD_9884;
        auto SUB_9914 = MUL_9523 - MUL_9800;
        auto ADD_9917 = INPUT_0 + SUB_9914;
        auto ADD_9915 = MUL_9804 + MUL_9526;
        auto SUB_9918 = INPUT_1 - ADD_9915;
        auto ADD_9942 = MUL_9800 + MUL_9492;
        auto SUB_9945 = INPUT_0 - ADD_9942;
        auto SUB_9944 = MUL_9495 - MUL_9804;
        auto ADD_9946 = INPUT_1 + SUB_9944;
        auto ADD_9970 = MUL_9800 + MUL_9523;
        auto SUB_9973 = INPUT_0 - ADD_9970;
        auto SUB_9972 = MUL_9526 - MUL_9804;
        auto ADD_9974 = INPUT_1 + SUB_9972;
        auto MUL_9977 = SUB_9211 * 0.213;
        auto MUL_9988 = MUL_9213 * 0.059;
        auto SUB_10001 = MUL_9988 - MUL_9977;
        auto ADD_10004 = INPUT_0 + SUB_10001;
        auto MUL_9981 = MUL_9213 * 0.213;
        auto MUL_9991 = SUB_9211 * 0.059;
        auto ADD_10002 = MUL_9981 + MUL_9991;
        auto SUB_10005 = INPUT_1 - ADD_10002;
        auto ADD_10029 = MUL_9977 + MUL_9988;
        auto SUB_10032 = INPUT_0 - ADD_10029;
        auto SUB_10031 = MUL_9991 - MUL_9981;
        auto ADD_10033 = INPUT_1 + SUB_10031;
        auto MUL_10036 = SUB_9211 * 0.192;
        auto MUL_10047 = MUL_9213 * 0.1;
        auto SUB_10060 = MUL_10047 - MUL_10036;
        auto ADD_10063 = INPUT_0 + SUB_10060;
        auto MUL_10040 = MUL_9213 * 0.192;
        auto MUL_10050 = SUB_9211 * 0.1;
        auto ADD_10061 = MUL_10040 + MUL_10050;
        auto SUB_10064 = INPUT_1 - ADD_10061;
        auto ADD_10088 = MUL_10036 + MUL_10047;
        auto SUB_10091 = INPUT_0 - ADD_10088;
        auto SUB_10090 = MUL_10050 - MUL_10040;
        auto ADD_10092 = INPUT_1 + SUB_10090;
        auto MUL_10095 = SUB_9211 * 0.23;
        auto SUB_10115 = INPUT_0 - MUL_10095;
        auto MUL_10099 = MUL_9213 * 0.23;
        auto SUB_10116 = INPUT_1 - MUL_10099;
        if(/*base_link*/ sphere_environment_in_collision(environment, SUB_9258, SUB_9259, 0.092, 0.23)){ if(sphere_environment_in_collision(environment, ADD_9277, ADD_9278, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9299, ADD_9300, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9324, ADD_9325, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9346, ADD_9347, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9371, ADD_9372, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9396, ADD_9397, 0.0944, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9418, ADD_9419, 0.0944, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9449, SUB_9450, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9477, ADD_9478, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9508, SUB_9509, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9539, SUB_9540, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9567, ADD_9568, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9595, ADD_9596, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9619, SUB_9620, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9650, SUB_9651, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9678, ADD_9679, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9709, SUB_9710, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9737, ADD_9738, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9768, SUB_9769, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9796, ADD_9797, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9827, SUB_9828, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9855, ADD_9856, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9886, SUB_9887, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_9917, SUB_9918, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9945, ADD_9946, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_9973, ADD_9974, 0.092, 0.079)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10004, SUB_10005, 0.095, 0.086)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_10032, ADD_10033, 0.095, 0.086)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10063, SUB_10064, 0.095, 0.086)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_10091, ADD_10092, 0.095, 0.086)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_10115, SUB_10116, 0.095, 0.086)){ return false; } } // (0, 163)
        auto MUL_148 = COS_114 * 0.7071081;
        auto MUL_164 = SIN_108 * 0.7071081;
        auto MUL_176 = COS_114 * 0.7071055;
        auto MUL_174 = SIN_108 * 0.7071055;
        auto MUL_10190 = MUL_148 * MUL_174;
        auto MUL_10184 = MUL_176 * MUL_164;
        auto ADD_10222 = MUL_10190 + MUL_10184;
        auto MUL_10225 = ADD_10222 * 2.0;
        auto MUL_188 = SIN_108 * 0.17035;
        auto MUL_199 = COS_114 * MUL_188;
        auto MUL_203 = MUL_199 * 2.0;
        auto ADD_220 = INPUT_0 + MUL_203;
        auto MUL_10272 = MUL_10225 * 0.014;
        auto SUB_10278 = ADD_220 - MUL_10272;
        auto MUL_208 = SIN_108 * MUL_188;
        auto MUL_210 = MUL_208 * 2.0;
        auto SUB_213 = MUL_210 - 0.17035;
        auto ADD_221 = INPUT_1 + SUB_213;
        auto MUL_10187 = MUL_176 * MUL_148;
        auto MUL_10192 = MUL_164 * MUL_174;
        auto SUB_10227 = MUL_10187 - MUL_10192;
        auto MUL_10229 = SUB_10227 * 2.0;
        auto MUL_10275 = MUL_10229 * 0.014;
        auto ADD_10279 = ADD_221 + MUL_10275;
        auto MUL_10186 = MUL_148 * MUL_148;
        auto MUL_10181 = MUL_164 * MUL_164;
        auto ADD_10230 = MUL_10181 + MUL_10186;
        auto MUL_10233 = ADD_10230 * 2.0;
        auto SUB_10236 = 1.0 - MUL_10233;
        auto MUL_10277 = SUB_10236 * 0.014;
        auto ADD_10280 = 0.0508 + MUL_10277;
        if(/*link_right_wheel*/ sphere_environment_in_collision(environment, SUB_10278, ADD_10279, ADD_10280, 0.052)){ return false; } // (163, 194)
        auto ADD_10324 = MUL_10190 + MUL_10184;
        auto MUL_10327 = ADD_10324 * 2.0;
        auto MUL_10396 = MUL_10327 * 0.014;
        auto MUL_337 = MUL_199 * 2.0;
        auto SUB_356 = INPUT_0 - MUL_337;
        auto MUL_10284 = MUL_174 * MUL_174;
        auto ADD_10296 = MUL_10181 + MUL_10284;
        auto MUL_10299 = ADD_10296 * 2.0;
        auto SUB_10302 = 1.0 - MUL_10299;
        auto MUL_10378 = SUB_10302 * 0.0;
        auto SUB_10405 = MUL_10396 - MUL_10378;
        auto ADD_10410 = SUB_356 + SUB_10405;
        auto SUB_10329 = MUL_10187 - MUL_10192;
        auto MUL_10291 = MUL_148 * MUL_164;
        auto MUL_10331 = SUB_10329 * 2.0;
        auto MUL_10399 = MUL_10331 * 0.014;
        auto MUL_346 = MUL_208 * 2.0;
        auto SUB_349 = 0.17035 - MUL_346;
        auto ADD_357 = INPUT_1 + SUB_349;
        auto MUL_10285 = MUL_176 * MUL_174;
        auto ADD_10303 = MUL_10291 + MUL_10285;
        auto MUL_10305 = ADD_10303 * 2.0;
        auto MUL_10382 = MUL_10305 * 0.0;
        auto ADD_10406 = MUL_10382 + MUL_10399;
        auto SUB_10411 = ADD_357 - ADD_10406;
        auto SUB_10306 = MUL_10184 - MUL_10190;
        auto ADD_10332 = MUL_10181 + MUL_10186;
        auto MUL_10335 = ADD_10332 * 2.0;
        auto SUB_10338 = 1.0 - MUL_10335;
        auto MUL_10403 = SUB_10338 * 0.014;
        auto MUL_10308 = SUB_10306 * 2.0;
        auto MUL_10386 = MUL_10308 * 0.0;
        auto ADD_10408 = MUL_10386 + MUL_10403;
        auto SUB_10412 = 0.0508 - ADD_10408;
        if(/*link_left_wheel*/ sphere_environment_in_collision(environment, ADD_10410, SUB_10411, SUB_10412, 0.052)){ return false; } // (194, 228)
        auto MUL_447 = COS_114 * 0.7071068;
        auto MUL_435 = SIN_108 * 0.7071068;
        auto MUL_10418 = MUL_447 * MUL_435;
        auto SUB_10441 = MUL_10418 - MUL_10418;
        auto ADD_10456 = MUL_10418 + MUL_10418;
        auto MUL_10459 = ADD_10456 * 2.0;
        auto MUL_10443 = SUB_10441 * 2.0;
        auto MUL_10416 = MUL_435 * MUL_435;
        auto ADD_10428 = MUL_10416 + MUL_10416;
        auto MUL_10431 = ADD_10428 * 2.0;
        auto SUB_10434 = 1.0 - MUL_10431;
        auto MUL_468 = SIN_108 * 0.245;
        auto MUL_472 = SIN_108 * MUL_468;
        auto MUL_474 = MUL_472 * 2.0;
        auto SUB_477 = MUL_474 - 0.245;
        auto ADD_493 = INPUT_0 + SUB_477;
        auto MUL_10507 = MUL_10443 * 0.000179;
        auto MUL_10514 = MUL_10459 * 0.0001548;
        auto MUL_10501 = SUB_10434 * 6.68e-05;
        auto ADD_10520 = MUL_10501 + MUL_10507;
        auto SUB_10523 = ADD_10520 - MUL_10514;
        auto ADD_10526 = ADD_493 + SUB_10523;
        auto ADD_10435 = MUL_10418 + MUL_10418;
        auto MUL_479 = COS_114 * MUL_468;
        auto MUL_10437 = ADD_10435 * 2.0;
        auto MUL_10503 = MUL_10437 * 6.68e-05;
        auto MUL_484 = MUL_479 * 2.0;
        auto SUB_494 = INPUT_1 - MUL_484;
        auto MUL_10421 = MUL_447 * MUL_447;
        auto SUB_10461 = MUL_10421 - MUL_10416;
        auto ADD_10444 = MUL_10416 + MUL_10421;
        auto MUL_10463 = SUB_10461 * 2.0;
        auto MUL_10517 = MUL_10463 * 0.0001548;
        auto MUL_10447 = ADD_10444 * 2.0;
        auto SUB_10450 = 1.0 - MUL_10447;
        auto MUL_10509 = SUB_10450 * 0.000179;
        auto ADD_10521 = MUL_10503 + MUL_10509;
        auto ADD_10524 = ADD_10521 + MUL_10517;
        auto ADD_10527 = SUB_494 + ADD_10524;
        auto SUB_10438 = MUL_10418 - MUL_10418;
        auto ADD_10464 = MUL_10416 + MUL_10421;
        auto MUL_10467 = ADD_10464 * 2.0;
        auto SUB_10470 = 1.0 - MUL_10467;
        auto MUL_10519 = SUB_10470 * 0.0001548;
        auto MUL_10454 = ADD_10464 * 2.0;
        auto MUL_10511 = MUL_10454 * 0.000179;
        auto MUL_10440 = SUB_10438 * 2.0;
        auto MUL_10505 = MUL_10440 * 6.68e-05;
        auto SUB_10522 = MUL_10505 - MUL_10511;
        auto ADD_10525 = SUB_10522 + MUL_10519;
        auto ADD_10528 = 0.032 + ADD_10525;
        if(/*caster_link*/ sphere_environment_in_collision(environment, ADD_10526, ADD_10527, ADD_10528, 0.0315)){ return false; } // (228, 279)
        auto SUB_10553 = MUL_10291 - MUL_10285;
        auto MUL_10555 = SUB_10553 * 2.0;
        auto MUL_10586 = MUL_10555 * 0.76;
        auto MUL_589 = SIN_108 * 0.135;
        auto MUL_602 = COS_114 * MUL_589;
        auto MUL_600 = SIN_108 * 0.067;
        auto MUL_605 = SIN_108 * MUL_600;
        auto SUB_606 = MUL_605 - MUL_602;
        auto MUL_608 = SUB_606 * 2.0;
        auto SUB_611 = MUL_608 - 0.067;
        auto ADD_631 = INPUT_0 + SUB_611;
        auto ADD_10597 = ADD_631 + MUL_10586;
        auto ADD_10556 = MUL_10284 + MUL_10186;
        auto MUL_613 = COS_114 * MUL_600;
        auto MUL_616 = SIN_108 * MUL_589;
        auto ADD_618 = MUL_613 + MUL_616;
        auto MUL_10559 = ADD_10556 * 2.0;
        auto SUB_10562 = 1.0 - MUL_10559;
        auto MUL_10588 = SUB_10562 * 0.76;
        auto MUL_621 = ADD_618 * 2.0;
        auto SUB_624 = 0.135 - MUL_621;
        auto ADD_632 = INPUT_1 + SUB_624;
        auto ADD_10598 = ADD_632 + MUL_10588;
        auto ADD_10563 = MUL_10192 + MUL_10187;
        auto MUL_10565 = ADD_10563 * 2.0;
        auto MUL_10590 = MUL_10565 * 0.76;
        auto ADD_10599 = 0.0284 + MUL_10590;
        auto MUL_10607 = MUL_10555 * 0.16;
        auto ADD_10618 = ADD_631 + MUL_10607;
        auto MUL_10609 = SUB_10562 * 0.16;
        auto ADD_10619 = ADD_632 + MUL_10609;
        auto MUL_10611 = MUL_10565 * 0.16;
        auto ADD_10620 = 0.0284 + MUL_10611;
        auto MUL_10628 = MUL_10555 * 0.23;
        auto ADD_10639 = ADD_631 + MUL_10628;
        auto MUL_10630 = SUB_10562 * 0.23;
        auto ADD_10640 = ADD_632 + MUL_10630;
        auto MUL_10632 = MUL_10565 * 0.23;
        auto ADD_10641 = 0.0284 + MUL_10632;
        auto MUL_10649 = MUL_10555 * 0.3;
        auto ADD_10660 = ADD_631 + MUL_10649;
        auto MUL_10651 = SUB_10562 * 0.3;
        auto ADD_10661 = ADD_632 + MUL_10651;
        auto MUL_10653 = MUL_10565 * 0.3;
        auto ADD_10662 = 0.0284 + MUL_10653;
        auto MUL_10670 = MUL_10555 * 0.37;
        auto ADD_10681 = ADD_631 + MUL_10670;
        auto MUL_10672 = SUB_10562 * 0.37;
        auto ADD_10682 = ADD_632 + MUL_10672;
        auto MUL_10674 = MUL_10565 * 0.37;
        auto ADD_10683 = 0.0284 + MUL_10674;
        auto MUL_10691 = MUL_10555 * 0.44;
        auto ADD_10702 = ADD_631 + MUL_10691;
        auto MUL_10693 = SUB_10562 * 0.44;
        auto ADD_10703 = ADD_632 + MUL_10693;
        auto MUL_10695 = MUL_10565 * 0.44;
        auto ADD_10704 = 0.0284 + MUL_10695;
        auto MUL_10712 = MUL_10555 * 0.51;
        auto ADD_10723 = ADD_631 + MUL_10712;
        auto MUL_10714 = SUB_10562 * 0.51;
        auto ADD_10724 = ADD_632 + MUL_10714;
        auto MUL_10716 = MUL_10565 * 0.51;
        auto ADD_10725 = 0.0284 + MUL_10716;
        auto MUL_10733 = MUL_10555 * 0.58;
        auto ADD_10744 = ADD_631 + MUL_10733;
        auto MUL_10735 = SUB_10562 * 0.58;
        auto ADD_10745 = ADD_632 + MUL_10735;
        auto MUL_10737 = MUL_10565 * 0.58;
        auto ADD_10746 = 0.0284 + MUL_10737;
        auto MUL_10754 = MUL_10555 * 0.65;
        auto ADD_10765 = ADD_631 + MUL_10754;
        auto MUL_10756 = SUB_10562 * 0.65;
        auto ADD_10766 = ADD_632 + MUL_10756;
        auto MUL_10758 = MUL_10565 * 0.65;
        auto ADD_10767 = 0.0284 + MUL_10758;
        auto MUL_10775 = MUL_10555 * 0.72;
        auto ADD_10786 = ADD_631 + MUL_10775;
        auto MUL_10777 = SUB_10562 * 0.72;
        auto ADD_10787 = ADD_632 + MUL_10777;
        auto MUL_10779 = MUL_10565 * 0.72;
        auto ADD_10788 = 0.0284 + MUL_10779;
        auto MUL_10796 = MUL_10555 * 0.79;
        auto ADD_10807 = ADD_631 + MUL_10796;
        auto MUL_10798 = SUB_10562 * 0.79;
        auto ADD_10808 = ADD_632 + MUL_10798;
        auto MUL_10800 = MUL_10565 * 0.79;
        auto ADD_10809 = 0.0284 + MUL_10800;
        auto MUL_10817 = MUL_10555 * 0.86;
        auto ADD_10828 = ADD_631 + MUL_10817;
        auto MUL_10819 = SUB_10562 * 0.86;
        auto ADD_10829 = ADD_632 + MUL_10819;
        auto MUL_10821 = MUL_10565 * 0.86;
        auto ADD_10830 = 0.0284 + MUL_10821;
        auto MUL_10838 = MUL_10555 * 0.93;
        auto ADD_10849 = ADD_631 + MUL_10838;
        auto MUL_10840 = SUB_10562 * 0.93;
        auto ADD_10850 = ADD_632 + MUL_10840;
        auto MUL_10842 = MUL_10565 * 0.93;
        auto ADD_10851 = 0.0284 + MUL_10842;
        auto ADD_10867 = ADD_631 + MUL_10555;
        auto ADD_10868 = ADD_632 + SUB_10562;
        auto ADD_10869 = 0.0284 + MUL_10565;
        auto MUL_10877 = MUL_10555 * 1.07;
        auto ADD_10888 = ADD_631 + MUL_10877;
        auto MUL_10879 = SUB_10562 * 1.07;
        auto ADD_10889 = ADD_632 + MUL_10879;
        auto MUL_10881 = MUL_10565 * 1.07;
        auto ADD_10890 = 0.0284 + MUL_10881;
        auto MUL_10898 = MUL_10555 * 1.14;
        auto ADD_10909 = ADD_631 + MUL_10898;
        auto MUL_10900 = SUB_10562 * 1.14;
        auto ADD_10910 = ADD_632 + MUL_10900;
        auto MUL_10902 = MUL_10565 * 1.14;
        auto ADD_10911 = 0.0284 + MUL_10902;
        auto MUL_10919 = MUL_10555 * 1.21;
        auto ADD_10930 = ADD_631 + MUL_10919;
        auto MUL_10921 = SUB_10562 * 1.21;
        auto ADD_10931 = ADD_632 + MUL_10921;
        auto MUL_10923 = MUL_10565 * 1.21;
        auto ADD_10932 = 0.0284 + MUL_10923;
        auto MUL_10940 = MUL_10555 * 1.28;
        auto ADD_10951 = ADD_631 + MUL_10940;
        auto MUL_10942 = SUB_10562 * 1.28;
        auto ADD_10952 = ADD_632 + MUL_10942;
        auto MUL_10944 = MUL_10565 * 1.28;
        auto ADD_10953 = 0.0284 + MUL_10944;
        if(/*link_mast*/ sphere_environment_in_collision(environment, ADD_10597, ADD_10598, ADD_10599, 0.55)){ if(sphere_environment_in_collision(environment, ADD_10618, ADD_10619, ADD_10620, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10639, ADD_10640, ADD_10641, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10660, ADD_10661, ADD_10662, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10681, ADD_10682, ADD_10683, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10702, ADD_10703, ADD_10704, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10723, ADD_10724, ADD_10725, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10744, ADD_10745, ADD_10746, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10765, ADD_10766, ADD_10767, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10786, ADD_10787, ADD_10788, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10807, ADD_10808, ADD_10809, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10828, ADD_10829, ADD_10830, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10849, ADD_10850, ADD_10851, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10867, ADD_10868, ADD_10869, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10888, ADD_10889, ADD_10890, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10909, ADD_10910, ADD_10911, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10930, ADD_10931, ADD_10932, 0.045)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_10951, ADD_10952, ADD_10953, 0.045)){ return false; } } // (279, 405)
        auto MUL_708 = MUL_148 * 0.5000018;
        auto MUL_742 = MUL_174 * 0.5000018;
        auto MUL_719 = MUL_176 * 0.5000018;
        auto MUL_700 = MUL_164 * 0.5000018;
        auto MUL_735 = MUL_148 * 0.5;
        auto ADD_722 = MUL_719 + MUL_735;
        auto MUL_715 = MUL_174 * 0.5;
        auto MUL_706 = MUL_176 * 0.5;
        auto SUB_709 = MUL_706 - MUL_708;
        auto MUL_739 = MUL_164 * 0.5;
        auto ADD_727 = ADD_722 + MUL_739;
        auto MUL_697 = MUL_148 * 0.4999982;
        auto SUB_698 = MUL_697 - MUL_706;
        auto ADD_701 = SUB_698 + MUL_700;
        auto SUB_704 = ADD_701 - MUL_715;
        auto MUL_729 = MUL_174 * 0.4999982;
        auto ADD_730 = ADD_727 + MUL_729;
        auto MUL_10991 = ADD_730 * ADD_730;
        auto MUL_10997 = SUB_704 * ADD_730;
        auto MUL_732 = MUL_176 * 0.4999982;
        auto ADD_737 = MUL_732 + MUL_735;
        auto SUB_740 = ADD_737 - MUL_739;
        auto SUB_743 = SUB_740 - MUL_742;
        auto MUL_10992 = SUB_743 * ADD_730;
        auto MUL_711 = MUL_164 * 0.4999982;
        auto ADD_712 = SUB_709 + MUL_711;
        auto SUB_717 = ADD_712 - MUL_715;
        auto MUL_10993 = SUB_743 * SUB_717;
        auto ADD_11025 = MUL_10997 + MUL_10993;
        auto MUL_11027 = ADD_11025 * 2.0;
        auto MUL_10990 = SUB_717 * SUB_717;
        auto ADD_10999 = MUL_10990 + MUL_10991;
        auto MUL_11002 = ADD_10999 * 2.0;
        auto SUB_11005 = 1.0 - MUL_11002;
        auto MUL_10996 = SUB_704 * SUB_717;
        auto SUB_11012 = MUL_10996 - MUL_10992;
        auto MUL_11014 = SUB_11012 * 2.0;
        auto MUL_750 = MUL_148 * 0.1666;
        auto MUL_747 = MUL_174 * 0.1666;
        auto MUL_762 = MUL_176 * MUL_747;
        auto MUL_11040 = SUB_11005 * 0.041;
        auto MUL_760 = MUL_174 * 0.037385;
        auto MUL_766 = MUL_174 * MUL_760;
        auto MUL_753 = MUL_164 * 0.037385;
        auto ADD_755 = MUL_750 + MUL_753;
        auto MUL_764 = MUL_164 * ADD_755;
        auto SUB_765 = MUL_764 - MUL_762;
        auto ADD_767 = SUB_765 + MUL_766;
        auto MUL_769 = ADD_767 * 2.0;
        auto SUB_772 = MUL_769 - 0.037385;
        auto ADD_797 = ADD_631 + SUB_772;
        auto MUL_11051 = MUL_11014 * 0.033;
        auto SUB_11062 = MUL_11051 - MUL_11040;
        auto MUL_11057 = MUL_11027 * 0.02;
        auto ADD_11065 = SUB_11062 + MUL_11057;
        auto INPUT_3 = q[3];
        auto MUL_835 = SUB_717 * INPUT_3;
        auto MUL_841 = SUB_743 * MUL_835;
        auto MUL_839 = SUB_704 * INPUT_3;
        auto MUL_843 = ADD_730 * MUL_839;
        auto ADD_844 = MUL_841 + MUL_843;
        auto MUL_846 = ADD_844 * 2.0;
        auto ADD_865 = ADD_797 + MUL_846;
        auto ADD_11068 = ADD_865 + ADD_11065;
        auto ADD_11006 = MUL_10996 + MUL_10992;
        auto MUL_776 = MUL_148 * ADD_755;
        auto MUL_11008 = ADD_11006 * 2.0;
        auto MUL_11044 = MUL_11008 * 0.041;
        auto MUL_848 = SUB_743 * MUL_839;
        auto MUL_10995 = SUB_743 * SUB_704;
        auto MUL_851 = ADD_730 * MUL_835;
        auto SUB_852 = MUL_851 - MUL_848;
        auto MUL_854 = SUB_852 * 2.0;
        auto MUL_10998 = SUB_717 * ADD_730;
        auto SUB_11028 = MUL_10998 - MUL_10995;
        auto MUL_11030 = SUB_11028 * 2.0;
        auto MUL_11059 = MUL_11030 * 0.02;
        auto MUL_10994 = SUB_704 * SUB_704;
        auto ADD_11015 = MUL_10991 + MUL_10994;
        auto MUL_11018 = ADD_11015 * 2.0;
        auto SUB_11021 = 1.0 - MUL_11018;
        auto MUL_11053 = SUB_11021 * 0.033;
        auto SUB_11063 = MUL_11053 - MUL_11044;
        auto ADD_11066 = SUB_11063 + MUL_11059;
        auto MUL_779 = MUL_174 * MUL_747;
        auto MUL_774 = MUL_176 * MUL_760;
        auto ADD_777 = MUL_774 + MUL_776;
        auto ADD_781 = ADD_777 + MUL_779;
        auto MUL_784 = ADD_781 * 2.0;
        auto SUB_787 = 0.1666 - MUL_784;
        auto ADD_798 = ADD_632 + SUB_787;
        auto ADD_866 = ADD_798 + MUL_854;
        auto ADD_11069 = ADD_866 + ADD_11066;
        auto SUB_11009 = MUL_10997 - MUL_10993;
        auto ADD_11022 = MUL_10998 + MUL_10995;
        auto ADD_11031 = MUL_10990 + MUL_10994;
        auto MUL_789 = MUL_148 * MUL_760;
        auto MUL_11034 = ADD_11031 * 2.0;
        auto SUB_11037 = 1.0 - MUL_11034;
        auto MUL_11061 = SUB_11037 * 0.02;
        auto MUL_11024 = ADD_11022 * 2.0;
        auto MUL_11055 = MUL_11024 * 0.033;
        auto MUL_11011 = SUB_11009 * 2.0;
        auto MUL_11048 = MUL_11011 * 0.041;
        auto SUB_11064 = MUL_11055 - MUL_11048;
        auto ADD_11067 = SUB_11064 + MUL_11061;
        auto MUL_858 = SUB_717 * MUL_835;
        auto MUL_856 = SUB_704 * MUL_839;
        auto ADD_859 = MUL_856 + MUL_858;
        auto MUL_862 = ADD_859 * 2.0;
        auto SUB_864 = INPUT_3 - MUL_862;
        auto MUL_788 = MUL_176 * ADD_755;
        auto SUB_790 = MUL_788 - MUL_789;
        auto MUL_791 = MUL_164 * MUL_747;
        auto ADD_793 = SUB_790 + MUL_791;
        auto MUL_795 = ADD_793 * 2.0;
        auto ADD_799 = 0.0284 + MUL_795;
        auto ADD_867 = ADD_799 + SUB_864;
        auto ADD_11070 = ADD_867 + ADD_11067;
        auto MUL_11079 = MUL_11014 * 0.02;
        auto MUL_11090 = MUL_11027 * 0.015;
        auto SUB_11095 = MUL_11090 - MUL_11079;
        auto ADD_11098 = ADD_865 + SUB_11095;
        auto MUL_11092 = MUL_11030 * 0.015;
        auto MUL_11083 = SUB_11021 * 0.02;
        auto SUB_11096 = MUL_11092 - MUL_11083;
        auto ADD_11099 = ADD_866 + SUB_11096;
        auto MUL_11094 = SUB_11037 * 0.015;
        auto MUL_11087 = MUL_11024 * 0.02;
        auto SUB_11097 = MUL_11094 - MUL_11087;
        auto ADD_11100 = ADD_867 + SUB_11097;
        auto MUL_11114 = MUL_11014 * 0.08;
        auto SUB_11125 = MUL_11114 - MUL_11040;
        auto ADD_11128 = SUB_11125 + MUL_11057;
        auto ADD_11131 = ADD_865 + ADD_11128;
        auto MUL_11116 = SUB_11021 * 0.08;
        auto SUB_11126 = MUL_11116 - MUL_11044;
        auto ADD_11129 = SUB_11126 + MUL_11059;
        auto ADD_11132 = ADD_866 + ADD_11129;
        auto MUL_11118 = MUL_11024 * 0.08;
        auto SUB_11127 = MUL_11118 - MUL_11048;
        auto ADD_11130 = SUB_11127 + MUL_11061;
        auto ADD_11133 = ADD_867 + ADD_11130;
        if(/*link_lift*/ sphere_environment_in_collision(environment, ADD_11068, ADD_11069, ADD_11070, 0.14)){ if(sphere_environment_in_collision(environment, ADD_11098, ADD_11099, ADD_11100, 0.08)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_11131, ADD_11132, ADD_11133, 0.105)){ return false; } } // (405, 548)
        auto MUL_885 = SUB_743 * 0.5000018;
        auto MUL_900 = SUB_743 * 0.5;
        auto MUL_916 = SUB_743 * 0.4999982;
        auto MUL_880 = ADD_730 * 0.5000018;
        auto MUL_896 = ADD_730 * 0.5;
        auto MUL_913 = ADD_730 * 0.4999982;
        auto MUL_922 = SUB_717 * 0.5000018;
        auto MUL_909 = SUB_717 * 0.5;
        auto MUL_893 = SUB_717 * 0.4999982;
        auto MUL_904 = SUB_704 * 0.5000018;
        auto ADD_906 = MUL_900 + MUL_904;
        auto ADD_910 = ADD_906 + MUL_909;
        auto SUB_914 = MUL_913 - ADD_910;
        auto MUL_889 = SUB_704 * 0.5;
        auto SUB_919 = MUL_916 - MUL_889;
        auto SUB_891 = MUL_889 - MUL_885;
        auto ADD_924 = SUB_919 + MUL_922;
        auto ADD_929 = ADD_924 + MUL_896;
        auto ADD_894 = SUB_891 + MUL_893;
        auto ADD_897 = ADD_894 + MUL_896;
        auto MUL_11143 = ADD_929 * ADD_897;
        auto MUL_871 = SUB_704 * 0.4999982;
        auto ADD_872 = MUL_900 + MUL_871;
        auto SUB_877 = ADD_872 - MUL_909;
        auto ADD_882 = SUB_877 + MUL_880;
        auto MUL_11147 = ADD_882 * SUB_914;
        auto ADD_11175 = MUL_11147 + MUL_11143;
        auto MUL_11177 = ADD_11175 * 2.0;
        auto MUL_944 = ADD_730 * 0.2547;
        auto MUL_948 = ADD_730 * MUL_944;
        auto MUL_938 = SUB_717 * 0.2547;
        auto MUL_947 = SUB_717 * MUL_938;
        auto ADD_949 = MUL_947 + MUL_948;
        auto MUL_951 = ADD_949 * 2.0;
        auto SUB_954 = MUL_951 - 0.2547;
        auto ADD_973 = ADD_865 + SUB_954;
        auto MUL_11202 = MUL_11177 * 0.11;
        auto SUB_11212 = ADD_973 - MUL_11202;
        auto MUL_11145 = ADD_929 * ADD_882;
        auto MUL_11148 = ADD_897 * SUB_914;
        auto SUB_11178 = MUL_11148 - MUL_11145;
        auto MUL_11180 = SUB_11178 * 2.0;
        auto MUL_11206 = MUL_11180 * 0.11;
        auto MUL_956 = SUB_743 * MUL_944;
        auto MUL_958 = SUB_704 * MUL_938;
        auto ADD_959 = MUL_956 + MUL_958;
        auto MUL_963 = ADD_959 * 2.0;
        auto SUB_974 = ADD_866 - MUL_963;
        auto SUB_11213 = SUB_974 - MUL_11206;
        auto MUL_11140 = ADD_897 * ADD_897;
        auto MUL_11144 = ADD_882 * ADD_882;
        auto ADD_11181 = MUL_11140 + MUL_11144;
        auto MUL_11184 = ADD_11181 * 2.0;
        auto SUB_11187 = 1.0 - MUL_11184;
        auto MUL_11210 = SUB_11187 * 0.11;
        auto MUL_966 = SUB_743 * MUL_938;
        auto MUL_967 = SUB_704 * MUL_944;
        auto SUB_968 = MUL_966 - MUL_967;
        auto MUL_971 = SUB_968 * 2.0;
        auto ADD_975 = ADD_867 + MUL_971;
        auto SUB_11214 = ADD_975 - MUL_11210;
        auto MUL_11229 = MUL_11177 * 0.01;
        auto SUB_11239 = ADD_973 - MUL_11229;
        auto MUL_11233 = MUL_11180 * 0.01;
        auto SUB_11240 = SUB_974 - MUL_11233;
        auto MUL_11237 = SUB_11187 * 0.01;
        auto SUB_11241 = ADD_975 - MUL_11237;
        auto MUL_11283 = MUL_11177 * 0.16;
        auto SUB_11293 = ADD_973 - MUL_11283;
        auto MUL_11287 = MUL_11180 * 0.16;
        auto SUB_11294 = SUB_974 - MUL_11287;
        auto MUL_11291 = SUB_11187 * 0.16;
        auto SUB_11295 = ADD_975 - MUL_11291;
        auto MUL_11310 = MUL_11177 * 0.06;
        auto SUB_11320 = ADD_973 - MUL_11310;
        auto MUL_11314 = MUL_11180 * 0.06;
        auto SUB_11321 = SUB_974 - MUL_11314;
        auto MUL_11318 = SUB_11187 * 0.06;
        auto SUB_11322 = ADD_975 - MUL_11318;
        if(/*link_arm_l4*/ sphere_environment_in_collision(environment, SUB_11212, SUB_11213, SUB_11214, 0.12)){ if(sphere_environment_in_collision(environment, SUB_11239, SUB_11240, SUB_11241, 0.042)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11212, SUB_11213, SUB_11214, 0.04)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11293, SUB_11294, SUB_11295, 0.04)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11320, SUB_11321, SUB_11322, 0.04)){ return false; } } // (548, 627)
        auto MUL_11370 = ADD_11175 * 2.0;
        auto MUL_11395 = MUL_11370 * 0.08;
        auto MUL_1064 = ADD_897 * 0.013;
        auto MUL_1075 = ADD_929 * MUL_1064;
        auto MUL_1072 = ADD_882 * 0.013;
        auto MUL_1077 = SUB_914 * MUL_1072;
        auto ADD_1078 = MUL_1075 + MUL_1077;
        auto MUL_1080 = ADD_1078 * 2.0;
        auto ADD_1102 = ADD_973 + MUL_1080;
        auto INPUT_4 = q[4];
        auto MUL_1140 = ADD_897 * INPUT_4;
        auto MUL_1146 = ADD_929 * MUL_1140;
        auto MUL_1144 = ADD_882 * INPUT_4;
        auto MUL_1148 = SUB_914 * MUL_1144;
        auto ADD_1149 = MUL_1146 + MUL_1148;
        auto MUL_1151 = ADD_1149 * 2.0;
        auto ADD_1170 = ADD_1102 + MUL_1151;
        auto SUB_11405 = ADD_1170 - MUL_11395;
        auto MUL_1153 = ADD_929 * MUL_1144;
        auto MUL_1083 = ADD_929 * MUL_1072;
        auto MUL_1156 = SUB_914 * MUL_1140;
        auto SUB_1157 = MUL_1156 - MUL_1153;
        auto MUL_1159 = SUB_1157 * 2.0;
        auto MUL_1086 = SUB_914 * MUL_1064;
        auto SUB_1087 = MUL_1086 - MUL_1083;
        auto MUL_1089 = SUB_1087 * 2.0;
        auto ADD_1103 = SUB_974 + MUL_1089;
        auto ADD_1171 = ADD_1103 + MUL_1159;
        auto MUL_11373 = SUB_11178 * 2.0;
        auto MUL_11399 = MUL_11373 * 0.08;
        auto SUB_11406 = ADD_1171 - MUL_11399;
        auto MUL_1163 = ADD_897 * MUL_1140;
        auto MUL_1094 = ADD_897 * MUL_1064;
        auto MUL_1161 = ADD_882 * MUL_1144;
        auto ADD_1164 = MUL_1161 + MUL_1163;
        auto MUL_1167 = ADD_1164 * 2.0;
        auto SUB_1169 = INPUT_4 - MUL_1167;
        auto MUL_1092 = ADD_882 * MUL_1072;
        auto ADD_1095 = MUL_1092 + MUL_1094;
        auto MUL_1098 = ADD_1095 * 2.0;
        auto SUB_1101 = 0.013 - MUL_1098;
        auto ADD_1104 = ADD_975 + SUB_1101;
        auto ADD_1172 = ADD_1104 + SUB_1169;
        auto MUL_11377 = ADD_11181 * 2.0;
        auto SUB_11380 = 1.0 - MUL_11377;
        auto MUL_11403 = SUB_11380 * 0.08;
        auto SUB_11407 = ADD_1172 - MUL_11403;
        auto MUL_11422 = MUL_11370 * 0.01;
        auto SUB_11432 = ADD_1170 - MUL_11422;
        auto MUL_11426 = MUL_11373 * 0.01;
        auto SUB_11433 = ADD_1171 - MUL_11426;
        auto MUL_11430 = SUB_11380 * 0.01;
        auto SUB_11434 = ADD_1172 - MUL_11430;
        auto MUL_11449 = MUL_11370 * 0.06;
        auto SUB_11459 = ADD_1170 - MUL_11449;
        auto MUL_11453 = MUL_11373 * 0.06;
        auto SUB_11460 = ADD_1171 - MUL_11453;
        auto MUL_11457 = SUB_11380 * 0.06;
        auto SUB_11461 = ADD_1172 - MUL_11457;
        auto MUL_11476 = MUL_11370 * 0.11;
        auto SUB_11486 = ADD_1170 - MUL_11476;
        auto MUL_11480 = MUL_11373 * 0.11;
        auto SUB_11487 = ADD_1171 - MUL_11480;
        auto MUL_11484 = SUB_11380 * 0.11;
        auto SUB_11488 = ADD_1172 - MUL_11484;
        if(/*link_arm_l3*/ sphere_environment_in_collision(environment, SUB_11405, SUB_11406, SUB_11407, 0.087)){ if(sphere_environment_in_collision(environment, SUB_11432, SUB_11433, SUB_11434, 0.042)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11459, SUB_11460, SUB_11461, 0.04)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11486, SUB_11487, SUB_11488, 0.04)){ return false; } } // (627, 692)
        auto ADD_1216 = MUL_1075 + MUL_1077;
        auto MUL_1218 = ADD_1216 * 2.0;
        auto ADD_1240 = ADD_1170 + MUL_1218;
        auto MUL_11534 = ADD_11175 * 2.0;
        auto MUL_11559 = MUL_11534 * 0.08;
        auto INPUT_5 = q[5];
        auto MUL_1278 = ADD_897 * INPUT_5;
        auto MUL_1284 = ADD_929 * MUL_1278;
        auto MUL_1282 = ADD_882 * INPUT_5;
        auto MUL_1286 = SUB_914 * MUL_1282;
        auto ADD_1287 = MUL_1284 + MUL_1286;
        auto MUL_1289 = ADD_1287 * 2.0;
        auto ADD_1308 = ADD_1240 + MUL_1289;
        auto SUB_11569 = ADD_1308 - MUL_11559;
        auto SUB_1225 = MUL_1086 - MUL_1083;
        auto MUL_1227 = SUB_1225 * 2.0;
        auto ADD_1241 = ADD_1171 + MUL_1227;
        auto MUL_1291 = ADD_929 * MUL_1282;
        auto MUL_1294 = SUB_914 * MUL_1278;
        auto SUB_1295 = MUL_1294 - MUL_1291;
        auto MUL_1297 = SUB_1295 * 2.0;
        auto ADD_1309 = ADD_1241 + MUL_1297;
        auto MUL_11537 = SUB_11178 * 2.0;
        auto MUL_11563 = MUL_11537 * 0.08;
        auto SUB_11570 = ADD_1309 - MUL_11563;
        auto ADD_1233 = MUL_1092 + MUL_1094;
        auto MUL_1236 = ADD_1233 * 2.0;
        auto SUB_1239 = 0.013 - MUL_1236;
        auto ADD_1242 = ADD_1172 + SUB_1239;
        auto MUL_1301 = ADD_897 * MUL_1278;
        auto MUL_1299 = ADD_882 * MUL_1282;
        auto ADD_1302 = MUL_1299 + MUL_1301;
        auto MUL_1305 = ADD_1302 * 2.0;
        auto SUB_1307 = INPUT_5 - MUL_1305;
        auto ADD_1310 = ADD_1242 + SUB_1307;
        auto MUL_11541 = ADD_11181 * 2.0;
        auto SUB_11544 = 1.0 - MUL_11541;
        auto MUL_11567 = SUB_11544 * 0.08;
        auto SUB_11571 = ADD_1310 - MUL_11567;
        auto MUL_11586 = MUL_11534 * 0.01;
        auto SUB_11596 = ADD_1308 - MUL_11586;
        auto MUL_11590 = MUL_11537 * 0.01;
        auto SUB_11597 = ADD_1309 - MUL_11590;
        auto MUL_11594 = SUB_11544 * 0.01;
        auto SUB_11598 = ADD_1310 - MUL_11594;
        auto MUL_11613 = MUL_11534 * 0.06;
        auto SUB_11623 = ADD_1308 - MUL_11613;
        auto MUL_11617 = MUL_11537 * 0.06;
        auto SUB_11624 = ADD_1309 - MUL_11617;
        auto MUL_11621 = SUB_11544 * 0.06;
        auto SUB_11625 = ADD_1310 - MUL_11621;
        auto MUL_11640 = MUL_11534 * 0.11;
        auto SUB_11650 = ADD_1308 - MUL_11640;
        auto MUL_11644 = MUL_11537 * 0.11;
        auto SUB_11651 = ADD_1309 - MUL_11644;
        auto MUL_11648 = SUB_11544 * 0.11;
        auto SUB_11652 = ADD_1310 - MUL_11648;
        if(/*link_arm_l2*/ sphere_environment_in_collision(environment, SUB_11569, SUB_11570, SUB_11571, 0.087)){ if(sphere_environment_in_collision(environment, SUB_11596, SUB_11597, SUB_11598, 0.04)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11623, SUB_11624, SUB_11625, 0.036)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11650, SUB_11651, SUB_11652, 0.036)){ return false; } } // (692, 749)
        auto ADD_1354 = MUL_1075 + MUL_1077;
        auto MUL_1356 = ADD_1354 * 2.0;
        auto ADD_1378 = ADD_1308 + MUL_1356;
        auto MUL_11698 = ADD_11175 * 2.0;
        auto MUL_11723 = MUL_11698 * 0.08;
        auto INPUT_6 = q[6];
        auto MUL_1416 = ADD_897 * INPUT_6;
        auto MUL_1422 = ADD_929 * MUL_1416;
        auto MUL_1420 = ADD_882 * INPUT_6;
        auto MUL_1424 = SUB_914 * MUL_1420;
        auto ADD_1425 = MUL_1422 + MUL_1424;
        auto MUL_1427 = ADD_1425 * 2.0;
        auto ADD_1446 = ADD_1378 + MUL_1427;
        auto SUB_11733 = ADD_1446 - MUL_11723;
        auto SUB_1363 = MUL_1086 - MUL_1083;
        auto MUL_1365 = SUB_1363 * 2.0;
        auto ADD_1379 = ADD_1309 + MUL_1365;
        auto MUL_1429 = ADD_929 * MUL_1420;
        auto MUL_1432 = SUB_914 * MUL_1416;
        auto SUB_1433 = MUL_1432 - MUL_1429;
        auto MUL_1435 = SUB_1433 * 2.0;
        auto ADD_1447 = ADD_1379 + MUL_1435;
        auto MUL_11701 = SUB_11178 * 2.0;
        auto MUL_11727 = MUL_11701 * 0.08;
        auto SUB_11734 = ADD_1447 - MUL_11727;
        auto ADD_1371 = MUL_1092 + MUL_1094;
        auto MUL_1374 = ADD_1371 * 2.0;
        auto SUB_1377 = 0.013 - MUL_1374;
        auto ADD_1380 = ADD_1310 + SUB_1377;
        auto MUL_1439 = ADD_897 * MUL_1416;
        auto MUL_1437 = ADD_882 * MUL_1420;
        auto ADD_1440 = MUL_1437 + MUL_1439;
        auto MUL_1443 = ADD_1440 * 2.0;
        auto SUB_1445 = INPUT_6 - MUL_1443;
        auto ADD_1448 = ADD_1380 + SUB_1445;
        auto MUL_11705 = ADD_11181 * 2.0;
        auto SUB_11708 = 1.0 - MUL_11705;
        auto MUL_11731 = SUB_11708 * 0.08;
        auto SUB_11735 = ADD_1448 - MUL_11731;
        auto MUL_11750 = MUL_11698 * 0.01;
        auto SUB_11760 = ADD_1446 - MUL_11750;
        auto MUL_11754 = MUL_11701 * 0.01;
        auto SUB_11761 = ADD_1447 - MUL_11754;
        auto MUL_11758 = SUB_11708 * 0.01;
        auto SUB_11762 = ADD_1448 - MUL_11758;
        auto MUL_11777 = MUL_11698 * 0.06;
        auto SUB_11787 = ADD_1446 - MUL_11777;
        auto MUL_11781 = MUL_11701 * 0.06;
        auto SUB_11788 = ADD_1447 - MUL_11781;
        auto MUL_11785 = SUB_11708 * 0.06;
        auto SUB_11789 = ADD_1448 - MUL_11785;
        auto MUL_11804 = MUL_11698 * 0.11;
        auto SUB_11814 = ADD_1446 - MUL_11804;
        auto MUL_11808 = MUL_11701 * 0.11;
        auto SUB_11815 = ADD_1447 - MUL_11808;
        auto MUL_11812 = SUB_11708 * 0.11;
        auto SUB_11816 = ADD_1448 - MUL_11812;
        if(/*link_arm_l1*/ sphere_environment_in_collision(environment, SUB_11733, SUB_11734, SUB_11735, 0.087)){ if(sphere_environment_in_collision(environment, SUB_11760, SUB_11761, SUB_11762, 0.034)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11787, SUB_11788, SUB_11789, 0.034)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11814, SUB_11815, SUB_11816, 0.034)){ return false; } } // (749, 806)
        auto MUL_11826 = SUB_914 * SUB_914;
        auto ADD_11834 = MUL_11140 + MUL_11826;
        auto MUL_11862 = ADD_11175 * 2.0;
        auto MUL_11837 = ADD_11834 * 2.0;
        auto SUB_11840 = 1.0 - MUL_11837;
        auto MUL_11887 = MUL_11862 * 0.05;
        auto MUL_11874 = SUB_11840 * 0.03;
        auto SUB_11897 = MUL_11874 - MUL_11887;
        auto MUL_1479 = ADD_897 * 0.01375;
        auto MUL_1493 = ADD_929 * MUL_1479;
        auto MUL_1489 = ADD_882 * 0.01375;
        auto MUL_1496 = SUB_914 * MUL_1489;
        auto ADD_1498 = MUL_1493 + MUL_1496;
        auto MUL_1501 = ADD_1498 * 2.0;
        auto SUB_1524 = ADD_1446 - MUL_1501;
        auto INPUT_7 = q[7];
        auto MUL_1562 = ADD_897 * INPUT_7;
        auto MUL_1568 = ADD_929 * MUL_1562;
        auto MUL_1566 = ADD_882 * INPUT_7;
        auto MUL_1570 = SUB_914 * MUL_1566;
        auto ADD_1571 = MUL_1568 + MUL_1570;
        auto MUL_1573 = ADD_1571 * 2.0;
        auto ADD_1592 = SUB_1524 + MUL_1573;
        auto ADD_11900 = ADD_1592 + SUB_11897;
        auto MUL_1575 = ADD_929 * MUL_1566;
        auto MUL_1505 = ADD_929 * MUL_1489;
        auto MUL_11827 = ADD_929 * SUB_914;
        auto MUL_1578 = SUB_914 * MUL_1562;
        auto SUB_1579 = MUL_1578 - MUL_1575;
        auto MUL_1581 = SUB_1579 * 2.0;
        auto MUL_1507 = SUB_914 * MUL_1479;
        auto SUB_1509 = MUL_1505 - MUL_1507;
        auto MUL_1511 = SUB_1509 * 2.0;
        auto ADD_1525 = ADD_1447 + MUL_1511;
        auto ADD_1593 = ADD_1525 + MUL_1581;
        auto MUL_11831 = ADD_882 * ADD_897;
        auto ADD_11841 = MUL_11831 + MUL_11827;
        auto MUL_11865 = SUB_11178 * 2.0;
        auto MUL_11891 = MUL_11865 * 0.05;
        auto MUL_11843 = ADD_11841 * 2.0;
        auto MUL_11876 = MUL_11843 * 0.03;
        auto SUB_11898 = MUL_11876 - MUL_11891;
        auto ADD_11901 = ADD_1593 + SUB_11898;
        auto SUB_11844 = MUL_11147 - MUL_11143;
        auto MUL_1585 = ADD_897 * MUL_1562;
        auto MUL_1516 = ADD_897 * MUL_1479;
        auto MUL_1583 = ADD_882 * MUL_1566;
        auto ADD_1586 = MUL_1583 + MUL_1585;
        auto MUL_1589 = ADD_1586 * 2.0;
        auto SUB_1591 = INPUT_7 - MUL_1589;
        auto MUL_1514 = ADD_882 * MUL_1489;
        auto ADD_1518 = MUL_1514 + MUL_1516;
        auto MUL_1520 = ADD_1518 * 2.0;
        auto SUB_1523 = MUL_1520 - 0.01375;
        auto ADD_1526 = ADD_1448 + SUB_1523;
        auto ADD_1594 = ADD_1526 + SUB_1591;
        auto MUL_11869 = ADD_11181 * 2.0;
        auto SUB_11872 = 1.0 - MUL_11869;
        auto MUL_11895 = SUB_11872 * 0.05;
        auto MUL_11846 = SUB_11844 * 2.0;
        auto MUL_11878 = MUL_11846 * 0.03;
        auto SUB_11899 = MUL_11878 - MUL_11895;
        auto ADD_11902 = ADD_1594 + SUB_11899;
        auto MUL_11904 = SUB_11840 * 0.003;
        auto ADD_11921 = ADD_1592 + MUL_11904;
        auto MUL_11906 = MUL_11843 * 0.003;
        auto ADD_11922 = ADD_1593 + MUL_11906;
        auto MUL_11908 = MUL_11846 * 0.003;
        auto ADD_11923 = ADD_1594 + MUL_11908;
        auto MUL_11938 = MUL_11862 * 0.035;
        auto SUB_11948 = ADD_1592 - MUL_11938;
        auto MUL_11942 = MUL_11865 * 0.035;
        auto SUB_11949 = ADD_1593 - MUL_11942;
        auto MUL_11946 = SUB_11872 * 0.035;
        auto SUB_11950 = ADD_1594 - MUL_11946;
        auto MUL_11965 = MUL_11862 * 0.075;
        auto SUB_11975 = ADD_1592 - MUL_11965;
        auto MUL_11969 = MUL_11865 * 0.075;
        auto SUB_11976 = ADD_1593 - MUL_11969;
        auto MUL_11973 = SUB_11872 * 0.075;
        auto SUB_11977 = ADD_1594 - MUL_11973;
        auto MUL_11979 = SUB_11840 * 0.047;
        auto MUL_11992 = MUL_11862 * 0.0025;
        auto SUB_12002 = MUL_11979 - MUL_11992;
        auto ADD_12005 = ADD_1592 + SUB_12002;
        auto MUL_11996 = MUL_11865 * 0.0025;
        auto MUL_11981 = MUL_11843 * 0.047;
        auto SUB_12003 = MUL_11981 - MUL_11996;
        auto ADD_12006 = ADD_1593 + SUB_12003;
        auto MUL_12000 = SUB_11872 * 0.0025;
        auto MUL_11983 = MUL_11846 * 0.047;
        auto SUB_12004 = MUL_11983 - MUL_12000;
        auto ADD_12007 = ADD_1594 + SUB_12004;
        auto MUL_12009 = SUB_11840 * 0.07;
        auto ADD_12026 = ADD_1592 + MUL_12009;
        auto MUL_12011 = MUL_11843 * 0.07;
        auto ADD_12027 = ADD_1593 + MUL_12011;
        auto MUL_12013 = MUL_11846 * 0.07;
        auto ADD_12028 = ADD_1594 + MUL_12013;
        if(/*link_arm_l0*/ sphere_environment_in_collision(environment, ADD_11900, ADD_11901, ADD_11902, 0.09)){ if(sphere_environment_in_collision(environment, ADD_11921, ADD_11922, ADD_11923, 0.0345)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11948, SUB_11949, SUB_11950, 0.034)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_11975, SUB_11976, SUB_11977, 0.034)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_12005, ADD_12006, ADD_12007, 0.042)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_12026, ADD_12027, ADD_12028, 0.034)){ return false; } } // (806, 905)
        auto MUL_1608 = ADD_929 * 0.7071068;
        auto MUL_1603 = SUB_914 * 0.7071068;
        auto MUL_1600 = ADD_897 * 0.7071068;
        auto SUB_1639 = MUL_1600 - MUL_1603;
        auto ADD_1605 = MUL_1600 + MUL_1603;
        auto MUL_1611 = ADD_882 * 0.7071068;
        auto SUB_1624 = MUL_1608 - MUL_1611;
        auto ADD_1612 = MUL_1608 + MUL_1611;
        auto MUL_1657 = SUB_914 * 0.083;
        auto MUL_1663 = SUB_914 * MUL_1657;
        auto MUL_1651 = ADD_897 * 0.083;
        auto MUL_1644 = SUB_914 * 0.03075;
        auto MUL_1659 = ADD_929 * MUL_1644;
        auto MUL_1648 = ADD_882 * 0.03075;
        auto ADD_1652 = MUL_1648 + MUL_1651;
        auto MUL_1660 = ADD_897 * ADD_1652;
        auto SUB_1662 = MUL_1659 - MUL_1660;
        auto SUB_1665 = SUB_1662 - MUL_1663;
        auto MUL_1667 = SUB_1665 * 2.0;
        auto ADD_1669 = MUL_1667 + 0.083;
        auto ADD_1692 = ADD_1592 + ADD_1669;
        auto INPUT_8 = q[8];
        auto DIV_1696 = INPUT_8 * 0.5;
        auto SIN_1697 = DIV_1696.sin();
        auto COS_1706 = DIV_1696.cos();
        auto MUL_1704 = SIN_1697 * 1.0;
        auto MUL_1719 = SUB_1639 * MUL_1704;
        auto MUL_1725 = SUB_1639 * COS_1706;
        auto MUL_1728 = SUB_1624 * MUL_1704;
        auto ADD_1730 = MUL_1725 + MUL_1728;
        auto MUL_1723 = SUB_1624 * COS_1706;
        auto SUB_1724 = MUL_1723 - MUL_1719;
        auto MUL_12043 = ADD_1730 * SUB_1724;
        auto MUL_12042 = SUB_1724 * SUB_1724;
        auto MUL_1709 = ADD_1612 * MUL_1704;
        auto MUL_1715 = ADD_1612 * COS_1706;
        auto MUL_1713 = ADD_1605 * MUL_1704;
        auto SUB_1717 = MUL_1713 - MUL_1715;
        auto MUL_12044 = ADD_1730 * SUB_1717;
        auto MUL_12041 = SUB_1717 * SUB_1717;
        auto ADD_12050 = MUL_12041 + MUL_12042;
        auto MUL_1708 = ADD_1605 * COS_1706;
        auto ADD_1710 = MUL_1708 + MUL_1709;
        auto MUL_12048 = ADD_1710 * SUB_1724;
        auto ADD_12076 = MUL_12048 + MUL_12044;
        auto MUL_12047 = ADD_1710 * SUB_1717;
        auto SUB_12063 = MUL_12047 - MUL_12043;
        auto MUL_12078 = ADD_12076 * 2.0;
        auto MUL_12148 = MUL_12078 * 0.019001;
        auto MUL_12065 = SUB_12063 * 2.0;
        auto MUL_12136 = MUL_12065 * 1e-06;
        auto MUL_12053 = ADD_12050 * 2.0;
        auto SUB_12056 = 1.0 - MUL_12053;
        auto MUL_12129 = SUB_12056 * 1e-06;
        auto SUB_12158 = MUL_12129 - MUL_12136;
        auto SUB_12161 = SUB_12158 - MUL_12148;
        auto ADD_12164 = ADD_1692 + SUB_12161;
        auto ADD_12057 = MUL_12047 + MUL_12043;
        auto MUL_12046 = ADD_1730 * ADD_1710;
        auto MUL_12049 = SUB_1717 * SUB_1724;
        auto SUB_12079 = MUL_12049 - MUL_12046;
        auto MUL_12045 = ADD_1710 * ADD_1710;
        auto ADD_12066 = MUL_12042 + MUL_12045;
        auto MUL_1671 = ADD_929 * MUL_1657;
        auto MUL_1675 = SUB_914 * MUL_1644;
        auto MUL_1672 = ADD_882 * ADD_1652;
        auto ADD_1674 = MUL_1671 + MUL_1672;
        auto ADD_1676 = ADD_1674 + MUL_1675;
        auto MUL_1678 = ADD_1676 * 2.0;
        auto SUB_1681 = MUL_1678 - 0.03075;
        auto ADD_1693 = ADD_1593 + SUB_1681;
        auto MUL_12081 = SUB_12079 * 2.0;
        auto MUL_12152 = MUL_12081 * 0.019001;
        auto MUL_12069 = ADD_12066 * 2.0;
        auto SUB_12072 = 1.0 - MUL_12069;
        auto MUL_12140 = SUB_12072 * 1e-06;
        auto MUL_12059 = ADD_12057 * 2.0;
        auto MUL_12131 = MUL_12059 * 1e-06;
        auto SUB_12159 = MUL_12131 - MUL_12140;
        auto SUB_12162 = SUB_12159 - MUL_12152;
        auto ADD_12165 = ADD_1693 + SUB_12162;
        auto SUB_12060 = MUL_12048 - MUL_12044;
        auto ADD_12073 = MUL_12049 + MUL_12046;
        auto ADD_12082 = MUL_12041 + MUL_12045;
        auto MUL_1682 = ADD_929 * ADD_1652;
        auto MUL_1687 = ADD_897 * MUL_1644;
        auto MUL_1684 = ADD_882 * MUL_1657;
        auto SUB_1686 = MUL_1684 - MUL_1682;
        auto SUB_1688 = SUB_1686 - MUL_1687;
        auto MUL_1690 = SUB_1688 * 2.0;
        auto ADD_1694 = ADD_1594 + MUL_1690;
        auto MUL_12085 = ADD_12082 * 2.0;
        auto SUB_12088 = 1.0 - MUL_12085;
        auto MUL_12156 = SUB_12088 * 0.019001;
        auto MUL_12075 = ADD_12073 * 2.0;
        auto MUL_12144 = MUL_12075 * 1e-06;
        auto MUL_12062 = SUB_12060 * 2.0;
        auto MUL_12133 = MUL_12062 * 1e-06;
        auto SUB_12160 = MUL_12133 - MUL_12144;
        auto SUB_12163 = SUB_12160 - MUL_12156;
        auto ADD_12166 = ADD_1694 + SUB_12163;
        if(/*link_wrist_yaw*/ sphere_environment_in_collision(environment, ADD_12164, ADD_12165, ADD_12166, 0.02848)){ return false; } // (905, 1006)
        auto SUB_1779 = MUL_719 - MUL_697;
        auto SUB_1784 = SUB_1779 - MUL_700;
        auto SUB_1768 = MUL_732 - MUL_708;
        auto ADD_1807 = MUL_719 + MUL_697;
        auto ADD_1811 = ADD_1807 + MUL_700;
        auto ADD_1815 = ADD_1811 + MUL_729;
        auto ADD_1792 = MUL_732 + MUL_708;
        auto SUB_1795 = ADD_1792 - MUL_711;
        auto SUB_1800 = SUB_1795 - MUL_742;
        auto ADD_1787 = SUB_1784 + MUL_729;
        auto ADD_1771 = SUB_1768 + MUL_711;
        auto SUB_1774 = ADD_1771 - MUL_742;
        auto MUL_12171 = ADD_1815 * SUB_1800;
        auto MUL_12173 = ADD_1815 * ADD_1787;
        auto MUL_12170 = SUB_1800 * SUB_1800;
        auto MUL_12169 = ADD_1787 * ADD_1787;
        auto ADD_12181 = MUL_12169 + MUL_12170;
        auto MUL_12179 = SUB_1774 * SUB_1800;
        auto SUB_12207 = MUL_12179 - MUL_12173;
        auto MUL_12178 = SUB_1774 * ADD_1787;
        auto ADD_12194 = MUL_12178 + MUL_12171;
        auto MUL_12209 = SUB_12207 * 2.0;
        auto MUL_12196 = ADD_12194 * 2.0;
        auto MUL_12184 = ADD_12181 * 2.0;
        auto SUB_12187 = 1.0 - MUL_12184;
        auto MUL_1823 = MUL_148 * 1.33;
        auto MUL_1820 = MUL_174 * 1.33;
        auto MUL_1830 = MUL_176 * MUL_1820;
        auto MUL_1832 = MUL_164 * MUL_1823;
        auto SUB_1833 = MUL_1832 - MUL_1830;
        auto MUL_1836 = SUB_1833 * 2.0;
        auto ADD_1859 = ADD_631 + MUL_1836;
        auto MUL_12221 = SUB_12187 * 0.053;
        auto MUL_12233 = MUL_12209 * 0.04;
        auto MUL_12227 = MUL_12196 * 0.04;
        auto ADD_12238 = MUL_12221 + MUL_12227;
        auto ADD_12241 = ADD_12238 + MUL_12233;
        auto ADD_12244 = ADD_1859 + ADD_12241;
        auto SUB_12188 = MUL_12178 - MUL_12171;
        auto MUL_1840 = MUL_148 * MUL_1823;
        auto MUL_12176 = ADD_1815 * SUB_1774;
        auto MUL_12180 = ADD_1787 * SUB_1800;
        auto ADD_12210 = MUL_12180 + MUL_12176;
        auto MUL_12175 = SUB_1774 * SUB_1774;
        auto ADD_12197 = MUL_12170 + MUL_12175;
        auto MUL_1842 = MUL_174 * MUL_1820;
        auto ADD_1844 = MUL_1840 + MUL_1842;
        auto MUL_1847 = ADD_1844 * 2.0;
        auto SUB_1850 = 1.33 - MUL_1847;
        auto ADD_1860 = ADD_632 + SUB_1850;
        auto MUL_12212 = ADD_12210 * 2.0;
        auto MUL_12235 = MUL_12212 * 0.04;
        auto MUL_12200 = ADD_12197 * 2.0;
        auto SUB_12203 = 1.0 - MUL_12200;
        auto MUL_12229 = SUB_12203 * 0.04;
        auto MUL_12190 = SUB_12188 * 2.0;
        auto MUL_12223 = MUL_12190 * 0.053;
        auto ADD_12239 = MUL_12223 + MUL_12229;
        auto ADD_12242 = ADD_12239 + MUL_12235;
        auto ADD_12245 = ADD_1860 + ADD_12242;
        auto SUB_12204 = MUL_12180 - MUL_12176;
        auto ADD_12191 = MUL_12179 + MUL_12173;
        auto ADD_12213 = MUL_12169 + MUL_12175;
        auto MUL_1851 = MUL_176 * MUL_1823;
        auto MUL_1853 = MUL_164 * MUL_1820;
        auto ADD_1855 = MUL_1851 + MUL_1853;
        auto MUL_1857 = ADD_1855 * 2.0;
        auto ADD_1861 = 0.0284 + MUL_1857;
        auto MUL_12216 = ADD_12213 * 2.0;
        auto SUB_12219 = 1.0 - MUL_12216;
        auto MUL_12237 = SUB_12219 * 0.04;
        auto MUL_12206 = SUB_12204 * 2.0;
        auto MUL_12231 = MUL_12206 * 0.04;
        auto MUL_12193 = ADD_12191 * 2.0;
        auto MUL_12225 = MUL_12193 * 0.053;
        auto ADD_12240 = MUL_12225 + MUL_12231;
        auto ADD_12243 = ADD_12240 + MUL_12237;
        auto ADD_12246 = ADD_1861 + ADD_12243;
        auto MUL_12254 = MUL_12196 * 0.028563;
        auto MUL_12260 = MUL_12209 * 0.0255641;
        auto MUL_12248 = SUB_12187 * 0.005;
        auto ADD_12265 = MUL_12248 + MUL_12254;
        auto ADD_12268 = ADD_12265 + MUL_12260;
        auto ADD_12271 = ADD_1859 + ADD_12268;
        auto MUL_12262 = MUL_12212 * 0.0255641;
        auto MUL_12256 = SUB_12203 * 0.028563;
        auto MUL_12250 = MUL_12190 * 0.005;
        auto ADD_12266 = MUL_12250 + MUL_12256;
        auto ADD_12269 = ADD_12266 + MUL_12262;
        auto ADD_12272 = ADD_1860 + ADD_12269;
        auto MUL_12264 = SUB_12219 * 0.0255641;
        auto MUL_12258 = MUL_12206 * 0.028563;
        auto MUL_12252 = MUL_12193 * 0.005;
        auto ADD_12267 = MUL_12252 + MUL_12258;
        auto ADD_12270 = ADD_12267 + MUL_12264;
        auto ADD_12273 = ADD_1861 + ADD_12270;
        auto MUL_12275 = SUB_12187 * 0.0899282;
        auto MUL_12281 = MUL_12196 * 0.064879;
        auto ADD_12292 = MUL_12275 + MUL_12281;
        auto ADD_12295 = ADD_12292 + MUL_12260;
        auto ADD_12298 = ADD_1859 + ADD_12295;
        auto MUL_12283 = SUB_12203 * 0.064879;
        auto MUL_12277 = MUL_12190 * 0.0899282;
        auto ADD_12293 = MUL_12277 + MUL_12283;
        auto ADD_12296 = ADD_12293 + MUL_12262;
        auto ADD_12299 = ADD_1860 + ADD_12296;
        auto MUL_12285 = MUL_12206 * 0.064879;
        auto MUL_12279 = MUL_12193 * 0.0899282;
        auto ADD_12294 = MUL_12279 + MUL_12285;
        auto ADD_12297 = ADD_12294 + MUL_12264;
        auto ADD_12300 = ADD_1861 + ADD_12297;
        if(/*link_head*/ sphere_environment_in_collision(environment, ADD_12244, ADD_12245, ADD_12246, 0.12)){ if(sphere_environment_in_collision(environment, ADD_12271, ADD_12272, ADD_12273, 0.074)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_12298, ADD_12299, ADD_12300, 0.074)){ return false; } } // (1006, 1117)
        auto MUL_1940 = ADD_1815 * 0.7071068;
        auto MUL_1947 = SUB_1800 * 0.7071068;
        auto SUB_1948 = MUL_1947 - MUL_1940;
        auto ADD_1958 = MUL_1940 + MUL_1947;
        auto MUL_1978 = SUB_1800 * 0.135;
        auto MUL_1935 = ADD_1787 * 0.7071068;
        auto MUL_1971 = ADD_1787 * 0.135;
        auto MUL_1932 = SUB_1774 * 0.7071068;
        auto SUB_1936 = MUL_1935 - MUL_1932;
        auto ADD_1926 = MUL_1932 + MUL_1935;
        auto MUL_1965 = SUB_1800 * 0.0731;
        auto MUL_1969 = SUB_1774 * 0.0731;
        auto SUB_1972 = MUL_1969 - MUL_1971;
        auto MUL_1982 = ADD_1787 * SUB_1972;
        auto MUL_1962 = ADD_1787 * 0.0031962;
        auto ADD_1966 = MUL_1962 + MUL_1965;
        auto MUL_1981 = ADD_1815 * ADD_1966;
        auto ADD_1983 = MUL_1981 + MUL_1982;
        auto MUL_1975 = SUB_1774 * 0.0031962;
        auto ADD_1979 = MUL_1975 + MUL_1978;
        auto MUL_1984 = SUB_1800 * ADD_1979;
        auto SUB_1986 = ADD_1983 - MUL_1984;
        auto MUL_1988 = SUB_1986 * 2.0;
        auto ADD_1990 = MUL_1988 + 0.135;
        auto ADD_2018 = ADD_1859 + ADD_1990;
        auto INPUT_10 = q[10];
        auto DIV_2185 = INPUT_10 * 0.5;
        auto SIN_2186 = DIV_2185.sin();
        auto COS_2192 = DIV_2185.cos();
        auto INPUT_9 = q[9];
        auto DIV_2022 = INPUT_9 * 0.5;
        auto SIN_2023 = DIV_2022.sin();
        auto COS_2029 = DIV_2022.cos();
        auto MUL_2047 = ADD_1958 * COS_2029;
        auto MUL_2041 = ADD_1958 * SIN_2023;
        auto MUL_2045 = SUB_1948 * COS_2029;
        auto SUB_2046 = MUL_2045 - MUL_2041;
        auto MUL_2101 = SUB_2046 * 0.7071068;
        auto MUL_2129 = SUB_2046 * 0.0277625;
        auto MUL_2145 = SUB_2046 * 0.0013;
        auto MUL_2051 = SUB_1948 * SIN_2023;
        auto ADD_2052 = MUL_2047 + MUL_2051;
        auto MUL_2085 = ADD_2052 * 0.7071068;
        auto MUL_2038 = SUB_1936 * COS_2029;
        auto MUL_2032 = SUB_1936 * SIN_2023;
        auto MUL_2031 = ADD_1926 * COS_2029;
        auto ADD_2033 = MUL_2031 + MUL_2032;
        auto MUL_2088 = ADD_2033 * 0.7071068;
        auto SUB_2089 = MUL_2088 - MUL_2085;
        auto ADD_2118 = MUL_2085 + MUL_2088;
        auto MUL_2210 = ADD_2118 * COS_2192;
        auto MUL_2204 = ADD_2118 * SIN_2186;
        auto MUL_2194 = SUB_2089 * COS_2192;
        auto MUL_2199 = SUB_2089 * SIN_2186;
        auto MUL_2141 = ADD_2033 * 0.0533108;
        auto SUB_2147 = MUL_2145 - MUL_2141;
        auto MUL_2151 = SUB_2046 * SUB_2147;
        auto MUL_2133 = ADD_2033 * 0.0277625;
        auto MUL_2036 = ADD_1926 * SIN_2023;
        auto SUB_2039 = MUL_2038 - MUL_2036;
        auto MUL_2099 = SUB_2039 * 0.7071068;
        auto SUB_2112 = MUL_2101 - MUL_2099;
        auto ADD_2102 = MUL_2099 + MUL_2101;
        auto MUL_2208 = SUB_2112 * COS_2192;
        auto SUB_2209 = MUL_2208 - MUL_2204;
        auto MUL_12359 = SUB_2209 * SUB_2209;
        auto MUL_2214 = SUB_2112 * SIN_2186;
        auto ADD_2215 = MUL_2210 + MUL_2214;
        auto MUL_12360 = ADD_2215 * SUB_2209;
        auto MUL_2201 = ADD_2102 * COS_2192;
        auto SUB_2202 = MUL_2201 - MUL_2199;
        auto MUL_12362 = ADD_2215 * SUB_2202;
        auto MUL_12358 = SUB_2202 * SUB_2202;
        auto ADD_12370 = MUL_12358 + MUL_12359;
        auto MUL_2195 = ADD_2102 * SIN_2186;
        auto ADD_2196 = MUL_2194 + MUL_2195;
        auto MUL_12368 = ADD_2196 * SUB_2209;
        auto SUB_12396 = MUL_12368 - MUL_12362;
        auto MUL_12367 = ADD_2196 * SUB_2202;
        auto ADD_12383 = MUL_12367 + MUL_12360;
        auto MUL_2126 = SUB_2039 * 0.0533108;
        auto ADD_2130 = MUL_2126 + MUL_2129;
        auto MUL_2148 = ADD_2052 * ADD_2130;
        auto MUL_2136 = SUB_2039 * 0.0013;
        auto ADD_2138 = MUL_2133 + MUL_2136;
        auto MUL_2149 = SUB_2039 * ADD_2138;
        auto ADD_2150 = MUL_2148 + MUL_2149;
        auto ADD_2152 = ADD_2150 + MUL_2151;
        auto MUL_2154 = ADD_2152 * 2.0;
        auto SUB_2157 = MUL_2154 - 0.0013;
        auto ADD_2181 = ADD_2018 + SUB_2157;
        auto MUL_12398 = SUB_12396 * 2.0;
        auto MUL_12428 = MUL_12398 * 0.0318223;
        auto MUL_12385 = ADD_12383 * 2.0;
        auto MUL_12417 = MUL_12385 * 0.02;
        auto MUL_12373 = ADD_12370 * 2.0;
        auto SUB_12376 = 1.0 - MUL_12373;
        auto MUL_12410 = SUB_12376 * 0.0105151;
        auto SUB_12433 = MUL_12410 - MUL_12417;
        auto ADD_12436 = SUB_12433 + MUL_12428;
        auto ADD_12439 = ADD_2181 + ADD_12436;
        auto SUB_12377 = MUL_12367 - MUL_12360;
        auto MUL_12365 = ADD_2215 * ADD_2196;
        auto MUL_12369 = SUB_2202 * SUB_2209;
        auto ADD_12399 = MUL_12369 + MUL_12365;
        auto MUL_12364 = ADD_2196 * ADD_2196;
        auto ADD_12386 = MUL_12359 + MUL_12364;
        auto MUL_2158 = ADD_2052 * SUB_2147;
        auto MUL_2161 = SUB_2046 * ADD_2130;
        auto MUL_2159 = ADD_2033 * ADD_2138;
        auto SUB_2160 = MUL_2158 - MUL_2159;
        auto SUB_2163 = SUB_2160 - MUL_2161;
        auto MUL_2165 = SUB_2163 * 2.0;
        auto ADD_2167 = MUL_2165 + 0.0277625;
        auto MUL_1991 = ADD_1815 * ADD_1979;
        auto MUL_1996 = SUB_1800 * ADD_1966;
        auto MUL_1993 = SUB_1774 * SUB_1972;
        auto ADD_1994 = MUL_1991 + MUL_1993;
        auto ADD_1998 = ADD_1994 + MUL_1996;
        auto MUL_2001 = ADD_1998 * 2.0;
        auto SUB_2004 = 0.0731 - MUL_2001;
        auto ADD_2019 = ADD_1860 + SUB_2004;
        auto ADD_2182 = ADD_2019 + ADD_2167;
        auto MUL_12401 = ADD_12399 * 2.0;
        auto MUL_12430 = MUL_12401 * 0.0318223;
        auto MUL_12389 = ADD_12386 * 2.0;
        auto SUB_12392 = 1.0 - MUL_12389;
        auto MUL_12421 = SUB_12392 * 0.02;
        auto MUL_12379 = SUB_12377 * 2.0;
        auto MUL_12412 = MUL_12379 * 0.0105151;
        auto SUB_12434 = MUL_12412 - MUL_12421;
        auto ADD_12437 = SUB_12434 + MUL_12430;
        auto ADD_12440 = ADD_2182 + ADD_12437;
        auto SUB_12393 = MUL_12369 - MUL_12365;
        auto ADD_12380 = MUL_12368 + MUL_12362;
        auto ADD_12402 = MUL_12358 + MUL_12364;
        auto MUL_2168 = ADD_2052 * ADD_2138;
        auto MUL_2173 = SUB_2039 * ADD_2130;
        auto MUL_2170 = ADD_2033 * SUB_2147;
        auto ADD_2171 = MUL_2168 + MUL_2170;
        auto SUB_2175 = MUL_2173 - ADD_2171;
        auto MUL_2177 = SUB_2175 * 2.0;
        auto SUB_2180 = MUL_2177 - 0.0533108;
        auto MUL_2005 = ADD_1815 * SUB_1972;
        auto MUL_2010 = ADD_1787 * ADD_1966;
        auto MUL_2007 = SUB_1774 * ADD_1979;
        auto SUB_2009 = MUL_2007 - MUL_2005;
        auto ADD_2012 = SUB_2009 + MUL_2010;
        auto MUL_2014 = ADD_2012 * 2.0;
        auto SUB_2017 = MUL_2014 - 0.0031962;
        auto ADD_2020 = ADD_1861 + SUB_2017;
        auto ADD_2183 = ADD_2020 + SUB_2180;
        auto MUL_12405 = ADD_12402 * 2.0;
        auto SUB_12408 = 1.0 - MUL_12405;
        auto MUL_12432 = SUB_12408 * 0.0318223;
        auto MUL_12395 = SUB_12393 * 2.0;
        auto MUL_12425 = MUL_12395 * 0.02;
        auto MUL_12382 = ADD_12380 * 2.0;
        auto MUL_12414 = MUL_12382 * 0.0105151;
        auto SUB_12435 = MUL_12414 - MUL_12425;
        auto ADD_12438 = SUB_12435 + MUL_12432;
        auto ADD_12441 = ADD_2183 + ADD_12438;
        auto MUL_12449 = MUL_12385 * 0.0053853;
        auto ADD_12460 = MUL_12410 + MUL_12449;
        auto ADD_12463 = ADD_12460 + MUL_12428;
        auto ADD_12466 = ADD_2181 + ADD_12463;
        auto MUL_12451 = SUB_12392 * 0.0053853;
        auto ADD_12461 = MUL_12412 + MUL_12451;
        auto ADD_12464 = ADD_12461 + MUL_12430;
        auto ADD_12467 = ADD_2182 + ADD_12464;
        auto MUL_12453 = MUL_12395 * 0.0053853;
        auto ADD_12462 = MUL_12414 + MUL_12453;
        auto ADD_12465 = ADD_12462 + MUL_12432;
        auto ADD_12468 = ADD_2183 + ADD_12465;
        auto MUL_12477 = MUL_12385 * 0.0619847;
        auto MUL_12470 = SUB_12376 * 0.0105141;
        auto SUB_12493 = MUL_12470 - MUL_12477;
        auto ADD_12496 = SUB_12493 + MUL_12428;
        auto ADD_12499 = ADD_2181 + ADD_12496;
        auto MUL_12481 = SUB_12392 * 0.0619847;
        auto MUL_12472 = MUL_12379 * 0.0105141;
        auto SUB_12494 = MUL_12472 - MUL_12481;
        auto ADD_12497 = SUB_12494 + MUL_12430;
        auto ADD_12500 = ADD_2182 + ADD_12497;
        auto MUL_12485 = MUL_12395 * 0.0619847;
        auto MUL_12474 = MUL_12382 * 0.0105141;
        auto SUB_12495 = MUL_12474 - MUL_12485;
        auto ADD_12498 = SUB_12495 + MUL_12432;
        auto ADD_12501 = ADD_2183 + ADD_12498;
        if(/*link_head_tilt*/ sphere_environment_in_collision(environment, ADD_12439, ADD_12440, ADD_12441, 0.07)){ if(sphere_environment_in_collision(environment, ADD_12466, ADD_12467, ADD_12468, 0.057304)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_12499, ADD_12500, ADD_12501, 0.056933)){ return false; } } // (1117, 1306)
        auto MUL_4981 = SIN_108 * 0.004;
        auto MUL_4985 = SIN_108 * MUL_4981;
        auto MUL_4988 = MUL_4985 * 2.0;
        auto SUB_4991 = 0.004 - MUL_4988;
        auto ADD_5005 = INPUT_0 + SUB_4991;
        auto MUL_4993 = COS_114 * MUL_4981;
        auto MUL_4997 = MUL_4993 * 2.0;
        auto ADD_5006 = INPUT_1 + MUL_4997;
        if(/*laser*/ sphere_environment_in_collision(environment, ADD_5005, ADD_5006, 0.1584, 0.04)){ return false; } // (1306, 1314)
        auto MUL_5395 = ADD_1730 * 0.7071068;
        auto MUL_5402 = SUB_1724 * 0.7071068;
        auto SUB_5415 = MUL_5395 - MUL_5402;
        auto ADD_5403 = MUL_5395 + MUL_5402;
        auto MUL_13624 = SUB_5415 * SUB_5415;
        auto MUL_13631 = ADD_5403 * SUB_5415;
        auto MUL_5422 = SUB_1717 * 0.7071068;
        auto MUL_5419 = ADD_1710 * 0.7071068;
        auto SUB_5435 = MUL_5419 - MUL_5422;
        auto ADD_5424 = MUL_5419 + MUL_5422;
        auto MUL_13626 = SUB_5435 * ADD_5424;
        auto ADD_13651 = MUL_13631 + MUL_13626;
        auto MUL_13627 = SUB_5435 * SUB_5415;
        auto MUL_13625 = ADD_5424 * ADD_5424;
        auto ADD_13636 = MUL_13624 + MUL_13625;
        auto MUL_13633 = ADD_5403 * ADD_5424;
        auto SUB_13666 = MUL_13627 - MUL_13633;
        auto MUL_13668 = SUB_13666 * 2.0;
        auto MUL_13639 = ADD_13636 * 2.0;
        auto SUB_13642 = 1.0 - MUL_13639;
        auto MUL_13654 = ADD_13651 * 2.0;
        auto MUL_13692 = MUL_13654 * 0.048456;
        auto MUL_13681 = SUB_13642 * 0.030127;
        auto SUB_13713 = MUL_13692 - MUL_13681;
        auto MUL_13703 = MUL_13668 * 0.029375;
        auto SUB_13717 = SUB_13713 - MUL_13703;
        auto ADD_13721 = ADD_1692 + SUB_13717;
        auto SUB_13643 = MUL_13626 - MUL_13631;
        auto MUL_13629 = SUB_5435 * ADD_5403;
        auto MUL_13635 = SUB_5415 * ADD_5424;
        auto ADD_13669 = MUL_13635 + MUL_13629;
        auto MUL_13628 = ADD_5403 * ADD_5403;
        auto ADD_13656 = MUL_13625 + MUL_13628;
        auto MUL_13671 = ADD_13669 * 2.0;
        auto MUL_13707 = MUL_13671 * 0.029375;
        auto MUL_13659 = ADD_13656 * 2.0;
        auto SUB_13662 = 1.0 - MUL_13659;
        auto MUL_13645 = SUB_13643 * 2.0;
        auto MUL_13685 = MUL_13645 * 0.030127;
        auto MUL_13695 = SUB_13662 * 0.048456;
        auto ADD_13714 = MUL_13685 + MUL_13695;
        auto ADD_13718 = ADD_13714 + MUL_13707;
        auto SUB_13722 = ADD_1693 - ADD_13718;
        auto SUB_13663 = MUL_13635 - MUL_13629;
        auto ADD_13672 = MUL_13624 + MUL_13628;
        auto ADD_13646 = MUL_13633 + MUL_13627;
        auto MUL_13675 = ADD_13672 * 2.0;
        auto SUB_13678 = 1.0 - MUL_13675;
        auto MUL_13711 = SUB_13678 * 0.029375;
        auto MUL_13649 = ADD_13646 * 2.0;
        auto MUL_13689 = MUL_13649 * 0.030127;
        auto MUL_13665 = SUB_13663 * 2.0;
        auto MUL_13699 = MUL_13665 * 0.048456;
        auto SUB_13716 = MUL_13689 - MUL_13699;
        auto SUB_13720 = SUB_13716 - MUL_13711;
        auto ADD_13723 = ADD_1694 + SUB_13720;
        auto SUB_13762 = SUB_13713 - MUL_13703;
        auto ADD_13766 = ADD_1692 + SUB_13762;
        auto ADD_13763 = ADD_13714 + MUL_13707;
        auto SUB_13767 = ADD_1693 - ADD_13763;
        auto SUB_13765 = SUB_13716 - MUL_13711;
        auto ADD_13768 = ADD_1694 + SUB_13765;
        auto MUL_13782 = MUL_13654 * 0.048455;
        auto MUL_13793 = MUL_13668 * 0.029374;
        auto MUL_13771 = SUB_13642 * 0.004374;
        auto SUB_13803 = MUL_13782 - MUL_13771;
        auto SUB_13807 = SUB_13803 - MUL_13793;
        auto ADD_13811 = ADD_1692 + SUB_13807;
        auto MUL_13797 = MUL_13671 * 0.029374;
        auto MUL_13775 = MUL_13645 * 0.004374;
        auto MUL_13785 = SUB_13662 * 0.048455;
        auto ADD_13804 = MUL_13775 + MUL_13785;
        auto ADD_13808 = ADD_13804 + MUL_13797;
        auto SUB_13812 = ADD_1693 - ADD_13808;
        auto MUL_13801 = SUB_13678 * 0.029374;
        auto MUL_13789 = MUL_13665 * 0.048455;
        auto MUL_13779 = MUL_13649 * 0.004374;
        auto SUB_13806 = MUL_13779 - MUL_13789;
        auto SUB_13810 = SUB_13806 - MUL_13801;
        auto ADD_13813 = ADD_1694 + SUB_13810;
        if(/*base_link vs. link_wrist_yaw_bottom*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, ADD_13721, SUB_13722, ADD_13723, 0.06)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; } } // (1314, 1394)
        if(/*link_wrist_yaw_bottom*/ sphere_environment_in_collision(environment, ADD_13721, SUB_13722, ADD_13723, 0.06)){ if(sphere_environment_in_collision(environment, ADD_13766, SUB_13767, ADD_13768, 0.046341)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_13811, SUB_13812, ADD_13813, 0.046342)){ return false; } } // (1394, 1394)
        auto MUL_5543 = SUB_5435 * 0.7071081;
        auto MUL_5559 = SUB_5435 * 0.7071055;
        auto MUL_5540 = ADD_5424 * 0.7071081;
        auto MUL_5588 = ADD_5424 * 0.7071055;
        auto MUL_5584 = SUB_5415 * 0.7071081;
        auto MUL_5537 = SUB_5415 * 0.7071055;
        auto MUL_5561 = ADD_5403 * 0.7071081;
        auto SUB_5563 = MUL_5559 - MUL_5561;
        auto MUL_5545 = ADD_5403 * 0.7071055;
        auto ADD_5547 = MUL_5543 + MUL_5545;
        auto MUL_5593 = SUB_5415 * 0.0305;
        auto MUL_5607 = ADD_5403 * 0.0305;
        auto MUL_5613 = ADD_5424 * MUL_5607;
        auto MUL_5597 = ADD_5424 * 0.019455;
        auto SUB_5599 = MUL_5597 - MUL_5593;
        auto MUL_5610 = SUB_5435 * SUB_5599;
        auto MUL_5602 = ADD_5403 * 0.019455;
        auto MUL_5611 = SUB_5415 * MUL_5602;
        auto ADD_5612 = MUL_5610 + MUL_5611;
        auto ADD_5614 = ADD_5612 + MUL_5613;
        auto MUL_5616 = ADD_5614 * 2.0;
        auto ADD_5642 = ADD_1692 + MUL_5616;
        auto MUL_5530 = SUB_5435 * 2.6e-06;
        auto MUL_5555 = ADD_5424 * 2.6e-06;
        auto MUL_5550 = SUB_5415 * 2.6e-06;
        auto SUB_5552 = ADD_5547 - MUL_5550;
        auto SUB_5557 = SUB_5552 - MUL_5555;
        auto ADD_5568 = SUB_5563 + MUL_5550;
        auto SUB_5573 = ADD_5568 - MUL_5555;
        auto MUL_5534 = ADD_5403 * 2.6e-06;
        auto SUB_5535 = MUL_5534 - MUL_5530;
        auto ADD_5538 = SUB_5535 + MUL_5537;
        auto SUB_5541 = ADD_5538 - MUL_5540;
        auto ADD_5581 = MUL_5530 + MUL_5534;
        auto ADD_5585 = ADD_5581 + MUL_5584;
        auto ADD_5589 = ADD_5585 + MUL_5588;
        auto INPUT_11 = q[11];
        auto DIV_5646 = INPUT_11 * 0.5;
        auto SIN_5647 = DIV_5646.sin();
        auto COS_5656 = DIV_5646.cos();
        auto MUL_5654 = SIN_5647 * 1.0;
        auto MUL_5669 = ADD_5589 * MUL_5654;
        auto MUL_5674 = ADD_5589 * COS_5656;
        auto MUL_5678 = SUB_5573 * MUL_5654;
        auto SUB_5680 = MUL_5678 - MUL_5674;
        auto MUL_5672 = SUB_5573 * COS_5656;
        auto ADD_5673 = MUL_5669 + MUL_5672;
        auto MUL_13822 = SUB_5680 * ADD_5673;
        auto MUL_13821 = ADD_5673 * ADD_5673;
        auto MUL_5659 = SUB_5557 * MUL_5654;
        auto MUL_5666 = SUB_5557 * COS_5656;
        auto MUL_5664 = SUB_5541 * MUL_5654;
        auto ADD_5667 = MUL_5664 + MUL_5666;
        auto MUL_13823 = SUB_5680 * ADD_5667;
        auto MUL_13820 = ADD_5667 * ADD_5667;
        auto ADD_13829 = MUL_13820 + MUL_13821;
        auto MUL_5658 = SUB_5541 * COS_5656;
        auto SUB_5661 = MUL_5658 - MUL_5659;
        auto MUL_13827 = SUB_5661 * ADD_5673;
        auto ADD_13855 = MUL_13827 + MUL_13823;
        auto MUL_13826 = SUB_5661 * ADD_5667;
        auto SUB_13842 = MUL_13826 - MUL_13822;
        auto MUL_13857 = ADD_13855 * 2.0;
        auto MUL_13887 = MUL_13857 * 0.018175;
        auto MUL_13844 = SUB_13842 * 2.0;
        auto MUL_13876 = MUL_13844 * 0.015;
        auto MUL_13832 = ADD_13829 * 2.0;
        auto SUB_13835 = 1.0 - MUL_13832;
        auto MUL_13869 = SUB_13835 * 0.0012589;
        auto SUB_13892 = MUL_13869 - MUL_13876;
        auto ADD_13895 = SUB_13892 + MUL_13887;
        auto ADD_13898 = ADD_5642 + ADD_13895;
        auto ADD_13836 = MUL_13826 + MUL_13822;
        auto MUL_13825 = SUB_5680 * SUB_5661;
        auto MUL_13828 = ADD_5667 * ADD_5673;
        auto SUB_13858 = MUL_13828 - MUL_13825;
        auto MUL_13824 = SUB_5661 * SUB_5661;
        auto ADD_13845 = MUL_13821 + MUL_13824;
        auto MUL_5619 = SUB_5435 * MUL_5607;
        auto MUL_5624 = ADD_5424 * SUB_5599;
        auto MUL_5621 = ADD_5403 * MUL_5602;
        auto SUB_5623 = MUL_5621 - MUL_5619;
        auto ADD_5625 = SUB_5623 + MUL_5624;
        auto MUL_5627 = ADD_5625 * 2.0;
        auto SUB_5630 = MUL_5627 - 0.019455;
        auto ADD_5643 = ADD_1693 + SUB_5630;
        auto MUL_13860 = SUB_13858 * 2.0;
        auto MUL_13889 = MUL_13860 * 0.018175;
        auto MUL_13848 = ADD_13845 * 2.0;
        auto SUB_13851 = 1.0 - MUL_13848;
        auto MUL_13880 = SUB_13851 * 0.015;
        auto MUL_13838 = ADD_13836 * 2.0;
        auto MUL_13871 = MUL_13838 * 0.0012589;
        auto SUB_13893 = MUL_13871 - MUL_13880;
        auto ADD_13896 = SUB_13893 + MUL_13889;
        auto ADD_13899 = ADD_5643 + ADD_13896;
        auto SUB_13839 = MUL_13827 - MUL_13823;
        auto ADD_13852 = MUL_13828 + MUL_13825;
        auto ADD_13861 = MUL_13820 + MUL_13824;
        auto MUL_5631 = SUB_5435 * MUL_5602;
        auto MUL_5635 = SUB_5415 * SUB_5599;
        auto MUL_5632 = ADD_5403 * MUL_5607;
        auto ADD_5634 = MUL_5631 + MUL_5632;
        auto SUB_5636 = ADD_5634 - MUL_5635;
        auto MUL_5638 = SUB_5636 * 2.0;
        auto SUB_5641 = MUL_5638 - 0.0305;
        auto ADD_5644 = ADD_1694 + SUB_5641;
        auto MUL_13864 = ADD_13861 * 2.0;
        auto SUB_13867 = 1.0 - MUL_13864;
        auto MUL_13891 = SUB_13867 * 0.018175;
        auto MUL_13854 = ADD_13852 * 2.0;
        auto MUL_13884 = MUL_13854 * 0.015;
        auto MUL_13841 = SUB_13839 * 2.0;
        auto MUL_13873 = MUL_13841 * 0.0012589;
        auto SUB_13894 = MUL_13873 - MUL_13884;
        auto ADD_13897 = SUB_13894 + MUL_13891;
        auto ADD_13900 = ADD_5644 + ADD_13897;
        auto MUL_13909 = MUL_13844 * 0.0373123;
        auto SUB_13925 = MUL_13869 - MUL_13909;
        auto ADD_13928 = SUB_13925 + MUL_13887;
        auto ADD_13931 = ADD_5642 + ADD_13928;
        auto MUL_13913 = SUB_13851 * 0.0373123;
        auto SUB_13926 = MUL_13871 - MUL_13913;
        auto ADD_13929 = SUB_13926 + MUL_13889;
        auto ADD_13932 = ADD_5643 + ADD_13929;
        auto MUL_13917 = MUL_13854 * 0.0373123;
        auto SUB_13927 = MUL_13873 - MUL_13917;
        auto ADD_13930 = SUB_13927 + MUL_13891;
        auto ADD_13933 = ADD_5644 + ADD_13930;
        auto MUL_13941 = MUL_13844 * 0.0028977;
        auto MUL_13935 = SUB_13835 * 0.0014159;
        auto ADD_13952 = MUL_13935 + MUL_13941;
        auto ADD_13955 = ADD_13952 + MUL_13887;
        auto ADD_13958 = ADD_5642 + ADD_13955;
        auto MUL_13943 = SUB_13851 * 0.0028977;
        auto MUL_13937 = MUL_13838 * 0.0014159;
        auto ADD_13953 = MUL_13937 + MUL_13943;
        auto ADD_13956 = ADD_13953 + MUL_13889;
        auto ADD_13959 = ADD_5643 + ADD_13956;
        auto MUL_13945 = MUL_13854 * 0.0028977;
        auto MUL_13939 = MUL_13841 * 0.0014159;
        auto ADD_13954 = MUL_13939 + MUL_13945;
        auto ADD_13957 = ADD_13954 + MUL_13891;
        auto ADD_13960 = ADD_5644 + ADD_13957;
        if(/*base_link vs. link_wrist_pitch*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, ADD_13898, ADD_13899, ADD_13900, 0.05)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; } } // (1394, 1538)
        if(/*link_wrist_pitch*/ sphere_environment_in_collision(environment, ADD_13898, ADD_13899, ADD_13900, 0.05)){ if(sphere_environment_in_collision(environment, ADD_13931, ADD_13932, ADD_13933, 0.038801)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_13958, ADD_13959, ADD_13960, 0.039007)){ return false; } } // (1538, 1538)
        auto MUL_5747 = SUB_5680 * 0.7071081;
        auto MUL_5712 = SUB_5680 * 0.7071055;
        auto MUL_5730 = SUB_5680 * 2.6e-06;
        auto MUL_5780 = ADD_5673 * 0.7071081;
        auto MUL_5743 = ADD_5673 * 0.7071055;
        auto MUL_5725 = ADD_5673 * 2.6e-06;
        auto MUL_5720 = ADD_5667 * 0.7071081;
        auto MUL_5756 = ADD_5667 * 0.7071055;
        auto MUL_5739 = ADD_5667 * 2.6e-06;
        auto MUL_5734 = SUB_5661 * 0.7071081;
        auto SUB_5736 = MUL_5734 - MUL_5730;
        auto SUB_5741 = SUB_5736 - MUL_5739;
        auto ADD_5744 = SUB_5741 + MUL_5743;
        auto MUL_5770 = SUB_5661 * 0.7071055;
        auto ADD_5771 = MUL_5730 + MUL_5770;
        auto SUB_5777 = MUL_5739 - ADD_5771;
        auto ADD_5782 = SUB_5777 + MUL_5780;
        auto MUL_5715 = SUB_5661 * 2.6e-06;
        auto SUB_5717 = MUL_5712 - MUL_5715;
        auto SUB_5722 = SUB_5717 - MUL_5720;
        auto ADD_5753 = MUL_5747 + MUL_5715;
        auto ADD_5757 = ADD_5753 + MUL_5756;
        auto ADD_5763 = ADD_5757 + MUL_5725;
        auto ADD_5727 = SUB_5722 + MUL_5725;
        auto MUL_5787 = ADD_5673 * 0.024;
        auto MUL_5792 = SUB_5661 * 0.024;
        auto MUL_5784 = ADD_5667 * 0.01955;
        auto ADD_5789 = MUL_5784 + MUL_5787;
        auto MUL_5806 = SUB_5680 * ADD_5789;
        auto MUL_5800 = SUB_5661 * 0.01955;
        auto MUL_5803 = ADD_5673 * 0.018859;
        auto ADD_5805 = MUL_5800 + MUL_5803;
        auto MUL_5809 = ADD_5673 * ADD_5805;
        auto MUL_5796 = ADD_5667 * 0.018859;
        auto SUB_5798 = MUL_5796 - MUL_5792;
        auto MUL_5807 = ADD_5667 * SUB_5798;
        auto ADD_5808 = MUL_5806 + MUL_5807;
        auto ADD_5810 = ADD_5808 + MUL_5809;
        auto MUL_5812 = ADD_5810 * 2.0;
        auto SUB_5815 = MUL_5812 - 0.018859;
        auto ADD_5838 = ADD_5642 + SUB_5815;
        auto INPUT_12 = q[12];
        auto DIV_5842 = INPUT_12 * 0.5;
        auto SIN_5843 = DIV_5842.sin();
        auto COS_5849 = DIV_5842.cos();
        auto MUL_5867 = ADD_5782 * COS_5849;
        auto MUL_5861 = ADD_5782 * SIN_5843;
        auto MUL_5864 = ADD_5763 * COS_5849;
        auto SUB_5866 = MUL_5861 - MUL_5864;
        auto MUL_6105 = SUB_5866 * 1.0;
        auto MUL_5870 = ADD_5763 * SIN_5843;
        auto ADD_5872 = MUL_5867 + MUL_5870;
        auto MUL_6089 = ADD_5872 * 1.0;
        auto MUL_14065 = MUL_6105 * MUL_6089;
        auto MUL_14064 = MUL_6089 * MUL_6089;
        auto MUL_5858 = ADD_5744 * COS_5849;
        auto MUL_5852 = ADD_5744 * SIN_5843;
        auto MUL_5851 = ADD_5727 * COS_5849;
        auto ADD_5853 = MUL_5851 + MUL_5852;
        auto MUL_6081 = ADD_5853 * 1.0;
        auto MUL_14067 = MUL_6105 * MUL_6081;
        auto MUL_14063 = MUL_6081 * MUL_6081;
        auto ADD_14076 = MUL_14063 + MUL_14064;
        auto MUL_6116 = ADD_5853 * 0.021;
        auto MUL_6121 = SUB_5866 * MUL_6116;
        auto MUL_5856 = ADD_5727 * SIN_5843;
        auto SUB_5859 = MUL_5858 - MUL_5856;
        auto MUL_6073 = SUB_5859 * 1.0;
        auto MUL_14073 = MUL_6073 * MUL_6089;
        auto ADD_14106 = MUL_14073 + MUL_14067;
        auto MUL_14071 = MUL_6073 * MUL_6081;
        auto SUB_14091 = MUL_14065 - MUL_14071;
        auto MUL_6108 = SUB_5859 * 0.021;
        auto MUL_6119 = ADD_5872 * MUL_6108;
        auto ADD_6122 = MUL_6119 + MUL_6121;
        auto MUL_6124 = ADD_6122 * 2.0;
        auto ADD_6146 = ADD_5838 + MUL_6124;
        auto MUL_14079 = ADD_14076 * 2.0;
        auto SUB_14082 = 1.0 - MUL_14079;
        auto MUL_14121 = SUB_14082 * 0.0004704;
        auto MUL_14093 = SUB_14091 * 2.0;
        auto MUL_14131 = MUL_14093 * 0.02;
        auto SUB_14143 = MUL_14131 - MUL_14121;
        auto MUL_14108 = ADD_14106 * 2.0;
        auto MUL_14138 = MUL_14108 * 0.0372672;
        auto ADD_14147 = SUB_14143 + MUL_14138;
        auto ADD_14150 = ADD_6146 + ADD_14147;
        auto ADD_14083 = MUL_14071 + MUL_14065;
        auto MUL_14069 = MUL_6105 * MUL_6073;
        auto MUL_14074 = MUL_6081 * MUL_6089;
        auto SUB_14109 = MUL_14069 - MUL_14074;
        auto MUL_14068 = MUL_6073 * MUL_6073;
        auto ADD_14094 = MUL_14064 + MUL_14068;
        auto MUL_6127 = ADD_5872 * MUL_6116;
        auto MUL_6130 = SUB_5866 * MUL_6108;
        auto SUB_6131 = MUL_6130 - MUL_6127;
        auto MUL_6133 = SUB_6131 * 2.0;
        auto MUL_5817 = SUB_5680 * ADD_5805;
        auto MUL_5822 = ADD_5673 * ADD_5789;
        auto MUL_5819 = SUB_5661 * SUB_5798;
        auto ADD_5820 = MUL_5817 + MUL_5819;
        auto SUB_5823 = MUL_5822 - ADD_5820;
        auto MUL_5825 = SUB_5823 * 2.0;
        auto SUB_5828 = MUL_5825 - 0.024;
        auto ADD_5839 = ADD_5643 + SUB_5828;
        auto ADD_6147 = ADD_5839 + MUL_6133;
        auto MUL_14086 = ADD_14083 * 2.0;
        auto MUL_14125 = MUL_14086 * 0.0004704;
        auto MUL_14097 = ADD_14094 * 2.0;
        auto SUB_14100 = 1.0 - MUL_14097;
        auto MUL_14133 = SUB_14100 * 0.02;
        auto ADD_14144 = MUL_14125 + MUL_14133;
        auto MUL_14111 = SUB_14109 * 2.0;
        auto MUL_14140 = MUL_14111 * 0.0372672;
        auto ADD_14148 = ADD_14144 + MUL_14140;
        auto ADD_14151 = ADD_6147 + ADD_14148;
        auto SUB_14088 = MUL_14073 - MUL_14067;
        auto ADD_14112 = MUL_14063 + MUL_14068;
        auto ADD_14101 = MUL_14074 + MUL_14069;
        auto MUL_6138 = SUB_5859 * MUL_6108;
        auto MUL_6136 = ADD_5853 * MUL_6116;
        auto ADD_6139 = MUL_6136 + MUL_6138;
        auto MUL_6142 = ADD_6139 * 2.0;
        auto SUB_6145 = 0.021 - MUL_6142;
        auto MUL_5829 = SUB_5680 * SUB_5798;
        auto MUL_5832 = ADD_5667 * ADD_5789;
        auto MUL_5830 = SUB_5661 * ADD_5805;
        auto SUB_5831 = MUL_5829 - MUL_5830;
        auto SUB_5833 = SUB_5831 - MUL_5832;
        auto MUL_5835 = SUB_5833 * 2.0;
        auto ADD_5837 = MUL_5835 + 0.01955;
        auto ADD_5840 = ADD_5644 + ADD_5837;
        auto ADD_6148 = ADD_5840 + SUB_6145;
        auto MUL_14115 = ADD_14112 * 2.0;
        auto SUB_14118 = 1.0 - MUL_14115;
        auto MUL_14142 = SUB_14118 * 0.0372672;
        auto MUL_14090 = SUB_14088 * 2.0;
        auto MUL_14104 = ADD_14101 * 2.0;
        auto MUL_14135 = MUL_14104 * 0.02;
        auto MUL_14128 = MUL_14090 * 0.0004704;
        auto ADD_14145 = MUL_14128 + MUL_14135;
        auto SUB_14149 = MUL_14142 - ADD_14145;
        auto ADD_14152 = ADD_6148 + SUB_14149;
        auto MUL_14161 = MUL_14093 * 0.0568983;
        auto MUL_14168 = MUL_14108 * 0.0401042;
        auto MUL_14154 = SUB_14082 * 0.001;
        auto ADD_14173 = MUL_14154 + MUL_14161;
        auto ADD_14176 = ADD_14173 + MUL_14168;
        auto ADD_14179 = ADD_6146 + ADD_14176;
        auto MUL_14170 = MUL_14111 * 0.0401042;
        auto MUL_14156 = MUL_14086 * 0.001;
        auto MUL_14163 = SUB_14100 * 0.0568983;
        auto SUB_14174 = MUL_14163 - MUL_14156;
        auto ADD_14177 = SUB_14174 + MUL_14170;
        auto ADD_14180 = ADD_6147 + ADD_14177;
        auto MUL_14172 = SUB_14118 * 0.0401042;
        auto MUL_14165 = MUL_14104 * 0.0568983;
        auto MUL_14159 = MUL_14090 * 0.001;
        auto SUB_14175 = MUL_14159 - MUL_14165;
        auto ADD_14178 = SUB_14175 + MUL_14172;
        auto ADD_14181 = ADD_6148 + ADD_14178;
        auto MUL_14195 = MUL_14108 * 0.03;
        auto ADD_14200 = ADD_6146 + MUL_14195;
        auto MUL_14197 = MUL_14111 * 0.03;
        auto ADD_14201 = ADD_6147 + MUL_14197;
        auto MUL_14199 = SUB_14118 * 0.03;
        auto ADD_14202 = ADD_6148 + MUL_14199;
        auto MUL_14216 = MUL_14108 * 0.0699512;
        auto ADD_14221 = ADD_6146 + MUL_14216;
        auto MUL_14218 = MUL_14111 * 0.0699512;
        auto ADD_14222 = ADD_6147 + MUL_14218;
        auto MUL_14220 = SUB_14118 * 0.0699512;
        auto ADD_14223 = ADD_6148 + MUL_14220;
        if(/*base_link vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1538, 1711)
        if(/*link_arm_l0 vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_11900, ADD_11901, ADD_11902, 0.09, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        if(/*link_arm_l1 vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11733, SUB_11734, SUB_11735, 0.087, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        if(/*link_arm_l2 vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11569, SUB_11570, SUB_11571, 0.087, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        if(/*link_arm_l3 vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11405, SUB_11406, SUB_11407, 0.087, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        if(/*link_arm_l4 vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.12, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        if(/*link_gripper_s3_body*/ sphere_environment_in_collision(environment, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_environment_in_collision(environment, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        if(/*link_head_tilt vs. link_gripper_s3_body*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_12439, ADD_12440, ADD_12441, 0.07, ADD_14150, ADD_14151, ADD_14152, 0.08)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14179, ADD_14180, ADD_14181, 0.04)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14200, ADD_14201, ADD_14202, 0.056)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14221, ADD_14222, ADD_14223, 0.05)){ return false; } } // (1711, 1711)
        auto MUL_6253 = MUL_6089 * 0.5284008;
        auto MUL_6248 = MUL_6081 * 0.5284008;
        auto MUL_6241 = MUL_6105 * 0.4698858;
        auto MUL_6244 = MUL_6073 * 0.4698858;
        auto SUB_6230 = MUL_6244 - MUL_6241;
        auto SUB_6235 = SUB_6230 - MUL_6248;
        auto ADD_6245 = MUL_6241 + MUL_6244;
        auto ADD_6250 = ADD_6245 + MUL_6248;
        auto ADD_6254 = ADD_6250 + MUL_6253;
        auto ADD_6239 = SUB_6235 + MUL_6253;
        auto MUL_14233 = ADD_6254 * ADD_6254;
        auto MUL_14232 = ADD_6239 * ADD_6239;
        auto ADD_14241 = MUL_14232 + MUL_14233;
        auto MUL_14244 = ADD_14241 * 2.0;
        auto SUB_14247 = 1.0 - MUL_14244;
        auto MUL_14282 = SUB_14247 * 0.1;
        auto MUL_6274 = MUL_6081 * 0.0371909;
        auto MUL_6291 = MUL_6105 * MUL_6274;
        auto MUL_6284 = MUL_6073 * 0.0371909;
        auto MUL_6288 = MUL_6089 * 0.0246929;
        auto ADD_6289 = MUL_6284 + MUL_6288;
        auto MUL_6294 = MUL_6089 * ADD_6289;
        auto MUL_6281 = MUL_6081 * 0.0246929;
        auto MUL_6292 = MUL_6081 * MUL_6281;
        auto ADD_6293 = MUL_6291 + MUL_6292;
        auto ADD_6295 = ADD_6293 + MUL_6294;
        auto MUL_6297 = ADD_6295 * 2.0;
        auto SUB_6300 = MUL_6297 - 0.0246929;
        auto ADD_6321 = ADD_6146 + SUB_6300;
        auto SUB_14304 = ADD_6321 - MUL_14282;
        auto MUL_6302 = MUL_6105 * ADD_6289;
        auto MUL_6257 = MUL_6105 * 0.5284008;
        auto MUL_6306 = MUL_6089 * MUL_6274;
        auto MUL_6270 = MUL_6089 * 0.4698858;
        auto MUL_6266 = MUL_6081 * 0.4698858;
        auto MUL_6303 = MUL_6073 * MUL_6281;
        auto ADD_6305 = MUL_6302 + MUL_6303;
        auto SUB_6308 = ADD_6305 - MUL_6306;
        auto MUL_6310 = SUB_6308 * 2.0;
        auto ADD_6322 = ADD_6147 + MUL_6310;
        auto MUL_6261 = MUL_6073 * 0.5284008;
        auto SUB_6215 = MUL_6261 - MUL_6257;
        auto ADD_6262 = MUL_6257 + MUL_6261;
        auto SUB_6268 = MUL_6266 - ADD_6262;
        auto ADD_6272 = SUB_6268 + MUL_6270;
        auto ADD_6218 = SUB_6215 + MUL_6266;
        auto SUB_6222 = ADD_6218 - MUL_6270;
        auto MUL_14234 = ADD_6272 * ADD_6254;
        auto MUL_14238 = SUB_6222 * ADD_6239;
        auto ADD_14248 = MUL_14238 + MUL_14234;
        auto MUL_14250 = ADD_14248 * 2.0;
        auto MUL_14286 = MUL_14250 * 0.1;
        auto SUB_14305 = ADD_6322 - MUL_14286;
        auto MUL_14235 = ADD_6272 * ADD_6239;
        auto MUL_14239 = SUB_6222 * ADD_6254;
        auto SUB_14251 = MUL_14239 - MUL_14235;
        auto MUL_6312 = MUL_6105 * MUL_6281;
        auto MUL_6315 = MUL_6081 * MUL_6274;
        auto MUL_6313 = MUL_6073 * ADD_6289;
        auto SUB_6314 = MUL_6312 - MUL_6313;
        auto SUB_6316 = SUB_6314 - MUL_6315;
        auto MUL_6318 = SUB_6316 * 2.0;
        auto ADD_6320 = MUL_6318 + 0.0371909;
        auto ADD_6323 = ADD_6148 + ADD_6320;
        auto MUL_14253 = SUB_14251 * 2.0;
        auto MUL_14290 = MUL_14253 * 0.1;
        auto SUB_14306 = ADD_6323 - MUL_14290;
        auto SUB_14254 = MUL_14238 - MUL_14234;
        auto MUL_14256 = SUB_14254 * 2.0;
        auto MUL_14321 = MUL_14256 * 0.003;
        auto MUL_14309 = SUB_14247 * 0.067;
        auto ADD_14337 = MUL_14309 + MUL_14321;
        auto SUB_14343 = ADD_6321 - ADD_14337;
        auto MUL_14236 = SUB_6222 * SUB_6222;
        auto ADD_14257 = MUL_14233 + MUL_14236;
        auto MUL_14260 = ADD_14257 * 2.0;
        auto SUB_14263 = 1.0 - MUL_14260;
        auto MUL_14325 = SUB_14263 * 0.003;
        auto MUL_14313 = MUL_14250 * 0.067;
        auto ADD_14339 = MUL_14313 + MUL_14325;
        auto SUB_14344 = ADD_6322 - ADD_14339;
        auto MUL_14237 = ADD_6272 * SUB_6222;
        auto MUL_14240 = ADD_6239 * ADD_6254;
        auto ADD_14264 = MUL_14240 + MUL_14237;
        auto MUL_14266 = ADD_14264 * 2.0;
        auto MUL_14329 = MUL_14266 * 0.003;
        auto MUL_14317 = MUL_14253 * 0.067;
        auto ADD_14341 = MUL_14317 + MUL_14329;
        auto SUB_14345 = ADD_6323 - ADD_14341;
        auto MUL_14348 = SUB_14247 * 0.08;
        auto ADD_14376 = MUL_14348 + MUL_14321;
        auto SUB_14382 = ADD_6321 - ADD_14376;
        auto MUL_14352 = MUL_14250 * 0.08;
        auto ADD_14378 = MUL_14352 + MUL_14325;
        auto SUB_14383 = ADD_6322 - ADD_14378;
        auto MUL_14356 = MUL_14253 * 0.08;
        auto ADD_14380 = MUL_14356 + MUL_14329;
        auto SUB_14384 = ADD_6323 - ADD_14380;
        auto MUL_14387 = SUB_14247 * 0.093;
        auto ADD_14415 = MUL_14387 + MUL_14321;
        auto SUB_14421 = ADD_6321 - ADD_14415;
        auto MUL_14391 = MUL_14250 * 0.093;
        auto ADD_14417 = MUL_14391 + MUL_14325;
        auto SUB_14422 = ADD_6322 - ADD_14417;
        auto MUL_14395 = MUL_14253 * 0.093;
        auto ADD_14419 = MUL_14395 + MUL_14329;
        auto SUB_14423 = ADD_6323 - ADD_14419;
        auto MUL_14426 = SUB_14247 * 0.106;
        auto ADD_14454 = MUL_14426 + MUL_14321;
        auto SUB_14460 = ADD_6321 - ADD_14454;
        auto MUL_14430 = MUL_14250 * 0.106;
        auto ADD_14456 = MUL_14430 + MUL_14325;
        auto SUB_14461 = ADD_6322 - ADD_14456;
        auto MUL_14434 = MUL_14253 * 0.106;
        auto ADD_14458 = MUL_14434 + MUL_14329;
        auto SUB_14462 = ADD_6323 - ADD_14458;
        auto MUL_14465 = SUB_14247 * 0.119;
        auto MUL_14477 = MUL_14256 * 0.002;
        auto ADD_14493 = MUL_14465 + MUL_14477;
        auto SUB_14499 = ADD_6321 - ADD_14493;
        auto MUL_14481 = SUB_14263 * 0.002;
        auto MUL_14469 = MUL_14250 * 0.119;
        auto ADD_14495 = MUL_14469 + MUL_14481;
        auto SUB_14500 = ADD_6322 - ADD_14495;
        auto MUL_14485 = MUL_14266 * 0.002;
        auto MUL_14473 = MUL_14253 * 0.119;
        auto ADD_14497 = MUL_14473 + MUL_14485;
        auto SUB_14501 = ADD_6323 - ADD_14497;
        auto MUL_14504 = SUB_14247 * 0.131;
        auto MUL_14516 = MUL_14256 * 0.0017;
        auto ADD_14532 = MUL_14504 + MUL_14516;
        auto SUB_14538 = ADD_6321 - ADD_14532;
        auto MUL_14520 = SUB_14263 * 0.0017;
        auto MUL_14508 = MUL_14250 * 0.131;
        auto ADD_14534 = MUL_14508 + MUL_14520;
        auto SUB_14539 = ADD_6322 - ADD_14534;
        auto MUL_14524 = MUL_14266 * 0.0017;
        auto MUL_14512 = MUL_14253 * 0.131;
        auto ADD_14536 = MUL_14512 + MUL_14524;
        auto SUB_14540 = ADD_6323 - ADD_14536;
        auto MUL_14543 = SUB_14247 * 0.144;
        auto MUL_14555 = MUL_14256 * 0.0032;
        auto ADD_14571 = MUL_14543 + MUL_14555;
        auto SUB_14577 = ADD_6321 - ADD_14571;
        auto MUL_14559 = SUB_14263 * 0.0032;
        auto MUL_14547 = MUL_14250 * 0.144;
        auto ADD_14573 = MUL_14547 + MUL_14559;
        auto SUB_14578 = ADD_6322 - ADD_14573;
        auto MUL_14563 = MUL_14266 * 0.0032;
        auto MUL_14551 = MUL_14253 * 0.144;
        auto ADD_14575 = MUL_14551 + MUL_14563;
        auto SUB_14579 = ADD_6323 - ADD_14575;
        auto MUL_14594 = MUL_14256 * 0.009;
        auto MUL_14582 = SUB_14247 * 0.16;
        auto ADD_14610 = MUL_14582 + MUL_14594;
        auto SUB_14616 = ADD_6321 - ADD_14610;
        auto MUL_14598 = SUB_14263 * 0.009;
        auto MUL_14586 = MUL_14250 * 0.16;
        auto ADD_14612 = MUL_14586 + MUL_14598;
        auto SUB_14617 = ADD_6322 - ADD_14612;
        auto MUL_14602 = MUL_14266 * 0.009;
        auto MUL_14590 = MUL_14253 * 0.16;
        auto ADD_14614 = MUL_14590 + MUL_14602;
        auto SUB_14618 = ADD_6323 - ADD_14614;
        auto MUL_14621 = SUB_14247 * 0.17;
        auto MUL_14633 = MUL_14256 * 0.011;
        auto ADD_14649 = MUL_14621 + MUL_14633;
        auto SUB_14655 = ADD_6321 - ADD_14649;
        auto MUL_14637 = SUB_14263 * 0.011;
        auto MUL_14625 = MUL_14250 * 0.17;
        auto ADD_14651 = MUL_14625 + MUL_14637;
        auto SUB_14656 = ADD_6322 - ADD_14651;
        auto MUL_14641 = MUL_14266 * 0.011;
        auto MUL_14629 = MUL_14253 * 0.17;
        auto ADD_14653 = MUL_14629 + MUL_14641;
        auto SUB_14657 = ADD_6323 - ADD_14653;
        auto MUL_14672 = MUL_14256 * 0.013;
        auto MUL_14660 = SUB_14247 * 0.178;
        auto ADD_14688 = MUL_14660 + MUL_14672;
        auto SUB_14694 = ADD_6321 - ADD_14688;
        auto MUL_14676 = SUB_14263 * 0.013;
        auto MUL_14664 = MUL_14250 * 0.178;
        auto ADD_14690 = MUL_14664 + MUL_14676;
        auto SUB_14695 = ADD_6322 - ADD_14690;
        auto MUL_14680 = MUL_14266 * 0.013;
        auto MUL_14668 = MUL_14253 * 0.178;
        auto ADD_14692 = MUL_14668 + MUL_14680;
        auto SUB_14696 = ADD_6323 - ADD_14692;
        if(/*base_link vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1711, 1899)
        if(/*link_arm_l0 vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_11900, ADD_11901, ADD_11902, 0.09, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        if(/*link_arm_l1 vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11733, SUB_11734, SUB_11735, 0.087, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        if(/*link_arm_l2 vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11569, SUB_11570, SUB_11571, 0.087, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        if(/*link_arm_l3 vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11405, SUB_11406, SUB_11407, 0.087, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        if(/*link_arm_l4 vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.12, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        if(/*link_gripper_finger_right*/ sphere_environment_in_collision(environment, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_environment_in_collision(environment, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_environment_in_collision(environment, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        if(/*link_head_tilt vs. link_gripper_finger_right*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_12439, ADD_12440, ADD_12441, 0.07, SUB_14304, SUB_14305, SUB_14306, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14343, SUB_14344, SUB_14345, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14382, SUB_14383, SUB_14384, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14421, SUB_14422, SUB_14423, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14460, SUB_14461, SUB_14462, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14499, SUB_14500, SUB_14501, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14538, SUB_14539, SUB_14540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14577, SUB_14578, SUB_14579, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14616, SUB_14617, SUB_14618, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14655, SUB_14656, SUB_14657, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, SUB_14694, SUB_14695, SUB_14696, 0.012)){ return false; } } // (1899, 1899)
        auto MUL_6430 = ADD_6272 * 0.7010574;
        auto MUL_6411 = ADD_6254 * 0.7010574;
        auto MUL_6407 = ADD_6239 * 0.7010574;
        auto MUL_6433 = SUB_6222 * 0.7010574;
        auto SUB_6389 = MUL_6433 - MUL_6430;
        auto ADD_6435 = MUL_6430 + MUL_6433;
        auto MUL_6463 = ADD_6254 * 0.1777845;
        auto MUL_6468 = ADD_6254 * MUL_6463;
        auto MUL_6456 = ADD_6239 * 0.1777845;
        auto MUL_6400 = ADD_6272 * 0.092296;
        auto MUL_6442 = ADD_6254 * 0.092296;
        auto MUL_6438 = ADD_6239 * 0.092296;
        auto ADD_6440 = ADD_6435 + MUL_6438;
        auto SUB_6443 = ADD_6440 - MUL_6442;
        auto ADD_6392 = SUB_6389 + MUL_6438;
        auto ADD_6397 = ADD_6392 + MUL_6442;
        auto MUL_6403 = SUB_6222 * 0.092296;
        auto SUB_6420 = MUL_6400 - MUL_6403;
        auto ADD_6425 = SUB_6420 + MUL_6407;
        auto ADD_6428 = ADD_6425 + MUL_6411;
        auto ADD_6404 = MUL_6400 + MUL_6403;
        auto SUB_6408 = MUL_6407 - ADD_6404;
        auto SUB_6413 = SUB_6408 - MUL_6411;
        auto MUL_14722 = SUB_6443 * SUB_6413;
        auto MUL_14720 = ADD_6428 * ADD_6428;
        auto MUL_14719 = SUB_6413 * SUB_6413;
        auto ADD_14728 = MUL_14719 + MUL_14720;
        auto MUL_14726 = ADD_6397 * ADD_6428;
        auto ADD_14754 = MUL_14726 + MUL_14722;
        auto MUL_14756 = ADD_14754 * 2.0;
        auto MUL_14731 = ADD_14728 * 2.0;
        auto SUB_14734 = 1.0 - MUL_14731;
        auto MUL_14768 = SUB_14734 * 1e-06;
        auto MUL_6448 = ADD_6254 * 0.0106722;
        auto MUL_6465 = ADD_6272 * MUL_6448;
        auto MUL_6452 = SUB_6222 * 0.0106722;
        auto SUB_6458 = MUL_6456 - MUL_6452;
        auto MUL_6466 = ADD_6239 * SUB_6458;
        auto ADD_6467 = MUL_6465 + MUL_6466;
        auto ADD_6469 = ADD_6467 + MUL_6468;
        auto MUL_6471 = ADD_6469 * 2.0;
        auto SUB_6474 = MUL_6471 - 0.1777845;
        auto ADD_6496 = ADD_6321 + SUB_6474;
        auto MUL_14780 = MUL_14756 * 0.0085;
        auto ADD_14785 = MUL_14768 + MUL_14780;
        auto ADD_14788 = ADD_6496 + ADD_14785;
        auto MUL_14721 = SUB_6443 * ADD_6428;
        auto MUL_14724 = SUB_6443 * ADD_6397;
        auto MUL_14727 = SUB_6413 * ADD_6428;
        auto SUB_14757 = MUL_14727 - MUL_14724;
        auto MUL_14725 = ADD_6397 * SUB_6413;
        auto ADD_14735 = MUL_14725 + MUL_14721;
        auto MUL_6476 = ADD_6272 * MUL_6463;
        auto MUL_6481 = ADD_6254 * MUL_6448;
        auto MUL_6478 = SUB_6222 * SUB_6458;
        auto ADD_6479 = MUL_6476 + MUL_6478;
        auto SUB_6482 = MUL_6481 - ADD_6479;
        auto MUL_6484 = SUB_6482 * 2.0;
        auto SUB_6487 = MUL_6484 - 0.0106722;
        auto ADD_6497 = ADD_6322 + SUB_6487;
        auto MUL_14759 = SUB_14757 * 2.0;
        auto MUL_14782 = MUL_14759 * 0.0085;
        auto MUL_14737 = ADD_14735 * 2.0;
        auto MUL_14770 = MUL_14737 * 1e-06;
        auto ADD_14786 = MUL_14770 + MUL_14782;
        auto ADD_14789 = ADD_6497 + ADD_14786;
        auto SUB_14738 = MUL_14726 - MUL_14722;
        auto MUL_14723 = ADD_6397 * ADD_6397;
        auto ADD_14760 = MUL_14719 + MUL_14723;
        auto MUL_6488 = ADD_6272 * SUB_6458;
        auto MUL_6491 = ADD_6239 * MUL_6448;
        auto MUL_6489 = SUB_6222 * MUL_6463;
        auto SUB_6490 = MUL_6488 - MUL_6489;
        auto SUB_6492 = SUB_6490 - MUL_6491;
        auto MUL_6494 = SUB_6492 * 2.0;
        auto ADD_6498 = ADD_6323 + MUL_6494;
        auto MUL_14763 = ADD_14760 * 2.0;
        auto SUB_14766 = 1.0 - MUL_14763;
        auto MUL_14784 = SUB_14766 * 0.0085;
        auto MUL_14740 = SUB_14738 * 2.0;
        auto MUL_14772 = MUL_14740 * 1e-06;
        auto ADD_14787 = MUL_14772 + MUL_14784;
        auto ADD_14790 = ADD_6498 + ADD_14787;
        auto MUL_14804 = MUL_14756 * 0.004;
        auto ADD_14809 = ADD_6496 + MUL_14804;
        auto MUL_14806 = MUL_14759 * 0.004;
        auto ADD_14810 = ADD_6497 + MUL_14806;
        auto MUL_14808 = SUB_14766 * 0.004;
        auto ADD_14811 = ADD_6498 + MUL_14808;
        auto SUB_14741 = MUL_14725 - MUL_14721;
        auto MUL_14743 = SUB_14741 * 2.0;
        auto MUL_14820 = MUL_14743 * 0.017;
        auto MUL_14831 = MUL_14756 * 0.0115;
        auto SUB_14836 = MUL_14831 - MUL_14820;
        auto ADD_14839 = ADD_6496 + SUB_14836;
        auto ADD_14744 = MUL_14720 + MUL_14723;
        auto MUL_14833 = MUL_14759 * 0.0115;
        auto MUL_14747 = ADD_14744 * 2.0;
        auto SUB_14750 = 1.0 - MUL_14747;
        auto MUL_14824 = SUB_14750 * 0.017;
        auto SUB_14837 = MUL_14833 - MUL_14824;
        auto ADD_14840 = ADD_6497 + SUB_14837;
        auto ADD_14751 = MUL_14727 + MUL_14724;
        auto MUL_14835 = SUB_14766 * 0.0115;
        auto MUL_14753 = ADD_14751 * 2.0;
        auto MUL_14828 = MUL_14753 * 0.017;
        auto SUB_14838 = MUL_14835 - MUL_14828;
        auto ADD_14841 = ADD_6498 + SUB_14838;
        auto ADD_14860 = MUL_14820 + MUL_14831;
        auto ADD_14863 = ADD_6496 + ADD_14860;
        auto ADD_14861 = MUL_14824 + MUL_14833;
        auto ADD_14864 = ADD_6497 + ADD_14861;
        auto ADD_14862 = MUL_14828 + MUL_14835;
        auto ADD_14865 = ADD_6498 + ADD_14862;
        auto MUL_14867 = SUB_14734 * 0.017;
        auto ADD_14884 = MUL_14867 + MUL_14831;
        auto ADD_14887 = ADD_6496 + ADD_14884;
        auto MUL_14869 = MUL_14737 * 0.017;
        auto ADD_14885 = MUL_14869 + MUL_14833;
        auto ADD_14888 = ADD_6497 + ADD_14885;
        auto MUL_14871 = MUL_14740 * 0.017;
        auto ADD_14886 = MUL_14871 + MUL_14835;
        auto ADD_14889 = ADD_6498 + ADD_14886;
        auto SUB_14914 = MUL_14831 - MUL_14867;
        auto ADD_14917 = ADD_6496 + SUB_14914;
        auto SUB_14915 = MUL_14833 - MUL_14869;
        auto ADD_14918 = ADD_6497 + SUB_14915;
        auto SUB_14916 = MUL_14835 - MUL_14871;
        auto ADD_14919 = ADD_6498 + SUB_14916;
        auto MUL_14928 = MUL_14743 * 0.012;
        auto MUL_14921 = SUB_14734 * 0.012;
        auto SUB_14944 = MUL_14921 - MUL_14928;
        auto ADD_14947 = SUB_14944 + MUL_14831;
        auto ADD_14950 = ADD_6496 + ADD_14947;
        auto MUL_14932 = SUB_14750 * 0.012;
        auto MUL_14923 = MUL_14737 * 0.012;
        auto SUB_14945 = MUL_14923 - MUL_14932;
        auto ADD_14948 = SUB_14945 + MUL_14833;
        auto ADD_14951 = ADD_6497 + ADD_14948;
        auto MUL_14936 = MUL_14753 * 0.012;
        auto MUL_14925 = MUL_14740 * 0.012;
        auto SUB_14946 = MUL_14925 - MUL_14936;
        auto ADD_14949 = SUB_14946 + MUL_14835;
        auto ADD_14952 = ADD_6498 + ADD_14949;
        auto ADD_14971 = MUL_14921 + MUL_14928;
        auto ADD_14974 = ADD_14971 + MUL_14831;
        auto ADD_14977 = ADD_6496 + ADD_14974;
        auto ADD_14972 = MUL_14923 + MUL_14932;
        auto ADD_14975 = ADD_14972 + MUL_14833;
        auto ADD_14978 = ADD_6497 + ADD_14975;
        auto ADD_14973 = MUL_14925 + MUL_14936;
        auto ADD_14976 = ADD_14973 + MUL_14835;
        auto ADD_14979 = ADD_6498 + ADD_14976;
        auto SUB_15004 = MUL_14928 - MUL_14921;
        auto ADD_15007 = SUB_15004 + MUL_14831;
        auto ADD_15010 = ADD_6496 + ADD_15007;
        auto SUB_15005 = MUL_14932 - MUL_14923;
        auto ADD_15008 = SUB_15005 + MUL_14833;
        auto ADD_15011 = ADD_6497 + ADD_15008;
        auto SUB_15006 = MUL_14936 - MUL_14925;
        auto ADD_15009 = SUB_15006 + MUL_14835;
        auto ADD_15012 = ADD_6498 + ADD_15009;
        auto SUB_15049 = MUL_14831 - ADD_14971;
        auto ADD_15052 = ADD_6496 + SUB_15049;
        auto SUB_15050 = MUL_14833 - ADD_14972;
        auto ADD_15053 = ADD_6497 + SUB_15050;
        auto SUB_15051 = MUL_14835 - ADD_14973;
        auto ADD_15054 = ADD_6498 + SUB_15051;
        auto MUL_15063 = MUL_14743 * 0.0157;
        auto MUL_15056 = SUB_14734 * 0.0065;
        auto SUB_15079 = MUL_15056 - MUL_15063;
        auto ADD_15082 = SUB_15079 + MUL_14831;
        auto ADD_15085 = ADD_6496 + ADD_15082;
        auto MUL_15067 = SUB_14750 * 0.0157;
        auto MUL_15058 = MUL_14737 * 0.0065;
        auto SUB_15080 = MUL_15058 - MUL_15067;
        auto ADD_15083 = SUB_15080 + MUL_14833;
        auto ADD_15086 = ADD_6497 + ADD_15083;
        auto MUL_15071 = MUL_14753 * 0.0157;
        auto MUL_15060 = MUL_14740 * 0.0065;
        auto SUB_15081 = MUL_15060 - MUL_15071;
        auto ADD_15084 = SUB_15081 + MUL_14835;
        auto ADD_15087 = ADD_6498 + ADD_15084;
        auto ADD_15106 = MUL_15056 + MUL_15063;
        auto ADD_15109 = ADD_15106 + MUL_14831;
        auto ADD_15112 = ADD_6496 + ADD_15109;
        auto ADD_15107 = MUL_15058 + MUL_15067;
        auto ADD_15110 = ADD_15107 + MUL_14833;
        auto ADD_15113 = ADD_6497 + ADD_15110;
        auto ADD_15108 = MUL_15060 + MUL_15071;
        auto ADD_15111 = ADD_15108 + MUL_14835;
        auto ADD_15114 = ADD_6498 + ADD_15111;
        auto SUB_15139 = MUL_15063 - MUL_15056;
        auto ADD_15142 = SUB_15139 + MUL_14831;
        auto ADD_15145 = ADD_6496 + ADD_15142;
        auto SUB_15140 = MUL_15067 - MUL_15058;
        auto ADD_15143 = SUB_15140 + MUL_14833;
        auto ADD_15146 = ADD_6497 + ADD_15143;
        auto SUB_15141 = MUL_15071 - MUL_15060;
        auto ADD_15144 = SUB_15141 + MUL_14835;
        auto ADD_15147 = ADD_6498 + ADD_15144;
        auto SUB_15184 = MUL_14831 - ADD_15106;
        auto ADD_15187 = ADD_6496 + SUB_15184;
        auto SUB_15185 = MUL_14833 - ADD_15107;
        auto ADD_15188 = ADD_6497 + SUB_15185;
        auto SUB_15186 = MUL_14835 - ADD_15108;
        auto ADD_15189 = ADD_6498 + SUB_15186;
        auto MUL_15198 = MUL_14743 * 0.0065;
        auto MUL_15191 = SUB_14734 * 0.0157;
        auto SUB_15214 = MUL_15191 - MUL_15198;
        auto ADD_15217 = SUB_15214 + MUL_14831;
        auto ADD_15220 = ADD_6496 + ADD_15217;
        auto MUL_15202 = SUB_14750 * 0.0065;
        auto MUL_15193 = MUL_14737 * 0.0157;
        auto SUB_15215 = MUL_15193 - MUL_15202;
        auto ADD_15218 = SUB_15215 + MUL_14833;
        auto ADD_15221 = ADD_6497 + ADD_15218;
        auto MUL_15206 = MUL_14753 * 0.0065;
        auto MUL_15195 = MUL_14740 * 0.0157;
        auto SUB_15216 = MUL_15195 - MUL_15206;
        auto ADD_15219 = SUB_15216 + MUL_14835;
        auto ADD_15222 = ADD_6498 + ADD_15219;
        auto ADD_15241 = MUL_15191 + MUL_15198;
        auto ADD_15244 = ADD_15241 + MUL_14831;
        auto ADD_15247 = ADD_6496 + ADD_15244;
        auto ADD_15242 = MUL_15193 + MUL_15202;
        auto ADD_15245 = ADD_15242 + MUL_14833;
        auto ADD_15248 = ADD_6497 + ADD_15245;
        auto ADD_15243 = MUL_15195 + MUL_15206;
        auto ADD_15246 = ADD_15243 + MUL_14835;
        auto ADD_15249 = ADD_6498 + ADD_15246;
        auto SUB_15274 = MUL_15198 - MUL_15191;
        auto ADD_15277 = SUB_15274 + MUL_14831;
        auto ADD_15280 = ADD_6496 + ADD_15277;
        auto SUB_15275 = MUL_15202 - MUL_15193;
        auto ADD_15278 = SUB_15275 + MUL_14833;
        auto ADD_15281 = ADD_6497 + ADD_15278;
        auto SUB_15276 = MUL_15206 - MUL_15195;
        auto ADD_15279 = SUB_15276 + MUL_14835;
        auto ADD_15282 = ADD_6498 + ADD_15279;
        auto SUB_15319 = MUL_14831 - ADD_15241;
        auto ADD_15322 = ADD_6496 + SUB_15319;
        auto SUB_15320 = MUL_14833 - ADD_15242;
        auto ADD_15323 = ADD_6497 + SUB_15320;
        auto SUB_15321 = MUL_14835 - ADD_15243;
        auto ADD_15324 = ADD_6498 + SUB_15321;
        if(/*base_link vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (1899, 2145)
        if(/*link_arm_l0 vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_11900, ADD_11901, ADD_11902, 0.09, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        if(/*link_arm_l1 vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11733, SUB_11734, SUB_11735, 0.087, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        if(/*link_arm_l2 vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11569, SUB_11570, SUB_11571, 0.087, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        if(/*link_arm_l3 vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11405, SUB_11406, SUB_11407, 0.087, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        if(/*link_arm_l4 vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.12, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        if(/*link_gripper_fingertip_right*/ sphere_environment_in_collision(environment, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_environment_in_collision(environment, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        if(/*link_head_tilt vs. link_gripper_fingertip_right*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_12439, ADD_12440, ADD_12441, 0.07, ADD_14788, ADD_14789, ADD_14790, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14809, ADD_14810, ADD_14811, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14839, ADD_14840, ADD_14841, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14863, ADD_14864, ADD_14865, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14887, ADD_14888, ADD_14889, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14917, ADD_14918, ADD_14919, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14950, ADD_14951, ADD_14952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_14977, ADD_14978, ADD_14979, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15010, ADD_15011, ADD_15012, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15052, ADD_15053, ADD_15054, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15085, ADD_15086, ADD_15087, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15112, ADD_15113, ADD_15114, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15145, ADD_15146, ADD_15147, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15187, ADD_15188, ADD_15189, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15220, ADD_15221, ADD_15222, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15247, ADD_15248, ADD_15249, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15280, ADD_15281, ADD_15282, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15322, ADD_15323, ADD_15324, 0.006)){ return false; } } // (2145, 2145)
        auto SUB_6785 = MUL_6248 - ADD_6245;
        auto SUB_6762 = MUL_6241 - MUL_6244;
        auto SUB_6767 = SUB_6762 - MUL_6248;
        auto ADD_6789 = SUB_6785 + MUL_6253;
        auto ADD_6771 = SUB_6767 + MUL_6253;
        auto MUL_15410 = ADD_6789 * ADD_6789;
        auto MUL_15409 = ADD_6771 * ADD_6771;
        auto ADD_15421 = MUL_15409 + MUL_15410;
        auto MUL_15424 = ADD_15421 * 2.0;
        auto SUB_15427 = 1.0 - MUL_15424;
        auto MUL_15461 = SUB_15427 * 0.1;
        auto MUL_6821 = MUL_6089 * 0.0245029;
        auto SUB_6823 = MUL_6821 - MUL_6284;
        auto MUL_6828 = MUL_6089 * SUB_6823;
        auto MUL_6815 = MUL_6081 * 0.0245029;
        auto MUL_6825 = MUL_6081 * MUL_6815;
        auto SUB_6827 = MUL_6291 - MUL_6825;
        auto SUB_6830 = SUB_6827 - MUL_6828;
        auto MUL_6832 = SUB_6830 * 2.0;
        auto ADD_6834 = MUL_6832 + 0.0245029;
        auto ADD_6860 = ADD_6146 + ADD_6834;
        auto ADD_15478 = ADD_6860 + MUL_15461;
        auto SUB_6752 = SUB_6215 - MUL_6266;
        auto ADD_6801 = ADD_6262 + MUL_6266;
        auto ADD_6806 = ADD_6801 + MUL_6270;
        auto ADD_6756 = SUB_6752 + MUL_6270;
        auto MUL_15411 = ADD_6806 * ADD_6789;
        auto MUL_15418 = ADD_6756 * ADD_6771;
        auto SUB_15428 = MUL_15418 - MUL_15411;
        auto MUL_6836 = MUL_6105 * SUB_6823;
        auto MUL_6838 = MUL_6073 * MUL_6815;
        auto ADD_6839 = MUL_6836 + MUL_6838;
        auto ADD_6843 = ADD_6839 + MUL_6306;
        auto MUL_6846 = ADD_6843 * 2.0;
        auto SUB_6861 = ADD_6147 - MUL_6846;
        auto MUL_15430 = SUB_15428 * 2.0;
        auto MUL_15463 = MUL_15430 * 0.1;
        auto ADD_15479 = SUB_6861 + MUL_15463;
        auto MUL_15413 = ADD_6806 * ADD_6771;
        auto MUL_15419 = ADD_6756 * ADD_6789;
        auto ADD_15431 = MUL_15419 + MUL_15413;
        auto MUL_6849 = MUL_6105 * MUL_6815;
        auto MUL_6851 = MUL_6073 * SUB_6823;
        auto SUB_6853 = MUL_6851 - MUL_6849;
        auto SUB_6855 = SUB_6853 - MUL_6315;
        auto MUL_6857 = SUB_6855 * 2.0;
        auto ADD_6859 = MUL_6857 + 0.0371909;
        auto ADD_6862 = ADD_6148 + ADD_6859;
        auto MUL_15433 = ADD_15431 * 2.0;
        auto MUL_15465 = MUL_15433 * 0.1;
        auto ADD_15480 = ADD_6862 + MUL_15465;
        auto ADD_15434 = MUL_15418 + MUL_15411;
        auto MUL_15436 = ADD_15434 * 2.0;
        auto MUL_15489 = MUL_15436 * 0.003;
        auto MUL_15482 = SUB_15427 * 0.067;
        auto SUB_15505 = MUL_15482 - MUL_15489;
        auto ADD_15508 = ADD_6860 + SUB_15505;
        auto MUL_15415 = ADD_6756 * ADD_6756;
        auto ADD_15437 = MUL_15410 + MUL_15415;
        auto MUL_15440 = ADD_15437 * 2.0;
        auto SUB_15443 = 1.0 - MUL_15440;
        auto MUL_15493 = SUB_15443 * 0.003;
        auto MUL_15484 = MUL_15430 * 0.067;
        auto SUB_15506 = MUL_15484 - MUL_15493;
        auto ADD_15509 = SUB_6861 + SUB_15506;
        auto MUL_15416 = ADD_6806 * ADD_6756;
        auto MUL_15420 = ADD_6771 * ADD_6789;
        auto SUB_15444 = MUL_15420 - MUL_15416;
        auto MUL_15446 = SUB_15444 * 2.0;
        auto MUL_15497 = MUL_15446 * 0.003;
        auto MUL_15486 = MUL_15433 * 0.067;
        auto SUB_15507 = MUL_15486 - MUL_15497;
        auto ADD_15510 = ADD_6862 + SUB_15507;
        auto MUL_15512 = SUB_15427 * 0.08;
        auto SUB_15535 = MUL_15512 - MUL_15489;
        auto ADD_15538 = ADD_6860 + SUB_15535;
        auto MUL_15514 = MUL_15430 * 0.08;
        auto SUB_15536 = MUL_15514 - MUL_15493;
        auto ADD_15539 = SUB_6861 + SUB_15536;
        auto MUL_15516 = MUL_15433 * 0.08;
        auto SUB_15537 = MUL_15516 - MUL_15497;
        auto ADD_15540 = ADD_6862 + SUB_15537;
        auto MUL_15542 = SUB_15427 * 0.093;
        auto SUB_15565 = MUL_15542 - MUL_15489;
        auto ADD_15568 = ADD_6860 + SUB_15565;
        auto MUL_15544 = MUL_15430 * 0.093;
        auto SUB_15566 = MUL_15544 - MUL_15493;
        auto ADD_15569 = SUB_6861 + SUB_15566;
        auto MUL_15546 = MUL_15433 * 0.093;
        auto SUB_15567 = MUL_15546 - MUL_15497;
        auto ADD_15570 = ADD_6862 + SUB_15567;
        auto MUL_15572 = SUB_15427 * 0.106;
        auto SUB_15595 = MUL_15572 - MUL_15489;
        auto ADD_15598 = ADD_6860 + SUB_15595;
        auto MUL_15574 = MUL_15430 * 0.106;
        auto SUB_15596 = MUL_15574 - MUL_15493;
        auto ADD_15599 = SUB_6861 + SUB_15596;
        auto MUL_15576 = MUL_15433 * 0.106;
        auto SUB_15597 = MUL_15576 - MUL_15497;
        auto ADD_15600 = ADD_6862 + SUB_15597;
        auto MUL_15609 = MUL_15436 * 0.002;
        auto MUL_15602 = SUB_15427 * 0.119;
        auto SUB_15625 = MUL_15602 - MUL_15609;
        auto ADD_15628 = ADD_6860 + SUB_15625;
        auto MUL_15613 = SUB_15443 * 0.002;
        auto MUL_15604 = MUL_15430 * 0.119;
        auto SUB_15626 = MUL_15604 - MUL_15613;
        auto ADD_15629 = SUB_6861 + SUB_15626;
        auto MUL_15617 = MUL_15446 * 0.002;
        auto MUL_15606 = MUL_15433 * 0.119;
        auto SUB_15627 = MUL_15606 - MUL_15617;
        auto ADD_15630 = ADD_6862 + SUB_15627;
        auto MUL_15639 = MUL_15436 * 0.0017;
        auto MUL_15632 = SUB_15427 * 0.131;
        auto SUB_15655 = MUL_15632 - MUL_15639;
        auto ADD_15658 = ADD_6860 + SUB_15655;
        auto MUL_15643 = SUB_15443 * 0.0017;
        auto MUL_15634 = MUL_15430 * 0.131;
        auto SUB_15656 = MUL_15634 - MUL_15643;
        auto ADD_15659 = SUB_6861 + SUB_15656;
        auto MUL_15647 = MUL_15446 * 0.0017;
        auto MUL_15636 = MUL_15433 * 0.131;
        auto SUB_15657 = MUL_15636 - MUL_15647;
        auto ADD_15660 = ADD_6862 + SUB_15657;
        auto MUL_15669 = MUL_15436 * 0.0032;
        auto MUL_15662 = SUB_15427 * 0.144;
        auto SUB_15685 = MUL_15662 - MUL_15669;
        auto ADD_15688 = ADD_6860 + SUB_15685;
        auto MUL_15673 = SUB_15443 * 0.0032;
        auto MUL_15664 = MUL_15430 * 0.144;
        auto SUB_15686 = MUL_15664 - MUL_15673;
        auto ADD_15689 = SUB_6861 + SUB_15686;
        auto MUL_15677 = MUL_15446 * 0.0032;
        auto MUL_15666 = MUL_15433 * 0.144;
        auto SUB_15687 = MUL_15666 - MUL_15677;
        auto ADD_15690 = ADD_6862 + SUB_15687;
        auto MUL_15699 = MUL_15436 * 0.009;
        auto MUL_15692 = SUB_15427 * 0.16;
        auto SUB_15715 = MUL_15692 - MUL_15699;
        auto ADD_15718 = ADD_6860 + SUB_15715;
        auto MUL_15703 = SUB_15443 * 0.009;
        auto MUL_15694 = MUL_15430 * 0.16;
        auto SUB_15716 = MUL_15694 - MUL_15703;
        auto ADD_15719 = SUB_6861 + SUB_15716;
        auto MUL_15707 = MUL_15446 * 0.009;
        auto MUL_15696 = MUL_15433 * 0.16;
        auto SUB_15717 = MUL_15696 - MUL_15707;
        auto ADD_15720 = ADD_6862 + SUB_15717;
        auto MUL_15729 = MUL_15436 * 0.011;
        auto MUL_15722 = SUB_15427 * 0.17;
        auto SUB_15745 = MUL_15722 - MUL_15729;
        auto ADD_15748 = ADD_6860 + SUB_15745;
        auto MUL_15733 = SUB_15443 * 0.011;
        auto MUL_15724 = MUL_15430 * 0.17;
        auto SUB_15746 = MUL_15724 - MUL_15733;
        auto ADD_15749 = SUB_6861 + SUB_15746;
        auto MUL_15737 = MUL_15446 * 0.011;
        auto MUL_15726 = MUL_15433 * 0.17;
        auto SUB_15747 = MUL_15726 - MUL_15737;
        auto ADD_15750 = ADD_6862 + SUB_15747;
        auto MUL_15759 = MUL_15436 * 0.013;
        auto MUL_15752 = SUB_15427 * 0.178;
        auto SUB_15775 = MUL_15752 - MUL_15759;
        auto ADD_15778 = ADD_6860 + SUB_15775;
        auto MUL_15763 = SUB_15443 * 0.013;
        auto MUL_15754 = MUL_15430 * 0.178;
        auto SUB_15776 = MUL_15754 - MUL_15763;
        auto ADD_15779 = SUB_6861 + SUB_15776;
        auto MUL_15767 = MUL_15446 * 0.013;
        auto MUL_15756 = MUL_15433 * 0.178;
        auto SUB_15777 = MUL_15756 - MUL_15767;
        auto ADD_15780 = ADD_6862 + SUB_15777;
        if(/*base_link vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2145, 2317)
        if(/*link_arm_l0 vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_11900, ADD_11901, ADD_11902, 0.09, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        if(/*link_arm_l1 vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11733, SUB_11734, SUB_11735, 0.087, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        if(/*link_arm_l2 vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11569, SUB_11570, SUB_11571, 0.087, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        if(/*link_arm_l3 vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11405, SUB_11406, SUB_11407, 0.087, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        if(/*link_arm_l4 vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.12, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        if(/*link_gripper_finger_left*/ sphere_environment_in_collision(environment, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_environment_in_collision(environment, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        if(/*link_head_tilt vs. link_gripper_finger_left*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_12439, ADD_12440, ADD_12441, 0.07, ADD_15478, ADD_15479, ADD_15480, 0.062)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15508, ADD_15509, ADD_15510, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15538, ADD_15539, ADD_15540, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15568, ADD_15569, ADD_15570, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15598, ADD_15599, ADD_15600, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15628, ADD_15629, ADD_15630, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15658, ADD_15659, ADD_15660, 0.02)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15688, ADD_15689, ADD_15690, 0.016)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15718, ADD_15719, ADD_15720, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15748, ADD_15749, ADD_15750, 0.013)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15778, ADD_15779, ADD_15780, 0.012)){ return false; } } // (2317, 2317)
        auto MUL_6934 = ADD_6806 * 0.7010574;
        auto MUL_6959 = ADD_6806 * 0.092296;
        auto MUL_6970 = ADD_6789 * 0.7010574;
        auto MUL_6990 = ADD_6789 * 0.1777845;
        auto MUL_6998 = ADD_6789 * MUL_6990;
        auto MUL_6944 = ADD_6789 * 0.092296;
        auto MUL_6977 = ADD_6789 * 0.0106722;
        auto MUL_6992 = ADD_6806 * MUL_6977;
        auto MUL_6966 = ADD_6771 * 0.7010574;
        auto MUL_6984 = ADD_6771 * 0.1777845;
        auto MUL_6941 = ADD_6771 * 0.092296;
        auto MUL_6937 = ADD_6756 * 0.7010574;
        auto SUB_6951 = MUL_6937 - MUL_6934;
        auto SUB_6954 = SUB_6951 - MUL_6941;
        auto ADD_6957 = SUB_6954 + MUL_6944;
        auto ADD_6938 = MUL_6934 + MUL_6937;
        auto SUB_6942 = MUL_6941 - ADD_6938;
        auto ADD_6945 = SUB_6942 + MUL_6944;
        auto MUL_15804 = ADD_6957 * ADD_6957;
        auto MUL_15803 = ADD_6945 * ADD_6945;
        auto ADD_15815 = MUL_15803 + MUL_15804;
        auto MUL_6962 = ADD_6756 * 0.092296;
        auto SUB_6926 = MUL_6962 - MUL_6959;
        auto ADD_6963 = MUL_6959 + MUL_6962;
        auto ADD_6967 = ADD_6963 + MUL_6966;
        auto ADD_6971 = ADD_6967 + MUL_6970;
        auto ADD_6929 = SUB_6926 + MUL_6966;
        auto SUB_6932 = ADD_6929 - MUL_6970;
        auto MUL_15807 = ADD_6971 * ADD_6945;
        auto MUL_15813 = SUB_6932 * ADD_6957;
        auto SUB_15841 = MUL_15813 - MUL_15807;
        auto MUL_6981 = ADD_6756 * 0.0106722;
        auto ADD_6985 = MUL_6981 + MUL_6984;
        auto MUL_6994 = ADD_6771 * ADD_6985;
        auto ADD_6996 = MUL_6992 + MUL_6994;
        auto ADD_7000 = ADD_6996 + MUL_6998;
        auto MUL_7003 = ADD_7000 * 2.0;
        auto SUB_7006 = 0.1777845 - MUL_7003;
        auto ADD_7028 = ADD_6860 + SUB_7006;
        auto MUL_15843 = SUB_15841 * 2.0;
        auto MUL_15867 = MUL_15843 * 0.0085;
        auto MUL_15818 = ADD_15815 * 2.0;
        auto SUB_15821 = 1.0 - MUL_15818;
        auto MUL_15855 = SUB_15821 * 1e-06;
        auto ADD_15872 = MUL_15855 + MUL_15867;
        auto ADD_15875 = ADD_7028 + ADD_15872;
        auto MUL_15805 = ADD_6971 * ADD_6957;
        auto MUL_15810 = ADD_6971 * SUB_6932;
        auto MUL_15814 = ADD_6945 * ADD_6957;
        auto ADD_15844 = MUL_15814 + MUL_15810;
        auto MUL_15812 = SUB_6932 * ADD_6945;
        auto SUB_15822 = MUL_15812 - MUL_15805;
        auto MUL_7007 = ADD_6806 * MUL_6990;
        auto MUL_7012 = ADD_6789 * MUL_6977;
        auto MUL_7009 = ADD_6756 * ADD_6985;
        auto SUB_7011 = MUL_7009 - MUL_7007;
        auto ADD_7013 = SUB_7011 + MUL_7012;
        auto MUL_7015 = ADD_7013 * 2.0;
        auto SUB_7018 = MUL_7015 - 0.0106722;
        auto ADD_7029 = SUB_6861 + SUB_7018;
        auto MUL_15846 = ADD_15844 * 2.0;
        auto MUL_15869 = MUL_15846 * 0.0085;
        auto MUL_15824 = SUB_15822 * 2.0;
        auto MUL_15857 = MUL_15824 * 1e-06;
        auto ADD_15873 = MUL_15857 + MUL_15869;
        auto ADD_15876 = ADD_7029 + ADD_15873;
        auto ADD_15825 = MUL_15813 + MUL_15807;
        auto MUL_15809 = SUB_6932 * SUB_6932;
        auto ADD_15847 = MUL_15803 + MUL_15809;
        auto MUL_7019 = ADD_6806 * ADD_6985;
        auto MUL_7023 = ADD_6771 * MUL_6977;
        auto MUL_7020 = ADD_6756 * MUL_6990;
        auto ADD_7022 = MUL_7019 + MUL_7020;
        auto SUB_7024 = ADD_7022 - MUL_7023;
        auto MUL_7026 = SUB_7024 * 2.0;
        auto ADD_7030 = ADD_6862 + MUL_7026;
        auto MUL_15850 = ADD_15847 * 2.0;
        auto SUB_15853 = 1.0 - MUL_15850;
        auto MUL_15871 = SUB_15853 * 0.0085;
        auto MUL_15827 = ADD_15825 * 2.0;
        auto MUL_15859 = MUL_15827 * 1e-06;
        auto ADD_15874 = MUL_15859 + MUL_15871;
        auto ADD_15877 = ADD_7030 + ADD_15874;
        auto MUL_15891 = MUL_15843 * 0.004;
        auto ADD_15896 = ADD_7028 + MUL_15891;
        auto MUL_15893 = MUL_15846 * 0.004;
        auto ADD_15897 = ADD_7029 + MUL_15893;
        auto MUL_15895 = SUB_15853 * 0.004;
        auto ADD_15898 = ADD_7030 + MUL_15895;
        auto ADD_15828 = MUL_15812 + MUL_15805;
        auto MUL_15918 = MUL_15843 * 0.0115;
        auto MUL_15830 = ADD_15828 * 2.0;
        auto MUL_15907 = MUL_15830 * 0.017;
        auto SUB_15923 = MUL_15918 - MUL_15907;
        auto ADD_15926 = ADD_7028 + SUB_15923;
        auto ADD_15831 = MUL_15804 + MUL_15809;
        auto MUL_15920 = MUL_15846 * 0.0115;
        auto MUL_15834 = ADD_15831 * 2.0;
        auto SUB_15837 = 1.0 - MUL_15834;
        auto MUL_15911 = SUB_15837 * 0.017;
        auto SUB_15924 = MUL_15920 - MUL_15911;
        auto ADD_15927 = ADD_7029 + SUB_15924;
        auto SUB_15838 = MUL_15814 - MUL_15810;
        auto MUL_15922 = SUB_15853 * 0.0115;
        auto MUL_15840 = SUB_15838 * 2.0;
        auto MUL_15915 = MUL_15840 * 0.017;
        auto SUB_15925 = MUL_15922 - MUL_15915;
        auto ADD_15928 = ADD_7030 + SUB_15925;
        auto ADD_15947 = MUL_15907 + MUL_15918;
        auto ADD_15950 = ADD_7028 + ADD_15947;
        auto ADD_15948 = MUL_15911 + MUL_15920;
        auto ADD_15951 = ADD_7029 + ADD_15948;
        auto ADD_15949 = MUL_15915 + MUL_15922;
        auto ADD_15952 = ADD_7030 + ADD_15949;
        auto MUL_15954 = SUB_15821 * 0.017;
        auto ADD_15971 = MUL_15954 + MUL_15918;
        auto ADD_15974 = ADD_7028 + ADD_15971;
        auto MUL_15956 = MUL_15824 * 0.017;
        auto ADD_15972 = MUL_15956 + MUL_15920;
        auto ADD_15975 = ADD_7029 + ADD_15972;
        auto MUL_15958 = MUL_15827 * 0.017;
        auto ADD_15973 = MUL_15958 + MUL_15922;
        auto ADD_15976 = ADD_7030 + ADD_15973;
        auto SUB_16001 = MUL_15918 - MUL_15954;
        auto ADD_16004 = ADD_7028 + SUB_16001;
        auto SUB_16002 = MUL_15920 - MUL_15956;
        auto ADD_16005 = ADD_7029 + SUB_16002;
        auto SUB_16003 = MUL_15922 - MUL_15958;
        auto ADD_16006 = ADD_7030 + SUB_16003;
        auto MUL_16015 = MUL_15830 * 0.012;
        auto MUL_16008 = SUB_15821 * 0.012;
        auto SUB_16031 = MUL_16008 - MUL_16015;
        auto ADD_16034 = SUB_16031 + MUL_15918;
        auto ADD_16037 = ADD_7028 + ADD_16034;
        auto MUL_16019 = SUB_15837 * 0.012;
        auto MUL_16010 = MUL_15824 * 0.012;
        auto SUB_16032 = MUL_16010 - MUL_16019;
        auto ADD_16035 = SUB_16032 + MUL_15920;
        auto ADD_16038 = ADD_7029 + ADD_16035;
        auto MUL_16023 = MUL_15840 * 0.012;
        auto MUL_16012 = MUL_15827 * 0.012;
        auto SUB_16033 = MUL_16012 - MUL_16023;
        auto ADD_16036 = SUB_16033 + MUL_15922;
        auto ADD_16039 = ADD_7030 + ADD_16036;
        auto ADD_16058 = MUL_16008 + MUL_16015;
        auto ADD_16061 = ADD_16058 + MUL_15918;
        auto ADD_16064 = ADD_7028 + ADD_16061;
        auto ADD_16059 = MUL_16010 + MUL_16019;
        auto ADD_16062 = ADD_16059 + MUL_15920;
        auto ADD_16065 = ADD_7029 + ADD_16062;
        auto ADD_16060 = MUL_16012 + MUL_16023;
        auto ADD_16063 = ADD_16060 + MUL_15922;
        auto ADD_16066 = ADD_7030 + ADD_16063;
        auto SUB_16091 = MUL_16015 - MUL_16008;
        auto ADD_16094 = SUB_16091 + MUL_15918;
        auto ADD_16097 = ADD_7028 + ADD_16094;
        auto SUB_16092 = MUL_16019 - MUL_16010;
        auto ADD_16095 = SUB_16092 + MUL_15920;
        auto ADD_16098 = ADD_7029 + ADD_16095;
        auto SUB_16093 = MUL_16023 - MUL_16012;
        auto ADD_16096 = SUB_16093 + MUL_15922;
        auto ADD_16099 = ADD_7030 + ADD_16096;
        auto SUB_16136 = MUL_15918 - ADD_16058;
        auto ADD_16139 = ADD_7028 + SUB_16136;
        auto SUB_16137 = MUL_15920 - ADD_16059;
        auto ADD_16140 = ADD_7029 + SUB_16137;
        auto SUB_16138 = MUL_15922 - ADD_16060;
        auto ADD_16141 = ADD_7030 + SUB_16138;
        auto MUL_16150 = MUL_15830 * 0.0157;
        auto MUL_16143 = SUB_15821 * 0.0065;
        auto SUB_16166 = MUL_16143 - MUL_16150;
        auto ADD_16169 = SUB_16166 + MUL_15918;
        auto ADD_16172 = ADD_7028 + ADD_16169;
        auto MUL_16154 = SUB_15837 * 0.0157;
        auto MUL_16145 = MUL_15824 * 0.0065;
        auto SUB_16167 = MUL_16145 - MUL_16154;
        auto ADD_16170 = SUB_16167 + MUL_15920;
        auto ADD_16173 = ADD_7029 + ADD_16170;
        auto MUL_16158 = MUL_15840 * 0.0157;
        auto MUL_16147 = MUL_15827 * 0.0065;
        auto SUB_16168 = MUL_16147 - MUL_16158;
        auto ADD_16171 = SUB_16168 + MUL_15922;
        auto ADD_16174 = ADD_7030 + ADD_16171;
        auto ADD_16193 = MUL_16143 + MUL_16150;
        auto ADD_16196 = ADD_16193 + MUL_15918;
        auto ADD_16199 = ADD_7028 + ADD_16196;
        auto ADD_16194 = MUL_16145 + MUL_16154;
        auto ADD_16197 = ADD_16194 + MUL_15920;
        auto ADD_16200 = ADD_7029 + ADD_16197;
        auto ADD_16195 = MUL_16147 + MUL_16158;
        auto ADD_16198 = ADD_16195 + MUL_15922;
        auto ADD_16201 = ADD_7030 + ADD_16198;
        auto SUB_16226 = MUL_16150 - MUL_16143;
        auto ADD_16229 = SUB_16226 + MUL_15918;
        auto ADD_16232 = ADD_7028 + ADD_16229;
        auto SUB_16227 = MUL_16154 - MUL_16145;
        auto ADD_16230 = SUB_16227 + MUL_15920;
        auto ADD_16233 = ADD_7029 + ADD_16230;
        auto SUB_16228 = MUL_16158 - MUL_16147;
        auto ADD_16231 = SUB_16228 + MUL_15922;
        auto ADD_16234 = ADD_7030 + ADD_16231;
        auto SUB_16271 = MUL_15918 - ADD_16193;
        auto ADD_16274 = ADD_7028 + SUB_16271;
        auto SUB_16272 = MUL_15920 - ADD_16194;
        auto ADD_16275 = ADD_7029 + SUB_16272;
        auto SUB_16273 = MUL_15922 - ADD_16195;
        auto ADD_16276 = ADD_7030 + SUB_16273;
        auto MUL_16285 = MUL_15830 * 0.0065;
        auto MUL_16278 = SUB_15821 * 0.0157;
        auto SUB_16301 = MUL_16278 - MUL_16285;
        auto ADD_16304 = SUB_16301 + MUL_15918;
        auto ADD_16307 = ADD_7028 + ADD_16304;
        auto MUL_16289 = SUB_15837 * 0.0065;
        auto MUL_16280 = MUL_15824 * 0.0157;
        auto SUB_16302 = MUL_16280 - MUL_16289;
        auto ADD_16305 = SUB_16302 + MUL_15920;
        auto ADD_16308 = ADD_7029 + ADD_16305;
        auto MUL_16293 = MUL_15840 * 0.0065;
        auto MUL_16282 = MUL_15827 * 0.0157;
        auto SUB_16303 = MUL_16282 - MUL_16293;
        auto ADD_16306 = SUB_16303 + MUL_15922;
        auto ADD_16309 = ADD_7030 + ADD_16306;
        auto ADD_16328 = MUL_16278 + MUL_16285;
        auto ADD_16331 = ADD_16328 + MUL_15918;
        auto ADD_16334 = ADD_7028 + ADD_16331;
        auto ADD_16329 = MUL_16280 + MUL_16289;
        auto ADD_16332 = ADD_16329 + MUL_15920;
        auto ADD_16335 = ADD_7029 + ADD_16332;
        auto ADD_16330 = MUL_16282 + MUL_16293;
        auto ADD_16333 = ADD_16330 + MUL_15922;
        auto ADD_16336 = ADD_7030 + ADD_16333;
        auto SUB_16361 = MUL_16285 - MUL_16278;
        auto ADD_16364 = SUB_16361 + MUL_15918;
        auto ADD_16367 = ADD_7028 + ADD_16364;
        auto SUB_16362 = MUL_16289 - MUL_16280;
        auto ADD_16365 = SUB_16362 + MUL_15920;
        auto ADD_16368 = ADD_7029 + ADD_16365;
        auto SUB_16363 = MUL_16293 - MUL_16282;
        auto ADD_16366 = SUB_16363 + MUL_15922;
        auto ADD_16369 = ADD_7030 + ADD_16366;
        auto SUB_16406 = MUL_15918 - ADD_16328;
        auto ADD_16409 = ADD_7028 + SUB_16406;
        auto SUB_16407 = MUL_15920 - ADD_16329;
        auto ADD_16410 = ADD_7029 + SUB_16407;
        auto SUB_16408 = MUL_15922 - ADD_16330;
        auto ADD_16411 = ADD_7030 + SUB_16408;
        if(/*base_link vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_9258, SUB_9259, 0.092, 0.23, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9277, ADD_9278, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9299, ADD_9300, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9324, ADD_9325, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9346, ADD_9347, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9371, ADD_9372, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9396, ADD_9397, 0.0944, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9418, ADD_9419, 0.0944, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9449, SUB_9450, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9477, ADD_9478, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9508, SUB_9509, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9539, SUB_9540, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9567, ADD_9568, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9595, ADD_9596, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9619, SUB_9620, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9650, SUB_9651, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9678, ADD_9679, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9709, SUB_9710, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9737, ADD_9738, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9768, SUB_9769, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9796, ADD_9797, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9827, SUB_9828, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9855, ADD_9856, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9886, SUB_9887, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_9917, SUB_9918, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9945, ADD_9946, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_9973, ADD_9974, 0.092, 0.079, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10004, SUB_10005, 0.095, 0.086, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10032, ADD_10033, 0.095, 0.086, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_10063, SUB_10064, 0.095, 0.086, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10091, ADD_10092, 0.095, 0.086, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_10115, SUB_10116, 0.095, 0.086, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2317, 2563)
        if(/*link_arm_l0 vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_11900, ADD_11901, ADD_11902, 0.09, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_11921, ADD_11922, ADD_11923, 0.0345, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11948, SUB_11949, SUB_11950, 0.034, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11975, SUB_11976, SUB_11977, 0.034, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12005, ADD_12006, ADD_12007, 0.042, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12026, ADD_12027, ADD_12028, 0.034, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        if(/*link_arm_l1 vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11733, SUB_11734, SUB_11735, 0.087, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11760, SUB_11761, SUB_11762, 0.034, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11787, SUB_11788, SUB_11789, 0.034, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11814, SUB_11815, SUB_11816, 0.034, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        if(/*link_arm_l2 vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11569, SUB_11570, SUB_11571, 0.087, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11596, SUB_11597, SUB_11598, 0.04, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11623, SUB_11624, SUB_11625, 0.036, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11650, SUB_11651, SUB_11652, 0.036, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        if(/*link_arm_l3 vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11405, SUB_11406, SUB_11407, 0.087, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11432, SUB_11433, SUB_11434, 0.042, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11459, SUB_11460, SUB_11461, 0.04, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11486, SUB_11487, SUB_11488, 0.04, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        if(/*link_arm_l4 vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.12, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11239, SUB_11240, SUB_11241, 0.042, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11212, SUB_11213, SUB_11214, 0.04, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11293, SUB_11294, SUB_11295, 0.04, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_11320, SUB_11321, SUB_11322, 0.04, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        if(/*link_gripper_fingertip_left*/ sphere_environment_in_collision(environment, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_environment_in_collision(environment, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_environment_in_collision(environment, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        if(/*link_head_tilt vs. link_gripper_fingertip_left*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_12439, ADD_12440, ADD_12441, 0.07, ADD_15875, ADD_15876, ADD_15877, 0.025)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12466, ADD_12467, ADD_12468, 0.057304, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15896, ADD_15897, ADD_15898, 0.014)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15926, ADD_15927, ADD_15928, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15950, ADD_15951, ADD_15952, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_15974, ADD_15975, ADD_15976, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16004, ADD_16005, ADD_16006, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16037, ADD_16038, ADD_16039, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16064, ADD_16065, ADD_16066, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16097, ADD_16098, ADD_16099, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16139, ADD_16140, ADD_16141, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16172, ADD_16173, ADD_16174, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16199, ADD_16200, ADD_16201, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16232, ADD_16233, ADD_16234, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16274, ADD_16275, ADD_16276, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16307, ADD_16308, ADD_16309, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16334, ADD_16335, ADD_16336, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16367, ADD_16368, ADD_16369, 0.006)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_12499, ADD_12500, ADD_12501, 0.056933, ADD_16409, ADD_16410, ADD_16411, 0.006)){ return false; } } // (2563, 2563)
        auto MUL_7622 = ADD_2215 * 0.7071068;
        auto MUL_7638 = SUB_2209 * 0.7071068;
        auto MUL_7627 = SUB_2202 * 0.7071068;
        auto SUB_7628 = MUL_7627 - MUL_7622;
        auto ADD_7647 = MUL_7622 + MUL_7627;
        auto MUL_16603 = ADD_7647 * SUB_7628;
        auto MUL_16599 = SUB_7628 * SUB_7628;
        auto MUL_7634 = ADD_2196 * 0.7071068;
        auto SUB_7620 = MUL_7634 - MUL_7638;
        auto ADD_7639 = MUL_7634 + MUL_7638;
        auto MUL_16601 = ADD_7647 * ADD_7639;
        auto MUL_16600 = ADD_7639 * ADD_7639;
        auto ADD_16611 = MUL_16599 + MUL_16600;
        auto MUL_16609 = SUB_7620 * ADD_7639;
        auto SUB_16637 = MUL_16609 - MUL_16603;
        auto MUL_16608 = SUB_7620 * SUB_7628;
        auto ADD_16624 = MUL_16608 + MUL_16601;
        auto MUL_16639 = SUB_16637 * 2.0;
        auto MUL_16614 = ADD_16611 * 2.0;
        auto SUB_16617 = 1.0 - MUL_16614;
        auto MUL_16626 = ADD_16624 * 2.0;
        auto MUL_7657 = SUB_2209 * 0.041173;
        auto MUL_7662 = ADD_2196 * 0.041173;
        auto MUL_7673 = SUB_2209 * 0.0402;
        auto MUL_7665 = SUB_2202 * 0.0402;
        auto ADD_7666 = MUL_7662 + MUL_7665;
        auto MUL_7678 = SUB_2202 * ADD_7666;
        auto MUL_7653 = SUB_2202 * 0.0245786;
        auto SUB_7659 = MUL_7657 - MUL_7653;
        auto MUL_7676 = ADD_2215 * SUB_7659;
        auto ADD_7680 = MUL_7676 + MUL_7678;
        auto MUL_7670 = ADD_2196 * 0.0245786;
        auto ADD_7674 = MUL_7670 + MUL_7673;
        auto MUL_7682 = SUB_2209 * ADD_7674;
        auto ADD_7684 = ADD_7680 + MUL_7682;
        auto MUL_7687 = ADD_7684 * 2.0;
        auto SUB_7690 = 0.0402 - MUL_7687;
        auto ADD_7714 = ADD_2181 + SUB_7690;
        auto MUL_16697 = MUL_16639 * 0.0171596;
        auto MUL_16690 = MUL_16626 * 0.0021271;
        auto MUL_16684 = SUB_16617 * 0.0003056;
        auto ADD_16707 = MUL_16684 + MUL_16690;
        auto SUB_16710 = ADD_16707 - MUL_16697;
        auto ADD_16713 = ADD_7714 + SUB_16710;
        auto SUB_16618 = MUL_16608 - MUL_16601;
        auto MUL_16606 = ADD_7647 * SUB_7620;
        auto MUL_16610 = SUB_7628 * ADD_7639;
        auto ADD_16640 = MUL_16610 + MUL_16606;
        auto MUL_16605 = SUB_7620 * SUB_7620;
        auto ADD_16627 = MUL_16600 + MUL_16605;
        auto MUL_7691 = ADD_2215 * ADD_7674;
        auto MUL_7696 = SUB_2209 * SUB_7659;
        auto MUL_7693 = ADD_2196 * ADD_7666;
        auto SUB_7695 = MUL_7693 - MUL_7691;
        auto ADD_7697 = SUB_7695 + MUL_7696;
        auto MUL_7699 = ADD_7697 * 2.0;
        auto SUB_7702 = MUL_7699 - 0.041173;
        auto ADD_7715 = ADD_2182 + SUB_7702;
        auto MUL_16642 = ADD_16640 * 2.0;
        auto MUL_16701 = MUL_16642 * 0.0171596;
        auto MUL_16620 = SUB_16618 * 2.0;
        auto MUL_16686 = MUL_16620 * 0.0003056;
        auto MUL_16630 = ADD_16627 * 2.0;
        auto SUB_16633 = 1.0 - MUL_16630;
        auto MUL_16692 = SUB_16633 * 0.0021271;
        auto ADD_16708 = MUL_16686 + MUL_16692;
        auto SUB_16711 = ADD_16708 - MUL_16701;
        auto ADD_16714 = ADD_7715 + SUB_16711;
        auto SUB_16634 = MUL_16610 - MUL_16606;
        auto ADD_16621 = MUL_16609 + MUL_16603;
        auto ADD_16643 = MUL_16599 + MUL_16605;
        auto MUL_7703 = ADD_2215 * ADD_7666;
        auto MUL_7707 = SUB_2202 * SUB_7659;
        auto MUL_7704 = ADD_2196 * ADD_7674;
        auto ADD_7706 = MUL_7703 + MUL_7704;
        auto SUB_7708 = ADD_7706 - MUL_7707;
        auto MUL_7710 = SUB_7708 * 2.0;
        auto SUB_7713 = MUL_7710 - 0.0245786;
        auto ADD_7716 = ADD_2183 + SUB_7713;
        auto MUL_16636 = SUB_16634 * 2.0;
        auto MUL_16694 = MUL_16636 * 0.0021271;
        auto MUL_16646 = ADD_16643 * 2.0;
        auto SUB_16649 = 1.0 - MUL_16646;
        auto MUL_16705 = SUB_16649 * 0.0171596;
        auto MUL_16623 = ADD_16621 * 2.0;
        auto MUL_16688 = MUL_16623 * 0.0003056;
        auto ADD_16709 = MUL_16688 + MUL_16694;
        auto SUB_16712 = ADD_16709 - MUL_16705;
        auto ADD_16715 = ADD_7716 + SUB_16712;
        if(/*link_head_nav_cam*/ sphere_environment_in_collision(environment, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; } // (2563, 2652)
        auto SUB_16677 = ADD_16707 - MUL_16697;
        auto ADD_16680 = ADD_7714 + SUB_16677;
        auto SUB_16678 = ADD_16708 - MUL_16701;
        auto ADD_16681 = ADD_7715 + SUB_16678;
        auto SUB_16679 = ADD_16709 - MUL_16705;
        auto ADD_16682 = ADD_7716 + SUB_16679;
        if(/*link_gripper_finger_left vs. link_head_nav_cam*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_15478, ADD_15479, ADD_15480, 0.062, ADD_16680, ADD_16681, ADD_16682, 0.050857)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15508, ADD_15509, ADD_15510, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15538, ADD_15539, ADD_15540, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15568, ADD_15569, ADD_15570, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15598, ADD_15599, ADD_15600, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15628, ADD_15629, ADD_15630, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15658, ADD_15659, ADD_15660, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15688, ADD_15689, ADD_15690, 0.016, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15718, ADD_15719, ADD_15720, 0.014, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15748, ADD_15749, ADD_15750, 0.013, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15778, ADD_15779, ADD_15780, 0.012, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; } } // (2652, 2658)
        if(/*link_gripper_finger_right vs. link_head_nav_cam*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_14304, SUB_14305, SUB_14306, 0.062, ADD_16680, ADD_16681, ADD_16682, 0.050857)){ if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14343, SUB_14344, SUB_14345, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14382, SUB_14383, SUB_14384, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14421, SUB_14422, SUB_14423, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14460, SUB_14461, SUB_14462, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14499, SUB_14500, SUB_14501, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14538, SUB_14539, SUB_14540, 0.02, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14577, SUB_14578, SUB_14579, 0.016, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14616, SUB_14617, SUB_14618, 0.014, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14655, SUB_14656, SUB_14657, 0.013, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(SUB_14694, SUB_14695, SUB_14696, 0.012, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; } } // (2658, 2658)
        if(/*link_gripper_fingertip_left vs. link_head_nav_cam*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_15875, ADD_15876, ADD_15877, 0.025, ADD_16680, ADD_16681, ADD_16682, 0.050857)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15896, ADD_15897, ADD_15898, 0.014, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15926, ADD_15927, ADD_15928, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15950, ADD_15951, ADD_15952, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15974, ADD_15975, ADD_15976, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16004, ADD_16005, ADD_16006, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16037, ADD_16038, ADD_16039, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16064, ADD_16065, ADD_16066, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16097, ADD_16098, ADD_16099, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16139, ADD_16140, ADD_16141, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16172, ADD_16173, ADD_16174, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16199, ADD_16200, ADD_16201, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16232, ADD_16233, ADD_16234, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16274, ADD_16275, ADD_16276, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16307, ADD_16308, ADD_16309, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16334, ADD_16335, ADD_16336, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16367, ADD_16368, ADD_16369, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_16409, ADD_16410, ADD_16411, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; } } // (2658, 2658)
        if(/*link_gripper_fingertip_right vs. link_head_nav_cam*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_14788, ADD_14789, ADD_14790, 0.025, ADD_16680, ADD_16681, ADD_16682, 0.050857)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14809, ADD_14810, ADD_14811, 0.014, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14839, ADD_14840, ADD_14841, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14863, ADD_14864, ADD_14865, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14887, ADD_14888, ADD_14889, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14917, ADD_14918, ADD_14919, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14950, ADD_14951, ADD_14952, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14977, ADD_14978, ADD_14979, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15010, ADD_15011, ADD_15012, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15052, ADD_15053, ADD_15054, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15085, ADD_15086, ADD_15087, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15112, ADD_15113, ADD_15114, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15145, ADD_15146, ADD_15147, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15187, ADD_15188, ADD_15189, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15220, ADD_15221, ADD_15222, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15247, ADD_15248, ADD_15249, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15280, ADD_15281, ADD_15282, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_15322, ADD_15323, ADD_15324, 0.006, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; } } // (2658, 2658)
        if(/*link_gripper_s3_body vs. link_head_nav_cam*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_14150, ADD_14151, ADD_14152, 0.08, ADD_16680, ADD_16681, ADD_16682, 0.050857)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14179, ADD_14180, ADD_14181, 0.04, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14200, ADD_14201, ADD_14202, 0.056, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; }
        if(sphere_sphere_self_collision<decltype(q[0])>(ADD_14221, ADD_14222, ADD_14223, 0.05, ADD_16713, ADD_16714, ADD_16715, 0.050857)){ return false; } } // (2658, 2658)
        return true;
    }
}