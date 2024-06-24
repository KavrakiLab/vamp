#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::panda
{
    using Configuration = FloatVector<7>;
    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, 7>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, 7> s_m_a{
        5.9342f,
        3.6652f,
        5.9342f,
        3.2289f,
        5.9342f,
        3.9095999999999997f,
        5.9342f};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 7> s_a_a{
        -2.9671f,
        -1.8326f,
        -2.9671f,
        -3.1416f,
        -2.9671f,
        -0.0873f,
        -2.9671f};

    const Configuration s_m(s_m_a);
    const Configuration s_a(s_a_a);

    inline void scale_configuration(Configuration &q) noexcept
    {
        q = q * s_m + s_a;
    }

    alignas(Configuration::S::Alignment) constexpr std::array<float, 7> d_m_a{
        0.1685147113342995f,
        0.2728364072901888f,
        0.1685147113342995f,
        0.30970299482796f,
        0.1685147113342995f,
        0.25578064252097404f,
        0.1685147113342995f};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 7> d_s_a{
        -2.9671f,
        -1.8326f,
        -2.9671f,
        -3.1416f,
        -2.9671f,
        -0.0873f,
        -2.9671f};

    const Configuration d_m(d_m_a);
    const Configuration d_s(d_s_a);

    inline void descale_configuration(Configuration &q) noexcept
    {
        q = (q - d_s) * d_m;
    }

    template <std::size_t rake>
    inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -2.9671f + (q[0] * 5.9342f);
        q[1] = -1.8326f + (q[1] * 3.6652f);
        q[2] = -2.9671f + (q[2] * 5.9342f);
        q[3] = -3.1416f + (q[3] * 3.2289f);
        q[4] = -2.9671f + (q[4] * 5.9342f);
        q[5] = -0.0873f + (q[5] * 3.9095999999999997f);
        q[6] = -2.9671f + (q[6] * 5.9342f);
    }

    template <std::size_t rake>
    inline void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = 0.1685147113342995f * (q[0] - -2.9671f);
        q[1] = 0.2728364072901888f * (q[1] - -1.8326f);
        q[2] = 0.1685147113342995f * (q[2] - -2.9671f);
        q[3] = 0.30970299482796f * (q[3] - -3.1416f);
        q[4] = 0.1685147113342995f * (q[4] - -2.9671f);
        q[5] = 0.25578064252097404f * (q[5] - -0.0873f);
        q[6] = 0.1685147113342995f * (q[6] - -2.9671f);
    }

    inline static auto space_measure() noexcept -> float
    {
        return 878819.1112640093;
    }

    constexpr auto n_spheres = 59;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, 59> x;
        FloatVector<rake, 59> y;
        FloatVector<rake, 59> z;
        FloatVector<rake, 59> r;
    };

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
        out.r[0] = 0.08;    // (0, 0)
        out.r[1] = 0.06;    // (0, 0)
        out.r[2] = 0.06;    // (0, 0)
        out.r[3] = 0.06;    // (0, 0)
        out.r[4] = 0.06;    // (0, 0)
        out.r[5] = 0.06;    // (0, 0)
        out.r[6] = 0.06;    // (0, 0)
        out.r[7] = 0.06;    // (0, 0)
        out.r[8] = 0.06;    // (0, 0)
        out.r[9] = 0.06;    // (0, 0)
        out.r[10] = 0.05;   // (0, 0)
        out.r[11] = 0.055;  // (0, 0)
        out.r[12] = 0.055;  // (0, 0)
        out.r[13] = 0.06;   // (0, 0)
        out.r[14] = 0.055;  // (0, 0)
        out.r[15] = 0.055;  // (0, 0)
        out.r[16] = 0.055;  // (0, 0)
        out.r[17] = 0.06;   // (0, 0)
        out.r[18] = 0.06;   // (0, 0)
        out.r[19] = 0.06;   // (0, 0)
        out.r[20] = 0.05;   // (0, 0)
        out.r[21] = 0.025;  // (0, 0)
        out.r[22] = 0.025;  // (0, 0)
        out.r[23] = 0.025;  // (0, 0)
        out.r[24] = 0.025;  // (0, 0)
        out.r[25] = 0.025;  // (0, 0)
        out.r[26] = 0.025;  // (0, 0)
        out.r[27] = 0.025;  // (0, 0)
        out.r[28] = 0.025;  // (0, 0)
        out.r[29] = 0.05;   // (0, 0)
        out.r[30] = 0.05;   // (0, 0)
        out.r[31] = 0.052;  // (0, 0)
        out.r[32] = 0.05;   // (0, 0)
        out.r[33] = 0.025;  // (0, 0)
        out.r[34] = 0.025;  // (0, 0)
        out.r[35] = 0.02;   // (0, 0)
        out.r[36] = 0.02;   // (0, 0)
        out.r[37] = 0.028;  // (0, 0)
        out.r[38] = 0.028;  // (0, 0)
        out.r[39] = 0.028;  // (0, 0)
        out.r[40] = 0.028;  // (0, 0)
        out.r[41] = 0.028;  // (0, 0)
        out.r[42] = 0.028;  // (0, 0)
        out.r[43] = 0.026;  // (0, 0)
        out.r[44] = 0.026;  // (0, 0)
        out.r[45] = 0.026;  // (0, 0)
        out.r[46] = 0.026;  // (0, 0)
        out.r[47] = 0.026;  // (0, 0)
        out.r[48] = 0.026;  // (0, 0)
        out.r[49] = 0.024;  // (0, 0)
        out.r[50] = 0.024;  // (0, 0)
        out.r[51] = 0.024;  // (0, 0)
        out.r[52] = 0.024;  // (0, 0)
        out.r[53] = 0.024;  // (0, 0)
        out.r[54] = 0.024;  // (0, 0)
        out.r[55] = 0.012;  // (0, 0)
        out.r[56] = 0.012;  // (0, 0)
        out.r[57] = 0.012;  // (0, 0)
        out.r[58] = 0.012;  // (0, 0)
        out.x[0] = 0.0;     // (0, 0)
        out.x[3] = 0.0;     // (0, 0)
        out.x[4] = 0.0;     // (0, 0)
        out.y[0] = 0.0;     // (0, 0)
        out.y[3] = 0.0;     // (0, 0)
        out.y[4] = 0.0;     // (0, 0)
        out.z[0] = 0.05;    // (0, 0)
        out.z[1] = 0.333;   // (0, 0)
        out.z[2] = 0.333;   // (0, 0)
        out.z[3] = 0.213;   // (0, 0)
        out.z[4] = 0.163;   // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_8 = INPUT_0 * 0.5;
        auto SIN_9 = DIV_8.sin();
        auto COS_15 = DIV_8.cos();
        auto MUL_1570 = COS_15 * SIN_9;
        auto MUL_1589 = MUL_1570 * 2.0;
        auto MUL_1615 = MUL_1589 * 0.08;
        out.x[1] = MUL_1615;  // (0, 7)
        auto MUL_1639 = MUL_1589 * 0.03;
        out.x[2] = MUL_1639;  // (7, 8)
        auto MUL_74 = COS_15 * 0.7071068;
        auto MUL_72 = SIN_9 * 0.7071068;
        auto INPUT_1 = q[1];
        auto DIV_117 = INPUT_1 * 0.5;
        auto SIN_118 = DIV_117.sin();
        auto COS_124 = DIV_117.cos();
        auto MUL_143 = MUL_72 * COS_124;
        auto MUL_128 = MUL_72 * SIN_118;
        auto MUL_126 = MUL_74 * COS_124;
        auto SUB_149 = MUL_126 - MUL_128;
        auto ADD_130 = MUL_126 + MUL_128;
        auto MUL_140 = MUL_74 * SIN_118;
        auto SUB_138 = MUL_140 - MUL_143;
        auto ADD_144 = MUL_140 + MUL_143;
        auto MUL_1706 = SUB_149 * SUB_138;
        auto MUL_1712 = ADD_130 * ADD_144;
        auto SUB_1745 = MUL_1706 - MUL_1712;
        auto MUL_1747 = SUB_1745 * 2.0;
        auto MUL_1771 = MUL_1747 * 0.03;
        out.x[5] = MUL_1771;  // (8, 27)
        auto MUL_1790 = MUL_1747 * 0.08;
        out.x[6] = MUL_1790;  // (27, 28)
        auto MUL_1705 = SUB_149 * ADD_144;
        auto MUL_1710 = ADD_130 * SUB_138;
        auto ADD_1730 = MUL_1710 + MUL_1705;
        auto MUL_1733 = ADD_1730 * 2.0;
        auto MUL_1804 = MUL_1733 * 0.12;
        out.x[7] = MUL_1804;  // (28, 33)
        auto MUL_1828 = MUL_1733 * 0.17;
        out.x[8] = MUL_1828;  // (33, 34)
        auto MUL_182 = SUB_149 * 0.7071068;
        auto MUL_198 = ADD_144 * 0.7071068;
        auto MUL_196 = SUB_138 * 0.7071068;
        auto SUB_209 = MUL_198 - MUL_196;
        auto ADD_199 = MUL_196 + MUL_198;
        auto MUL_184 = ADD_130 * 0.7071068;
        auto SUB_186 = MUL_182 - MUL_184;
        auto ADD_215 = MUL_182 + MUL_184;
        auto MUL_224 = ADD_144 * 0.316;
        auto MUL_235 = SUB_149 * MUL_224;
        auto MUL_228 = ADD_130 * 0.316;
        auto MUL_236 = SUB_138 * MUL_228;
        auto ADD_237 = MUL_235 + MUL_236;
        auto MUL_240 = ADD_237 * 2.0;
        auto INPUT_2 = q[2];
        auto DIV_262 = INPUT_2 * 0.5;
        auto SIN_263 = DIV_262.sin();
        auto COS_269 = DIV_262.cos();
        auto MUL_286 = ADD_215 * COS_269;
        auto MUL_281 = ADD_215 * SIN_263;
        auto MUL_284 = SUB_209 * COS_269;
        auto ADD_285 = MUL_281 + MUL_284;
        auto MUL_289 = SUB_209 * SIN_263;
        auto SUB_290 = MUL_286 - MUL_289;
        auto MUL_271 = SUB_186 * COS_269;
        auto MUL_276 = SUB_186 * SIN_263;
        auto MUL_278 = ADD_199 * COS_269;
        auto SUB_279 = MUL_278 - MUL_276;
        auto MUL_1847 = SUB_290 * SUB_279;
        auto MUL_272 = ADD_199 * SIN_263;
        auto ADD_273 = MUL_271 + MUL_272;
        auto MUL_1851 = ADD_273 * ADD_285;
        auto ADD_1879 = MUL_1851 + MUL_1847;
        auto MUL_1881 = ADD_1879 * 2.0;
        auto MUL_1906 = MUL_1881 * 0.1;
        auto SUB_1916 = MUL_240 - MUL_1906;
        out.x[9] = SUB_1916;  // (34, 70)
        auto MUL_1933 = MUL_1881 * 0.06;
        auto SUB_1943 = MUL_240 - MUL_1933;
        out.x[10] = SUB_1943;  // (70, 72)
        auto MUL_1846 = SUB_290 * ADD_285;
        auto MUL_1845 = ADD_285 * ADD_285;
        auto MUL_1844 = SUB_279 * SUB_279;
        auto ADD_1853 = MUL_1844 + MUL_1845;
        auto MUL_1856 = ADD_1853 * 2.0;
        auto MUL_1850 = ADD_273 * SUB_279;
        auto SUB_1866 = MUL_1850 - MUL_1846;
        auto MUL_1868 = SUB_1866 * 2.0;
        auto MUL_1953 = MUL_1868 * 0.06;
        auto SUB_1859 = 1.0 - MUL_1856;
        auto MUL_1947 = SUB_1859 * 0.08;
        auto ADD_1964 = MUL_1947 + MUL_1953;
        auto ADD_1967 = MUL_240 + ADD_1964;
        out.x[11] = ADD_1967;  // (72, 85)
        auto MUL_1977 = MUL_1868 * 0.02;
        auto ADD_1988 = MUL_1947 + MUL_1977;
        auto ADD_1991 = MUL_240 + ADD_1988;
        out.x[12] = ADD_1991;  // (85, 88)
        auto MUL_351 = SUB_290 * 0.7071068;
        auto MUL_348 = ADD_285 * 0.7071068;
        auto MUL_345 = SUB_279 * 0.7071068;
        auto SUB_349 = MUL_348 - MUL_345;
        auto ADD_339 = MUL_345 + MUL_348;
        auto MUL_353 = ADD_273 * 0.7071068;
        auto SUB_354 = MUL_351 - MUL_353;
        auto ADD_326 = MUL_351 + MUL_353;
        auto MUL_371 = ADD_285 * 0.0825;
        auto MUL_376 = ADD_285 * MUL_371;
        auto MUL_366 = SUB_279 * 0.0825;
        auto MUL_374 = SUB_279 * MUL_366;
        auto ADD_378 = MUL_374 + MUL_376;
        auto MUL_381 = ADD_378 * 2.0;
        auto SUB_384 = 0.0825 - MUL_381;
        auto ADD_403 = MUL_240 + SUB_384;
        auto INPUT_3 = q[3];
        auto DIV_407 = INPUT_3 * 0.5;
        auto SIN_408 = DIV_407.sin();
        auto COS_414 = DIV_407.cos();
        auto MUL_431 = SUB_354 * COS_414;
        auto MUL_426 = SUB_354 * SIN_408;
        auto MUL_429 = SUB_349 * COS_414;
        auto ADD_430 = MUL_426 + MUL_429;
        auto MUL_1995 = ADD_430 * ADD_430;
        auto MUL_434 = SUB_349 * SIN_408;
        auto SUB_435 = MUL_431 - MUL_434;
        auto MUL_1996 = SUB_435 * ADD_430;
        auto MUL_416 = ADD_326 * COS_414;
        auto MUL_421 = ADD_326 * SIN_408;
        auto MUL_423 = ADD_339 * COS_414;
        auto SUB_424 = MUL_423 - MUL_421;
        auto MUL_1994 = SUB_424 * SUB_424;
        auto ADD_2003 = MUL_1994 + MUL_1995;
        auto MUL_2006 = ADD_2003 * 2.0;
        auto SUB_2009 = 1.0 - MUL_2006;
        auto MUL_2044 = SUB_2009 * 0.08;
        auto MUL_417 = ADD_339 * SIN_408;
        auto ADD_418 = MUL_416 + MUL_417;
        auto MUL_2000 = ADD_418 * SUB_424;
        auto SUB_2016 = MUL_2000 - MUL_1996;
        auto MUL_2018 = SUB_2016 * 2.0;
        auto MUL_2055 = MUL_2018 * 0.095;
        auto SUB_2066 = MUL_2055 - MUL_2044;
        auto ADD_2069 = ADD_403 + SUB_2066;
        out.x[13] = ADD_2069;  // (88, 133)
        auto MUL_1997 = SUB_435 * SUB_424;
        auto MUL_2001 = ADD_418 * ADD_430;
        auto ADD_2029 = MUL_2001 + MUL_1997;
        auto MUL_2031 = ADD_2029 * 2.0;
        auto MUL_2085 = MUL_2031 * 0.02;
        auto ADD_2090 = ADD_403 + MUL_2085;
        out.x[14] = ADD_2090;  // (133, 139)
        auto MUL_2106 = MUL_2031 * 0.06;
        auto ADD_2111 = ADD_403 + MUL_2106;
        out.x[15] = ADD_2111;  // (139, 141)
        auto MUL_2127 = MUL_2018 * 0.06;
        auto SUB_2138 = MUL_2127 - MUL_2044;
        auto ADD_2141 = ADD_403 + SUB_2138;
        out.x[16] = ADD_2141;  // (141, 144)
        auto MUL_501 = SUB_435 * 0.7071068;
        auto MUL_498 = ADD_430 * 0.7071068;
        auto MUL_527 = ADD_430 * 0.0825;
        auto MUL_533 = ADD_430 * MUL_527;
        auto MUL_495 = SUB_424 * 0.7071068;
        auto SUB_488 = MUL_495 - MUL_498;
        auto ADD_499 = MUL_495 + MUL_498;
        auto MUL_520 = SUB_424 * 0.0825;
        auto MUL_504 = ADD_418 * 0.7071068;
        auto SUB_473 = MUL_504 - MUL_501;
        auto ADD_506 = MUL_501 + MUL_504;
        auto MUL_514 = ADD_430 * 0.384;
        auto MUL_529 = SUB_435 * MUL_514;
        auto MUL_517 = ADD_418 * 0.384;
        auto ADD_522 = MUL_517 + MUL_520;
        auto MUL_531 = SUB_424 * ADD_522;
        auto SUB_532 = MUL_531 - MUL_529;
        auto ADD_534 = SUB_532 + MUL_533;
        auto MUL_536 = ADD_534 * 2.0;
        auto SUB_539 = MUL_536 - 0.0825;
        auto ADD_564 = ADD_403 + SUB_539;
        out.x[29] = ADD_564;  // (144, 165)
        auto INPUT_4 = q[4];
        auto DIV_568 = INPUT_4 * 0.5;
        auto SIN_569 = DIV_568.sin();
        auto COS_575 = DIV_568.cos();
        auto MUL_592 = ADD_506 * COS_575;
        auto MUL_587 = ADD_506 * SIN_569;
        auto MUL_590 = ADD_499 * COS_575;
        auto ADD_591 = MUL_587 + MUL_590;
        auto MUL_595 = ADD_499 * SIN_569;
        auto SUB_596 = MUL_592 - MUL_595;
        auto MUL_2146 = SUB_596 * ADD_591;
        auto MUL_577 = SUB_473 * COS_575;
        auto MUL_582 = SUB_473 * SIN_569;
        auto MUL_584 = SUB_488 * COS_575;
        auto SUB_585 = MUL_584 - MUL_582;
        auto MUL_578 = SUB_488 * SIN_569;
        auto ADD_579 = MUL_577 + MUL_578;
        auto MUL_2150 = ADD_579 * SUB_585;
        auto SUB_2166 = MUL_2150 - MUL_2146;
        auto MUL_2168 = SUB_2166 * 2.0;
        auto MUL_2199 = MUL_2168 * 0.055;
        auto ADD_2210 = ADD_564 + MUL_2199;
        out.x[17] = ADD_2210;  // (165, 187)
        auto MUL_2220 = MUL_2168 * 0.075;
        auto ADD_2231 = ADD_564 + MUL_2220;
        out.x[18] = ADD_2231;  // (187, 189)
        auto MUL_2147 = SUB_596 * SUB_585;
        auto MUL_2151 = ADD_579 * ADD_591;
        auto ADD_2179 = MUL_2151 + MUL_2147;
        auto MUL_2181 = ADD_2179 * 2.0;
        auto MUL_2248 = MUL_2181 * 0.22;
        auto SUB_2258 = ADD_564 - MUL_2248;
        out.x[19] = SUB_2258;  // (189, 195)
        auto MUL_2275 = MUL_2181 * 0.18;
        auto MUL_2268 = MUL_2168 * 0.05;
        auto SUB_2285 = MUL_2268 - MUL_2275;
        auto ADD_2288 = ADD_564 + SUB_2285;
        out.x[20] = ADD_2288;  // (195, 199)
        auto MUL_2298 = MUL_2168 * 0.08;
        auto MUL_2145 = ADD_591 * ADD_591;
        auto MUL_2144 = SUB_585 * SUB_585;
        auto ADD_2153 = MUL_2144 + MUL_2145;
        auto MUL_2156 = ADD_2153 * 2.0;
        auto SUB_2159 = 1.0 - MUL_2156;
        auto MUL_2305 = MUL_2181 * 0.14;
        auto MUL_2292 = SUB_2159 * 0.01;
        auto ADD_2315 = MUL_2292 + MUL_2298;
        auto SUB_2318 = ADD_2315 - MUL_2305;
        auto ADD_2321 = ADD_564 + SUB_2318;
        out.x[21] = ADD_2321;  // (199, 210)
        auto MUL_2338 = MUL_2181 * 0.11;
        auto MUL_2331 = MUL_2168 * 0.085;
        auto ADD_2348 = MUL_2292 + MUL_2331;
        auto SUB_2351 = ADD_2348 - MUL_2338;
        auto ADD_2354 = ADD_564 + SUB_2351;
        out.x[22] = ADD_2354;  // (210, 215)
        auto MUL_2371 = MUL_2181 * 0.08;
        auto MUL_2364 = MUL_2168 * 0.09;
        auto ADD_2381 = MUL_2292 + MUL_2364;
        auto SUB_2384 = ADD_2381 - MUL_2371;
        auto ADD_2387 = ADD_564 + SUB_2384;
        out.x[23] = ADD_2387;  // (215, 220)
        auto MUL_2404 = MUL_2181 * 0.05;
        auto MUL_2397 = MUL_2168 * 0.095;
        auto ADD_2414 = MUL_2292 + MUL_2397;
        auto SUB_2417 = ADD_2414 - MUL_2404;
        auto ADD_2420 = ADD_564 + SUB_2417;
        out.x[24] = ADD_2420;  // (220, 225)
        auto SUB_2453 = MUL_2298 - MUL_2292;
        auto SUB_2456 = SUB_2453 - MUL_2305;
        auto ADD_2459 = ADD_564 + SUB_2456;
        out.x[25] = ADD_2459;  // (225, 228)
        auto SUB_2492 = MUL_2331 - MUL_2292;
        auto SUB_2495 = SUB_2492 - MUL_2338;
        auto ADD_2498 = ADD_564 + SUB_2495;
        out.x[26] = ADD_2498;  // (228, 231)
        auto SUB_2531 = MUL_2364 - MUL_2292;
        auto SUB_2534 = SUB_2531 - MUL_2371;
        auto ADD_2537 = ADD_564 + SUB_2534;
        out.x[27] = ADD_2537;  // (231, 234)
        auto SUB_2570 = MUL_2397 - MUL_2292;
        auto SUB_2573 = SUB_2570 - MUL_2404;
        auto ADD_2576 = ADD_564 + SUB_2573;
        out.x[28] = ADD_2576;  // (234, 237)
        auto MUL_657 = SUB_596 * 0.7071068;
        auto MUL_654 = ADD_591 * 0.7071068;
        auto MUL_651 = SUB_585 * 0.7071068;
        auto SUB_655 = MUL_654 - MUL_651;
        auto ADD_645 = MUL_651 + MUL_654;
        auto MUL_659 = ADD_579 * 0.7071068;
        auto SUB_660 = MUL_657 - MUL_659;
        auto ADD_632 = MUL_657 + MUL_659;
        auto INPUT_5 = q[5];
        auto DIV_697 = INPUT_5 * 0.5;
        auto SIN_698 = DIV_697.sin();
        auto COS_704 = DIV_697.cos();
        auto MUL_716 = SUB_660 * SIN_698;
        auto MUL_721 = SUB_660 * COS_704;
        auto MUL_724 = SUB_655 * SIN_698;
        auto SUB_725 = MUL_721 - MUL_724;
        auto MUL_719 = SUB_655 * COS_704;
        auto ADD_720 = MUL_716 + MUL_719;
        auto MUL_2581 = SUB_725 * ADD_720;
        auto MUL_2580 = ADD_720 * ADD_720;
        auto MUL_711 = ADD_632 * SIN_698;
        auto MUL_706 = ADD_632 * COS_704;
        auto MUL_707 = ADD_645 * SIN_698;
        auto ADD_708 = MUL_706 + MUL_707;
        auto MUL_713 = ADD_645 * COS_704;
        auto SUB_714 = MUL_713 - MUL_711;
        auto MUL_2579 = SUB_714 * SUB_714;
        auto ADD_2588 = MUL_2579 + MUL_2580;
        auto MUL_2591 = ADD_2588 * 2.0;
        auto SUB_2594 = 1.0 - MUL_2591;
        auto MUL_2646 = SUB_2594 * 0.08;
        auto MUL_2585 = ADD_708 * SUB_714;
        auto SUB_2601 = MUL_2585 - MUL_2581;
        auto MUL_2603 = SUB_2601 * 2.0;
        auto MUL_2653 = MUL_2603 * 0.01;
        auto SUB_2669 = MUL_2646 - MUL_2653;
        auto ADD_2672 = ADD_564 + SUB_2669;
        out.x[30] = ADD_2672;  // (237, 274)
        auto MUL_2682 = MUL_2603 * 0.035;
        auto ADD_2693 = MUL_2646 + MUL_2682;
        auto ADD_2696 = ADD_564 + ADD_2693;
        out.x[31] = ADD_2696;  // (274, 277)
        auto MUL_758 = SUB_725 * 0.7071068;
        auto MUL_773 = ADD_720 * 0.7071068;
        auto MUL_771 = SUB_714 * 0.7071068;
        auto SUB_784 = MUL_773 - MUL_771;
        auto ADD_774 = MUL_771 + MUL_773;
        auto MUL_760 = ADD_708 * 0.7071068;
        auto SUB_789 = MUL_758 - MUL_760;
        auto ADD_761 = MUL_758 + MUL_760;
        auto MUL_806 = ADD_720 * 0.088;
        auto MUL_811 = ADD_720 * MUL_806;
        auto MUL_801 = SUB_714 * 0.088;
        auto MUL_809 = SUB_714 * MUL_801;
        auto ADD_813 = MUL_809 + MUL_811;
        auto MUL_816 = ADD_813 * 2.0;
        auto SUB_819 = 0.088 - MUL_816;
        auto ADD_838 = ADD_564 + SUB_819;
        auto INPUT_6 = q[6];
        auto DIV_842 = INPUT_6 * 0.5;
        auto SIN_843 = DIV_842.sin();
        auto COS_849 = DIV_842.cos();
        auto MUL_866 = SUB_789 * COS_849;
        auto MUL_861 = SUB_789 * SIN_843;
        auto MUL_864 = SUB_784 * COS_849;
        auto ADD_865 = MUL_861 + MUL_864;
        auto MUL_869 = SUB_784 * SIN_843;
        auto SUB_870 = MUL_866 - MUL_869;
        auto MUL_851 = ADD_761 * COS_849;
        auto MUL_856 = ADD_761 * SIN_843;
        auto MUL_858 = ADD_774 * COS_849;
        auto SUB_859 = MUL_858 - MUL_856;
        auto MUL_2702 = SUB_870 * SUB_859;
        auto MUL_852 = ADD_774 * SIN_843;
        auto ADD_853 = MUL_851 + MUL_852;
        auto MUL_2706 = ADD_853 * ADD_865;
        auto ADD_2734 = MUL_2706 + MUL_2702;
        auto MUL_2736 = ADD_2734 * 2.0;
        auto MUL_2760 = MUL_2736 * 0.07;
        auto ADD_2765 = ADD_838 + MUL_2760;
        out.x[32] = ADD_2765;  // (277, 315)
        auto MUL_2781 = MUL_2736 * 0.08;
        auto MUL_2701 = SUB_870 * ADD_865;
        auto MUL_2700 = ADD_865 * ADD_865;
        auto MUL_2699 = SUB_859 * SUB_859;
        auto ADD_2708 = MUL_2699 + MUL_2700;
        auto MUL_2711 = ADD_2708 * 2.0;
        auto SUB_2714 = 1.0 - MUL_2711;
        auto MUL_2769 = SUB_2714 * 0.02;
        auto MUL_2705 = ADD_853 * SUB_859;
        auto SUB_2721 = MUL_2705 - MUL_2701;
        auto MUL_2723 = SUB_2721 * 2.0;
        auto MUL_2775 = MUL_2723 * 0.04;
        auto ADD_2786 = MUL_2769 + MUL_2775;
        auto ADD_2789 = ADD_2786 + MUL_2781;
        auto ADD_2792 = ADD_838 + ADD_2789;
        out.x[33] = ADD_2792;  // (315, 330)
        auto MUL_2802 = MUL_2723 * 0.02;
        auto MUL_2796 = SUB_2714 * 0.04;
        auto ADD_2813 = MUL_2796 + MUL_2802;
        auto ADD_2816 = ADD_2813 + MUL_2781;
        auto ADD_2819 = ADD_838 + ADD_2816;
        out.x[34] = ADD_2819;  // (330, 335)
        auto MUL_2835 = MUL_2736 * 0.085;
        auto MUL_2829 = MUL_2723 * 0.06;
        auto ADD_2840 = MUL_2796 + MUL_2829;
        auto ADD_2843 = ADD_2840 + MUL_2835;
        auto ADD_2846 = ADD_838 + ADD_2843;
        out.x[35] = ADD_2846;  // (335, 340)
        auto MUL_2850 = SUB_2714 * 0.06;
        auto ADD_2867 = MUL_2850 + MUL_2775;
        auto ADD_2870 = ADD_2867 + MUL_2835;
        auto ADD_2873 = ADD_838 + ADD_2870;
        out.x[36] = ADD_2873;  // (340, 344)
        auto MUL_1065 = SUB_870 * 0.9238795;
        auto MUL_1062 = ADD_865 * 0.9238795;
        auto MUL_1049 = SUB_859 * 0.9238795;
        auto MUL_1034 = ADD_853 * 0.9238795;
        auto MUL_1055 = SUB_870 * 0.3826834;
        auto SUB_1063 = MUL_1062 - MUL_1055;
        auto MUL_1072 = ADD_865 * 0.3826834;
        auto ADD_1074 = MUL_1065 + MUL_1072;
        auto MUL_2926 = ADD_1074 * SUB_1063;
        auto MUL_1037 = SUB_859 * 0.3826834;
        auto SUB_1039 = MUL_1034 - MUL_1037;
        auto MUL_2931 = SUB_1039 * SUB_1063;
        auto MUL_1046 = ADD_853 * 0.3826834;
        auto ADD_1050 = MUL_1046 + MUL_1049;
        auto MUL_2927 = ADD_1074 * ADD_1050;
        auto ADD_2959 = MUL_2931 + MUL_2927;
        auto MUL_2961 = ADD_2959 * 2.0;
        auto MUL_2991 = MUL_2961 * 0.01;
        auto MUL_2930 = SUB_1039 * ADD_1050;
        auto SUB_2946 = MUL_2930 - MUL_2926;
        auto MUL_2948 = SUB_2946 * 2.0;
        auto MUL_2980 = MUL_2948 * 0.075;
        auto SUB_2996 = MUL_2991 - MUL_2980;
        auto MUL_931 = SUB_859 * 0.107;
        auto MUL_942 = SUB_870 * MUL_931;
        auto MUL_939 = ADD_853 * 0.107;
        auto MUL_944 = ADD_865 * MUL_939;
        auto ADD_945 = MUL_942 + MUL_944;
        auto MUL_947 = ADD_945 * 2.0;
        auto ADD_969 = ADD_838 + MUL_947;
        auto ADD_2999 = ADD_969 + SUB_2996;
        out.x[37] = ADD_2999;  // (344, 375)
        auto MUL_3010 = MUL_2948 * 0.045;
        auto SUB_3026 = MUL_2991 - MUL_3010;
        auto ADD_3029 = ADD_969 + SUB_3026;
        out.x[38] = ADD_3029;  // (375, 378)
        auto MUL_3040 = MUL_2948 * 0.015;
        auto SUB_3056 = MUL_2991 - MUL_3040;
        auto ADD_3059 = ADD_969 + SUB_3056;
        out.x[39] = ADD_3059;  // (378, 381)
        auto ADD_3080 = MUL_3040 + MUL_2991;
        auto ADD_3083 = ADD_969 + ADD_3080;
        out.x[40] = ADD_3083;  // (381, 383)
        auto ADD_3104 = MUL_3010 + MUL_2991;
        auto ADD_3107 = ADD_969 + ADD_3104;
        out.x[41] = ADD_3107;  // (383, 385)
        auto ADD_3128 = MUL_2980 + MUL_2991;
        auto ADD_3131 = ADD_969 + ADD_3128;
        out.x[42] = ADD_3131;  // (385, 387)
        auto MUL_3153 = MUL_2961 * 0.03;
        auto SUB_3158 = MUL_3153 - MUL_2980;
        auto ADD_3161 = ADD_969 + SUB_3158;
        out.x[43] = ADD_3161;  // (387, 390)
        auto SUB_3188 = MUL_3153 - MUL_3010;
        auto ADD_3191 = ADD_969 + SUB_3188;
        out.x[44] = ADD_3191;  // (390, 392)
        auto SUB_3218 = MUL_3153 - MUL_3040;
        auto ADD_3221 = ADD_969 + SUB_3218;
        out.x[45] = ADD_3221;  // (392, 394)
        auto ADD_3242 = MUL_3040 + MUL_3153;
        auto ADD_3245 = ADD_969 + ADD_3242;
        out.x[46] = ADD_3245;  // (394, 396)
        auto ADD_3266 = MUL_3010 + MUL_3153;
        auto ADD_3269 = ADD_969 + ADD_3266;
        out.x[47] = ADD_3269;  // (396, 398)
        auto ADD_3290 = MUL_2980 + MUL_3153;
        auto ADD_3293 = ADD_969 + ADD_3290;
        out.x[48] = ADD_3293;  // (398, 400)
        auto MUL_3315 = MUL_2961 * 0.05;
        auto SUB_3320 = MUL_3315 - MUL_2980;
        auto ADD_3323 = ADD_969 + SUB_3320;
        out.x[49] = ADD_3323;  // (400, 403)
        auto SUB_3350 = MUL_3315 - MUL_3010;
        auto ADD_3353 = ADD_969 + SUB_3350;
        out.x[50] = ADD_3353;  // (403, 405)
        auto SUB_3380 = MUL_3315 - MUL_3040;
        auto ADD_3383 = ADD_969 + SUB_3380;
        out.x[51] = ADD_3383;  // (405, 407)
        auto ADD_3404 = MUL_3040 + MUL_3315;
        auto ADD_3407 = ADD_969 + ADD_3404;
        out.x[52] = ADD_3407;  // (407, 409)
        auto ADD_3428 = MUL_3010 + MUL_3315;
        auto ADD_3431 = ADD_969 + ADD_3428;
        out.x[53] = ADD_3431;  // (409, 411)
        auto ADD_3452 = MUL_2980 + MUL_3315;
        auto ADD_3455 = ADD_969 + ADD_3452;
        out.x[54] = ADD_3455;  // (411, 413)
        auto MUL_3495 = ADD_2959 * 2.0;
        auto MUL_3482 = SUB_2946 * 2.0;
        auto MUL_3513 = MUL_3482 * 0.015;
        auto MUL_1196 = SUB_1063 * 0.065;
        auto MUL_1199 = SUB_1039 * 0.065;
        auto MUL_1207 = ADD_1050 * MUL_1199;
        auto MUL_1203 = SUB_1039 * 0.0584;
        auto MUL_1209 = SUB_1063 * MUL_1203;
        auto MUL_1194 = ADD_1050 * 0.0584;
        auto SUB_1197 = MUL_1194 - MUL_1196;
        auto MUL_1206 = ADD_1074 * SUB_1197;
        auto ADD_1208 = MUL_1206 + MUL_1207;
        auto ADD_1210 = ADD_1208 + MUL_1209;
        auto MUL_1212 = ADD_1210 * 2.0;
        auto ADD_1235 = ADD_969 + MUL_1212;
        auto MUL_3519 = MUL_3495 * 0.022;
        auto ADD_3524 = MUL_3513 + MUL_3519;
        auto ADD_3527 = ADD_1235 + ADD_3524;
        out.x[55] = ADD_3527;  // (413, 431)
        auto MUL_3543 = MUL_3495 * 0.044;
        auto MUL_3537 = MUL_3482 * 0.008;
        auto ADD_3548 = MUL_3537 + MUL_3543;
        auto ADD_3551 = ADD_1235 + ADD_3548;
        out.x[56] = ADD_3551;  // (431, 435)
        auto ADD_1331 = MUL_1194 + MUL_1196;
        auto MUL_3591 = ADD_2959 * 2.0;
        auto MUL_3621 = MUL_3591 * 0.022;
        auto MUL_3578 = SUB_2946 * 2.0;
        auto MUL_3610 = MUL_3578 * 0.015;
        auto SUB_3626 = MUL_3621 - MUL_3610;
        auto MUL_1342 = ADD_1074 * ADD_1331;
        auto SUB_1345 = MUL_1342 - MUL_1207;
        auto ADD_1347 = SUB_1345 + MUL_1209;
        auto MUL_1349 = ADD_1347 * 2.0;
        auto ADD_1377 = ADD_969 + MUL_1349;
        auto ADD_3629 = ADD_1377 + SUB_3626;
        out.x[57] = ADD_3629;  // (435, 447)
        auto MUL_3651 = MUL_3591 * 0.044;
        auto MUL_3640 = MUL_3578 * 0.008;
        auto SUB_3656 = MUL_3651 - MUL_3640;
        auto ADD_3659 = ADD_1377 + SUB_3656;
        out.x[58] = ADD_3659;  // (447, 451)
        auto MUL_1569 = SIN_9 * SIN_9;
        auto MUL_1593 = MUL_1569 * 2.0;
        auto SUB_1596 = 1.0 - MUL_1593;
        auto MUL_1618 = SUB_1596 * 0.08;
        auto NEGATE_1619 = -MUL_1618;
        out.y[1] = NEGATE_1619;  // (451, 456)
        auto MUL_1642 = SUB_1596 * 0.03;
        auto NEGATE_1643 = -MUL_1642;
        out.y[2] = NEGATE_1643;  // (456, 458)
        auto MUL_1708 = SUB_149 * ADD_130;
        auto MUL_1714 = SUB_138 * ADD_144;
        auto ADD_1748 = MUL_1714 + MUL_1708;
        auto MUL_1750 = ADD_1748 * 2.0;
        auto MUL_1773 = MUL_1750 * 0.03;
        out.y[5] = MUL_1773;  // (458, 463)
        auto MUL_1792 = MUL_1750 * 0.08;
        out.y[6] = MUL_1792;  // (463, 464)
        auto MUL_1704 = ADD_144 * ADD_144;
        auto MUL_1707 = ADD_130 * ADD_130;
        auto ADD_1735 = MUL_1704 + MUL_1707;
        auto MUL_1738 = ADD_1735 * 2.0;
        auto SUB_1741 = 1.0 - MUL_1738;
        auto MUL_1807 = SUB_1741 * 0.12;
        auto NEGATE_1808 = -MUL_1807;
        out.y[7] = NEGATE_1808;  // (464, 471)
        auto MUL_1831 = SUB_1741 * 0.17;
        auto NEGATE_1832 = -MUL_1831;
        out.y[8] = NEGATE_1832;  // (471, 473)
        auto MUL_1849 = SUB_290 * ADD_273;
        auto MUL_1852 = SUB_279 * ADD_285;
        auto SUB_1882 = MUL_1852 - MUL_1849;
        auto MUL_1884 = SUB_1882 * 2.0;
        auto MUL_1910 = MUL_1884 * 0.1;
        auto MUL_246 = ADD_144 * MUL_224;
        auto MUL_244 = ADD_130 * MUL_228;
        auto ADD_247 = MUL_244 + MUL_246;
        auto MUL_249 = ADD_247 * 2.0;
        auto SUB_252 = MUL_249 - 0.316;
        auto SUB_1917 = SUB_252 - MUL_1910;
        out.y[9] = SUB_1917;  // (473, 484)
        auto MUL_1937 = MUL_1884 * 0.06;
        auto SUB_1944 = SUB_252 - MUL_1937;
        out.y[10] = SUB_1944;  // (484, 486)
        auto ADD_1860 = MUL_1850 + MUL_1846;
        auto MUL_1862 = ADD_1860 * 2.0;
        auto MUL_1949 = MUL_1862 * 0.08;
        auto MUL_1848 = ADD_273 * ADD_273;
        auto ADD_1869 = MUL_1845 + MUL_1848;
        auto MUL_1872 = ADD_1869 * 2.0;
        auto SUB_1875 = 1.0 - MUL_1872;
        auto MUL_1955 = SUB_1875 * 0.06;
        auto ADD_1965 = MUL_1949 + MUL_1955;
        auto ADD_1968 = SUB_252 + ADD_1965;
        out.y[11] = ADD_1968;  // (486, 496)
        auto MUL_1979 = SUB_1875 * 0.02;
        auto ADD_1989 = MUL_1949 + MUL_1979;
        auto ADD_1992 = SUB_252 + ADD_1989;
        out.y[12] = ADD_1992;  // (496, 499)
        auto ADD_2010 = MUL_2000 + MUL_1996;
        auto MUL_2012 = ADD_2010 * 2.0;
        auto MUL_2048 = MUL_2012 * 0.08;
        auto MUL_1998 = ADD_418 * ADD_418;
        auto ADD_2019 = MUL_1995 + MUL_1998;
        auto MUL_2022 = ADD_2019 * 2.0;
        auto SUB_2025 = 1.0 - MUL_2022;
        auto MUL_2057 = SUB_2025 * 0.095;
        auto SUB_2067 = MUL_2057 - MUL_2048;
        auto MUL_386 = SUB_290 * MUL_371;
        auto MUL_387 = ADD_273 * MUL_366;
        auto ADD_389 = MUL_386 + MUL_387;
        auto MUL_392 = ADD_389 * 2.0;
        auto ADD_404 = SUB_252 + MUL_392;
        auto ADD_2070 = ADD_404 + SUB_2067;
        out.y[13] = ADD_2070;  // (499, 514)
        auto MUL_1999 = SUB_435 * ADD_418;
        auto MUL_2002 = SUB_424 * ADD_430;
        auto SUB_2032 = MUL_2002 - MUL_1999;
        auto MUL_2034 = SUB_2032 * 2.0;
        auto MUL_2087 = MUL_2034 * 0.02;
        auto ADD_2091 = ADD_404 + MUL_2087;
        out.y[14] = ADD_2091;  // (514, 520)
        auto MUL_2108 = MUL_2034 * 0.06;
        auto ADD_2112 = ADD_404 + MUL_2108;
        out.y[15] = ADD_2112;  // (520, 522)
        auto MUL_2129 = SUB_2025 * 0.06;
        auto SUB_2139 = MUL_2129 - MUL_2048;
        auto ADD_2142 = ADD_404 + SUB_2139;
        out.y[16] = ADD_2142;  // (522, 525)
        auto MUL_2148 = ADD_579 * ADD_579;
        auto ADD_2169 = MUL_2145 + MUL_2148;
        auto MUL_2172 = ADD_2169 * 2.0;
        auto SUB_2175 = 1.0 - MUL_2172;
        auto MUL_2201 = SUB_2175 * 0.055;
        auto MUL_541 = SUB_435 * MUL_527;
        auto MUL_546 = ADD_430 * MUL_514;
        auto MUL_543 = ADD_418 * ADD_522;
        auto ADD_544 = MUL_541 + MUL_543;
        auto ADD_548 = ADD_544 + MUL_546;
        auto MUL_551 = ADD_548 * 2.0;
        auto SUB_554 = 0.384 - MUL_551;
        auto ADD_565 = ADD_404 + SUB_554;
        out.y[29] = ADD_565;  // (525, 538)
        auto ADD_2211 = ADD_565 + MUL_2201;
        out.y[17] = ADD_2211;  // (538, 539)
        auto MUL_2222 = SUB_2175 * 0.075;
        auto ADD_2232 = ADD_565 + MUL_2222;
        out.y[18] = ADD_2232;  // (539, 541)
        auto MUL_2149 = SUB_596 * ADD_579;
        auto MUL_2152 = SUB_585 * ADD_591;
        auto SUB_2182 = MUL_2152 - MUL_2149;
        auto MUL_2184 = SUB_2182 * 2.0;
        auto MUL_2252 = MUL_2184 * 0.22;
        auto SUB_2259 = ADD_565 - MUL_2252;
        out.y[19] = SUB_2259;  // (541, 547)
        auto MUL_2279 = MUL_2184 * 0.18;
        auto MUL_2270 = SUB_2175 * 0.05;
        auto SUB_2286 = MUL_2270 - MUL_2279;
        auto ADD_2289 = ADD_565 + SUB_2286;
        out.y[20] = ADD_2289;  // (547, 551)
        auto ADD_2160 = MUL_2150 + MUL_2146;
        auto MUL_2309 = MUL_2184 * 0.14;
        auto MUL_2300 = SUB_2175 * 0.08;
        auto MUL_2162 = ADD_2160 * 2.0;
        auto MUL_2294 = MUL_2162 * 0.01;
        auto ADD_2316 = MUL_2294 + MUL_2300;
        auto SUB_2319 = ADD_2316 - MUL_2309;
        auto ADD_2322 = ADD_565 + SUB_2319;
        out.y[21] = ADD_2322;  // (551, 559)
        auto MUL_2342 = MUL_2184 * 0.11;
        auto MUL_2333 = SUB_2175 * 0.085;
        auto ADD_2349 = MUL_2294 + MUL_2333;
        auto SUB_2352 = ADD_2349 - MUL_2342;
        auto ADD_2355 = ADD_565 + SUB_2352;
        out.y[22] = ADD_2355;  // (559, 564)
        auto MUL_2375 = MUL_2184 * 0.08;
        auto MUL_2366 = SUB_2175 * 0.09;
        auto ADD_2382 = MUL_2294 + MUL_2366;
        auto SUB_2385 = ADD_2382 - MUL_2375;
        auto ADD_2388 = ADD_565 + SUB_2385;
        out.y[23] = ADD_2388;  // (564, 569)
        auto MUL_2408 = MUL_2184 * 0.05;
        auto MUL_2399 = SUB_2175 * 0.095;
        auto ADD_2415 = MUL_2294 + MUL_2399;
        auto SUB_2418 = ADD_2415 - MUL_2408;
        auto ADD_2421 = ADD_565 + SUB_2418;
        out.y[24] = ADD_2421;  // (569, 574)
        auto SUB_2454 = MUL_2300 - MUL_2294;
        auto SUB_2457 = SUB_2454 - MUL_2309;
        auto ADD_2460 = ADD_565 + SUB_2457;
        out.y[25] = ADD_2460;  // (574, 577)
        auto SUB_2493 = MUL_2333 - MUL_2294;
        auto SUB_2496 = SUB_2493 - MUL_2342;
        auto ADD_2499 = ADD_565 + SUB_2496;
        out.y[26] = ADD_2499;  // (577, 580)
        auto SUB_2532 = MUL_2366 - MUL_2294;
        auto SUB_2535 = SUB_2532 - MUL_2375;
        auto ADD_2538 = ADD_565 + SUB_2535;
        out.y[27] = ADD_2538;  // (580, 583)
        auto SUB_2571 = MUL_2399 - MUL_2294;
        auto SUB_2574 = SUB_2571 - MUL_2408;
        auto ADD_2577 = ADD_565 + SUB_2574;
        out.y[28] = ADD_2577;  // (583, 586)
        auto ADD_2595 = MUL_2585 + MUL_2581;
        auto MUL_2597 = ADD_2595 * 2.0;
        auto MUL_2648 = MUL_2597 * 0.08;
        auto MUL_2583 = ADD_708 * ADD_708;
        auto ADD_2604 = MUL_2580 + MUL_2583;
        auto MUL_2607 = ADD_2604 * 2.0;
        auto SUB_2610 = 1.0 - MUL_2607;
        auto MUL_2657 = SUB_2610 * 0.01;
        auto SUB_2670 = MUL_2648 - MUL_2657;
        auto ADD_2673 = ADD_565 + SUB_2670;
        out.y[30] = ADD_2673;  // (586, 596)
        auto MUL_2684 = SUB_2610 * 0.035;
        auto ADD_2694 = MUL_2648 + MUL_2684;
        auto ADD_2697 = ADD_565 + ADD_2694;
        out.y[31] = ADD_2697;  // (596, 599)
        auto MUL_2704 = SUB_870 * ADD_853;
        auto MUL_2707 = SUB_859 * ADD_865;
        auto SUB_2737 = MUL_2707 - MUL_2704;
        auto MUL_2739 = SUB_2737 * 2.0;
        auto MUL_2762 = MUL_2739 * 0.07;
        auto MUL_821 = SUB_725 * MUL_806;
        auto MUL_822 = ADD_708 * MUL_801;
        auto ADD_824 = MUL_821 + MUL_822;
        auto MUL_827 = ADD_824 * 2.0;
        auto ADD_839 = ADD_565 + MUL_827;
        auto ADD_2766 = ADD_839 + MUL_2762;
        out.y[32] = ADD_2766;  // (599, 610)
        auto ADD_2715 = MUL_2705 + MUL_2701;
        auto MUL_2783 = MUL_2739 * 0.08;
        auto MUL_2717 = ADD_2715 * 2.0;
        auto MUL_2771 = MUL_2717 * 0.02;
        auto MUL_2703 = ADD_853 * ADD_853;
        auto ADD_2724 = MUL_2700 + MUL_2703;
        auto MUL_2727 = ADD_2724 * 2.0;
        auto SUB_2730 = 1.0 - MUL_2727;
        auto MUL_2777 = SUB_2730 * 0.04;
        auto ADD_2787 = MUL_2771 + MUL_2777;
        auto ADD_2790 = ADD_2787 + MUL_2783;
        auto ADD_2793 = ADD_839 + ADD_2790;
        out.y[33] = ADD_2793;  // (610, 622)
        auto MUL_2804 = SUB_2730 * 0.02;
        auto MUL_2798 = MUL_2717 * 0.04;
        auto ADD_2814 = MUL_2798 + MUL_2804;
        auto ADD_2817 = ADD_2814 + MUL_2783;
        auto ADD_2820 = ADD_839 + ADD_2817;
        out.y[34] = ADD_2820;  // (622, 627)
        auto MUL_2837 = MUL_2739 * 0.085;
        auto MUL_2831 = SUB_2730 * 0.06;
        auto ADD_2841 = MUL_2798 + MUL_2831;
        auto ADD_2844 = ADD_2841 + MUL_2837;
        auto ADD_2847 = ADD_839 + ADD_2844;
        out.y[35] = ADD_2847;  // (627, 632)
        auto MUL_2852 = MUL_2717 * 0.06;
        auto ADD_2868 = MUL_2852 + MUL_2777;
        auto ADD_2871 = ADD_2868 + MUL_2837;
        auto ADD_2874 = ADD_839 + ADD_2871;
        out.y[36] = ADD_2874;  // (632, 636)
        auto MUL_2929 = ADD_1074 * SUB_1039;
        auto MUL_2925 = SUB_1063 * SUB_1063;
        auto MUL_2928 = SUB_1039 * SUB_1039;
        auto ADD_2949 = MUL_2925 + MUL_2928;
        auto MUL_2952 = ADD_2949 * 2.0;
        auto SUB_2955 = 1.0 - MUL_2952;
        auto MUL_2984 = SUB_2955 * 0.075;
        auto MUL_2932 = ADD_1050 * SUB_1063;
        auto SUB_2962 = MUL_2932 - MUL_2929;
        auto MUL_2964 = SUB_2962 * 2.0;
        auto MUL_2993 = MUL_2964 * 0.01;
        auto SUB_2997 = MUL_2993 - MUL_2984;
        auto MUL_950 = SUB_870 * MUL_939;
        auto MUL_953 = ADD_865 * MUL_931;
        auto SUB_954 = MUL_953 - MUL_950;
        auto MUL_956 = SUB_954 * 2.0;
        auto ADD_970 = ADD_839 + MUL_956;
        auto ADD_3000 = ADD_970 + SUB_2997;
        out.y[37] = ADD_3000;  // (636, 654)
        auto MUL_3014 = SUB_2955 * 0.045;
        auto SUB_3027 = MUL_2993 - MUL_3014;
        auto ADD_3030 = ADD_970 + SUB_3027;
        out.y[38] = ADD_3030;  // (654, 657)
        auto MUL_3044 = SUB_2955 * 0.015;
        auto SUB_3057 = MUL_2993 - MUL_3044;
        auto ADD_3060 = ADD_970 + SUB_3057;
        out.y[39] = ADD_3060;  // (657, 660)
        auto ADD_3081 = MUL_3044 + MUL_2993;
        auto ADD_3084 = ADD_970 + ADD_3081;
        out.y[40] = ADD_3084;  // (660, 662)
        auto ADD_3105 = MUL_3014 + MUL_2993;
        auto ADD_3108 = ADD_970 + ADD_3105;
        out.y[41] = ADD_3108;  // (662, 664)
        auto ADD_3129 = MUL_2984 + MUL_2993;
        auto ADD_3132 = ADD_970 + ADD_3129;
        out.y[42] = ADD_3132;  // (664, 666)
        auto MUL_3155 = MUL_2964 * 0.03;
        auto SUB_3159 = MUL_3155 - MUL_2984;
        auto ADD_3162 = ADD_970 + SUB_3159;
        out.y[43] = ADD_3162;  // (666, 669)
        auto SUB_3189 = MUL_3155 - MUL_3014;
        auto ADD_3192 = ADD_970 + SUB_3189;
        out.y[44] = ADD_3192;  // (669, 671)
        auto SUB_3219 = MUL_3155 - MUL_3044;
        auto ADD_3222 = ADD_970 + SUB_3219;
        out.y[45] = ADD_3222;  // (671, 673)
        auto ADD_3243 = MUL_3044 + MUL_3155;
        auto ADD_3246 = ADD_970 + ADD_3243;
        out.y[46] = ADD_3246;  // (673, 675)
        auto ADD_3267 = MUL_3014 + MUL_3155;
        auto ADD_3270 = ADD_970 + ADD_3267;
        out.y[47] = ADD_3270;  // (675, 677)
        auto ADD_3291 = MUL_2984 + MUL_3155;
        auto ADD_3294 = ADD_970 + ADD_3291;
        out.y[48] = ADD_3294;  // (677, 679)
        auto MUL_3317 = MUL_2964 * 0.05;
        auto SUB_3321 = MUL_3317 - MUL_2984;
        auto ADD_3324 = ADD_970 + SUB_3321;
        out.y[49] = ADD_3324;  // (679, 682)
        auto SUB_3351 = MUL_3317 - MUL_3014;
        auto ADD_3354 = ADD_970 + SUB_3351;
        out.y[50] = ADD_3354;  // (682, 684)
        auto SUB_3381 = MUL_3317 - MUL_3044;
        auto ADD_3384 = ADD_970 + SUB_3381;
        out.y[51] = ADD_3384;  // (684, 686)
        auto ADD_3405 = MUL_3044 + MUL_3317;
        auto ADD_3408 = ADD_970 + ADD_3405;
        out.y[52] = ADD_3408;  // (686, 688)
        auto ADD_3429 = MUL_3014 + MUL_3317;
        auto ADD_3432 = ADD_970 + ADD_3429;
        out.y[53] = ADD_3432;  // (688, 690)
        auto ADD_3453 = MUL_2984 + MUL_3317;
        auto ADD_3456 = ADD_970 + ADD_3453;
        out.y[54] = ADD_3456;  // (690, 692)
        auto MUL_3498 = SUB_2962 * 2.0;
        auto MUL_3521 = MUL_3498 * 0.022;
        auto MUL_3486 = ADD_2949 * 2.0;
        auto SUB_3489 = 1.0 - MUL_3486;
        auto MUL_3515 = SUB_3489 * 0.015;
        auto ADD_3525 = MUL_3515 + MUL_3521;
        auto MUL_1215 = ADD_1074 * MUL_1203;
        auto MUL_1220 = SUB_1063 * SUB_1197;
        auto MUL_1217 = SUB_1039 * MUL_1199;
        auto ADD_1218 = MUL_1215 + MUL_1217;
        auto SUB_1221 = MUL_1220 - ADD_1218;
        auto MUL_1223 = SUB_1221 * 2.0;
        auto ADD_1225 = MUL_1223 + 0.065;
        auto ADD_1236 = ADD_970 + ADD_1225;
        auto ADD_3528 = ADD_1236 + ADD_3525;
        out.y[55] = ADD_3528;  // (692, 707)
        auto MUL_3545 = MUL_3498 * 0.044;
        auto MUL_3539 = SUB_3489 * 0.008;
        auto ADD_3549 = MUL_3539 + MUL_3545;
        auto ADD_3552 = ADD_1236 + ADD_3549;
        out.y[56] = ADD_3552;  // (707, 711)
        auto SUB_1356 = MUL_1217 - MUL_1215;
        auto MUL_3594 = SUB_2962 * 2.0;
        auto MUL_3623 = MUL_3594 * 0.022;
        auto MUL_3582 = ADD_2949 * 2.0;
        auto SUB_3585 = 1.0 - MUL_3582;
        auto MUL_3614 = SUB_3585 * 0.015;
        auto SUB_3627 = MUL_3623 - MUL_3614;
        auto MUL_1357 = SUB_1063 * ADD_1331;
        auto ADD_1358 = SUB_1356 + MUL_1357;
        auto MUL_1360 = ADD_1358 * 2.0;
        auto SUB_1363 = MUL_1360 - 0.065;
        auto ADD_1378 = ADD_970 + SUB_1363;
        auto ADD_3630 = ADD_1378 + SUB_3627;
        out.y[57] = ADD_3630;  // (711, 724)
        auto MUL_3653 = MUL_3594 * 0.044;
        auto MUL_3644 = SUB_3585 * 0.008;
        auto SUB_3657 = MUL_3653 - MUL_3644;
        auto ADD_3660 = ADD_1378 + SUB_3657;
        out.y[58] = ADD_3660;  // (724, 728)
        auto MUL_1703 = SUB_138 * SUB_138;
        auto ADD_1751 = MUL_1703 + MUL_1707;
        auto MUL_1754 = ADD_1751 * 2.0;
        auto SUB_1757 = 1.0 - MUL_1754;
        auto MUL_1775 = SUB_1757 * 0.03;
        auto ADD_1776 = 0.333 + MUL_1775;
        out.z[5] = ADD_1776;  // (728, 734)
        auto MUL_1794 = SUB_1757 * 0.08;
        auto ADD_1795 = 0.333 + MUL_1794;
        out.z[6] = ADD_1795;  // (734, 736)
        auto SUB_1742 = MUL_1714 - MUL_1708;
        auto MUL_1744 = SUB_1742 * 2.0;
        auto MUL_1811 = MUL_1744 * 0.12;
        auto SUB_1819 = 0.333 - MUL_1811;
        out.z[7] = SUB_1819;  // (736, 740)
        auto MUL_1835 = MUL_1744 * 0.17;
        auto SUB_1843 = 0.333 - MUL_1835;
        out.z[8] = SUB_1843;  // (740, 742)
        auto ADD_1885 = MUL_1844 + MUL_1848;
        auto MUL_1888 = ADD_1885 * 2.0;
        auto SUB_1891 = 1.0 - MUL_1888;
        auto MUL_1914 = SUB_1891 * 0.1;
        auto MUL_253 = SUB_149 * MUL_228;
        auto MUL_255 = SUB_138 * MUL_224;
        auto SUB_256 = MUL_253 - MUL_255;
        auto MUL_258 = SUB_256 * 2.0;
        auto ADD_260 = 0.333 + MUL_258;
        auto SUB_1918 = ADD_260 - MUL_1914;
        out.z[9] = SUB_1918;  // (742, 752)
        auto MUL_1941 = SUB_1891 * 0.06;
        auto SUB_1945 = ADD_260 - MUL_1941;
        out.z[10] = SUB_1945;  // (752, 754)
        auto SUB_1863 = MUL_1851 - MUL_1847;
        auto ADD_1876 = MUL_1852 + MUL_1849;
        auto MUL_1878 = ADD_1876 * 2.0;
        auto MUL_1957 = MUL_1878 * 0.06;
        auto MUL_1865 = SUB_1863 * 2.0;
        auto MUL_1951 = MUL_1865 * 0.08;
        auto ADD_1966 = MUL_1951 + MUL_1957;
        auto ADD_1969 = ADD_260 + ADD_1966;
        out.z[11] = ADD_1969;  // (754, 762)
        auto MUL_1981 = MUL_1878 * 0.02;
        auto ADD_1990 = MUL_1951 + MUL_1981;
        auto ADD_1993 = ADD_260 + ADD_1990;
        out.z[12] = ADD_1993;  // (762, 765)
        auto SUB_2013 = MUL_2001 - MUL_1997;
        auto ADD_2026 = MUL_2002 + MUL_1999;
        auto MUL_2028 = ADD_2026 * 2.0;
        auto MUL_2059 = MUL_2028 * 0.095;
        auto MUL_2015 = SUB_2013 * 2.0;
        auto MUL_2052 = MUL_2015 * 0.08;
        auto SUB_2068 = MUL_2059 - MUL_2052;
        auto MUL_394 = SUB_290 * MUL_366;
        auto MUL_396 = ADD_273 * MUL_371;
        auto SUB_398 = MUL_396 - MUL_394;
        auto MUL_401 = SUB_398 * 2.0;
        auto ADD_405 = ADD_260 + MUL_401;
        auto ADD_2071 = ADD_405 + SUB_2068;
        out.z[13] = ADD_2071;  // (765, 778)
        auto ADD_2035 = MUL_1994 + MUL_1998;
        auto MUL_2038 = ADD_2035 * 2.0;
        auto SUB_2041 = 1.0 - MUL_2038;
        auto MUL_2089 = SUB_2041 * 0.02;
        auto ADD_2092 = ADD_405 + MUL_2089;
        out.z[14] = ADD_2092;  // (778, 783)
        auto MUL_2110 = SUB_2041 * 0.06;
        auto ADD_2113 = ADD_405 + MUL_2110;
        out.z[15] = ADD_2113;  // (783, 785)
        auto MUL_2131 = MUL_2028 * 0.06;
        auto SUB_2140 = MUL_2131 - MUL_2052;
        auto ADD_2143 = ADD_405 + SUB_2140;
        out.z[16] = ADD_2143;  // (785, 788)
        auto ADD_2176 = MUL_2152 + MUL_2149;
        auto MUL_2178 = ADD_2176 * 2.0;
        auto MUL_2203 = MUL_2178 * 0.055;
        auto MUL_555 = SUB_435 * ADD_522;
        auto MUL_558 = SUB_424 * MUL_514;
        auto MUL_556 = ADD_418 * MUL_527;
        auto SUB_557 = MUL_555 - MUL_556;
        auto ADD_560 = SUB_557 + MUL_558;
        auto MUL_562 = ADD_560 * 2.0;
        auto ADD_566 = ADD_405 + MUL_562;
        out.z[29] = ADD_566;  // (788, 798)
        auto ADD_2212 = ADD_566 + MUL_2203;
        out.z[17] = ADD_2212;  // (798, 799)
        auto MUL_2224 = MUL_2178 * 0.075;
        auto ADD_2233 = ADD_566 + MUL_2224;
        out.z[18] = ADD_2233;  // (799, 801)
        auto ADD_2185 = MUL_2144 + MUL_2148;
        auto MUL_2188 = ADD_2185 * 2.0;
        auto SUB_2191 = 1.0 - MUL_2188;
        auto MUL_2256 = SUB_2191 * 0.22;
        auto SUB_2260 = ADD_566 - MUL_2256;
        out.z[19] = SUB_2260;  // (801, 806)
        auto MUL_2283 = SUB_2191 * 0.18;
        auto MUL_2272 = MUL_2178 * 0.05;
        auto SUB_2287 = MUL_2272 - MUL_2283;
        auto ADD_2290 = ADD_566 + SUB_2287;
        out.z[20] = ADD_2290;  // (806, 810)
        auto SUB_2163 = MUL_2151 - MUL_2147;
        auto MUL_2313 = SUB_2191 * 0.14;
        auto MUL_2302 = MUL_2178 * 0.08;
        auto MUL_2165 = SUB_2163 * 2.0;
        auto MUL_2296 = MUL_2165 * 0.01;
        auto ADD_2317 = MUL_2296 + MUL_2302;
        auto SUB_2320 = ADD_2317 - MUL_2313;
        auto ADD_2323 = ADD_566 + SUB_2320;
        out.z[21] = ADD_2323;  // (810, 818)
        auto MUL_2346 = SUB_2191 * 0.11;
        auto MUL_2335 = MUL_2178 * 0.085;
        auto ADD_2350 = MUL_2296 + MUL_2335;
        auto SUB_2353 = ADD_2350 - MUL_2346;
        auto ADD_2356 = ADD_566 + SUB_2353;
        out.z[22] = ADD_2356;  // (818, 823)
        auto MUL_2379 = SUB_2191 * 0.08;
        auto MUL_2368 = MUL_2178 * 0.09;
        auto ADD_2383 = MUL_2296 + MUL_2368;
        auto SUB_2386 = ADD_2383 - MUL_2379;
        auto ADD_2389 = ADD_566 + SUB_2386;
        out.z[23] = ADD_2389;  // (823, 828)
        auto MUL_2412 = SUB_2191 * 0.05;
        auto MUL_2401 = MUL_2178 * 0.095;
        auto ADD_2416 = MUL_2296 + MUL_2401;
        auto SUB_2419 = ADD_2416 - MUL_2412;
        auto ADD_2422 = ADD_566 + SUB_2419;
        out.z[24] = ADD_2422;  // (828, 833)
        auto SUB_2455 = MUL_2302 - MUL_2296;
        auto SUB_2458 = SUB_2455 - MUL_2313;
        auto ADD_2461 = ADD_566 + SUB_2458;
        out.z[25] = ADD_2461;  // (833, 836)
        auto SUB_2494 = MUL_2335 - MUL_2296;
        auto SUB_2497 = SUB_2494 - MUL_2346;
        auto ADD_2500 = ADD_566 + SUB_2497;
        out.z[26] = ADD_2500;  // (836, 839)
        auto SUB_2533 = MUL_2368 - MUL_2296;
        auto SUB_2536 = SUB_2533 - MUL_2379;
        auto ADD_2539 = ADD_566 + SUB_2536;
        out.z[27] = ADD_2539;  // (839, 842)
        auto SUB_2572 = MUL_2401 - MUL_2296;
        auto SUB_2575 = SUB_2572 - MUL_2412;
        auto ADD_2578 = ADD_566 + SUB_2575;
        out.z[28] = ADD_2578;  // (842, 845)
        auto MUL_2582 = SUB_725 * SUB_714;
        auto MUL_2584 = SUB_725 * ADD_708;
        auto MUL_2587 = SUB_714 * ADD_720;
        auto ADD_2611 = MUL_2587 + MUL_2584;
        auto MUL_2613 = ADD_2611 * 2.0;
        auto MUL_2661 = MUL_2613 * 0.01;
        auto MUL_2586 = ADD_708 * ADD_720;
        auto SUB_2598 = MUL_2586 - MUL_2582;
        auto MUL_2600 = SUB_2598 * 2.0;
        auto MUL_2650 = MUL_2600 * 0.08;
        auto SUB_2671 = MUL_2650 - MUL_2661;
        auto ADD_2674 = ADD_566 + SUB_2671;
        out.z[30] = ADD_2674;  // (845, 857)
        auto MUL_2686 = MUL_2613 * 0.035;
        auto ADD_2695 = MUL_2650 + MUL_2686;
        auto ADD_2698 = ADD_566 + ADD_2695;
        out.z[31] = ADD_2698;  // (857, 860)
        auto ADD_2740 = MUL_2699 + MUL_2703;
        auto MUL_2743 = ADD_2740 * 2.0;
        auto SUB_2746 = 1.0 - MUL_2743;
        auto MUL_2764 = SUB_2746 * 0.07;
        auto MUL_829 = SUB_725 * MUL_801;
        auto MUL_831 = ADD_708 * MUL_806;
        auto SUB_833 = MUL_831 - MUL_829;
        auto MUL_836 = SUB_833 * 2.0;
        auto ADD_840 = ADD_566 + MUL_836;
        auto ADD_2767 = ADD_840 + MUL_2764;
        out.z[32] = ADD_2767;  // (860, 870)
        auto SUB_2718 = MUL_2706 - MUL_2702;
        auto ADD_2731 = MUL_2707 + MUL_2704;
        auto MUL_2785 = SUB_2746 * 0.08;
        auto MUL_2733 = ADD_2731 * 2.0;
        auto MUL_2779 = MUL_2733 * 0.04;
        auto MUL_2720 = SUB_2718 * 2.0;
        auto MUL_2773 = MUL_2720 * 0.02;
        auto ADD_2788 = MUL_2773 + MUL_2779;
        auto ADD_2791 = ADD_2788 + MUL_2785;
        auto ADD_2794 = ADD_840 + ADD_2791;
        out.z[33] = ADD_2794;  // (870, 880)
        auto MUL_2806 = MUL_2733 * 0.02;
        auto MUL_2800 = MUL_2720 * 0.04;
        auto ADD_2815 = MUL_2800 + MUL_2806;
        auto ADD_2818 = ADD_2815 + MUL_2785;
        auto ADD_2821 = ADD_840 + ADD_2818;
        out.z[34] = ADD_2821;  // (880, 885)
        auto MUL_2839 = SUB_2746 * 0.085;
        auto MUL_2833 = MUL_2733 * 0.06;
        auto ADD_2842 = MUL_2800 + MUL_2833;
        auto ADD_2845 = ADD_2842 + MUL_2839;
        auto ADD_2848 = ADD_840 + ADD_2845;
        out.z[35] = ADD_2848;  // (885, 890)
        auto MUL_2854 = MUL_2720 * 0.06;
        auto ADD_2869 = MUL_2854 + MUL_2779;
        auto ADD_2872 = ADD_2869 + MUL_2839;
        auto ADD_2875 = ADD_840 + ADD_2872;
        out.z[36] = ADD_2875;  // (890, 894)
        auto ADD_2956 = MUL_2932 + MUL_2929;
        auto MUL_2958 = ADD_2956 * 2.0;
        auto MUL_2988 = MUL_2958 * 0.075;
        auto MUL_2924 = ADD_1050 * ADD_1050;
        auto ADD_2965 = MUL_2924 + MUL_2928;
        auto MUL_2968 = ADD_2965 * 2.0;
        auto SUB_2971 = 1.0 - MUL_2968;
        auto MUL_2995 = SUB_2971 * 0.01;
        auto SUB_2998 = MUL_2995 - MUL_2988;
        auto MUL_961 = SUB_859 * MUL_931;
        auto MUL_959 = ADD_853 * MUL_939;
        auto ADD_962 = MUL_959 + MUL_961;
        auto MUL_965 = ADD_962 * 2.0;
        auto SUB_968 = 0.107 - MUL_965;
        auto ADD_971 = ADD_840 + SUB_968;
        auto ADD_3001 = ADD_971 + SUB_2998;
        out.z[37] = ADD_3001;  // (894, 910)
        auto MUL_3018 = MUL_2958 * 0.045;
        auto SUB_3028 = MUL_2995 - MUL_3018;
        auto ADD_3031 = ADD_971 + SUB_3028;
        out.z[38] = ADD_3031;  // (910, 913)
        auto MUL_3048 = MUL_2958 * 0.015;
        auto SUB_3058 = MUL_2995 - MUL_3048;
        auto ADD_3061 = ADD_971 + SUB_3058;
        out.z[39] = ADD_3061;  // (913, 916)
        auto ADD_3082 = MUL_3048 + MUL_2995;
        auto ADD_3085 = ADD_971 + ADD_3082;
        out.z[40] = ADD_3085;  // (916, 918)
        auto ADD_3106 = MUL_3018 + MUL_2995;
        auto ADD_3109 = ADD_971 + ADD_3106;
        out.z[41] = ADD_3109;  // (918, 920)
        auto ADD_3130 = MUL_2988 + MUL_2995;
        auto ADD_3133 = ADD_971 + ADD_3130;
        out.z[42] = ADD_3133;  // (920, 922)
        auto MUL_3157 = SUB_2971 * 0.03;
        auto SUB_3160 = MUL_3157 - MUL_2988;
        auto ADD_3163 = ADD_971 + SUB_3160;
        out.z[43] = ADD_3163;  // (922, 925)
        auto SUB_3190 = MUL_3157 - MUL_3018;
        auto ADD_3193 = ADD_971 + SUB_3190;
        out.z[44] = ADD_3193;  // (925, 927)
        auto SUB_3220 = MUL_3157 - MUL_3048;
        auto ADD_3223 = ADD_971 + SUB_3220;
        out.z[45] = ADD_3223;  // (927, 929)
        auto ADD_3244 = MUL_3048 + MUL_3157;
        auto ADD_3247 = ADD_971 + ADD_3244;
        out.z[46] = ADD_3247;  // (929, 931)
        auto ADD_3268 = MUL_3018 + MUL_3157;
        auto ADD_3271 = ADD_971 + ADD_3268;
        out.z[47] = ADD_3271;  // (931, 933)
        auto ADD_3292 = MUL_2988 + MUL_3157;
        auto ADD_3295 = ADD_971 + ADD_3292;
        out.z[48] = ADD_3295;  // (933, 935)
        auto MUL_3319 = SUB_2971 * 0.05;
        auto SUB_3322 = MUL_3319 - MUL_2988;
        auto ADD_3325 = ADD_971 + SUB_3322;
        out.z[49] = ADD_3325;  // (935, 938)
        auto SUB_3352 = MUL_3319 - MUL_3018;
        auto ADD_3355 = ADD_971 + SUB_3352;
        out.z[50] = ADD_3355;  // (938, 940)
        auto SUB_3382 = MUL_3319 - MUL_3048;
        auto ADD_3385 = ADD_971 + SUB_3382;
        out.z[51] = ADD_3385;  // (940, 942)
        auto ADD_3406 = MUL_3048 + MUL_3319;
        auto ADD_3409 = ADD_971 + ADD_3406;
        out.z[52] = ADD_3409;  // (942, 944)
        auto ADD_3430 = MUL_3018 + MUL_3319;
        auto ADD_3433 = ADD_971 + ADD_3430;
        out.z[53] = ADD_3433;  // (944, 946)
        auto ADD_3454 = MUL_2988 + MUL_3319;
        auto ADD_3457 = ADD_971 + ADD_3454;
        out.z[54] = ADD_3457;  // (946, 948)
        auto MUL_3502 = ADD_2965 * 2.0;
        auto SUB_3505 = 1.0 - MUL_3502;
        auto MUL_3523 = SUB_3505 * 0.022;
        auto MUL_3492 = ADD_2956 * 2.0;
        auto MUL_3517 = MUL_3492 * 0.015;
        auto ADD_3526 = MUL_3517 + MUL_3523;
        auto MUL_1226 = ADD_1074 * MUL_1199;
        auto MUL_1227 = SUB_1039 * MUL_1203;
        auto SUB_1228 = MUL_1226 - MUL_1227;
        auto MUL_1229 = ADD_1050 * SUB_1197;
        auto SUB_1230 = SUB_1228 - MUL_1229;
        auto MUL_1232 = SUB_1230 * 2.0;
        auto ADD_1234 = MUL_1232 + 0.0584;
        auto ADD_1237 = ADD_971 + ADD_1234;
        auto ADD_3529 = ADD_1237 + ADD_3526;
        out.z[55] = ADD_3529;  // (948, 963)
        auto MUL_3547 = SUB_3505 * 0.044;
        auto MUL_3541 = MUL_3492 * 0.008;
        auto ADD_3550 = MUL_3541 + MUL_3547;
        auto ADD_3553 = ADD_1237 + ADD_3550;
        out.z[56] = ADD_3553;  // (963, 967)
        auto ADD_1367 = MUL_1226 + MUL_1227;
        auto MUL_3598 = ADD_2965 * 2.0;
        auto SUB_3601 = 1.0 - MUL_3598;
        auto MUL_3625 = SUB_3601 * 0.022;
        auto MUL_3588 = ADD_2956 * 2.0;
        auto MUL_3618 = MUL_3588 * 0.015;
        auto SUB_3628 = MUL_3625 - MUL_3618;
        auto MUL_1369 = ADD_1050 * ADD_1331;
        auto ADD_1370 = ADD_1367 + MUL_1369;
        auto MUL_1373 = ADD_1370 * 2.0;
        auto SUB_1376 = 0.0584 - MUL_1373;
        auto ADD_1379 = ADD_971 + SUB_1376;
        auto ADD_3631 = ADD_1379 + SUB_3628;
        out.z[57] = ADD_3631;  // (967, 980)
        auto MUL_3655 = SUB_3601 * 0.044;
        auto MUL_3648 = MUL_3588 * 0.008;
        auto SUB_3658 = MUL_3655 - MUL_3648;
        auto ADD_3661 = ADD_1379 + SUB_3658;
        out.z[58] = ADD_3661;  // (980, 984)
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        // Ignore static frame collisions - needed for some evaluation problems
        // if (/*panda_link0*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.05, 0.08))
        // {
        //     return false;
        // }  // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_8 = INPUT_0 * 0.5;
        auto SIN_9 = DIV_8.sin();
        auto COS_15 = DIV_8.cos();
        auto MUL_1575 = COS_15 * SIN_9;
        auto MUL_1594 = MUL_1575 * 2.0;
        auto MUL_1625 = MUL_1594 * 0.039;
        auto MUL_1574 = SIN_9 * SIN_9;
        auto MUL_1584 = MUL_1574 * 2.0;
        auto SUB_1587 = 1.0 - MUL_1584;
        auto MUL_1614 = SUB_1587 * 0.001;
        auto SUB_1641 = MUL_1625 - MUL_1614;
        auto MUL_1628 = SUB_1587 * 0.039;
        auto MUL_1618 = MUL_1594 * 0.001;
        auto ADD_1642 = MUL_1618 + MUL_1628;
        auto NEGATE_1643 = -ADD_1642;
        auto MUL_1656 = MUL_1594 * 0.08;
        auto MUL_1659 = SUB_1587 * 0.08;
        auto NEGATE_1660 = -MUL_1659;
        auto MUL_1680 = MUL_1594 * 0.03;
        auto MUL_1683 = SUB_1587 * 0.03;
        auto NEGATE_1684 = -MUL_1683;
        if (/*panda_link1*/ sphere_environment_in_collision(environment, SUB_1641, NEGATE_1643, 0.248, 0.154))
        {
            if (sphere_environment_in_collision(environment, MUL_1656, NEGATE_1660, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1680, NEGATE_1684, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0, 0.0, 0.213, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0, 0.0, 0.163, 0.06))
            {
                return false;
            }
        }  // (0, 22)
        auto MUL_74 = COS_15 * 0.7071068;
        auto MUL_72 = SIN_9 * 0.7071068;
        auto INPUT_1 = q[1];
        auto DIV_117 = INPUT_1 * 0.5;
        auto SIN_118 = DIV_117.sin();
        auto COS_124 = DIV_117.cos();
        auto MUL_143 = MUL_72 * COS_124;
        auto MUL_128 = MUL_72 * SIN_118;
        auto MUL_126 = MUL_74 * COS_124;
        auto SUB_149 = MUL_126 - MUL_128;
        auto ADD_130 = MUL_126 + MUL_128;
        auto MUL_140 = MUL_74 * SIN_118;
        auto SUB_138 = MUL_140 - MUL_143;
        auto ADD_144 = MUL_140 + MUL_143;
        auto MUL_1756 = SUB_149 * ADD_144;
        auto MUL_1757 = SUB_149 * SUB_138;
        auto MUL_1763 = ADD_130 * ADD_144;
        auto SUB_1796 = MUL_1757 - MUL_1763;
        auto MUL_1798 = SUB_1796 * 2.0;
        auto MUL_1827 = MUL_1798 * 0.04;
        auto MUL_1761 = ADD_130 * SUB_138;
        auto ADD_1781 = MUL_1761 + MUL_1756;
        auto MUL_1784 = ADD_1781 * 2.0;
        auto MUL_1817 = MUL_1784 * 0.085;
        auto ADD_1832 = MUL_1817 + MUL_1827;
        auto MUL_1759 = SUB_149 * ADD_130;
        auto MUL_1755 = ADD_144 * ADD_144;
        auto MUL_1765 = SUB_138 * ADD_144;
        auto ADD_1799 = MUL_1765 + MUL_1759;
        auto MUL_1801 = ADD_1799 * 2.0;
        auto MUL_1829 = MUL_1801 * 0.04;
        auto MUL_1758 = ADD_130 * ADD_130;
        auto ADD_1786 = MUL_1755 + MUL_1758;
        auto MUL_1789 = ADD_1786 * 2.0;
        auto SUB_1792 = 1.0 - MUL_1789;
        auto MUL_1820 = SUB_1792 * 0.085;
        auto SUB_1833 = MUL_1829 - MUL_1820;
        auto SUB_1793 = MUL_1765 - MUL_1759;
        auto MUL_1795 = SUB_1793 * 2.0;
        auto MUL_1824 = MUL_1795 * 0.085;
        auto MUL_1754 = SUB_138 * SUB_138;
        auto ADD_1802 = MUL_1754 + MUL_1758;
        auto MUL_1805 = ADD_1802 * 2.0;
        auto SUB_1808 = 1.0 - MUL_1805;
        auto MUL_1831 = SUB_1808 * 0.04;
        auto SUB_1834 = MUL_1831 - MUL_1824;
        auto ADD_1835 = 0.333 + SUB_1834;
        auto MUL_1849 = MUL_1798 * 0.03;
        auto MUL_1851 = MUL_1801 * 0.03;
        auto MUL_1853 = SUB_1808 * 0.03;
        auto ADD_1854 = 0.333 + MUL_1853;
        auto MUL_1868 = MUL_1798 * 0.08;
        auto MUL_1870 = MUL_1801 * 0.08;
        auto MUL_1872 = SUB_1808 * 0.08;
        auto ADD_1873 = 0.333 + MUL_1872;
        auto MUL_1882 = MUL_1784 * 0.12;
        auto MUL_1885 = SUB_1792 * 0.12;
        auto NEGATE_1886 = -MUL_1885;
        auto MUL_1889 = MUL_1795 * 0.12;
        auto SUB_1897 = 0.333 - MUL_1889;
        auto MUL_1906 = MUL_1784 * 0.17;
        auto MUL_1909 = SUB_1792 * 0.17;
        auto NEGATE_1910 = -MUL_1909;
        auto MUL_1913 = MUL_1795 * 0.17;
        auto SUB_1921 = 0.333 - MUL_1913;
        if (/*panda_link2*/ sphere_environment_in_collision(environment, ADD_1832, SUB_1833, ADD_1835, 0.154))
        {
            if (sphere_environment_in_collision(environment, MUL_1849, MUL_1851, ADD_1854, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1868, MUL_1870, ADD_1873, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1882, NEGATE_1886, SUB_1897, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1906, NEGATE_1910, SUB_1921, 0.06))
            {
                return false;
            }
        }  // (22, 87)
        auto MUL_182 = SUB_149 * 0.7071068;
        auto MUL_198 = ADD_144 * 0.7071068;
        auto MUL_196 = SUB_138 * 0.7071068;
        auto SUB_209 = MUL_198 - MUL_196;
        auto ADD_199 = MUL_196 + MUL_198;
        auto MUL_184 = ADD_130 * 0.7071068;
        auto SUB_186 = MUL_182 - MUL_184;
        auto ADD_215 = MUL_182 + MUL_184;
        auto MUL_224 = ADD_144 * 0.316;
        auto MUL_235 = SUB_149 * MUL_224;
        auto MUL_228 = ADD_130 * 0.316;
        auto MUL_236 = SUB_138 * MUL_228;
        auto ADD_237 = MUL_235 + MUL_236;
        auto MUL_240 = ADD_237 * 2.0;
        auto INPUT_2 = q[2];
        auto DIV_262 = INPUT_2 * 0.5;
        auto SIN_263 = DIV_262.sin();
        auto COS_269 = DIV_262.cos();
        auto MUL_286 = ADD_215 * COS_269;
        auto MUL_281 = ADD_215 * SIN_263;
        auto MUL_284 = SUB_209 * COS_269;
        auto ADD_285 = MUL_281 + MUL_284;
        auto MUL_1933 = ADD_285 * ADD_285;
        auto MUL_289 = SUB_209 * SIN_263;
        auto SUB_290 = MUL_286 - MUL_289;
        auto MUL_1934 = SUB_290 * ADD_285;
        auto MUL_271 = SUB_186 * COS_269;
        auto MUL_276 = SUB_186 * SIN_263;
        auto MUL_278 = ADD_199 * COS_269;
        auto SUB_279 = MUL_278 - MUL_276;
        auto MUL_1935 = SUB_290 * SUB_279;
        auto MUL_1932 = SUB_279 * SUB_279;
        auto ADD_1941 = MUL_1932 + MUL_1933;
        auto MUL_1944 = ADD_1941 * 2.0;
        auto SUB_1947 = 1.0 - MUL_1944;
        auto MUL_1981 = SUB_1947 * 0.039;
        auto MUL_272 = ADD_199 * SIN_263;
        auto ADD_273 = MUL_271 + MUL_272;
        auto MUL_1939 = ADD_273 * ADD_285;
        auto ADD_1967 = MUL_1939 + MUL_1935;
        auto MUL_1969 = ADD_1967 * 2.0;
        auto MUL_1994 = MUL_1969 * 0.052;
        auto MUL_1938 = ADD_273 * SUB_279;
        auto SUB_1954 = MUL_1938 - MUL_1934;
        auto MUL_1956 = SUB_1954 * 2.0;
        auto MUL_1987 = MUL_1956 * 0.028;
        auto ADD_2004 = MUL_1981 + MUL_1987;
        auto SUB_2007 = ADD_2004 - MUL_1994;
        auto ADD_2010 = MUL_240 + SUB_2007;
        auto ADD_1948 = MUL_1938 + MUL_1934;
        auto MUL_1950 = ADD_1948 * 2.0;
        auto MUL_1983 = MUL_1950 * 0.039;
        auto MUL_1937 = SUB_290 * ADD_273;
        auto MUL_1940 = SUB_279 * ADD_285;
        auto SUB_1970 = MUL_1940 - MUL_1937;
        auto MUL_1972 = SUB_1970 * 2.0;
        auto MUL_1998 = MUL_1972 * 0.052;
        auto MUL_1936 = ADD_273 * ADD_273;
        auto ADD_1957 = MUL_1933 + MUL_1936;
        auto MUL_1960 = ADD_1957 * 2.0;
        auto SUB_1963 = 1.0 - MUL_1960;
        auto MUL_1989 = SUB_1963 * 0.028;
        auto ADD_2005 = MUL_1983 + MUL_1989;
        auto SUB_2008 = ADD_2005 - MUL_1998;
        auto MUL_246 = ADD_144 * MUL_224;
        auto MUL_244 = ADD_130 * MUL_228;
        auto ADD_247 = MUL_244 + MUL_246;
        auto MUL_249 = ADD_247 * 2.0;
        auto SUB_252 = MUL_249 - 0.316;
        auto ADD_2011 = SUB_252 + SUB_2008;
        auto SUB_1951 = MUL_1939 - MUL_1935;
        auto ADD_1964 = MUL_1940 + MUL_1937;
        auto ADD_1973 = MUL_1932 + MUL_1936;
        auto MUL_1976 = ADD_1973 * 2.0;
        auto SUB_1979 = 1.0 - MUL_1976;
        auto MUL_2002 = SUB_1979 * 0.052;
        auto MUL_1966 = ADD_1964 * 2.0;
        auto MUL_1991 = MUL_1966 * 0.028;
        auto MUL_1953 = SUB_1951 * 2.0;
        auto MUL_1985 = MUL_1953 * 0.039;
        auto ADD_2006 = MUL_1985 + MUL_1991;
        auto SUB_2009 = ADD_2006 - MUL_2002;
        auto MUL_253 = SUB_149 * MUL_228;
        auto MUL_255 = SUB_138 * MUL_224;
        auto SUB_256 = MUL_253 - MUL_255;
        auto MUL_258 = SUB_256 * 2.0;
        auto ADD_260 = 0.333 + MUL_258;
        auto ADD_2012 = ADD_260 + SUB_2009;
        auto MUL_2027 = MUL_1969 * 0.1;
        auto SUB_2037 = MUL_240 - MUL_2027;
        auto MUL_2031 = MUL_1972 * 0.1;
        auto SUB_2038 = SUB_252 - MUL_2031;
        auto MUL_2035 = SUB_1979 * 0.1;
        auto SUB_2039 = ADD_260 - MUL_2035;
        auto MUL_2054 = MUL_1969 * 0.06;
        auto SUB_2064 = MUL_240 - MUL_2054;
        auto MUL_2058 = MUL_1972 * 0.06;
        auto SUB_2065 = SUB_252 - MUL_2058;
        auto MUL_2062 = SUB_1979 * 0.06;
        auto SUB_2066 = ADD_260 - MUL_2062;
        auto MUL_2074 = MUL_1956 * 0.06;
        auto MUL_2068 = SUB_1947 * 0.08;
        auto ADD_2085 = MUL_2068 + MUL_2074;
        auto ADD_2088 = MUL_240 + ADD_2085;
        auto MUL_2076 = SUB_1963 * 0.06;
        auto MUL_2070 = MUL_1950 * 0.08;
        auto ADD_2086 = MUL_2070 + MUL_2076;
        auto ADD_2089 = SUB_252 + ADD_2086;
        auto MUL_2078 = MUL_1966 * 0.06;
        auto MUL_2072 = MUL_1953 * 0.08;
        auto ADD_2087 = MUL_2072 + MUL_2078;
        auto ADD_2090 = ADD_260 + ADD_2087;
        auto MUL_2098 = MUL_1956 * 0.02;
        auto ADD_2109 = MUL_2068 + MUL_2098;
        auto ADD_2112 = MUL_240 + ADD_2109;
        auto MUL_2100 = SUB_1963 * 0.02;
        auto ADD_2110 = MUL_2070 + MUL_2100;
        auto ADD_2113 = SUB_252 + ADD_2110;
        auto MUL_2102 = MUL_1966 * 0.02;
        auto ADD_2111 = MUL_2072 + MUL_2102;
        auto ADD_2114 = ADD_260 + ADD_2111;
        if (/*panda_link3*/ sphere_environment_in_collision(environment, ADD_2010, ADD_2011, ADD_2012, 0.128))
        {
            if (sphere_environment_in_collision(environment, SUB_2037, SUB_2038, SUB_2039, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2064, SUB_2065, SUB_2066, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2088, ADD_2089, ADD_2090, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2112, ADD_2113, ADD_2114, 0.055))
            {
                return false;
            }
        }  // (87, 208)
        auto MUL_323 = SUB_290 * 0.7071068;
        auto MUL_338 = ADD_285 * 0.7071068;
        auto MUL_336 = SUB_279 * 0.7071068;
        auto SUB_349 = MUL_338 - MUL_336;
        auto ADD_339 = MUL_336 + MUL_338;
        auto MUL_325 = ADD_273 * 0.7071068;
        auto SUB_354 = MUL_323 - MUL_325;
        auto ADD_326 = MUL_323 + MUL_325;
        auto MUL_371 = ADD_285 * 0.0825;
        auto MUL_376 = ADD_285 * MUL_371;
        auto MUL_366 = SUB_279 * 0.0825;
        auto MUL_374 = SUB_279 * MUL_366;
        auto ADD_378 = MUL_374 + MUL_376;
        auto MUL_381 = ADD_378 * 2.0;
        auto SUB_384 = 0.0825 - MUL_381;
        auto ADD_403 = MUL_240 + SUB_384;
        auto INPUT_3 = q[3];
        auto DIV_407 = INPUT_3 * 0.5;
        auto SIN_408 = DIV_407.sin();
        auto COS_414 = DIV_407.cos();
        auto MUL_431 = SUB_354 * COS_414;
        auto MUL_426 = SUB_354 * SIN_408;
        auto MUL_429 = SUB_349 * COS_414;
        auto ADD_430 = MUL_426 + MUL_429;
        auto MUL_2126 = ADD_430 * ADD_430;
        auto MUL_434 = SUB_349 * SIN_408;
        auto SUB_435 = MUL_431 - MUL_434;
        auto MUL_2127 = SUB_435 * ADD_430;
        auto MUL_416 = ADD_326 * COS_414;
        auto MUL_421 = ADD_326 * SIN_408;
        auto MUL_423 = ADD_339 * COS_414;
        auto SUB_424 = MUL_423 - MUL_421;
        auto MUL_2128 = SUB_435 * SUB_424;
        auto MUL_2125 = SUB_424 * SUB_424;
        auto ADD_2134 = MUL_2125 + MUL_2126;
        auto MUL_2137 = ADD_2134 * 2.0;
        auto SUB_2140 = 1.0 - MUL_2137;
        auto MUL_2175 = SUB_2140 * 0.042;
        auto MUL_417 = ADD_339 * SIN_408;
        auto ADD_418 = MUL_416 + MUL_417;
        auto MUL_2132 = ADD_418 * ADD_430;
        auto ADD_2160 = MUL_2132 + MUL_2128;
        auto MUL_2162 = ADD_2160 * 2.0;
        auto MUL_2192 = MUL_2162 * 0.029;
        auto MUL_2131 = ADD_418 * SUB_424;
        auto SUB_2147 = MUL_2131 - MUL_2127;
        auto MUL_2149 = SUB_2147 * 2.0;
        auto MUL_2186 = MUL_2149 * 0.049;
        auto SUB_2197 = MUL_2186 - MUL_2175;
        auto ADD_2200 = SUB_2197 + MUL_2192;
        auto ADD_2203 = ADD_403 + ADD_2200;
        auto ADD_2141 = MUL_2131 + MUL_2127;
        auto MUL_2143 = ADD_2141 * 2.0;
        auto MUL_2179 = MUL_2143 * 0.042;
        auto MUL_2130 = SUB_435 * ADD_418;
        auto MUL_2133 = SUB_424 * ADD_430;
        auto SUB_2163 = MUL_2133 - MUL_2130;
        auto MUL_2165 = SUB_2163 * 2.0;
        auto MUL_2194 = MUL_2165 * 0.029;
        auto MUL_2129 = ADD_418 * ADD_418;
        auto ADD_2150 = MUL_2126 + MUL_2129;
        auto MUL_2153 = ADD_2150 * 2.0;
        auto SUB_2156 = 1.0 - MUL_2153;
        auto MUL_2188 = SUB_2156 * 0.049;
        auto SUB_2198 = MUL_2188 - MUL_2179;
        auto ADD_2201 = SUB_2198 + MUL_2194;
        auto MUL_386 = SUB_290 * MUL_371;
        auto MUL_387 = ADD_273 * MUL_366;
        auto ADD_389 = MUL_386 + MUL_387;
        auto MUL_392 = ADD_389 * 2.0;
        auto ADD_404 = SUB_252 + MUL_392;
        auto ADD_2204 = ADD_404 + ADD_2201;
        auto SUB_2144 = MUL_2132 - MUL_2128;
        auto ADD_2157 = MUL_2133 + MUL_2130;
        auto ADD_2166 = MUL_2125 + MUL_2129;
        auto MUL_2169 = ADD_2166 * 2.0;
        auto SUB_2172 = 1.0 - MUL_2169;
        auto MUL_2196 = SUB_2172 * 0.029;
        auto MUL_2159 = ADD_2157 * 2.0;
        auto MUL_2190 = MUL_2159 * 0.049;
        auto MUL_2146 = SUB_2144 * 2.0;
        auto MUL_2183 = MUL_2146 * 0.042;
        auto SUB_2199 = MUL_2190 - MUL_2183;
        auto ADD_2202 = SUB_2199 + MUL_2196;
        auto MUL_394 = SUB_290 * MUL_366;
        auto MUL_396 = ADD_273 * MUL_371;
        auto SUB_398 = MUL_396 - MUL_394;
        auto MUL_401 = SUB_398 * 2.0;
        auto ADD_405 = ADD_260 + MUL_401;
        auto ADD_2205 = ADD_405 + ADD_2202;
        auto MUL_2208 = SUB_2140 * 0.08;
        auto MUL_2219 = MUL_2149 * 0.095;
        auto SUB_2230 = MUL_2219 - MUL_2208;
        auto ADD_2233 = ADD_403 + SUB_2230;
        auto MUL_2221 = SUB_2156 * 0.095;
        auto MUL_2212 = MUL_2143 * 0.08;
        auto SUB_2231 = MUL_2221 - MUL_2212;
        auto ADD_2234 = ADD_404 + SUB_2231;
        auto MUL_2223 = MUL_2159 * 0.095;
        auto MUL_2216 = MUL_2146 * 0.08;
        auto SUB_2232 = MUL_2223 - MUL_2216;
        auto ADD_2235 = ADD_405 + SUB_2232;
        auto MUL_2249 = MUL_2162 * 0.02;
        auto ADD_2254 = ADD_403 + MUL_2249;
        auto MUL_2251 = MUL_2165 * 0.02;
        auto ADD_2255 = ADD_404 + MUL_2251;
        auto MUL_2253 = SUB_2172 * 0.02;
        auto ADD_2256 = ADD_405 + MUL_2253;
        auto MUL_2270 = MUL_2162 * 0.06;
        auto ADD_2275 = ADD_403 + MUL_2270;
        auto MUL_2272 = MUL_2165 * 0.06;
        auto ADD_2276 = ADD_404 + MUL_2272;
        auto MUL_2274 = SUB_2172 * 0.06;
        auto ADD_2277 = ADD_405 + MUL_2274;
        auto MUL_2291 = MUL_2149 * 0.06;
        auto SUB_2302 = MUL_2291 - MUL_2208;
        auto ADD_2305 = ADD_403 + SUB_2302;
        auto MUL_2293 = SUB_2156 * 0.06;
        auto SUB_2303 = MUL_2293 - MUL_2212;
        auto ADD_2306 = ADD_404 + SUB_2303;
        auto MUL_2295 = MUL_2159 * 0.06;
        auto SUB_2304 = MUL_2295 - MUL_2216;
        auto ADD_2307 = ADD_405 + SUB_2304;
        if (/*panda_link4*/ sphere_environment_in_collision(environment, ADD_2203, ADD_2204, ADD_2205, 0.126))
        {
            if (sphere_environment_in_collision(environment, ADD_2233, ADD_2234, ADD_2235, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2254, ADD_2255, ADD_2256, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2275, ADD_2276, ADD_2277, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2305, ADD_2306, ADD_2307, 0.055))
            {
                return false;
            }
        }  // (208, 331)
        auto MUL_469 = SUB_435 * 0.7071068;
        auto MUL_486 = ADD_430 * 0.7071068;
        auto MUL_527 = ADD_430 * 0.0825;
        auto MUL_533 = ADD_430 * MUL_527;
        auto MUL_483 = SUB_424 * 0.7071068;
        auto SUB_488 = MUL_483 - MUL_486;
        auto ADD_499 = MUL_483 + MUL_486;
        auto MUL_520 = SUB_424 * 0.0825;
        auto MUL_472 = ADD_418 * 0.7071068;
        auto SUB_473 = MUL_472 - MUL_469;
        auto ADD_506 = MUL_469 + MUL_472;
        auto MUL_514 = ADD_430 * 0.384;
        auto MUL_529 = SUB_435 * MUL_514;
        auto MUL_517 = ADD_418 * 0.384;
        auto ADD_522 = MUL_517 + MUL_520;
        auto MUL_531 = SUB_424 * ADD_522;
        auto SUB_532 = MUL_531 - MUL_529;
        auto ADD_534 = SUB_532 + MUL_533;
        auto MUL_536 = ADD_534 * 2.0;
        auto SUB_539 = MUL_536 - 0.0825;
        auto ADD_564 = ADD_403 + SUB_539;
        auto INPUT_4 = q[4];
        auto DIV_568 = INPUT_4 * 0.5;
        auto SIN_569 = DIV_568.sin();
        auto COS_575 = DIV_568.cos();
        auto MUL_592 = ADD_506 * COS_575;
        auto MUL_587 = ADD_506 * SIN_569;
        auto MUL_590 = ADD_499 * COS_575;
        auto ADD_591 = MUL_587 + MUL_590;
        auto MUL_2319 = ADD_591 * ADD_591;
        auto MUL_595 = ADD_499 * SIN_569;
        auto SUB_596 = MUL_592 - MUL_595;
        auto MUL_2320 = SUB_596 * ADD_591;
        auto MUL_577 = SUB_473 * COS_575;
        auto MUL_582 = SUB_473 * SIN_569;
        auto MUL_584 = SUB_488 * COS_575;
        auto SUB_585 = MUL_584 - MUL_582;
        auto MUL_2321 = SUB_596 * SUB_585;
        auto MUL_2318 = SUB_585 * SUB_585;
        auto ADD_2327 = MUL_2318 + MUL_2319;
        auto MUL_2330 = ADD_2327 * 2.0;
        auto SUB_2333 = 1.0 - MUL_2330;
        auto MUL_2368 = SUB_2333 * 0.001;
        auto MUL_578 = SUB_488 * SIN_569;
        auto ADD_579 = MUL_577 + MUL_578;
        auto MUL_2325 = ADD_579 * ADD_591;
        auto ADD_2353 = MUL_2325 + MUL_2321;
        auto MUL_2355 = ADD_2353 * 2.0;
        auto MUL_2386 = MUL_2355 * 0.11;
        auto MUL_2324 = ADD_579 * SUB_585;
        auto SUB_2340 = MUL_2324 - MUL_2320;
        auto MUL_2342 = SUB_2340 * 2.0;
        auto MUL_2379 = MUL_2342 * 0.037;
        auto SUB_2396 = MUL_2379 - MUL_2368;
        auto SUB_2399 = SUB_2396 - MUL_2386;
        auto ADD_2402 = ADD_564 + SUB_2399;
        auto ADD_2334 = MUL_2324 + MUL_2320;
        auto MUL_2336 = ADD_2334 * 2.0;
        auto MUL_2372 = MUL_2336 * 0.001;
        auto MUL_2323 = SUB_596 * ADD_579;
        auto MUL_2326 = SUB_585 * ADD_591;
        auto SUB_2356 = MUL_2326 - MUL_2323;
        auto MUL_2358 = SUB_2356 * 2.0;
        auto MUL_2390 = MUL_2358 * 0.11;
        auto MUL_2322 = ADD_579 * ADD_579;
        auto ADD_2343 = MUL_2319 + MUL_2322;
        auto MUL_2346 = ADD_2343 * 2.0;
        auto SUB_2349 = 1.0 - MUL_2346;
        auto MUL_2381 = SUB_2349 * 0.037;
        auto SUB_2397 = MUL_2381 - MUL_2372;
        auto SUB_2400 = SUB_2397 - MUL_2390;
        auto MUL_541 = SUB_435 * MUL_527;
        auto MUL_546 = ADD_430 * MUL_514;
        auto MUL_543 = ADD_418 * ADD_522;
        auto ADD_544 = MUL_541 + MUL_543;
        auto ADD_548 = ADD_544 + MUL_546;
        auto MUL_551 = ADD_548 * 2.0;
        auto SUB_554 = 0.384 - MUL_551;
        auto ADD_565 = ADD_404 + SUB_554;
        auto ADD_2403 = ADD_565 + SUB_2400;
        auto SUB_2337 = MUL_2325 - MUL_2321;
        auto ADD_2350 = MUL_2326 + MUL_2323;
        auto ADD_2359 = MUL_2318 + MUL_2322;
        auto MUL_2362 = ADD_2359 * 2.0;
        auto SUB_2365 = 1.0 - MUL_2362;
        auto MUL_2394 = SUB_2365 * 0.11;
        auto MUL_2352 = ADD_2350 * 2.0;
        auto MUL_2383 = MUL_2352 * 0.037;
        auto MUL_2339 = SUB_2337 * 2.0;
        auto MUL_2376 = MUL_2339 * 0.001;
        auto SUB_2398 = MUL_2383 - MUL_2376;
        auto SUB_2401 = SUB_2398 - MUL_2394;
        auto MUL_555 = SUB_435 * ADD_522;
        auto MUL_558 = SUB_424 * MUL_514;
        auto MUL_556 = ADD_418 * MUL_527;
        auto SUB_557 = MUL_555 - MUL_556;
        auto ADD_560 = SUB_557 + MUL_558;
        auto MUL_562 = ADD_560 * 2.0;
        auto ADD_566 = ADD_405 + MUL_562;
        auto ADD_2404 = ADD_566 + SUB_2401;
        auto MUL_2412 = MUL_2342 * 0.055;
        auto ADD_2423 = ADD_564 + MUL_2412;
        auto MUL_2414 = SUB_2349 * 0.055;
        auto ADD_2424 = ADD_565 + MUL_2414;
        auto MUL_2416 = MUL_2352 * 0.055;
        auto ADD_2425 = ADD_566 + MUL_2416;
        auto MUL_2433 = MUL_2342 * 0.075;
        auto ADD_2444 = ADD_564 + MUL_2433;
        auto MUL_2435 = SUB_2349 * 0.075;
        auto ADD_2445 = ADD_565 + MUL_2435;
        auto MUL_2437 = MUL_2352 * 0.075;
        auto ADD_2446 = ADD_566 + MUL_2437;
        auto MUL_2461 = MUL_2355 * 0.22;
        auto SUB_2471 = ADD_564 - MUL_2461;
        auto MUL_2465 = MUL_2358 * 0.22;
        auto SUB_2472 = ADD_565 - MUL_2465;
        auto MUL_2469 = SUB_2365 * 0.22;
        auto SUB_2473 = ADD_566 - MUL_2469;
        auto MUL_2488 = MUL_2355 * 0.18;
        auto MUL_2481 = MUL_2342 * 0.05;
        auto SUB_2498 = MUL_2481 - MUL_2488;
        auto ADD_2501 = ADD_564 + SUB_2498;
        auto MUL_2492 = MUL_2358 * 0.18;
        auto MUL_2483 = SUB_2349 * 0.05;
        auto SUB_2499 = MUL_2483 - MUL_2492;
        auto ADD_2502 = ADD_565 + SUB_2499;
        auto MUL_2496 = SUB_2365 * 0.18;
        auto MUL_2485 = MUL_2352 * 0.05;
        auto SUB_2500 = MUL_2485 - MUL_2496;
        auto ADD_2503 = ADD_566 + SUB_2500;
        auto MUL_2511 = MUL_2342 * 0.08;
        auto MUL_2518 = MUL_2355 * 0.14;
        auto MUL_2505 = SUB_2333 * 0.01;
        auto ADD_2528 = MUL_2505 + MUL_2511;
        auto SUB_2531 = ADD_2528 - MUL_2518;
        auto ADD_2534 = ADD_564 + SUB_2531;
        auto MUL_2522 = MUL_2358 * 0.14;
        auto MUL_2513 = SUB_2349 * 0.08;
        auto MUL_2507 = MUL_2336 * 0.01;
        auto ADD_2529 = MUL_2507 + MUL_2513;
        auto SUB_2532 = ADD_2529 - MUL_2522;
        auto ADD_2535 = ADD_565 + SUB_2532;
        auto MUL_2526 = SUB_2365 * 0.14;
        auto MUL_2515 = MUL_2352 * 0.08;
        auto MUL_2509 = MUL_2339 * 0.01;
        auto ADD_2530 = MUL_2509 + MUL_2515;
        auto SUB_2533 = ADD_2530 - MUL_2526;
        auto ADD_2536 = ADD_566 + SUB_2533;
        auto MUL_2544 = MUL_2342 * 0.085;
        auto ADD_2561 = MUL_2505 + MUL_2544;
        auto SUB_2564 = ADD_2561 - MUL_2386;
        auto ADD_2567 = ADD_564 + SUB_2564;
        auto MUL_2546 = SUB_2349 * 0.085;
        auto ADD_2562 = MUL_2507 + MUL_2546;
        auto SUB_2565 = ADD_2562 - MUL_2390;
        auto ADD_2568 = ADD_565 + SUB_2565;
        auto MUL_2548 = MUL_2352 * 0.085;
        auto ADD_2563 = MUL_2509 + MUL_2548;
        auto SUB_2566 = ADD_2563 - MUL_2394;
        auto ADD_2569 = ADD_566 + SUB_2566;
        auto MUL_2584 = MUL_2355 * 0.08;
        auto MUL_2577 = MUL_2342 * 0.09;
        auto ADD_2594 = MUL_2505 + MUL_2577;
        auto SUB_2597 = ADD_2594 - MUL_2584;
        auto ADD_2600 = ADD_564 + SUB_2597;
        auto MUL_2588 = MUL_2358 * 0.08;
        auto MUL_2579 = SUB_2349 * 0.09;
        auto ADD_2595 = MUL_2507 + MUL_2579;
        auto SUB_2598 = ADD_2595 - MUL_2588;
        auto ADD_2601 = ADD_565 + SUB_2598;
        auto MUL_2592 = SUB_2365 * 0.08;
        auto MUL_2581 = MUL_2352 * 0.09;
        auto ADD_2596 = MUL_2509 + MUL_2581;
        auto SUB_2599 = ADD_2596 - MUL_2592;
        auto ADD_2602 = ADD_566 + SUB_2599;
        auto MUL_2617 = MUL_2355 * 0.05;
        auto MUL_2610 = MUL_2342 * 0.095;
        auto ADD_2627 = MUL_2505 + MUL_2610;
        auto SUB_2630 = ADD_2627 - MUL_2617;
        auto ADD_2633 = ADD_564 + SUB_2630;
        auto MUL_2621 = MUL_2358 * 0.05;
        auto MUL_2612 = SUB_2349 * 0.095;
        auto ADD_2628 = MUL_2507 + MUL_2612;
        auto SUB_2631 = ADD_2628 - MUL_2621;
        auto ADD_2634 = ADD_565 + SUB_2631;
        auto MUL_2625 = SUB_2365 * 0.05;
        auto MUL_2614 = MUL_2352 * 0.095;
        auto ADD_2629 = MUL_2509 + MUL_2614;
        auto SUB_2632 = ADD_2629 - MUL_2625;
        auto ADD_2635 = ADD_566 + SUB_2632;
        auto SUB_2666 = MUL_2511 - MUL_2505;
        auto SUB_2669 = SUB_2666 - MUL_2518;
        auto ADD_2672 = ADD_564 + SUB_2669;
        auto SUB_2667 = MUL_2513 - MUL_2507;
        auto SUB_2670 = SUB_2667 - MUL_2522;
        auto ADD_2673 = ADD_565 + SUB_2670;
        auto SUB_2668 = MUL_2515 - MUL_2509;
        auto SUB_2671 = SUB_2668 - MUL_2526;
        auto ADD_2674 = ADD_566 + SUB_2671;
        auto SUB_2705 = MUL_2544 - MUL_2505;
        auto SUB_2708 = SUB_2705 - MUL_2386;
        auto ADD_2711 = ADD_564 + SUB_2708;
        auto SUB_2706 = MUL_2546 - MUL_2507;
        auto SUB_2709 = SUB_2706 - MUL_2390;
        auto ADD_2712 = ADD_565 + SUB_2709;
        auto SUB_2707 = MUL_2548 - MUL_2509;
        auto SUB_2710 = SUB_2707 - MUL_2394;
        auto ADD_2713 = ADD_566 + SUB_2710;
        auto SUB_2744 = MUL_2577 - MUL_2505;
        auto SUB_2747 = SUB_2744 - MUL_2584;
        auto ADD_2750 = ADD_564 + SUB_2747;
        auto SUB_2745 = MUL_2579 - MUL_2507;
        auto SUB_2748 = SUB_2745 - MUL_2588;
        auto ADD_2751 = ADD_565 + SUB_2748;
        auto SUB_2746 = MUL_2581 - MUL_2509;
        auto SUB_2749 = SUB_2746 - MUL_2592;
        auto ADD_2752 = ADD_566 + SUB_2749;
        auto SUB_2783 = MUL_2610 - MUL_2505;
        auto SUB_2786 = SUB_2783 - MUL_2617;
        auto ADD_2789 = ADD_564 + SUB_2786;
        auto SUB_2784 = MUL_2612 - MUL_2507;
        auto SUB_2787 = SUB_2784 - MUL_2621;
        auto ADD_2790 = ADD_565 + SUB_2787;
        auto SUB_2785 = MUL_2614 - MUL_2509;
        auto SUB_2788 = SUB_2785 - MUL_2625;
        auto ADD_2791 = ADD_566 + SUB_2788;
        if (/*panda_link0 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (331, 557)
        if (/*panda_link1 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        if (/*panda_link2 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        if (/*panda_link5*/ sphere_environment_in_collision(environment, ADD_2402, ADD_2403, ADD_2404, 0.176))
        {
            if (sphere_environment_in_collision(environment, ADD_2423, ADD_2424, ADD_2425, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2444, ADD_2445, ADD_2446, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2471, SUB_2472, SUB_2473, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2501, ADD_2502, ADD_2503, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2534, ADD_2535, ADD_2536, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2567, ADD_2568, ADD_2569, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2600, ADD_2601, ADD_2602, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2633, ADD_2634, ADD_2635, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2672, ADD_2673, ADD_2674, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2711, ADD_2712, ADD_2713, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2750, ADD_2751, ADD_2752, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2789, ADD_2790, ADD_2791, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        auto MUL_657 = SUB_596 * 0.7071068;
        auto MUL_654 = ADD_591 * 0.7071068;
        auto MUL_651 = SUB_585 * 0.7071068;
        auto SUB_655 = MUL_654 - MUL_651;
        auto ADD_645 = MUL_651 + MUL_654;
        auto MUL_659 = ADD_579 * 0.7071068;
        auto SUB_660 = MUL_657 - MUL_659;
        auto ADD_632 = MUL_657 + MUL_659;
        auto INPUT_5 = q[5];
        auto DIV_697 = INPUT_5 * 0.5;
        auto SIN_698 = DIV_697.sin();
        auto COS_704 = DIV_697.cos();
        auto MUL_716 = SUB_660 * SIN_698;
        auto MUL_721 = SUB_660 * COS_704;
        auto MUL_724 = SUB_655 * SIN_698;
        auto SUB_725 = MUL_721 - MUL_724;
        auto MUL_719 = SUB_655 * COS_704;
        auto ADD_720 = MUL_716 + MUL_719;
        auto MUL_2820 = SUB_725 * ADD_720;
        auto MUL_2819 = ADD_720 * ADD_720;
        auto MUL_711 = ADD_632 * SIN_698;
        auto MUL_706 = ADD_632 * COS_704;
        auto MUL_707 = ADD_645 * SIN_698;
        auto ADD_708 = MUL_706 + MUL_707;
        auto MUL_713 = ADD_645 * COS_704;
        auto SUB_714 = MUL_713 - MUL_711;
        auto MUL_2818 = SUB_714 * SUB_714;
        auto ADD_2827 = MUL_2818 + MUL_2819;
        auto MUL_2830 = ADD_2827 * 2.0;
        auto SUB_2833 = 1.0 - MUL_2830;
        auto MUL_2867 = SUB_2833 * 0.042;
        auto MUL_2824 = ADD_708 * SUB_714;
        auto SUB_2840 = MUL_2824 - MUL_2820;
        auto MUL_2842 = SUB_2840 * 2.0;
        auto MUL_2873 = MUL_2842 * 0.014;
        auto ADD_2884 = MUL_2867 + MUL_2873;
        auto ADD_2887 = ADD_564 + ADD_2884;
        auto ADD_2834 = MUL_2824 + MUL_2820;
        auto MUL_2836 = ADD_2834 * 2.0;
        auto MUL_2869 = MUL_2836 * 0.042;
        auto MUL_2822 = ADD_708 * ADD_708;
        auto ADD_2843 = MUL_2819 + MUL_2822;
        auto MUL_2846 = ADD_2843 * 2.0;
        auto SUB_2849 = 1.0 - MUL_2846;
        auto MUL_2875 = SUB_2849 * 0.014;
        auto ADD_2885 = MUL_2869 + MUL_2875;
        auto ADD_2888 = ADD_565 + ADD_2885;
        auto MUL_2821 = SUB_725 * SUB_714;
        auto MUL_2823 = SUB_725 * ADD_708;
        auto MUL_2826 = SUB_714 * ADD_720;
        auto ADD_2850 = MUL_2826 + MUL_2823;
        auto MUL_2852 = ADD_2850 * 2.0;
        auto MUL_2877 = MUL_2852 * 0.014;
        auto MUL_2825 = ADD_708 * ADD_720;
        auto SUB_2837 = MUL_2825 - MUL_2821;
        auto MUL_2839 = SUB_2837 * 2.0;
        auto MUL_2871 = MUL_2839 * 0.042;
        auto ADD_2886 = MUL_2871 + MUL_2877;
        auto ADD_2889 = ADD_566 + ADD_2886;
        auto MUL_2916 = MUL_2842 * 0.01;
        auto MUL_2909 = SUB_2833 * 0.08;
        auto SUB_2932 = MUL_2909 - MUL_2916;
        auto ADD_2935 = ADD_564 + SUB_2932;
        auto MUL_2920 = SUB_2849 * 0.01;
        auto MUL_2911 = MUL_2836 * 0.08;
        auto SUB_2933 = MUL_2911 - MUL_2920;
        auto ADD_2936 = ADD_565 + SUB_2933;
        auto MUL_2924 = MUL_2852 * 0.01;
        auto MUL_2913 = MUL_2839 * 0.08;
        auto SUB_2934 = MUL_2913 - MUL_2924;
        auto ADD_2937 = ADD_566 + SUB_2934;
        auto MUL_2945 = MUL_2842 * 0.035;
        auto ADD_2956 = MUL_2909 + MUL_2945;
        auto ADD_2959 = ADD_564 + ADD_2956;
        auto MUL_2947 = SUB_2849 * 0.035;
        auto ADD_2957 = MUL_2911 + MUL_2947;
        auto ADD_2960 = ADD_565 + ADD_2957;
        auto MUL_2949 = MUL_2852 * 0.035;
        auto ADD_2958 = MUL_2913 + MUL_2949;
        auto ADD_2961 = ADD_566 + ADD_2958;
        if (/*panda_link0 vs. panda_link6*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (557, 637)
        if (/*panda_link1 vs. panda_link6*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (637, 637)
        if (/*panda_link6*/ sphere_environment_in_collision(environment, ADD_2887, ADD_2888, ADD_2889, 0.095))
        {
            if (sphere_environment_in_collision(environment, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (637, 637)
        auto MUL_758 = SUB_725 * 0.7071068;
        auto MUL_773 = ADD_720 * 0.7071068;
        auto MUL_771 = SUB_714 * 0.7071068;
        auto SUB_784 = MUL_773 - MUL_771;
        auto ADD_774 = MUL_771 + MUL_773;
        auto MUL_760 = ADD_708 * 0.7071068;
        auto SUB_789 = MUL_758 - MUL_760;
        auto ADD_761 = MUL_758 + MUL_760;
        auto MUL_806 = ADD_720 * 0.088;
        auto MUL_811 = ADD_720 * MUL_806;
        auto MUL_801 = SUB_714 * 0.088;
        auto MUL_809 = SUB_714 * MUL_801;
        auto ADD_813 = MUL_809 + MUL_811;
        auto MUL_816 = ADD_813 * 2.0;
        auto SUB_819 = 0.088 - MUL_816;
        auto ADD_838 = ADD_564 + SUB_819;
        auto INPUT_6 = q[6];
        auto DIV_842 = INPUT_6 * 0.5;
        auto SIN_843 = DIV_842.sin();
        auto COS_849 = DIV_842.cos();
        auto MUL_866 = SUB_789 * COS_849;
        auto MUL_861 = SUB_789 * SIN_843;
        auto MUL_864 = SUB_784 * COS_849;
        auto ADD_865 = MUL_861 + MUL_864;
        auto MUL_2971 = ADD_865 * ADD_865;
        auto MUL_869 = SUB_784 * SIN_843;
        auto SUB_870 = MUL_866 - MUL_869;
        auto MUL_2972 = SUB_870 * ADD_865;
        auto MUL_851 = ADD_761 * COS_849;
        auto MUL_856 = ADD_761 * SIN_843;
        auto MUL_858 = ADD_774 * COS_849;
        auto SUB_859 = MUL_858 - MUL_856;
        auto MUL_2973 = SUB_870 * SUB_859;
        auto MUL_2970 = SUB_859 * SUB_859;
        auto ADD_2979 = MUL_2970 + MUL_2971;
        auto MUL_2982 = ADD_2979 * 2.0;
        auto SUB_2985 = 1.0 - MUL_2982;
        auto MUL_3019 = SUB_2985 * 0.015;
        auto MUL_852 = ADD_774 * SIN_843;
        auto ADD_853 = MUL_851 + MUL_852;
        auto MUL_2977 = ADD_853 * ADD_865;
        auto ADD_3005 = MUL_2977 + MUL_2973;
        auto MUL_3007 = ADD_3005 * 2.0;
        auto MUL_3031 = MUL_3007 * 0.075;
        auto MUL_2976 = ADD_853 * SUB_859;
        auto SUB_2992 = MUL_2976 - MUL_2972;
        auto MUL_2994 = SUB_2992 * 2.0;
        auto MUL_3025 = MUL_2994 * 0.015;
        auto ADD_3036 = MUL_3019 + MUL_3025;
        auto ADD_3039 = ADD_3036 + MUL_3031;
        auto ADD_3042 = ADD_838 + ADD_3039;
        auto ADD_2986 = MUL_2976 + MUL_2972;
        auto MUL_2988 = ADD_2986 * 2.0;
        auto MUL_3021 = MUL_2988 * 0.015;
        auto MUL_2975 = SUB_870 * ADD_853;
        auto MUL_2978 = SUB_859 * ADD_865;
        auto SUB_3008 = MUL_2978 - MUL_2975;
        auto MUL_3010 = SUB_3008 * 2.0;
        auto MUL_3033 = MUL_3010 * 0.075;
        auto MUL_2974 = ADD_853 * ADD_853;
        auto ADD_2995 = MUL_2971 + MUL_2974;
        auto MUL_2998 = ADD_2995 * 2.0;
        auto SUB_3001 = 1.0 - MUL_2998;
        auto MUL_3027 = SUB_3001 * 0.015;
        auto ADD_3037 = MUL_3021 + MUL_3027;
        auto ADD_3040 = ADD_3037 + MUL_3033;
        auto MUL_821 = SUB_725 * MUL_806;
        auto MUL_822 = ADD_708 * MUL_801;
        auto ADD_824 = MUL_821 + MUL_822;
        auto MUL_827 = ADD_824 * 2.0;
        auto ADD_839 = ADD_565 + MUL_827;
        auto ADD_3043 = ADD_839 + ADD_3040;
        auto SUB_2989 = MUL_2977 - MUL_2973;
        auto ADD_3002 = MUL_2978 + MUL_2975;
        auto ADD_3011 = MUL_2970 + MUL_2974;
        auto MUL_3014 = ADD_3011 * 2.0;
        auto SUB_3017 = 1.0 - MUL_3014;
        auto MUL_3035 = SUB_3017 * 0.075;
        auto MUL_3004 = ADD_3002 * 2.0;
        auto MUL_3029 = MUL_3004 * 0.015;
        auto MUL_2991 = SUB_2989 * 2.0;
        auto MUL_3023 = MUL_2991 * 0.015;
        auto ADD_3038 = MUL_3023 + MUL_3029;
        auto ADD_3041 = ADD_3038 + MUL_3035;
        auto MUL_829 = SUB_725 * MUL_801;
        auto MUL_831 = ADD_708 * MUL_806;
        auto SUB_833 = MUL_831 - MUL_829;
        auto MUL_836 = SUB_833 * 2.0;
        auto ADD_840 = ADD_566 + MUL_836;
        auto ADD_3044 = ADD_840 + ADD_3041;
        auto MUL_3058 = MUL_3007 * 0.07;
        auto ADD_3063 = ADD_838 + MUL_3058;
        auto MUL_3060 = MUL_3010 * 0.07;
        auto ADD_3064 = ADD_839 + MUL_3060;
        auto MUL_3062 = SUB_3017 * 0.07;
        auto ADD_3065 = ADD_840 + MUL_3062;
        auto MUL_3079 = MUL_3007 * 0.08;
        auto MUL_3073 = MUL_2994 * 0.04;
        auto MUL_3067 = SUB_2985 * 0.02;
        auto ADD_3084 = MUL_3067 + MUL_3073;
        auto ADD_3087 = ADD_3084 + MUL_3079;
        auto ADD_3090 = ADD_838 + ADD_3087;
        auto MUL_3081 = MUL_3010 * 0.08;
        auto MUL_3075 = SUB_3001 * 0.04;
        auto MUL_3069 = MUL_2988 * 0.02;
        auto ADD_3085 = MUL_3069 + MUL_3075;
        auto ADD_3088 = ADD_3085 + MUL_3081;
        auto ADD_3091 = ADD_839 + ADD_3088;
        auto MUL_3083 = SUB_3017 * 0.08;
        auto MUL_3077 = MUL_3004 * 0.04;
        auto MUL_3071 = MUL_2991 * 0.02;
        auto ADD_3086 = MUL_3071 + MUL_3077;
        auto ADD_3089 = ADD_3086 + MUL_3083;
        auto ADD_3092 = ADD_840 + ADD_3089;
        auto MUL_3100 = MUL_2994 * 0.02;
        auto MUL_3094 = SUB_2985 * 0.04;
        auto ADD_3111 = MUL_3094 + MUL_3100;
        auto ADD_3114 = ADD_3111 + MUL_3079;
        auto ADD_3117 = ADD_838 + ADD_3114;
        auto MUL_3102 = SUB_3001 * 0.02;
        auto MUL_3096 = MUL_2988 * 0.04;
        auto ADD_3112 = MUL_3096 + MUL_3102;
        auto ADD_3115 = ADD_3112 + MUL_3081;
        auto ADD_3118 = ADD_839 + ADD_3115;
        auto MUL_3104 = MUL_3004 * 0.02;
        auto MUL_3098 = MUL_2991 * 0.04;
        auto ADD_3113 = MUL_3098 + MUL_3104;
        auto ADD_3116 = ADD_3113 + MUL_3083;
        auto ADD_3119 = ADD_840 + ADD_3116;
        auto MUL_3133 = MUL_3007 * 0.085;
        auto MUL_3127 = MUL_2994 * 0.06;
        auto ADD_3138 = MUL_3094 + MUL_3127;
        auto ADD_3141 = ADD_3138 + MUL_3133;
        auto ADD_3144 = ADD_838 + ADD_3141;
        auto MUL_3135 = MUL_3010 * 0.085;
        auto MUL_3129 = SUB_3001 * 0.06;
        auto ADD_3139 = MUL_3096 + MUL_3129;
        auto ADD_3142 = ADD_3139 + MUL_3135;
        auto ADD_3145 = ADD_839 + ADD_3142;
        auto MUL_3137 = SUB_3017 * 0.085;
        auto MUL_3131 = MUL_3004 * 0.06;
        auto ADD_3140 = MUL_3098 + MUL_3131;
        auto ADD_3143 = ADD_3140 + MUL_3137;
        auto ADD_3146 = ADD_840 + ADD_3143;
        auto MUL_3148 = SUB_2985 * 0.06;
        auto ADD_3165 = MUL_3148 + MUL_3073;
        auto ADD_3168 = ADD_3165 + MUL_3133;
        auto ADD_3171 = ADD_838 + ADD_3168;
        auto MUL_3150 = MUL_2988 * 0.06;
        auto ADD_3166 = MUL_3150 + MUL_3075;
        auto ADD_3169 = ADD_3166 + MUL_3135;
        auto ADD_3172 = ADD_839 + ADD_3169;
        auto MUL_3152 = MUL_2991 * 0.06;
        auto ADD_3167 = MUL_3152 + MUL_3077;
        auto ADD_3170 = ADD_3167 + MUL_3137;
        auto ADD_3173 = ADD_840 + ADD_3170;
        if (/*panda_link0 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (637, 793)
        if (/*panda_link1 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link2 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link5 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link7*/ sphere_environment_in_collision(environment, ADD_3042, ADD_3043, ADD_3044, 0.072))
        {
            if (sphere_environment_in_collision(environment, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        auto MUL_1065 = SUB_870 * 0.9238795;
        auto MUL_1062 = ADD_865 * 0.9238795;
        auto MUL_1049 = SUB_859 * 0.9238795;
        auto MUL_1034 = ADD_853 * 0.9238795;
        auto MUL_1055 = SUB_870 * 0.3826834;
        auto SUB_1063 = MUL_1062 - MUL_1055;
        auto MUL_1072 = ADD_865 * 0.3826834;
        auto ADD_1074 = MUL_1065 + MUL_1072;
        auto MUL_1037 = SUB_859 * 0.3826834;
        auto SUB_1039 = MUL_1034 - MUL_1037;
        auto MUL_3241 = SUB_1039 * SUB_1063;
        auto MUL_1046 = ADD_853 * 0.3826834;
        auto ADD_1050 = MUL_1046 + MUL_1049;
        auto MUL_3237 = ADD_1074 * ADD_1050;
        auto ADD_3269 = MUL_3241 + MUL_3237;
        auto MUL_3271 = ADD_3269 * 2.0;
        auto MUL_931 = SUB_859 * 0.107;
        auto MUL_942 = SUB_870 * MUL_931;
        auto MUL_939 = ADD_853 * 0.107;
        auto MUL_944 = ADD_865 * MUL_939;
        auto ADD_945 = MUL_942 + MUL_944;
        auto MUL_947 = ADD_945 * 2.0;
        auto ADD_969 = ADD_838 + MUL_947;
        auto MUL_3295 = MUL_3271 * 0.022;
        auto ADD_3300 = ADD_969 + MUL_3295;
        auto MUL_3239 = ADD_1074 * SUB_1039;
        auto MUL_3242 = ADD_1050 * SUB_1063;
        auto SUB_3272 = MUL_3242 - MUL_3239;
        auto MUL_3274 = SUB_3272 * 2.0;
        auto MUL_3297 = MUL_3274 * 0.022;
        auto MUL_950 = SUB_870 * MUL_939;
        auto MUL_953 = ADD_865 * MUL_931;
        auto SUB_954 = MUL_953 - MUL_950;
        auto MUL_956 = SUB_954 * 2.0;
        auto ADD_970 = ADD_839 + MUL_956;
        auto ADD_3301 = ADD_970 + MUL_3297;
        auto MUL_3238 = SUB_1039 * SUB_1039;
        auto MUL_3234 = ADD_1050 * ADD_1050;
        auto ADD_3275 = MUL_3234 + MUL_3238;
        auto MUL_3278 = ADD_3275 * 2.0;
        auto SUB_3281 = 1.0 - MUL_3278;
        auto MUL_3299 = SUB_3281 * 0.022;
        auto MUL_961 = SUB_859 * MUL_931;
        auto MUL_959 = ADD_853 * MUL_939;
        auto ADD_962 = MUL_959 + MUL_961;
        auto MUL_965 = ADD_962 * 2.0;
        auto SUB_968 = 0.107 - MUL_965;
        auto ADD_971 = ADD_840 + SUB_968;
        auto ADD_3302 = ADD_971 + MUL_3299;
        auto MUL_3322 = MUL_3271 * 0.01;
        auto MUL_3236 = ADD_1074 * SUB_1063;
        auto MUL_3240 = SUB_1039 * ADD_1050;
        auto SUB_3256 = MUL_3240 - MUL_3236;
        auto MUL_3258 = SUB_3256 * 2.0;
        auto MUL_3311 = MUL_3258 * 0.075;
        auto SUB_3327 = MUL_3322 - MUL_3311;
        auto ADD_3330 = ADD_969 + SUB_3327;
        auto MUL_3324 = MUL_3274 * 0.01;
        auto MUL_3235 = SUB_1063 * SUB_1063;
        auto ADD_3259 = MUL_3235 + MUL_3238;
        auto MUL_3262 = ADD_3259 * 2.0;
        auto SUB_3265 = 1.0 - MUL_3262;
        auto MUL_3315 = SUB_3265 * 0.075;
        auto SUB_3328 = MUL_3324 - MUL_3315;
        auto ADD_3331 = ADD_970 + SUB_3328;
        auto ADD_3266 = MUL_3242 + MUL_3239;
        auto MUL_3326 = SUB_3281 * 0.01;
        auto MUL_3268 = ADD_3266 * 2.0;
        auto MUL_3319 = MUL_3268 * 0.075;
        auto SUB_3329 = MUL_3326 - MUL_3319;
        auto ADD_3332 = ADD_971 + SUB_3329;
        auto MUL_3341 = MUL_3258 * 0.045;
        auto SUB_3357 = MUL_3322 - MUL_3341;
        auto ADD_3360 = ADD_969 + SUB_3357;
        auto MUL_3345 = SUB_3265 * 0.045;
        auto SUB_3358 = MUL_3324 - MUL_3345;
        auto ADD_3361 = ADD_970 + SUB_3358;
        auto MUL_3349 = MUL_3268 * 0.045;
        auto SUB_3359 = MUL_3326 - MUL_3349;
        auto ADD_3362 = ADD_971 + SUB_3359;
        auto MUL_3371 = MUL_3258 * 0.015;
        auto SUB_3387 = MUL_3322 - MUL_3371;
        auto ADD_3390 = ADD_969 + SUB_3387;
        auto MUL_3375 = SUB_3265 * 0.015;
        auto SUB_3388 = MUL_3324 - MUL_3375;
        auto ADD_3391 = ADD_970 + SUB_3388;
        auto MUL_3379 = MUL_3268 * 0.015;
        auto SUB_3389 = MUL_3326 - MUL_3379;
        auto ADD_3392 = ADD_971 + SUB_3389;
        auto ADD_3411 = MUL_3371 + MUL_3322;
        auto ADD_3414 = ADD_969 + ADD_3411;
        auto ADD_3412 = MUL_3375 + MUL_3324;
        auto ADD_3415 = ADD_970 + ADD_3412;
        auto ADD_3413 = MUL_3379 + MUL_3326;
        auto ADD_3416 = ADD_971 + ADD_3413;
        auto ADD_3435 = MUL_3341 + MUL_3322;
        auto ADD_3438 = ADD_969 + ADD_3435;
        auto ADD_3436 = MUL_3345 + MUL_3324;
        auto ADD_3439 = ADD_970 + ADD_3436;
        auto ADD_3437 = MUL_3349 + MUL_3326;
        auto ADD_3440 = ADD_971 + ADD_3437;
        auto ADD_3459 = MUL_3311 + MUL_3322;
        auto ADD_3462 = ADD_969 + ADD_3459;
        auto ADD_3460 = MUL_3315 + MUL_3324;
        auto ADD_3463 = ADD_970 + ADD_3460;
        auto ADD_3461 = MUL_3319 + MUL_3326;
        auto ADD_3464 = ADD_971 + ADD_3461;
        auto MUL_3484 = MUL_3271 * 0.03;
        auto SUB_3489 = MUL_3484 - MUL_3311;
        auto ADD_3492 = ADD_969 + SUB_3489;
        auto MUL_3486 = MUL_3274 * 0.03;
        auto SUB_3490 = MUL_3486 - MUL_3315;
        auto ADD_3493 = ADD_970 + SUB_3490;
        auto MUL_3488 = SUB_3281 * 0.03;
        auto SUB_3491 = MUL_3488 - MUL_3319;
        auto ADD_3494 = ADD_971 + SUB_3491;
        auto SUB_3519 = MUL_3484 - MUL_3341;
        auto ADD_3522 = ADD_969 + SUB_3519;
        auto SUB_3520 = MUL_3486 - MUL_3345;
        auto ADD_3523 = ADD_970 + SUB_3520;
        auto SUB_3521 = MUL_3488 - MUL_3349;
        auto ADD_3524 = ADD_971 + SUB_3521;
        auto SUB_3549 = MUL_3484 - MUL_3371;
        auto ADD_3552 = ADD_969 + SUB_3549;
        auto SUB_3550 = MUL_3486 - MUL_3375;
        auto ADD_3553 = ADD_970 + SUB_3550;
        auto SUB_3551 = MUL_3488 - MUL_3379;
        auto ADD_3554 = ADD_971 + SUB_3551;
        auto ADD_3573 = MUL_3371 + MUL_3484;
        auto ADD_3576 = ADD_969 + ADD_3573;
        auto ADD_3574 = MUL_3375 + MUL_3486;
        auto ADD_3577 = ADD_970 + ADD_3574;
        auto ADD_3575 = MUL_3379 + MUL_3488;
        auto ADD_3578 = ADD_971 + ADD_3575;
        auto ADD_3597 = MUL_3341 + MUL_3484;
        auto ADD_3600 = ADD_969 + ADD_3597;
        auto ADD_3598 = MUL_3345 + MUL_3486;
        auto ADD_3601 = ADD_970 + ADD_3598;
        auto ADD_3599 = MUL_3349 + MUL_3488;
        auto ADD_3602 = ADD_971 + ADD_3599;
        auto ADD_3621 = MUL_3311 + MUL_3484;
        auto ADD_3624 = ADD_969 + ADD_3621;
        auto ADD_3622 = MUL_3315 + MUL_3486;
        auto ADD_3625 = ADD_970 + ADD_3622;
        auto ADD_3623 = MUL_3319 + MUL_3488;
        auto ADD_3626 = ADD_971 + ADD_3623;
        auto MUL_3646 = MUL_3271 * 0.05;
        auto SUB_3651 = MUL_3646 - MUL_3311;
        auto ADD_3654 = ADD_969 + SUB_3651;
        auto MUL_3648 = MUL_3274 * 0.05;
        auto SUB_3652 = MUL_3648 - MUL_3315;
        auto ADD_3655 = ADD_970 + SUB_3652;
        auto MUL_3650 = SUB_3281 * 0.05;
        auto SUB_3653 = MUL_3650 - MUL_3319;
        auto ADD_3656 = ADD_971 + SUB_3653;
        auto SUB_3681 = MUL_3646 - MUL_3341;
        auto ADD_3684 = ADD_969 + SUB_3681;
        auto SUB_3682 = MUL_3648 - MUL_3345;
        auto ADD_3685 = ADD_970 + SUB_3682;
        auto SUB_3683 = MUL_3650 - MUL_3349;
        auto ADD_3686 = ADD_971 + SUB_3683;
        auto SUB_3711 = MUL_3646 - MUL_3371;
        auto ADD_3714 = ADD_969 + SUB_3711;
        auto SUB_3712 = MUL_3648 - MUL_3375;
        auto ADD_3715 = ADD_970 + SUB_3712;
        auto SUB_3713 = MUL_3650 - MUL_3379;
        auto ADD_3716 = ADD_971 + SUB_3713;
        auto ADD_3735 = MUL_3371 + MUL_3646;
        auto ADD_3738 = ADD_969 + ADD_3735;
        auto ADD_3736 = MUL_3375 + MUL_3648;
        auto ADD_3739 = ADD_970 + ADD_3736;
        auto ADD_3737 = MUL_3379 + MUL_3650;
        auto ADD_3740 = ADD_971 + ADD_3737;
        auto ADD_3759 = MUL_3341 + MUL_3646;
        auto ADD_3762 = ADD_969 + ADD_3759;
        auto ADD_3760 = MUL_3345 + MUL_3648;
        auto ADD_3763 = ADD_970 + ADD_3760;
        auto ADD_3761 = MUL_3349 + MUL_3650;
        auto ADD_3764 = ADD_971 + ADD_3761;
        auto ADD_3783 = MUL_3311 + MUL_3646;
        auto ADD_3786 = ADD_969 + ADD_3783;
        auto ADD_3784 = MUL_3315 + MUL_3648;
        auto ADD_3787 = ADD_970 + ADD_3784;
        auto ADD_3785 = MUL_3319 + MUL_3650;
        auto ADD_3788 = ADD_971 + ADD_3785;
        if (/*panda_hand*/ sphere_environment_in_collision(environment, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_environment_in_collision(environment, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (793, 978)
        if (/*panda_link0 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        if (/*panda_link1 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        if (/*panda_link2 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        if (/*panda_link5 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_3300, ADD_3301, ADD_3302, 0.104))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3330, ADD_3331, ADD_3332, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3360, ADD_3361, ADD_3362, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3390, ADD_3391, ADD_3392, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3414, ADD_3415, ADD_3416, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3438, ADD_3439, ADD_3440, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3462, ADD_3463, ADD_3464, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3492, ADD_3493, ADD_3494, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3522, ADD_3523, ADD_3524, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3552, ADD_3553, ADD_3554, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3576, ADD_3577, ADD_3578, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3600, ADD_3601, ADD_3602, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3624, ADD_3625, ADD_3626, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3654, ADD_3655, ADD_3656, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3684, ADD_3685, ADD_3686, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3714, ADD_3715, ADD_3716, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3738, ADD_3739, ADD_3740, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3762, ADD_3763, ADD_3764, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3786, ADD_3787, ADD_3788, 0.024))
            {
                return false;
            }
        }  // (978, 978)
        auto MUL_3864 = ADD_3269 * 2.0;
        auto MUL_3851 = SUB_3256 * 2.0;
        auto MUL_1196 = SUB_1063 * 0.065;
        auto MUL_1199 = SUB_1039 * 0.065;
        auto MUL_1207 = ADD_1050 * MUL_1199;
        auto MUL_1203 = SUB_1039 * 0.0584;
        auto MUL_1209 = SUB_1063 * MUL_1203;
        auto MUL_1194 = ADD_1050 * 0.0584;
        auto SUB_1197 = MUL_1194 - MUL_1196;
        auto MUL_1206 = ADD_1074 * SUB_1197;
        auto ADD_1208 = MUL_1206 + MUL_1207;
        auto ADD_1210 = ADD_1208 + MUL_1209;
        auto MUL_1212 = ADD_1210 * 2.0;
        auto ADD_1235 = ADD_969 + MUL_1212;
        auto MUL_3888 = MUL_3864 * 0.033;
        auto MUL_3882 = MUL_3851 * 0.012;
        auto ADD_3893 = MUL_3882 + MUL_3888;
        auto ADD_3896 = ADD_1235 + ADD_3893;
        auto MUL_3867 = SUB_3272 * 2.0;
        auto MUL_3890 = MUL_3867 * 0.033;
        auto MUL_3855 = ADD_3259 * 2.0;
        auto SUB_3858 = 1.0 - MUL_3855;
        auto MUL_3884 = SUB_3858 * 0.012;
        auto ADD_3894 = MUL_3884 + MUL_3890;
        auto MUL_1215 = ADD_1074 * MUL_1203;
        auto MUL_1220 = SUB_1063 * SUB_1197;
        auto MUL_1217 = SUB_1039 * MUL_1199;
        auto ADD_1218 = MUL_1215 + MUL_1217;
        auto SUB_1221 = MUL_1220 - ADD_1218;
        auto MUL_1223 = SUB_1221 * 2.0;
        auto ADD_1225 = MUL_1223 + 0.065;
        auto ADD_1236 = ADD_970 + ADD_1225;
        auto ADD_3897 = ADD_1236 + ADD_3894;
        auto MUL_3871 = ADD_3275 * 2.0;
        auto SUB_3874 = 1.0 - MUL_3871;
        auto MUL_3892 = SUB_3874 * 0.033;
        auto MUL_3861 = ADD_3266 * 2.0;
        auto MUL_3886 = MUL_3861 * 0.012;
        auto ADD_3895 = MUL_3886 + MUL_3892;
        auto MUL_1226 = ADD_1074 * MUL_1199;
        auto MUL_1227 = SUB_1039 * MUL_1203;
        auto SUB_1228 = MUL_1226 - MUL_1227;
        auto MUL_1229 = ADD_1050 * SUB_1197;
        auto SUB_1230 = SUB_1228 - MUL_1229;
        auto MUL_1232 = SUB_1230 * 2.0;
        auto ADD_1234 = MUL_1232 + 0.0584;
        auto ADD_1237 = ADD_971 + ADD_1234;
        auto ADD_3898 = ADD_1237 + ADD_3895;
        auto MUL_3912 = MUL_3864 * 0.022;
        auto MUL_3906 = MUL_3851 * 0.015;
        auto ADD_3917 = MUL_3906 + MUL_3912;
        auto ADD_3920 = ADD_1235 + ADD_3917;
        auto MUL_3914 = MUL_3867 * 0.022;
        auto MUL_3908 = SUB_3858 * 0.015;
        auto ADD_3918 = MUL_3908 + MUL_3914;
        auto ADD_3921 = ADD_1236 + ADD_3918;
        auto MUL_3916 = SUB_3874 * 0.022;
        auto MUL_3910 = MUL_3861 * 0.015;
        auto ADD_3919 = MUL_3910 + MUL_3916;
        auto ADD_3922 = ADD_1237 + ADD_3919;
        auto MUL_3936 = MUL_3864 * 0.044;
        auto MUL_3930 = MUL_3851 * 0.008;
        auto ADD_3941 = MUL_3930 + MUL_3936;
        auto ADD_3944 = ADD_1235 + ADD_3941;
        auto MUL_3938 = MUL_3867 * 0.044;
        auto MUL_3932 = SUB_3858 * 0.008;
        auto ADD_3942 = MUL_3932 + MUL_3938;
        auto ADD_3945 = ADD_1236 + ADD_3942;
        auto MUL_3940 = SUB_3874 * 0.044;
        auto MUL_3934 = MUL_3861 * 0.008;
        auto ADD_3943 = MUL_3934 + MUL_3940;
        auto ADD_3946 = ADD_1237 + ADD_3943;
        if (/*panda_leftfinger*/ sphere_environment_in_collision(
            environment, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_environment_in_collision(environment, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (978, 1050)
        if (/*panda_link0 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        if (/*panda_link1 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        if (/*panda_link2 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        if (/*panda_link5 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_3896, ADD_3897, ADD_3898, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3920, ADD_3921, ADD_3922, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_3944, ADD_3945, ADD_3946, 0.012))
            {
                return false;
            }
        }  // (1050, 1050)
        auto ADD_1331 = MUL_1194 + MUL_1196;
        auto MUL_3990 = ADD_3269 * 2.0;
        auto MUL_4020 = MUL_3990 * 0.033;
        auto MUL_3977 = SUB_3256 * 2.0;
        auto MUL_4009 = MUL_3977 * 0.012;
        auto SUB_4025 = MUL_4020 - MUL_4009;
        auto MUL_1342 = ADD_1074 * ADD_1331;
        auto SUB_1345 = MUL_1342 - MUL_1207;
        auto ADD_1347 = SUB_1345 + MUL_1209;
        auto MUL_1349 = ADD_1347 * 2.0;
        auto ADD_1377 = ADD_969 + MUL_1349;
        auto ADD_4028 = ADD_1377 + SUB_4025;
        auto SUB_1356 = MUL_1217 - MUL_1215;
        auto MUL_3993 = SUB_3272 * 2.0;
        auto MUL_4022 = MUL_3993 * 0.033;
        auto MUL_3981 = ADD_3259 * 2.0;
        auto SUB_3984 = 1.0 - MUL_3981;
        auto MUL_4013 = SUB_3984 * 0.012;
        auto SUB_4026 = MUL_4022 - MUL_4013;
        auto MUL_1357 = SUB_1063 * ADD_1331;
        auto ADD_1358 = SUB_1356 + MUL_1357;
        auto MUL_1360 = ADD_1358 * 2.0;
        auto SUB_1363 = MUL_1360 - 0.065;
        auto ADD_1378 = ADD_970 + SUB_1363;
        auto ADD_4029 = ADD_1378 + SUB_4026;
        auto ADD_1367 = MUL_1226 + MUL_1227;
        auto MUL_3997 = ADD_3275 * 2.0;
        auto SUB_4000 = 1.0 - MUL_3997;
        auto MUL_4024 = SUB_4000 * 0.033;
        auto MUL_3987 = ADD_3266 * 2.0;
        auto MUL_4017 = MUL_3987 * 0.012;
        auto SUB_4027 = MUL_4024 - MUL_4017;
        auto MUL_1369 = ADD_1050 * ADD_1331;
        auto ADD_1370 = ADD_1367 + MUL_1369;
        auto MUL_1373 = ADD_1370 * 2.0;
        auto SUB_1376 = 0.0584 - MUL_1373;
        auto ADD_1379 = ADD_971 + SUB_1376;
        auto ADD_4030 = ADD_1379 + SUB_4027;
        auto MUL_4050 = MUL_3990 * 0.022;
        auto MUL_4039 = MUL_3977 * 0.015;
        auto SUB_4055 = MUL_4050 - MUL_4039;
        auto ADD_4058 = ADD_1377 + SUB_4055;
        auto MUL_4052 = MUL_3993 * 0.022;
        auto MUL_4043 = SUB_3984 * 0.015;
        auto SUB_4056 = MUL_4052 - MUL_4043;
        auto ADD_4059 = ADD_1378 + SUB_4056;
        auto MUL_4054 = SUB_4000 * 0.022;
        auto MUL_4047 = MUL_3987 * 0.015;
        auto SUB_4057 = MUL_4054 - MUL_4047;
        auto ADD_4060 = ADD_1379 + SUB_4057;
        auto MUL_4080 = MUL_3990 * 0.044;
        auto MUL_4069 = MUL_3977 * 0.008;
        auto SUB_4085 = MUL_4080 - MUL_4069;
        auto ADD_4088 = ADD_1377 + SUB_4085;
        auto MUL_4082 = MUL_3993 * 0.044;
        auto MUL_4073 = SUB_3984 * 0.008;
        auto SUB_4086 = MUL_4082 - MUL_4073;
        auto ADD_4089 = ADD_1378 + SUB_4086;
        auto MUL_4084 = SUB_4000 * 0.044;
        auto MUL_4077 = MUL_3987 * 0.008;
        auto SUB_4087 = MUL_4084 - MUL_4077;
        auto ADD_4090 = ADD_1379 + SUB_4087;
        if (/*panda_link0 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1050, 1112)
        if (/*panda_link1 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_1641, NEGATE_1643, 0.248, 0.154, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1656, NEGATE_1660, 0.333, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1680, NEGATE_1684, 0.333, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)
        if (/*panda_link2 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1832, SUB_1833, ADD_1835, 0.154, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1849, MUL_1851, ADD_1854, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1868, MUL_1870, ADD_1873, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1882, NEGATE_1886, SUB_1897, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1906, NEGATE_1910, SUB_1921, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)
        if (/*panda_link5 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2402, ADD_2403, ADD_2404, 0.176, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2423, ADD_2424, ADD_2425, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2444, ADD_2445, ADD_2446, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2471, SUB_2472, SUB_2473, 0.06, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2501, ADD_2502, ADD_2503, 0.05, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2534, ADD_2535, ADD_2536, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2567, ADD_2568, ADD_2569, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2600, ADD_2601, ADD_2602, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2633, ADD_2634, ADD_2635, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2672, ADD_2673, ADD_2674, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2711, ADD_2712, ADD_2713, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2750, ADD_2751, ADD_2752, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2789, ADD_2790, ADD_2791, 0.025, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)
        if (/*panda_rightfinger*/ sphere_environment_in_collision(
            environment, ADD_4028, ADD_4029, ADD_4030, 0.024))
        {
            if (sphere_environment_in_collision(environment, ADD_4058, ADD_4059, ADD_4060, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4088, ADD_4089, ADD_4090, 0.012))
            {
                return false;
            }
        }  // (1112, 1112)

        return true;
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk_attachment(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        // ignore collision of static link
        // if(/*panda_link0*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.05, 0.08)){ return
        // false; } // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_8 = INPUT_0 * 0.5;
        auto SIN_9 = DIV_8.sin();
        auto COS_15 = DIV_8.cos();
        auto MUL_1575 = COS_15 * SIN_9;
        auto MUL_1574 = SIN_9 * SIN_9;
        auto MUL_1594 = MUL_1575 * 2.0;
        auto MUL_1620 = MUL_1594 * 0.0265023;
        auto MUL_1584 = MUL_1574 * 2.0;
        auto SUB_1587 = 1.0 - MUL_1584;
        auto MUL_1613 = SUB_1587 * 4.21e-05;
        auto ADD_1636 = MUL_1613 + MUL_1620;
        auto MUL_1623 = SUB_1587 * 0.0265023;
        auto MUL_1615 = MUL_1594 * 4.21e-05;
        auto SUB_1637 = MUL_1615 - MUL_1623;
        auto MUL_1650 = MUL_1594 * 0.08;
        auto MUL_1653 = SUB_1587 * 0.08;
        auto NEGATE_1654 = -MUL_1653;
        auto MUL_1674 = MUL_1594 * 0.03;
        auto MUL_1677 = SUB_1587 * 0.03;
        auto NEGATE_1678 = -MUL_1677;
        if (/*panda_link1*/ sphere_environment_in_collision(
            environment, ADD_1636, SUB_1637, 0.2598976, 0.144259))
        {
            if (sphere_environment_in_collision(environment, MUL_1650, NEGATE_1654, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1674, NEGATE_1678, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0, 0.0, 0.213, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0, 0.0, 0.163, 0.06))
            {
                return false;
            }
        }  // (0, 21)
        auto MUL_74 = COS_15 * 0.7071068;
        auto MUL_72 = SIN_9 * 0.7071068;
        auto INPUT_1 = q[1];
        auto DIV_117 = INPUT_1 * 0.5;
        auto SIN_118 = DIV_117.sin();
        auto COS_124 = DIV_117.cos();
        auto MUL_143 = MUL_72 * COS_124;
        auto MUL_128 = MUL_72 * SIN_118;
        auto MUL_126 = MUL_74 * COS_124;
        auto SUB_149 = MUL_126 - MUL_128;
        auto ADD_130 = MUL_126 + MUL_128;
        auto MUL_140 = MUL_74 * SIN_118;
        auto SUB_138 = MUL_140 - MUL_143;
        auto ADD_144 = MUL_140 + MUL_143;
        auto MUL_1750 = SUB_149 * ADD_144;
        auto MUL_1751 = SUB_149 * SUB_138;
        auto MUL_1749 = ADD_144 * ADD_144;
        auto MUL_1748 = SUB_138 * SUB_138;
        auto ADD_1760 = MUL_1748 + MUL_1749;
        auto MUL_1763 = ADD_1760 * 2.0;
        auto SUB_1766 = 1.0 - MUL_1763;
        auto MUL_1805 = SUB_1766 * 2.72e-05;
        auto MUL_1757 = ADD_130 * ADD_144;
        auto SUB_1790 = MUL_1751 - MUL_1757;
        auto MUL_1792 = SUB_1790 * 2.0;
        auto MUL_1826 = MUL_1792 * 0.0265382;
        auto MUL_1755 = ADD_130 * SUB_138;
        auto ADD_1775 = MUL_1755 + MUL_1750;
        auto MUL_1778 = ADD_1775 * 2.0;
        auto MUL_1816 = MUL_1778 * 0.074083;
        auto SUB_1831 = MUL_1816 - MUL_1805;
        auto ADD_1835 = SUB_1831 + MUL_1826;
        auto SUB_1767 = MUL_1750 - MUL_1755;
        auto MUL_1769 = SUB_1767 * 2.0;
        auto MUL_1809 = MUL_1769 * 2.72e-05;
        auto MUL_1753 = SUB_149 * ADD_130;
        auto MUL_1759 = SUB_138 * ADD_144;
        auto ADD_1793 = MUL_1759 + MUL_1753;
        auto MUL_1795 = ADD_1793 * 2.0;
        auto MUL_1828 = MUL_1795 * 0.0265382;
        auto MUL_1752 = ADD_130 * ADD_130;
        auto ADD_1780 = MUL_1749 + MUL_1752;
        auto MUL_1783 = ADD_1780 * 2.0;
        auto SUB_1786 = 1.0 - MUL_1783;
        auto MUL_1819 = SUB_1786 * 0.074083;
        auto ADD_1832 = MUL_1809 + MUL_1819;
        auto SUB_1836 = MUL_1828 - ADD_1832;
        auto SUB_1787 = MUL_1759 - MUL_1753;
        auto ADD_1770 = MUL_1757 + MUL_1751;
        auto ADD_1796 = MUL_1748 + MUL_1752;
        auto MUL_1799 = ADD_1796 * 2.0;
        auto SUB_1802 = 1.0 - MUL_1799;
        auto MUL_1830 = SUB_1802 * 0.0265382;
        auto MUL_1789 = SUB_1787 * 2.0;
        auto MUL_1823 = MUL_1789 * 0.074083;
        auto MUL_1773 = ADD_1770 * 2.0;
        auto MUL_1813 = MUL_1773 * 2.72e-05;
        auto SUB_1834 = MUL_1813 - MUL_1823;
        auto ADD_1837 = SUB_1834 + MUL_1830;
        auto ADD_1838 = 0.333 + ADD_1837;
        auto MUL_1852 = MUL_1792 * 0.03;
        auto MUL_1854 = MUL_1795 * 0.03;
        auto MUL_1856 = SUB_1802 * 0.03;
        auto ADD_1857 = 0.333 + MUL_1856;
        auto MUL_1871 = MUL_1792 * 0.08;
        auto MUL_1873 = MUL_1795 * 0.08;
        auto MUL_1875 = SUB_1802 * 0.08;
        auto ADD_1876 = 0.333 + MUL_1875;
        auto MUL_1885 = MUL_1778 * 0.12;
        auto MUL_1888 = SUB_1786 * 0.12;
        auto NEGATE_1889 = -MUL_1888;
        auto MUL_1892 = MUL_1789 * 0.12;
        auto SUB_1900 = 0.333 - MUL_1892;
        auto MUL_1909 = MUL_1778 * 0.17;
        auto MUL_1912 = SUB_1786 * 0.17;
        auto NEGATE_1913 = -MUL_1912;
        auto MUL_1916 = MUL_1789 * 0.17;
        auto SUB_1924 = 0.333 - MUL_1916;
        if (/*panda_link2*/ sphere_environment_in_collision(
            environment, ADD_1835, SUB_1836, ADD_1838, 0.145067))
        {
            if (sphere_environment_in_collision(environment, MUL_1852, MUL_1854, ADD_1857, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1871, MUL_1873, ADD_1876, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1885, NEGATE_1889, SUB_1900, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_1909, NEGATE_1913, SUB_1924, 0.06))
            {
                return false;
            }
        }  // (21, 99)
        auto MUL_182 = SUB_149 * 0.7071068;
        auto MUL_198 = ADD_144 * 0.7071068;
        auto MUL_196 = SUB_138 * 0.7071068;
        auto SUB_209 = MUL_198 - MUL_196;
        auto ADD_199 = MUL_196 + MUL_198;
        auto MUL_184 = ADD_130 * 0.7071068;
        auto SUB_186 = MUL_182 - MUL_184;
        auto ADD_215 = MUL_182 + MUL_184;
        auto MUL_224 = ADD_144 * 0.316;
        auto MUL_235 = SUB_149 * MUL_224;
        auto MUL_228 = ADD_130 * 0.316;
        auto MUL_236 = SUB_138 * MUL_228;
        auto ADD_237 = MUL_235 + MUL_236;
        auto MUL_240 = ADD_237 * 2.0;
        auto INPUT_2 = q[2];
        auto DIV_262 = INPUT_2 * 0.5;
        auto SIN_263 = DIV_262.sin();
        auto COS_269 = DIV_262.cos();
        auto MUL_286 = ADD_215 * COS_269;
        auto MUL_281 = ADD_215 * SIN_263;
        auto MUL_284 = SUB_209 * COS_269;
        auto ADD_285 = MUL_281 + MUL_284;
        auto MUL_1936 = ADD_285 * ADD_285;
        auto MUL_289 = SUB_209 * SIN_263;
        auto SUB_290 = MUL_286 - MUL_289;
        auto MUL_1937 = SUB_290 * ADD_285;
        auto MUL_271 = SUB_186 * COS_269;
        auto MUL_276 = SUB_186 * SIN_263;
        auto MUL_278 = ADD_199 * COS_269;
        auto SUB_279 = MUL_278 - MUL_276;
        auto MUL_1938 = SUB_290 * SUB_279;
        auto MUL_1935 = SUB_279 * SUB_279;
        auto ADD_1944 = MUL_1935 + MUL_1936;
        auto MUL_1947 = ADD_1944 * 2.0;
        auto SUB_1950 = 1.0 - MUL_1947;
        auto MUL_1984 = SUB_1950 * 0.0404726;
        auto MUL_272 = ADD_199 * SIN_263;
        auto ADD_273 = MUL_271 + MUL_272;
        auto MUL_1942 = ADD_273 * ADD_285;
        auto ADD_1970 = MUL_1942 + MUL_1938;
        auto MUL_1972 = ADD_1970 * 2.0;
        auto MUL_1997 = MUL_1972 * 0.0439448;
        auto MUL_1941 = ADD_273 * SUB_279;
        auto SUB_1957 = MUL_1941 - MUL_1937;
        auto MUL_1959 = SUB_1957 * 2.0;
        auto MUL_1990 = MUL_1959 * 0.0229569;
        auto ADD_2007 = MUL_1984 + MUL_1990;
        auto SUB_2010 = ADD_2007 - MUL_1997;
        auto ADD_2013 = MUL_240 + SUB_2010;
        auto ADD_1951 = MUL_1941 + MUL_1937;
        auto MUL_1953 = ADD_1951 * 2.0;
        auto MUL_1986 = MUL_1953 * 0.0404726;
        auto MUL_1940 = SUB_290 * ADD_273;
        auto MUL_1943 = SUB_279 * ADD_285;
        auto SUB_1973 = MUL_1943 - MUL_1940;
        auto MUL_1975 = SUB_1973 * 2.0;
        auto MUL_2001 = MUL_1975 * 0.0439448;
        auto MUL_1939 = ADD_273 * ADD_273;
        auto ADD_1960 = MUL_1936 + MUL_1939;
        auto MUL_1963 = ADD_1960 * 2.0;
        auto SUB_1966 = 1.0 - MUL_1963;
        auto MUL_1992 = SUB_1966 * 0.0229569;
        auto ADD_2008 = MUL_1986 + MUL_1992;
        auto SUB_2011 = ADD_2008 - MUL_2001;
        auto MUL_246 = ADD_144 * MUL_224;
        auto MUL_244 = ADD_130 * MUL_228;
        auto ADD_247 = MUL_244 + MUL_246;
        auto MUL_249 = ADD_247 * 2.0;
        auto SUB_252 = MUL_249 - 0.316;
        auto ADD_2014 = SUB_252 + SUB_2011;
        auto SUB_1954 = MUL_1942 - MUL_1938;
        auto ADD_1967 = MUL_1943 + MUL_1940;
        auto ADD_1976 = MUL_1935 + MUL_1939;
        auto MUL_1979 = ADD_1976 * 2.0;
        auto SUB_1982 = 1.0 - MUL_1979;
        auto MUL_2005 = SUB_1982 * 0.0439448;
        auto MUL_1969 = ADD_1967 * 2.0;
        auto MUL_1994 = MUL_1969 * 0.0229569;
        auto MUL_1956 = SUB_1954 * 2.0;
        auto MUL_1988 = MUL_1956 * 0.0404726;
        auto ADD_2009 = MUL_1988 + MUL_1994;
        auto SUB_2012 = ADD_2009 - MUL_2005;
        auto MUL_253 = SUB_149 * MUL_228;
        auto MUL_255 = SUB_138 * MUL_224;
        auto SUB_256 = MUL_253 - MUL_255;
        auto MUL_258 = SUB_256 * 2.0;
        auto ADD_260 = 0.333 + MUL_258;
        auto ADD_2015 = ADD_260 + SUB_2012;
        auto MUL_2030 = MUL_1972 * 0.1;
        auto SUB_2040 = MUL_240 - MUL_2030;
        auto MUL_2034 = MUL_1975 * 0.1;
        auto SUB_2041 = SUB_252 - MUL_2034;
        auto MUL_2038 = SUB_1982 * 0.1;
        auto SUB_2042 = ADD_260 - MUL_2038;
        auto MUL_2057 = MUL_1972 * 0.06;
        auto SUB_2067 = MUL_240 - MUL_2057;
        auto MUL_2061 = MUL_1975 * 0.06;
        auto SUB_2068 = SUB_252 - MUL_2061;
        auto MUL_2065 = SUB_1982 * 0.06;
        auto SUB_2069 = ADD_260 - MUL_2065;
        auto MUL_2077 = MUL_1959 * 0.06;
        auto MUL_2071 = SUB_1950 * 0.08;
        auto ADD_2088 = MUL_2071 + MUL_2077;
        auto ADD_2091 = MUL_240 + ADD_2088;
        auto MUL_2079 = SUB_1966 * 0.06;
        auto MUL_2073 = MUL_1953 * 0.08;
        auto ADD_2089 = MUL_2073 + MUL_2079;
        auto ADD_2092 = SUB_252 + ADD_2089;
        auto MUL_2081 = MUL_1969 * 0.06;
        auto MUL_2075 = MUL_1956 * 0.08;
        auto ADD_2090 = MUL_2075 + MUL_2081;
        auto ADD_2093 = ADD_260 + ADD_2090;
        auto MUL_2101 = MUL_1959 * 0.02;
        auto ADD_2112 = MUL_2071 + MUL_2101;
        auto ADD_2115 = MUL_240 + ADD_2112;
        auto MUL_2103 = SUB_1966 * 0.02;
        auto ADD_2113 = MUL_2073 + MUL_2103;
        auto ADD_2116 = SUB_252 + ADD_2113;
        auto MUL_2105 = MUL_1969 * 0.02;
        auto ADD_2114 = MUL_2075 + MUL_2105;
        auto ADD_2117 = ADD_260 + ADD_2114;
        if (/*panda_link3*/ sphere_environment_in_collision(
            environment, ADD_2013, ADD_2014, ADD_2015, 0.127656))
        {
            if (sphere_environment_in_collision(environment, SUB_2040, SUB_2041, SUB_2042, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2067, SUB_2068, SUB_2069, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2091, ADD_2092, ADD_2093, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2115, ADD_2116, ADD_2117, 0.055))
            {
                return false;
            }
        }  // (99, 220)
        auto MUL_323 = SUB_290 * 0.7071068;
        auto MUL_338 = ADD_285 * 0.7071068;
        auto MUL_336 = SUB_279 * 0.7071068;
        auto SUB_349 = MUL_338 - MUL_336;
        auto ADD_339 = MUL_336 + MUL_338;
        auto MUL_325 = ADD_273 * 0.7071068;
        auto SUB_354 = MUL_323 - MUL_325;
        auto ADD_326 = MUL_323 + MUL_325;
        auto MUL_371 = ADD_285 * 0.0825;
        auto MUL_376 = ADD_285 * MUL_371;
        auto MUL_366 = SUB_279 * 0.0825;
        auto MUL_374 = SUB_279 * MUL_366;
        auto ADD_378 = MUL_374 + MUL_376;
        auto MUL_381 = ADD_378 * 2.0;
        auto SUB_384 = 0.0825 - MUL_381;
        auto ADD_403 = MUL_240 + SUB_384;
        auto INPUT_3 = q[3];
        auto DIV_407 = INPUT_3 * 0.5;
        auto SIN_408 = DIV_407.sin();
        auto COS_414 = DIV_407.cos();
        auto MUL_431 = SUB_354 * COS_414;
        auto MUL_426 = SUB_354 * SIN_408;
        auto MUL_429 = SUB_349 * COS_414;
        auto ADD_430 = MUL_426 + MUL_429;
        auto MUL_2129 = ADD_430 * ADD_430;
        auto MUL_434 = SUB_349 * SIN_408;
        auto SUB_435 = MUL_431 - MUL_434;
        auto MUL_2130 = SUB_435 * ADD_430;
        auto MUL_416 = ADD_326 * COS_414;
        auto MUL_421 = ADD_326 * SIN_408;
        auto MUL_423 = ADD_339 * COS_414;
        auto SUB_424 = MUL_423 - MUL_421;
        auto MUL_2131 = SUB_435 * SUB_424;
        auto MUL_2128 = SUB_424 * SUB_424;
        auto ADD_2137 = MUL_2128 + MUL_2129;
        auto MUL_2140 = ADD_2137 * 2.0;
        auto SUB_2143 = 1.0 - MUL_2140;
        auto MUL_2178 = SUB_2143 * 0.0422068;
        auto MUL_417 = ADD_339 * SIN_408;
        auto ADD_418 = MUL_416 + MUL_417;
        auto MUL_2135 = ADD_418 * ADD_430;
        auto ADD_2163 = MUL_2135 + MUL_2131;
        auto MUL_2165 = ADD_2163 * 2.0;
        auto MUL_2195 = MUL_2165 * 0.0226917;
        auto MUL_2134 = ADD_418 * SUB_424;
        auto SUB_2150 = MUL_2134 - MUL_2130;
        auto MUL_2152 = SUB_2150 * 2.0;
        auto MUL_2189 = MUL_2152 * 0.0449876;
        auto SUB_2200 = MUL_2189 - MUL_2178;
        auto ADD_2203 = SUB_2200 + MUL_2195;
        auto ADD_2206 = ADD_403 + ADD_2203;
        auto ADD_2144 = MUL_2134 + MUL_2130;
        auto MUL_2146 = ADD_2144 * 2.0;
        auto MUL_2182 = MUL_2146 * 0.0422068;
        auto MUL_2133 = SUB_435 * ADD_418;
        auto MUL_2136 = SUB_424 * ADD_430;
        auto SUB_2166 = MUL_2136 - MUL_2133;
        auto MUL_2168 = SUB_2166 * 2.0;
        auto MUL_2197 = MUL_2168 * 0.0226917;
        auto MUL_2132 = ADD_418 * ADD_418;
        auto ADD_2153 = MUL_2129 + MUL_2132;
        auto MUL_2156 = ADD_2153 * 2.0;
        auto SUB_2159 = 1.0 - MUL_2156;
        auto MUL_2191 = SUB_2159 * 0.0449876;
        auto SUB_2201 = MUL_2191 - MUL_2182;
        auto ADD_2204 = SUB_2201 + MUL_2197;
        auto MUL_386 = SUB_290 * MUL_371;
        auto MUL_387 = ADD_273 * MUL_366;
        auto ADD_389 = MUL_386 + MUL_387;
        auto MUL_392 = ADD_389 * 2.0;
        auto ADD_404 = SUB_252 + MUL_392;
        auto ADD_2207 = ADD_404 + ADD_2204;
        auto SUB_2147 = MUL_2135 - MUL_2131;
        auto ADD_2160 = MUL_2136 + MUL_2133;
        auto ADD_2169 = MUL_2128 + MUL_2132;
        auto MUL_2172 = ADD_2169 * 2.0;
        auto SUB_2175 = 1.0 - MUL_2172;
        auto MUL_2199 = SUB_2175 * 0.0226917;
        auto MUL_2162 = ADD_2160 * 2.0;
        auto MUL_2193 = MUL_2162 * 0.0449876;
        auto MUL_2149 = SUB_2147 * 2.0;
        auto MUL_2186 = MUL_2149 * 0.0422068;
        auto SUB_2202 = MUL_2193 - MUL_2186;
        auto ADD_2205 = SUB_2202 + MUL_2199;
        auto MUL_394 = SUB_290 * MUL_366;
        auto MUL_396 = ADD_273 * MUL_371;
        auto SUB_398 = MUL_396 - MUL_394;
        auto MUL_401 = SUB_398 * 2.0;
        auto ADD_405 = ADD_260 + MUL_401;
        auto ADD_2208 = ADD_405 + ADD_2205;
        auto MUL_2211 = SUB_2143 * 0.08;
        auto MUL_2222 = MUL_2152 * 0.095;
        auto SUB_2233 = MUL_2222 - MUL_2211;
        auto ADD_2236 = ADD_403 + SUB_2233;
        auto MUL_2224 = SUB_2159 * 0.095;
        auto MUL_2215 = MUL_2146 * 0.08;
        auto SUB_2234 = MUL_2224 - MUL_2215;
        auto ADD_2237 = ADD_404 + SUB_2234;
        auto MUL_2226 = MUL_2162 * 0.095;
        auto MUL_2219 = MUL_2149 * 0.08;
        auto SUB_2235 = MUL_2226 - MUL_2219;
        auto ADD_2238 = ADD_405 + SUB_2235;
        auto MUL_2252 = MUL_2165 * 0.02;
        auto ADD_2257 = ADD_403 + MUL_2252;
        auto MUL_2254 = MUL_2168 * 0.02;
        auto ADD_2258 = ADD_404 + MUL_2254;
        auto MUL_2256 = SUB_2175 * 0.02;
        auto ADD_2259 = ADD_405 + MUL_2256;
        auto MUL_2273 = MUL_2165 * 0.06;
        auto ADD_2278 = ADD_403 + MUL_2273;
        auto MUL_2275 = MUL_2168 * 0.06;
        auto ADD_2279 = ADD_404 + MUL_2275;
        auto MUL_2277 = SUB_2175 * 0.06;
        auto ADD_2280 = ADD_405 + MUL_2277;
        auto MUL_2294 = MUL_2152 * 0.06;
        auto SUB_2305 = MUL_2294 - MUL_2211;
        auto ADD_2308 = ADD_403 + SUB_2305;
        auto MUL_2296 = SUB_2159 * 0.06;
        auto SUB_2306 = MUL_2296 - MUL_2215;
        auto ADD_2309 = ADD_404 + SUB_2306;
        auto MUL_2298 = MUL_2162 * 0.06;
        auto SUB_2307 = MUL_2298 - MUL_2219;
        auto ADD_2310 = ADD_405 + SUB_2307;
        if (/*panda_link4*/ sphere_environment_in_collision(
            environment, ADD_2206, ADD_2207, ADD_2208, 0.12849))
        {
            if (sphere_environment_in_collision(environment, ADD_2236, ADD_2237, ADD_2238, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2257, ADD_2258, ADD_2259, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2278, ADD_2279, ADD_2280, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2308, ADD_2309, ADD_2310, 0.055))
            {
                return false;
            }
        }  // (220, 343)
        auto MUL_469 = SUB_435 * 0.7071068;
        auto MUL_486 = ADD_430 * 0.7071068;
        auto MUL_527 = ADD_430 * 0.0825;
        auto MUL_533 = ADD_430 * MUL_527;
        auto MUL_483 = SUB_424 * 0.7071068;
        auto SUB_488 = MUL_483 - MUL_486;
        auto ADD_499 = MUL_483 + MUL_486;
        auto MUL_520 = SUB_424 * 0.0825;
        auto MUL_472 = ADD_418 * 0.7071068;
        auto SUB_473 = MUL_472 - MUL_469;
        auto ADD_506 = MUL_469 + MUL_472;
        auto MUL_514 = ADD_430 * 0.384;
        auto MUL_529 = SUB_435 * MUL_514;
        auto MUL_517 = ADD_418 * 0.384;
        auto ADD_522 = MUL_517 + MUL_520;
        auto MUL_531 = SUB_424 * ADD_522;
        auto SUB_532 = MUL_531 - MUL_529;
        auto ADD_534 = SUB_532 + MUL_533;
        auto MUL_536 = ADD_534 * 2.0;
        auto SUB_539 = MUL_536 - 0.0825;
        auto ADD_564 = ADD_403 + SUB_539;
        auto INPUT_4 = q[4];
        auto DIV_568 = INPUT_4 * 0.5;
        auto SIN_569 = DIV_568.sin();
        auto COS_575 = DIV_568.cos();
        auto MUL_592 = ADD_506 * COS_575;
        auto MUL_587 = ADD_506 * SIN_569;
        auto MUL_590 = ADD_499 * COS_575;
        auto ADD_591 = MUL_587 + MUL_590;
        auto MUL_2322 = ADD_591 * ADD_591;
        auto MUL_595 = ADD_499 * SIN_569;
        auto SUB_596 = MUL_592 - MUL_595;
        auto MUL_2323 = SUB_596 * ADD_591;
        auto MUL_577 = SUB_473 * COS_575;
        auto MUL_582 = SUB_473 * SIN_569;
        auto MUL_584 = SUB_488 * COS_575;
        auto SUB_585 = MUL_584 - MUL_582;
        auto MUL_2324 = SUB_596 * SUB_585;
        auto MUL_2321 = SUB_585 * SUB_585;
        auto ADD_2330 = MUL_2321 + MUL_2322;
        auto MUL_2333 = ADD_2330 * 2.0;
        auto SUB_2336 = 1.0 - MUL_2333;
        auto MUL_2370 = SUB_2336 * 3.14e-05;
        auto MUL_578 = SUB_488 * SIN_569;
        auto ADD_579 = MUL_577 + MUL_578;
        auto MUL_2328 = ADD_579 * ADD_591;
        auto ADD_2356 = MUL_2328 + MUL_2324;
        auto MUL_2358 = ADD_2356 * 2.0;
        auto MUL_2383 = MUL_2358 * 0.1065559;
        auto MUL_2327 = ADD_579 * SUB_585;
        auto SUB_2343 = MUL_2327 - MUL_2323;
        auto MUL_2345 = SUB_2343 * 2.0;
        auto MUL_2376 = MUL_2345 * 0.027819;
        auto ADD_2393 = MUL_2370 + MUL_2376;
        auto SUB_2396 = ADD_2393 - MUL_2383;
        auto ADD_2399 = ADD_564 + SUB_2396;
        auto ADD_2337 = MUL_2327 + MUL_2323;
        auto MUL_2339 = ADD_2337 * 2.0;
        auto MUL_2372 = MUL_2339 * 3.14e-05;
        auto MUL_2326 = SUB_596 * ADD_579;
        auto MUL_2329 = SUB_585 * ADD_591;
        auto SUB_2359 = MUL_2329 - MUL_2326;
        auto MUL_2361 = SUB_2359 * 2.0;
        auto MUL_2387 = MUL_2361 * 0.1065559;
        auto MUL_2325 = ADD_579 * ADD_579;
        auto ADD_2346 = MUL_2322 + MUL_2325;
        auto MUL_2349 = ADD_2346 * 2.0;
        auto SUB_2352 = 1.0 - MUL_2349;
        auto MUL_2378 = SUB_2352 * 0.027819;
        auto ADD_2394 = MUL_2372 + MUL_2378;
        auto SUB_2397 = ADD_2394 - MUL_2387;
        auto MUL_541 = SUB_435 * MUL_527;
        auto MUL_546 = ADD_430 * MUL_514;
        auto MUL_543 = ADD_418 * ADD_522;
        auto ADD_544 = MUL_541 + MUL_543;
        auto ADD_548 = ADD_544 + MUL_546;
        auto MUL_551 = ADD_548 * 2.0;
        auto SUB_554 = 0.384 - MUL_551;
        auto ADD_565 = ADD_404 + SUB_554;
        auto ADD_2400 = ADD_565 + SUB_2397;
        auto SUB_2340 = MUL_2328 - MUL_2324;
        auto ADD_2353 = MUL_2329 + MUL_2326;
        auto ADD_2362 = MUL_2321 + MUL_2325;
        auto MUL_2365 = ADD_2362 * 2.0;
        auto SUB_2368 = 1.0 - MUL_2365;
        auto MUL_2391 = SUB_2368 * 0.1065559;
        auto MUL_2355 = ADD_2353 * 2.0;
        auto MUL_2380 = MUL_2355 * 0.027819;
        auto MUL_2342 = SUB_2340 * 2.0;
        auto MUL_2374 = MUL_2342 * 3.14e-05;
        auto ADD_2395 = MUL_2374 + MUL_2380;
        auto SUB_2398 = ADD_2395 - MUL_2391;
        auto MUL_555 = SUB_435 * ADD_522;
        auto MUL_558 = SUB_424 * MUL_514;
        auto MUL_556 = ADD_418 * MUL_527;
        auto SUB_557 = MUL_555 - MUL_556;
        auto ADD_560 = SUB_557 + MUL_558;
        auto MUL_562 = ADD_560 * 2.0;
        auto ADD_566 = ADD_405 + MUL_562;
        auto ADD_2401 = ADD_566 + SUB_2398;
        auto MUL_2409 = MUL_2345 * 0.055;
        auto ADD_2420 = ADD_564 + MUL_2409;
        auto MUL_2411 = SUB_2352 * 0.055;
        auto ADD_2421 = ADD_565 + MUL_2411;
        auto MUL_2413 = MUL_2355 * 0.055;
        auto ADD_2422 = ADD_566 + MUL_2413;
        auto MUL_2430 = MUL_2345 * 0.075;
        auto ADD_2441 = ADD_564 + MUL_2430;
        auto MUL_2432 = SUB_2352 * 0.075;
        auto ADD_2442 = ADD_565 + MUL_2432;
        auto MUL_2434 = MUL_2355 * 0.075;
        auto ADD_2443 = ADD_566 + MUL_2434;
        auto MUL_2458 = MUL_2358 * 0.22;
        auto SUB_2468 = ADD_564 - MUL_2458;
        auto MUL_2462 = MUL_2361 * 0.22;
        auto SUB_2469 = ADD_565 - MUL_2462;
        auto MUL_2466 = SUB_2368 * 0.22;
        auto SUB_2470 = ADD_566 - MUL_2466;
        auto MUL_2485 = MUL_2358 * 0.18;
        auto MUL_2478 = MUL_2345 * 0.05;
        auto SUB_2495 = MUL_2478 - MUL_2485;
        auto ADD_2498 = ADD_564 + SUB_2495;
        auto MUL_2489 = MUL_2361 * 0.18;
        auto MUL_2480 = SUB_2352 * 0.05;
        auto SUB_2496 = MUL_2480 - MUL_2489;
        auto ADD_2499 = ADD_565 + SUB_2496;
        auto MUL_2493 = SUB_2368 * 0.18;
        auto MUL_2482 = MUL_2355 * 0.05;
        auto SUB_2497 = MUL_2482 - MUL_2493;
        auto ADD_2500 = ADD_566 + SUB_2497;
        auto MUL_2508 = MUL_2345 * 0.08;
        auto MUL_2515 = MUL_2358 * 0.14;
        auto MUL_2502 = SUB_2336 * 0.01;
        auto ADD_2525 = MUL_2502 + MUL_2508;
        auto SUB_2528 = ADD_2525 - MUL_2515;
        auto ADD_2531 = ADD_564 + SUB_2528;
        auto MUL_2519 = MUL_2361 * 0.14;
        auto MUL_2510 = SUB_2352 * 0.08;
        auto MUL_2504 = MUL_2339 * 0.01;
        auto ADD_2526 = MUL_2504 + MUL_2510;
        auto SUB_2529 = ADD_2526 - MUL_2519;
        auto ADD_2532 = ADD_565 + SUB_2529;
        auto MUL_2523 = SUB_2368 * 0.14;
        auto MUL_2512 = MUL_2355 * 0.08;
        auto MUL_2506 = MUL_2342 * 0.01;
        auto ADD_2527 = MUL_2506 + MUL_2512;
        auto SUB_2530 = ADD_2527 - MUL_2523;
        auto ADD_2533 = ADD_566 + SUB_2530;
        auto MUL_2548 = MUL_2358 * 0.11;
        auto MUL_2541 = MUL_2345 * 0.085;
        auto ADD_2558 = MUL_2502 + MUL_2541;
        auto SUB_2561 = ADD_2558 - MUL_2548;
        auto ADD_2564 = ADD_564 + SUB_2561;
        auto MUL_2552 = MUL_2361 * 0.11;
        auto MUL_2543 = SUB_2352 * 0.085;
        auto ADD_2559 = MUL_2504 + MUL_2543;
        auto SUB_2562 = ADD_2559 - MUL_2552;
        auto ADD_2565 = ADD_565 + SUB_2562;
        auto MUL_2556 = SUB_2368 * 0.11;
        auto MUL_2545 = MUL_2355 * 0.085;
        auto ADD_2560 = MUL_2506 + MUL_2545;
        auto SUB_2563 = ADD_2560 - MUL_2556;
        auto ADD_2566 = ADD_566 + SUB_2563;
        auto MUL_2581 = MUL_2358 * 0.08;
        auto MUL_2574 = MUL_2345 * 0.09;
        auto ADD_2591 = MUL_2502 + MUL_2574;
        auto SUB_2594 = ADD_2591 - MUL_2581;
        auto ADD_2597 = ADD_564 + SUB_2594;
        auto MUL_2585 = MUL_2361 * 0.08;
        auto MUL_2576 = SUB_2352 * 0.09;
        auto ADD_2592 = MUL_2504 + MUL_2576;
        auto SUB_2595 = ADD_2592 - MUL_2585;
        auto ADD_2598 = ADD_565 + SUB_2595;
        auto MUL_2589 = SUB_2368 * 0.08;
        auto MUL_2578 = MUL_2355 * 0.09;
        auto ADD_2593 = MUL_2506 + MUL_2578;
        auto SUB_2596 = ADD_2593 - MUL_2589;
        auto ADD_2599 = ADD_566 + SUB_2596;
        auto MUL_2614 = MUL_2358 * 0.05;
        auto MUL_2607 = MUL_2345 * 0.095;
        auto ADD_2624 = MUL_2502 + MUL_2607;
        auto SUB_2627 = ADD_2624 - MUL_2614;
        auto ADD_2630 = ADD_564 + SUB_2627;
        auto MUL_2618 = MUL_2361 * 0.05;
        auto MUL_2609 = SUB_2352 * 0.095;
        auto ADD_2625 = MUL_2504 + MUL_2609;
        auto SUB_2628 = ADD_2625 - MUL_2618;
        auto ADD_2631 = ADD_565 + SUB_2628;
        auto MUL_2622 = SUB_2368 * 0.05;
        auto MUL_2611 = MUL_2355 * 0.095;
        auto ADD_2626 = MUL_2506 + MUL_2611;
        auto SUB_2629 = ADD_2626 - MUL_2622;
        auto ADD_2632 = ADD_566 + SUB_2629;
        auto SUB_2663 = MUL_2508 - MUL_2502;
        auto SUB_2666 = SUB_2663 - MUL_2515;
        auto ADD_2669 = ADD_564 + SUB_2666;
        auto SUB_2664 = MUL_2510 - MUL_2504;
        auto SUB_2667 = SUB_2664 - MUL_2519;
        auto ADD_2670 = ADD_565 + SUB_2667;
        auto SUB_2665 = MUL_2512 - MUL_2506;
        auto SUB_2668 = SUB_2665 - MUL_2523;
        auto ADD_2671 = ADD_566 + SUB_2668;
        auto SUB_2702 = MUL_2541 - MUL_2502;
        auto SUB_2705 = SUB_2702 - MUL_2548;
        auto ADD_2708 = ADD_564 + SUB_2705;
        auto SUB_2703 = MUL_2543 - MUL_2504;
        auto SUB_2706 = SUB_2703 - MUL_2552;
        auto ADD_2709 = ADD_565 + SUB_2706;
        auto SUB_2704 = MUL_2545 - MUL_2506;
        auto SUB_2707 = SUB_2704 - MUL_2556;
        auto ADD_2710 = ADD_566 + SUB_2707;
        auto SUB_2741 = MUL_2574 - MUL_2502;
        auto SUB_2744 = SUB_2741 - MUL_2581;
        auto ADD_2747 = ADD_564 + SUB_2744;
        auto SUB_2742 = MUL_2576 - MUL_2504;
        auto SUB_2745 = SUB_2742 - MUL_2585;
        auto ADD_2748 = ADD_565 + SUB_2745;
        auto SUB_2743 = MUL_2578 - MUL_2506;
        auto SUB_2746 = SUB_2743 - MUL_2589;
        auto ADD_2749 = ADD_566 + SUB_2746;
        auto SUB_2780 = MUL_2607 - MUL_2502;
        auto SUB_2783 = SUB_2780 - MUL_2614;
        auto ADD_2786 = ADD_564 + SUB_2783;
        auto SUB_2781 = MUL_2609 - MUL_2504;
        auto SUB_2784 = SUB_2781 - MUL_2618;
        auto ADD_2787 = ADD_565 + SUB_2784;
        auto SUB_2782 = MUL_2611 - MUL_2506;
        auto SUB_2785 = SUB_2782 - MUL_2622;
        auto ADD_2788 = ADD_566 + SUB_2785;
        if (/*panda_link0 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.043343, 1.4e-06, 0.0629063, 0.130366, ADD_2399, ADD_2400, ADD_2401, 0.173531))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
        }  // (343, 572)
        if (/*panda_link1 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1636, SUB_1637, 0.2598976, 0.144259, ADD_2399, ADD_2400, ADD_2401, 0.173531))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
        }  // (572, 572)
        if (/*panda_link2 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1835, SUB_1836, ADD_1838, 0.145067, ADD_2399, ADD_2400, ADD_2401, 0.173531))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
        }  // (572, 572)
        if (/*panda_link5*/ sphere_environment_in_collision(
            environment, ADD_2399, ADD_2400, ADD_2401, 0.173531))
        {
            if (sphere_environment_in_collision(environment, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
        }  // (572, 572)
        auto MUL_629 = SUB_596 * 0.7071068;
        auto MUL_644 = ADD_591 * 0.7071068;
        auto MUL_642 = SUB_585 * 0.7071068;
        auto SUB_655 = MUL_644 - MUL_642;
        auto ADD_645 = MUL_642 + MUL_644;
        auto MUL_631 = ADD_579 * 0.7071068;
        auto SUB_660 = MUL_629 - MUL_631;
        auto ADD_632 = MUL_629 + MUL_631;
        auto INPUT_5 = q[5];
        auto DIV_697 = INPUT_5 * 0.5;
        auto SIN_698 = DIV_697.sin();
        auto COS_704 = DIV_697.cos();
        auto MUL_716 = SUB_660 * SIN_698;
        auto MUL_721 = SUB_660 * COS_704;
        auto MUL_724 = SUB_655 * SIN_698;
        auto SUB_725 = MUL_721 - MUL_724;
        auto MUL_719 = SUB_655 * COS_704;
        auto ADD_720 = MUL_716 + MUL_719;
        auto MUL_2817 = SUB_725 * ADD_720;
        auto MUL_2816 = ADD_720 * ADD_720;
        auto MUL_711 = ADD_632 * SIN_698;
        auto MUL_706 = ADD_632 * COS_704;
        auto MUL_707 = ADD_645 * SIN_698;
        auto ADD_708 = MUL_706 + MUL_707;
        auto MUL_2822 = ADD_708 * ADD_720;
        auto MUL_713 = ADD_645 * COS_704;
        auto SUB_714 = MUL_713 - MUL_711;
        auto MUL_2818 = SUB_725 * SUB_714;
        auto ADD_2850 = MUL_2822 + MUL_2818;
        auto MUL_2852 = ADD_2850 * 2.0;
        auto MUL_2876 = MUL_2852 * 0.0108239;
        auto MUL_2815 = SUB_714 * SUB_714;
        auto ADD_2824 = MUL_2815 + MUL_2816;
        auto MUL_2827 = ADD_2824 * 2.0;
        auto SUB_2830 = 1.0 - MUL_2827;
        auto MUL_2864 = SUB_2830 * 0.0485274;
        auto MUL_2821 = ADD_708 * SUB_714;
        auto SUB_2837 = MUL_2821 - MUL_2817;
        auto MUL_2839 = SUB_2837 * 2.0;
        auto MUL_2870 = MUL_2839 * 0.0033602;
        auto ADD_2881 = MUL_2864 + MUL_2870;
        auto ADD_2884 = ADD_2881 + MUL_2876;
        auto ADD_2887 = ADD_564 + ADD_2884;
        auto ADD_2831 = MUL_2821 + MUL_2817;
        auto MUL_2833 = ADD_2831 * 2.0;
        auto MUL_2866 = MUL_2833 * 0.0485274;
        auto MUL_2820 = SUB_725 * ADD_708;
        auto MUL_2823 = SUB_714 * ADD_720;
        auto SUB_2853 = MUL_2823 - MUL_2820;
        auto MUL_2855 = SUB_2853 * 2.0;
        auto MUL_2878 = MUL_2855 * 0.0108239;
        auto MUL_2819 = ADD_708 * ADD_708;
        auto ADD_2840 = MUL_2816 + MUL_2819;
        auto MUL_2843 = ADD_2840 * 2.0;
        auto SUB_2846 = 1.0 - MUL_2843;
        auto MUL_2872 = SUB_2846 * 0.0033602;
        auto ADD_2882 = MUL_2866 + MUL_2872;
        auto ADD_2885 = ADD_2882 + MUL_2878;
        auto ADD_2888 = ADD_565 + ADD_2885;
        auto SUB_2834 = MUL_2822 - MUL_2818;
        auto ADD_2847 = MUL_2823 + MUL_2820;
        auto ADD_2856 = MUL_2815 + MUL_2819;
        auto MUL_2859 = ADD_2856 * 2.0;
        auto SUB_2862 = 1.0 - MUL_2859;
        auto MUL_2880 = SUB_2862 * 0.0108239;
        auto MUL_2849 = ADD_2847 * 2.0;
        auto MUL_2874 = MUL_2849 * 0.0033602;
        auto MUL_2836 = SUB_2834 * 2.0;
        auto MUL_2868 = MUL_2836 * 0.0485274;
        auto ADD_2883 = MUL_2868 + MUL_2874;
        auto ADD_2886 = ADD_2883 + MUL_2880;
        auto ADD_2889 = ADD_566 + ADD_2886;
        auto MUL_2916 = MUL_2839 * 0.01;
        auto MUL_2909 = SUB_2830 * 0.08;
        auto SUB_2932 = MUL_2909 - MUL_2916;
        auto ADD_2935 = ADD_564 + SUB_2932;
        auto MUL_2920 = SUB_2846 * 0.01;
        auto MUL_2911 = MUL_2833 * 0.08;
        auto SUB_2933 = MUL_2911 - MUL_2920;
        auto ADD_2936 = ADD_565 + SUB_2933;
        auto MUL_2924 = MUL_2849 * 0.01;
        auto MUL_2913 = MUL_2836 * 0.08;
        auto SUB_2934 = MUL_2913 - MUL_2924;
        auto ADD_2937 = ADD_566 + SUB_2934;
        auto MUL_2945 = MUL_2839 * 0.035;
        auto ADD_2956 = MUL_2909 + MUL_2945;
        auto ADD_2959 = ADD_564 + ADD_2956;
        auto MUL_2947 = SUB_2846 * 0.035;
        auto ADD_2957 = MUL_2911 + MUL_2947;
        auto ADD_2960 = ADD_565 + ADD_2957;
        auto MUL_2949 = MUL_2849 * 0.035;
        auto ADD_2958 = MUL_2913 + MUL_2949;
        auto ADD_2961 = ADD_566 + ADD_2958;
        if (/*panda_link0 vs. panda_link6*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.043343, 1.4e-06, 0.0629063, 0.130366, ADD_2887, ADD_2888, ADD_2889, 0.104795))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (572, 665)
        if (/*panda_link1 vs. panda_link6*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1636, SUB_1637, 0.2598976, 0.144259, ADD_2887, ADD_2888, ADD_2889, 0.104795))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (665, 665)
        if (/*panda_link6*/ sphere_environment_in_collision(
            environment, ADD_2887, ADD_2888, ADD_2889, 0.104795))
        {
            if (sphere_environment_in_collision(environment, ADD_564, ADD_565, ADD_566, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2935, ADD_2936, ADD_2937, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2959, ADD_2960, ADD_2961, 0.052))
            {
                return false;
            }
        }  // (665, 665)
        auto MUL_758 = SUB_725 * 0.7071068;
        auto MUL_773 = ADD_720 * 0.7071068;
        auto MUL_771 = SUB_714 * 0.7071068;
        auto SUB_784 = MUL_773 - MUL_771;
        auto ADD_774 = MUL_771 + MUL_773;
        auto MUL_760 = ADD_708 * 0.7071068;
        auto SUB_789 = MUL_758 - MUL_760;
        auto ADD_761 = MUL_758 + MUL_760;
        auto MUL_806 = ADD_720 * 0.088;
        auto MUL_811 = ADD_720 * MUL_806;
        auto MUL_801 = SUB_714 * 0.088;
        auto MUL_809 = SUB_714 * MUL_801;
        auto ADD_813 = MUL_809 + MUL_811;
        auto MUL_816 = ADD_813 * 2.0;
        auto SUB_819 = 0.088 - MUL_816;
        auto ADD_838 = ADD_564 + SUB_819;
        auto INPUT_6 = q[6];
        auto DIV_842 = INPUT_6 * 0.5;
        auto SIN_843 = DIV_842.sin();
        auto COS_849 = DIV_842.cos();
        auto MUL_866 = SUB_789 * COS_849;
        auto MUL_861 = SUB_789 * SIN_843;
        auto MUL_864 = SUB_784 * COS_849;
        auto ADD_865 = MUL_861 + MUL_864;
        auto MUL_2971 = ADD_865 * ADD_865;
        auto MUL_869 = SUB_784 * SIN_843;
        auto SUB_870 = MUL_866 - MUL_869;
        auto MUL_2972 = SUB_870 * ADD_865;
        auto MUL_851 = ADD_761 * COS_849;
        auto MUL_856 = ADD_761 * SIN_843;
        auto MUL_858 = ADD_774 * COS_849;
        auto SUB_859 = MUL_858 - MUL_856;
        auto MUL_2973 = SUB_870 * SUB_859;
        auto MUL_2970 = SUB_859 * SUB_859;
        auto ADD_2979 = MUL_2970 + MUL_2971;
        auto MUL_2982 = ADD_2979 * 2.0;
        auto SUB_2985 = 1.0 - MUL_2982;
        auto MUL_3019 = SUB_2985 * 0.0132698;
        auto MUL_852 = ADD_774 * SIN_843;
        auto ADD_853 = MUL_851 + MUL_852;
        auto MUL_2977 = ADD_853 * ADD_865;
        auto ADD_3005 = MUL_2977 + MUL_2973;
        auto MUL_3007 = ADD_3005 * 2.0;
        auto MUL_3031 = MUL_3007 * 0.0793978;
        auto MUL_2976 = ADD_853 * SUB_859;
        auto SUB_2992 = MUL_2976 - MUL_2972;
        auto MUL_2994 = SUB_2992 * 2.0;
        auto MUL_3025 = MUL_2994 * 0.0133909;
        auto ADD_3036 = MUL_3019 + MUL_3025;
        auto ADD_3039 = ADD_3036 + MUL_3031;
        auto ADD_3042 = ADD_838 + ADD_3039;
        auto ADD_2986 = MUL_2976 + MUL_2972;
        auto MUL_2988 = ADD_2986 * 2.0;
        auto MUL_3021 = MUL_2988 * 0.0132698;
        auto MUL_2975 = SUB_870 * ADD_853;
        auto MUL_2978 = SUB_859 * ADD_865;
        auto SUB_3008 = MUL_2978 - MUL_2975;
        auto MUL_3010 = SUB_3008 * 2.0;
        auto MUL_3033 = MUL_3010 * 0.0793978;
        auto MUL_2974 = ADD_853 * ADD_853;
        auto ADD_2995 = MUL_2971 + MUL_2974;
        auto MUL_2998 = ADD_2995 * 2.0;
        auto SUB_3001 = 1.0 - MUL_2998;
        auto MUL_3027 = SUB_3001 * 0.0133909;
        auto ADD_3037 = MUL_3021 + MUL_3027;
        auto ADD_3040 = ADD_3037 + MUL_3033;
        auto MUL_821 = SUB_725 * MUL_806;
        auto MUL_822 = ADD_708 * MUL_801;
        auto ADD_824 = MUL_821 + MUL_822;
        auto MUL_827 = ADD_824 * 2.0;
        auto ADD_839 = ADD_565 + MUL_827;
        auto ADD_3043 = ADD_839 + ADD_3040;
        auto SUB_2989 = MUL_2977 - MUL_2973;
        auto ADD_3002 = MUL_2978 + MUL_2975;
        auto ADD_3011 = MUL_2970 + MUL_2974;
        auto MUL_3014 = ADD_3011 * 2.0;
        auto SUB_3017 = 1.0 - MUL_3014;
        auto MUL_3035 = SUB_3017 * 0.0793978;
        auto MUL_3004 = ADD_3002 * 2.0;
        auto MUL_3029 = MUL_3004 * 0.0133909;
        auto MUL_2991 = SUB_2989 * 2.0;
        auto MUL_3023 = MUL_2991 * 0.0132698;
        auto ADD_3038 = MUL_3023 + MUL_3029;
        auto ADD_3041 = ADD_3038 + MUL_3035;
        auto MUL_829 = SUB_725 * MUL_801;
        auto MUL_831 = ADD_708 * MUL_806;
        auto SUB_833 = MUL_831 - MUL_829;
        auto MUL_836 = SUB_833 * 2.0;
        auto ADD_840 = ADD_566 + MUL_836;
        auto ADD_3044 = ADD_840 + ADD_3041;
        auto MUL_3058 = MUL_3007 * 0.07;
        auto ADD_3063 = ADD_838 + MUL_3058;
        auto MUL_3060 = MUL_3010 * 0.07;
        auto ADD_3064 = ADD_839 + MUL_3060;
        auto MUL_3062 = SUB_3017 * 0.07;
        auto ADD_3065 = ADD_840 + MUL_3062;
        auto MUL_3079 = MUL_3007 * 0.08;
        auto MUL_3067 = SUB_2985 * 0.02;
        auto MUL_3073 = MUL_2994 * 0.04;
        auto ADD_3084 = MUL_3067 + MUL_3073;
        auto ADD_3087 = ADD_3084 + MUL_3079;
        auto ADD_3090 = ADD_838 + ADD_3087;
        auto MUL_3081 = MUL_3010 * 0.08;
        auto MUL_3075 = SUB_3001 * 0.04;
        auto MUL_3069 = MUL_2988 * 0.02;
        auto ADD_3085 = MUL_3069 + MUL_3075;
        auto ADD_3088 = ADD_3085 + MUL_3081;
        auto ADD_3091 = ADD_839 + ADD_3088;
        auto MUL_3083 = SUB_3017 * 0.08;
        auto MUL_3077 = MUL_3004 * 0.04;
        auto MUL_3071 = MUL_2991 * 0.02;
        auto ADD_3086 = MUL_3071 + MUL_3077;
        auto ADD_3089 = ADD_3086 + MUL_3083;
        auto ADD_3092 = ADD_840 + ADD_3089;
        auto MUL_3100 = MUL_2994 * 0.02;
        auto MUL_3094 = SUB_2985 * 0.04;
        auto ADD_3111 = MUL_3094 + MUL_3100;
        auto ADD_3114 = ADD_3111 + MUL_3079;
        auto ADD_3117 = ADD_838 + ADD_3114;
        auto MUL_3102 = SUB_3001 * 0.02;
        auto MUL_3096 = MUL_2988 * 0.04;
        auto ADD_3112 = MUL_3096 + MUL_3102;
        auto ADD_3115 = ADD_3112 + MUL_3081;
        auto ADD_3118 = ADD_839 + ADD_3115;
        auto MUL_3104 = MUL_3004 * 0.02;
        auto MUL_3098 = MUL_2991 * 0.04;
        auto ADD_3113 = MUL_3098 + MUL_3104;
        auto ADD_3116 = ADD_3113 + MUL_3083;
        auto ADD_3119 = ADD_840 + ADD_3116;
        auto MUL_3133 = MUL_3007 * 0.085;
        auto MUL_3127 = MUL_2994 * 0.06;
        auto ADD_3138 = MUL_3094 + MUL_3127;
        auto ADD_3141 = ADD_3138 + MUL_3133;
        auto ADD_3144 = ADD_838 + ADD_3141;
        auto MUL_3135 = MUL_3010 * 0.085;
        auto MUL_3129 = SUB_3001 * 0.06;
        auto ADD_3139 = MUL_3096 + MUL_3129;
        auto ADD_3142 = ADD_3139 + MUL_3135;
        auto ADD_3145 = ADD_839 + ADD_3142;
        auto MUL_3137 = SUB_3017 * 0.085;
        auto MUL_3131 = MUL_3004 * 0.06;
        auto ADD_3140 = MUL_3098 + MUL_3131;
        auto ADD_3143 = ADD_3140 + MUL_3137;
        auto ADD_3146 = ADD_840 + ADD_3143;
        auto MUL_3148 = SUB_2985 * 0.06;
        auto ADD_3165 = MUL_3148 + MUL_3073;
        auto ADD_3168 = ADD_3165 + MUL_3133;
        auto ADD_3171 = ADD_838 + ADD_3168;
        auto MUL_3150 = MUL_2988 * 0.06;
        auto ADD_3166 = MUL_3150 + MUL_3075;
        auto ADD_3169 = ADD_3166 + MUL_3135;
        auto ADD_3172 = ADD_839 + ADD_3169;
        auto MUL_3152 = MUL_2991 * 0.06;
        auto ADD_3167 = MUL_3152 + MUL_3077;
        auto ADD_3170 = ADD_3167 + MUL_3137;
        auto ADD_3173 = ADD_840 + ADD_3170;
        if (/*panda_link0 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.043343, 1.4e-06, 0.0629063, 0.130366, ADD_3042, ADD_3043, ADD_3044, 0.073242))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (665, 821)
        if (/*panda_link1 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1636, SUB_1637, 0.2598976, 0.144259, ADD_3042, ADD_3043, ADD_3044, 0.073242))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (821, 821)
        if (/*panda_link2 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1835, SUB_1836, ADD_1838, 0.145067, ADD_3042, ADD_3043, ADD_3044, 0.073242))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (821, 821)
        if (/*panda_link5 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2399, ADD_2400, ADD_2401, 0.173531, ADD_3042, ADD_3043, ADD_3044, 0.073242))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (821, 821)
        if (/*panda_link7*/ sphere_environment_in_collision(
            environment, ADD_3042, ADD_3043, ADD_3044, 0.073242))
        {
            if (sphere_environment_in_collision(environment, ADD_3063, ADD_3064, ADD_3065, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3090, ADD_3091, ADD_3092, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3117, ADD_3118, ADD_3119, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3144, ADD_3145, ADD_3146, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3171, ADD_3172, ADD_3173, 0.02))
            {
                return false;
            }
        }  // (821, 821)
        auto MUL_1065 = SUB_870 * 0.9238795;
        auto MUL_1062 = ADD_865 * 0.9238795;
        auto MUL_1049 = SUB_859 * 0.9238795;
        auto MUL_1034 = ADD_853 * 0.9238795;
        auto MUL_1055 = SUB_870 * 0.3826834;
        auto SUB_1063 = MUL_1062 - MUL_1055;
        auto MUL_3235 = SUB_1063 * SUB_1063;
        auto MUL_1072 = ADD_865 * 0.3826834;
        auto ADD_1074 = MUL_1065 + MUL_1072;
        auto MUL_3236 = ADD_1074 * SUB_1063;
        auto MUL_1037 = SUB_859 * 0.3826834;
        auto SUB_1039 = MUL_1034 - MUL_1037;
        auto MUL_3241 = SUB_1039 * SUB_1063;
        auto MUL_1046 = ADD_853 * 0.3826834;
        auto ADD_1050 = MUL_1046 + MUL_1049;
        auto MUL_3237 = ADD_1074 * ADD_1050;
        auto ADD_3269 = MUL_3241 + MUL_3237;
        auto MUL_3271 = ADD_3269 * 2.0;
        auto MUL_3240 = SUB_1039 * ADD_1050;
        auto SUB_3256 = MUL_3240 - MUL_3236;
        auto MUL_3258 = SUB_3256 * 2.0;
        auto MUL_3234 = ADD_1050 * ADD_1050;
        auto ADD_3243 = MUL_3234 + MUL_3235;
        auto MUL_3246 = ADD_3243 * 2.0;
        auto SUB_3249 = 1.0 - MUL_3246;
        auto MUL_931 = SUB_859 * 0.107;
        auto MUL_942 = SUB_870 * MUL_931;
        auto MUL_939 = ADD_853 * 0.107;
        auto MUL_944 = ADD_865 * MUL_939;
        auto ADD_945 = MUL_942 + MUL_944;
        auto MUL_947 = ADD_945 * 2.0;
        auto ADD_969 = ADD_838 + MUL_947;
        auto MUL_3301 = MUL_3271 * 0.0196227;
        auto MUL_3295 = MUL_3258 * 2.7e-05;
        auto MUL_3284 = SUB_3249 * 5.1e-06;
        auto SUB_3306 = MUL_3295 - MUL_3284;
        auto ADD_3309 = SUB_3306 + MUL_3301;
        auto ADD_3312 = ADD_969 + ADD_3309;
        auto ADD_3250 = MUL_3240 + MUL_3236;
        auto MUL_3252 = ADD_3250 * 2.0;
        auto MUL_3288 = MUL_3252 * 5.1e-06;
        auto MUL_3239 = ADD_1074 * SUB_1039;
        auto MUL_3238 = SUB_1039 * SUB_1039;
        auto ADD_3259 = MUL_3235 + MUL_3238;
        auto MUL_3262 = ADD_3259 * 2.0;
        auto SUB_3265 = 1.0 - MUL_3262;
        auto MUL_3297 = SUB_3265 * 2.7e-05;
        auto SUB_3307 = MUL_3297 - MUL_3288;
        auto MUL_3242 = ADD_1050 * SUB_1063;
        auto SUB_3272 = MUL_3242 - MUL_3239;
        auto MUL_3274 = SUB_3272 * 2.0;
        auto MUL_3303 = MUL_3274 * 0.0196227;
        auto ADD_3310 = SUB_3307 + MUL_3303;
        auto MUL_950 = SUB_870 * MUL_939;
        auto MUL_953 = ADD_865 * MUL_931;
        auto SUB_954 = MUL_953 - MUL_950;
        auto MUL_956 = SUB_954 * 2.0;
        auto ADD_970 = ADD_839 + MUL_956;
        auto ADD_3313 = ADD_970 + ADD_3310;
        auto SUB_3253 = MUL_3241 - MUL_3237;
        auto ADD_3266 = MUL_3242 + MUL_3239;
        auto ADD_3275 = MUL_3234 + MUL_3238;
        auto MUL_3278 = ADD_3275 * 2.0;
        auto SUB_3281 = 1.0 - MUL_3278;
        auto MUL_3305 = SUB_3281 * 0.0196227;
        auto MUL_3268 = ADD_3266 * 2.0;
        auto MUL_3299 = MUL_3268 * 2.7e-05;
        auto MUL_3255 = SUB_3253 * 2.0;
        auto MUL_3292 = MUL_3255 * 5.1e-06;
        auto SUB_3308 = MUL_3299 - MUL_3292;
        auto ADD_3311 = SUB_3308 + MUL_3305;
        auto MUL_961 = SUB_859 * MUL_931;
        auto MUL_959 = ADD_853 * MUL_939;
        auto ADD_962 = MUL_959 + MUL_961;
        auto MUL_965 = ADD_962 * 2.0;
        auto SUB_968 = 0.107 - MUL_965;
        auto ADD_971 = ADD_840 + SUB_968;
        auto ADD_3314 = ADD_971 + ADD_3311;
        auto MUL_3334 = MUL_3271 * 0.01;
        auto MUL_3323 = MUL_3258 * 0.075;
        auto SUB_3339 = MUL_3334 - MUL_3323;
        auto ADD_3342 = ADD_969 + SUB_3339;
        auto MUL_3336 = MUL_3274 * 0.01;
        auto MUL_3327 = SUB_3265 * 0.075;
        auto SUB_3340 = MUL_3336 - MUL_3327;
        auto ADD_3343 = ADD_970 + SUB_3340;
        auto MUL_3338 = SUB_3281 * 0.01;
        auto MUL_3331 = MUL_3268 * 0.075;
        auto SUB_3341 = MUL_3338 - MUL_3331;
        auto ADD_3344 = ADD_971 + SUB_3341;
        auto MUL_3353 = MUL_3258 * 0.045;
        auto SUB_3369 = MUL_3334 - MUL_3353;
        auto ADD_3372 = ADD_969 + SUB_3369;
        auto MUL_3357 = SUB_3265 * 0.045;
        auto SUB_3370 = MUL_3336 - MUL_3357;
        auto ADD_3373 = ADD_970 + SUB_3370;
        auto MUL_3361 = MUL_3268 * 0.045;
        auto SUB_3371 = MUL_3338 - MUL_3361;
        auto ADD_3374 = ADD_971 + SUB_3371;
        auto MUL_3383 = MUL_3258 * 0.015;
        auto SUB_3399 = MUL_3334 - MUL_3383;
        auto ADD_3402 = ADD_969 + SUB_3399;
        auto MUL_3387 = SUB_3265 * 0.015;
        auto SUB_3400 = MUL_3336 - MUL_3387;
        auto ADD_3403 = ADD_970 + SUB_3400;
        auto MUL_3391 = MUL_3268 * 0.015;
        auto SUB_3401 = MUL_3338 - MUL_3391;
        auto ADD_3404 = ADD_971 + SUB_3401;
        auto ADD_3423 = MUL_3383 + MUL_3334;
        auto ADD_3426 = ADD_969 + ADD_3423;
        auto ADD_3424 = MUL_3387 + MUL_3336;
        auto ADD_3427 = ADD_970 + ADD_3424;
        auto ADD_3425 = MUL_3391 + MUL_3338;
        auto ADD_3428 = ADD_971 + ADD_3425;
        auto ADD_3447 = MUL_3353 + MUL_3334;
        auto ADD_3450 = ADD_969 + ADD_3447;
        auto ADD_3448 = MUL_3357 + MUL_3336;
        auto ADD_3451 = ADD_970 + ADD_3448;
        auto ADD_3449 = MUL_3361 + MUL_3338;
        auto ADD_3452 = ADD_971 + ADD_3449;
        auto ADD_3471 = MUL_3323 + MUL_3334;
        auto ADD_3474 = ADD_969 + ADD_3471;
        auto ADD_3472 = MUL_3327 + MUL_3336;
        auto ADD_3475 = ADD_970 + ADD_3472;
        auto ADD_3473 = MUL_3331 + MUL_3338;
        auto ADD_3476 = ADD_971 + ADD_3473;
        auto MUL_3496 = MUL_3271 * 0.03;
        auto SUB_3501 = MUL_3496 - MUL_3323;
        auto ADD_3504 = ADD_969 + SUB_3501;
        auto MUL_3498 = MUL_3274 * 0.03;
        auto SUB_3502 = MUL_3498 - MUL_3327;
        auto ADD_3505 = ADD_970 + SUB_3502;
        auto MUL_3500 = SUB_3281 * 0.03;
        auto SUB_3503 = MUL_3500 - MUL_3331;
        auto ADD_3506 = ADD_971 + SUB_3503;
        auto SUB_3531 = MUL_3496 - MUL_3353;
        auto ADD_3534 = ADD_969 + SUB_3531;
        auto SUB_3532 = MUL_3498 - MUL_3357;
        auto ADD_3535 = ADD_970 + SUB_3532;
        auto SUB_3533 = MUL_3500 - MUL_3361;
        auto ADD_3536 = ADD_971 + SUB_3533;
        auto SUB_3561 = MUL_3496 - MUL_3383;
        auto ADD_3564 = ADD_969 + SUB_3561;
        auto SUB_3562 = MUL_3498 - MUL_3387;
        auto ADD_3565 = ADD_970 + SUB_3562;
        auto SUB_3563 = MUL_3500 - MUL_3391;
        auto ADD_3566 = ADD_971 + SUB_3563;
        auto ADD_3585 = MUL_3383 + MUL_3496;
        auto ADD_3588 = ADD_969 + ADD_3585;
        auto ADD_3586 = MUL_3387 + MUL_3498;
        auto ADD_3589 = ADD_970 + ADD_3586;
        auto ADD_3587 = MUL_3391 + MUL_3500;
        auto ADD_3590 = ADD_971 + ADD_3587;
        auto ADD_3609 = MUL_3353 + MUL_3496;
        auto ADD_3612 = ADD_969 + ADD_3609;
        auto ADD_3610 = MUL_3357 + MUL_3498;
        auto ADD_3613 = ADD_970 + ADD_3610;
        auto ADD_3611 = MUL_3361 + MUL_3500;
        auto ADD_3614 = ADD_971 + ADD_3611;
        auto ADD_3633 = MUL_3323 + MUL_3496;
        auto ADD_3636 = ADD_969 + ADD_3633;
        auto ADD_3634 = MUL_3327 + MUL_3498;
        auto ADD_3637 = ADD_970 + ADD_3634;
        auto ADD_3635 = MUL_3331 + MUL_3500;
        auto ADD_3638 = ADD_971 + ADD_3635;
        auto MUL_3658 = MUL_3271 * 0.05;
        auto SUB_3663 = MUL_3658 - MUL_3323;
        auto ADD_3666 = ADD_969 + SUB_3663;
        auto MUL_3660 = MUL_3274 * 0.05;
        auto SUB_3664 = MUL_3660 - MUL_3327;
        auto ADD_3667 = ADD_970 + SUB_3664;
        auto MUL_3662 = SUB_3281 * 0.05;
        auto SUB_3665 = MUL_3662 - MUL_3331;
        auto ADD_3668 = ADD_971 + SUB_3665;
        auto SUB_3693 = MUL_3658 - MUL_3353;
        auto ADD_3696 = ADD_969 + SUB_3693;
        auto SUB_3694 = MUL_3660 - MUL_3357;
        auto ADD_3697 = ADD_970 + SUB_3694;
        auto SUB_3695 = MUL_3662 - MUL_3361;
        auto ADD_3698 = ADD_971 + SUB_3695;
        auto SUB_3723 = MUL_3658 - MUL_3383;
        auto ADD_3726 = ADD_969 + SUB_3723;
        auto SUB_3724 = MUL_3660 - MUL_3387;
        auto ADD_3727 = ADD_970 + SUB_3724;
        auto SUB_3725 = MUL_3662 - MUL_3391;
        auto ADD_3728 = ADD_971 + SUB_3725;
        auto ADD_3747 = MUL_3383 + MUL_3658;
        auto ADD_3750 = ADD_969 + ADD_3747;
        auto ADD_3748 = MUL_3387 + MUL_3660;
        auto ADD_3751 = ADD_970 + ADD_3748;
        auto ADD_3749 = MUL_3391 + MUL_3662;
        auto ADD_3752 = ADD_971 + ADD_3749;
        auto ADD_3771 = MUL_3353 + MUL_3658;
        auto ADD_3774 = ADD_969 + ADD_3771;
        auto ADD_3772 = MUL_3357 + MUL_3660;
        auto ADD_3775 = ADD_970 + ADD_3772;
        auto ADD_3773 = MUL_3361 + MUL_3662;
        auto ADD_3776 = ADD_971 + ADD_3773;
        auto ADD_3795 = MUL_3323 + MUL_3658;
        auto ADD_3798 = ADD_969 + ADD_3795;
        auto ADD_3796 = MUL_3327 + MUL_3660;
        auto ADD_3799 = ADD_970 + ADD_3796;
        auto ADD_3797 = MUL_3331 + MUL_3662;
        auto ADD_3800 = ADD_971 + ADD_3797;
        if (/*panda_hand*/ sphere_environment_in_collision(
            environment, ADD_3312, ADD_3313, ADD_3314, 0.107701))
        {
            if (sphere_environment_in_collision(environment, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
        }  // (821, 1025)
        if (/*panda_link0 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.043343, 1.4e-06, 0.0629063, 0.130366, ADD_3312, ADD_3313, ADD_3314, 0.107701))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
        }  // (1025, 1025)
        if (/*panda_link1 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1636, SUB_1637, 0.2598976, 0.144259, ADD_3312, ADD_3313, ADD_3314, 0.107701))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
        }  // (1025, 1025)
        if (/*panda_link2 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1835, SUB_1836, ADD_1838, 0.145067, ADD_3312, ADD_3313, ADD_3314, 0.107701))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
        }  // (1025, 1025)
        if (/*panda_link5 vs. panda_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2399, ADD_2400, ADD_2401, 0.173531, ADD_3312, ADD_3313, ADD_3314, 0.107701))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3342, ADD_3343, ADD_3344, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3372, ADD_3373, ADD_3374, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3402, ADD_3403, ADD_3404, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3426, ADD_3427, ADD_3428, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3450, ADD_3451, ADD_3452, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3474, ADD_3475, ADD_3476, 0.028))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3504, ADD_3505, ADD_3506, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3534, ADD_3535, ADD_3536, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3564, ADD_3565, ADD_3566, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3588, ADD_3589, ADD_3590, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3612, ADD_3613, ADD_3614, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3636, ADD_3637, ADD_3638, 0.026))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3666, ADD_3667, ADD_3668, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3696, ADD_3697, ADD_3698, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3726, ADD_3727, ADD_3728, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3750, ADD_3751, ADD_3752, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3774, ADD_3775, ADD_3776, 0.024))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3798, ADD_3799, ADD_3800, 0.024))
            {
                return false;
            }
        }  // (1025, 1025)
        auto MUL_3876 = ADD_3269 * 2.0;
        auto MUL_3863 = SUB_3256 * 2.0;
        auto MUL_3851 = ADD_3243 * 2.0;
        auto SUB_3854 = 1.0 - MUL_3851;
        auto MUL_1196 = SUB_1063 * 0.065;
        auto MUL_1199 = SUB_1039 * 0.065;
        auto MUL_1207 = ADD_1050 * MUL_1199;
        auto MUL_1203 = SUB_1039 * 0.0584;
        auto MUL_1209 = SUB_1063 * MUL_1203;
        auto MUL_1194 = ADD_1050 * 0.0584;
        auto SUB_1197 = MUL_1194 - MUL_1196;
        auto MUL_1206 = ADD_1074 * SUB_1197;
        auto ADD_1208 = MUL_1206 + MUL_1207;
        auto ADD_1210 = ADD_1208 + MUL_1209;
        auto MUL_1212 = ADD_1210 * 2.0;
        auto ADD_1235 = ADD_969 + MUL_1212;
        auto MUL_3906 = MUL_3876 * 0.0261043;
        auto MUL_3900 = MUL_3863 * 0.0129294;
        auto MUL_3889 = SUB_3854 * 4.3e-06;
        auto SUB_3911 = MUL_3900 - MUL_3889;
        auto ADD_3914 = SUB_3911 + MUL_3906;
        auto ADD_3917 = ADD_1235 + ADD_3914;
        auto MUL_3879 = SUB_3272 * 2.0;
        auto MUL_3908 = MUL_3879 * 0.0261043;
        auto MUL_3867 = ADD_3259 * 2.0;
        auto SUB_3870 = 1.0 - MUL_3867;
        auto MUL_3902 = SUB_3870 * 0.0129294;
        auto MUL_3857 = ADD_3250 * 2.0;
        auto MUL_3893 = MUL_3857 * 4.3e-06;
        auto SUB_3912 = MUL_3902 - MUL_3893;
        auto ADD_3915 = SUB_3912 + MUL_3908;
        auto MUL_1215 = ADD_1074 * MUL_1203;
        auto MUL_1220 = SUB_1063 * SUB_1197;
        auto MUL_1217 = SUB_1039 * MUL_1199;
        auto ADD_1218 = MUL_1215 + MUL_1217;
        auto SUB_1221 = MUL_1220 - ADD_1218;
        auto MUL_1223 = SUB_1221 * 2.0;
        auto ADD_1225 = MUL_1223 + 0.065;
        auto ADD_1236 = ADD_970 + ADD_1225;
        auto ADD_3918 = ADD_1236 + ADD_3915;
        auto MUL_3883 = ADD_3275 * 2.0;
        auto SUB_3886 = 1.0 - MUL_3883;
        auto MUL_3910 = SUB_3886 * 0.0261043;
        auto MUL_3873 = ADD_3266 * 2.0;
        auto MUL_3904 = MUL_3873 * 0.0129294;
        auto MUL_3860 = SUB_3253 * 2.0;
        auto MUL_3897 = MUL_3860 * 4.3e-06;
        auto SUB_3913 = MUL_3904 - MUL_3897;
        auto ADD_3916 = SUB_3913 + MUL_3910;
        auto MUL_1226 = ADD_1074 * MUL_1199;
        auto MUL_1227 = SUB_1039 * MUL_1203;
        auto SUB_1228 = MUL_1226 - MUL_1227;
        auto MUL_1229 = ADD_1050 * SUB_1197;
        auto SUB_1230 = SUB_1228 - MUL_1229;
        auto MUL_1232 = SUB_1230 * 2.0;
        auto ADD_1234 = MUL_1232 + 0.0584;
        auto ADD_1237 = ADD_971 + ADD_1234;
        auto ADD_3919 = ADD_1237 + ADD_3916;
        auto MUL_3927 = MUL_3863 * 0.015;
        auto MUL_3933 = MUL_3876 * 0.022;
        auto ADD_3938 = MUL_3927 + MUL_3933;
        auto ADD_3941 = ADD_1235 + ADD_3938;
        auto MUL_3935 = MUL_3879 * 0.022;
        auto MUL_3929 = SUB_3870 * 0.015;
        auto ADD_3939 = MUL_3929 + MUL_3935;
        auto ADD_3942 = ADD_1236 + ADD_3939;
        auto MUL_3937 = SUB_3886 * 0.022;
        auto MUL_3931 = MUL_3873 * 0.015;
        auto ADD_3940 = MUL_3931 + MUL_3937;
        auto ADD_3943 = ADD_1237 + ADD_3940;
        auto MUL_3957 = MUL_3876 * 0.044;
        auto MUL_3951 = MUL_3863 * 0.008;
        auto ADD_3962 = MUL_3951 + MUL_3957;
        auto ADD_3965 = ADD_1235 + ADD_3962;
        auto MUL_3959 = MUL_3879 * 0.044;
        auto MUL_3953 = SUB_3870 * 0.008;
        auto ADD_3963 = MUL_3953 + MUL_3959;
        auto ADD_3966 = ADD_1236 + ADD_3963;
        auto MUL_3961 = SUB_3886 * 0.044;
        auto MUL_3955 = MUL_3873 * 0.008;
        auto ADD_3964 = MUL_3955 + MUL_3961;
        auto ADD_3967 = ADD_1237 + ADD_3964;
        if (/*panda_leftfinger*/ sphere_environment_in_collision(
            environment, ADD_3917, ADD_3918, ADD_3919, 0.031022))
        {
            if (sphere_environment_in_collision(environment, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
        }  // (1025, 1107)
        if (/*panda_link0 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.043343, 1.4e-06, 0.0629063, 0.130366, ADD_3917, ADD_3918, ADD_3919, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
        }  // (1107, 1107)
        if (/*panda_link1 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1636, SUB_1637, 0.2598976, 0.144259, ADD_3917, ADD_3918, ADD_3919, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
        }  // (1107, 1107)
        if (/*panda_link2 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1835, SUB_1836, ADD_1838, 0.145067, ADD_3917, ADD_3918, ADD_3919, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
        }  // (1107, 1107)
        if (/*panda_link5 vs. panda_leftfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2399, ADD_2400, ADD_2401, 0.173531, ADD_3917, ADD_3918, ADD_3919, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3941, ADD_3942, ADD_3943, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_3965, ADD_3966, ADD_3967, 0.012))
            {
                return false;
            }
        }  // (1107, 1107)
        auto ADD_1331 = MUL_1194 + MUL_1196;
        auto MUL_4011 = ADD_3269 * 2.0;
        auto MUL_3998 = SUB_3256 * 2.0;
        auto MUL_3986 = ADD_3243 * 2.0;
        auto SUB_3989 = 1.0 - MUL_3986;
        auto MUL_1342 = ADD_1074 * ADD_1331;
        auto SUB_1345 = MUL_1342 - MUL_1207;
        auto ADD_1347 = SUB_1345 + MUL_1209;
        auto MUL_1349 = ADD_1347 * 2.0;
        auto ADD_1377 = ADD_969 + MUL_1349;
        auto MUL_4041 = MUL_4011 * 0.0261203;
        auto MUL_4030 = MUL_3998 * 0.0129624;
        auto MUL_4023 = SUB_3989 * 5.4e-06;
        auto SUB_4046 = MUL_4023 - MUL_4030;
        auto ADD_4049 = SUB_4046 + MUL_4041;
        auto ADD_4052 = ADD_1377 + ADD_4049;
        auto SUB_1356 = MUL_1217 - MUL_1215;
        auto MUL_4014 = SUB_3272 * 2.0;
        auto MUL_4043 = MUL_4014 * 0.0261203;
        auto MUL_4002 = ADD_3259 * 2.0;
        auto SUB_4005 = 1.0 - MUL_4002;
        auto MUL_4034 = SUB_4005 * 0.0129624;
        auto MUL_3992 = ADD_3250 * 2.0;
        auto MUL_4025 = MUL_3992 * 5.4e-06;
        auto SUB_4047 = MUL_4025 - MUL_4034;
        auto ADD_4050 = SUB_4047 + MUL_4043;
        auto MUL_1357 = SUB_1063 * ADD_1331;
        auto ADD_1358 = SUB_1356 + MUL_1357;
        auto MUL_1360 = ADD_1358 * 2.0;
        auto SUB_1363 = MUL_1360 - 0.065;
        auto ADD_1378 = ADD_970 + SUB_1363;
        auto ADD_4053 = ADD_1378 + ADD_4050;
        auto ADD_1367 = MUL_1226 + MUL_1227;
        auto MUL_4018 = ADD_3275 * 2.0;
        auto SUB_4021 = 1.0 - MUL_4018;
        auto MUL_4045 = SUB_4021 * 0.0261203;
        auto MUL_4008 = ADD_3266 * 2.0;
        auto MUL_4038 = MUL_4008 * 0.0129624;
        auto MUL_3995 = SUB_3253 * 2.0;
        auto MUL_4027 = MUL_3995 * 5.4e-06;
        auto SUB_4048 = MUL_4027 - MUL_4038;
        auto ADD_4051 = SUB_4048 + MUL_4045;
        auto MUL_1369 = ADD_1050 * ADD_1331;
        auto ADD_1370 = ADD_1367 + MUL_1369;
        auto MUL_1373 = ADD_1370 * 2.0;
        auto SUB_1376 = 0.0584 - MUL_1373;
        auto ADD_1379 = ADD_971 + SUB_1376;
        auto ADD_4054 = ADD_1379 + ADD_4051;
        auto MUL_4074 = MUL_4011 * 0.022;
        auto MUL_4063 = MUL_3998 * 0.015;
        auto SUB_4079 = MUL_4074 - MUL_4063;
        auto ADD_4082 = ADD_1377 + SUB_4079;
        auto MUL_4076 = MUL_4014 * 0.022;
        auto MUL_4067 = SUB_4005 * 0.015;
        auto SUB_4080 = MUL_4076 - MUL_4067;
        auto ADD_4083 = ADD_1378 + SUB_4080;
        auto MUL_4078 = SUB_4021 * 0.022;
        auto MUL_4071 = MUL_4008 * 0.015;
        auto SUB_4081 = MUL_4078 - MUL_4071;
        auto ADD_4084 = ADD_1379 + SUB_4081;
        auto MUL_4104 = MUL_4011 * 0.044;
        auto MUL_4093 = MUL_3998 * 0.008;
        auto SUB_4109 = MUL_4104 - MUL_4093;
        auto ADD_4112 = ADD_1377 + SUB_4109;
        auto MUL_4106 = MUL_4014 * 0.044;
        auto MUL_4097 = SUB_4005 * 0.008;
        auto SUB_4110 = MUL_4106 - MUL_4097;
        auto ADD_4113 = ADD_1378 + SUB_4110;
        auto MUL_4108 = SUB_4021 * 0.044;
        auto MUL_4101 = MUL_4008 * 0.008;
        auto SUB_4111 = MUL_4108 - MUL_4101;
        auto ADD_4114 = ADD_1379 + SUB_4111;
        if (/*panda_link0 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.043343, 1.4e-06, 0.0629063, 0.130366, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
        }  // (1107, 1179)
        if (/*panda_link1 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1636, SUB_1637, 0.2598976, 0.144259, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1650, NEGATE_1654, 0.333, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1674, NEGATE_1678, 0.333, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
        }  // (1179, 1179)
        if (/*panda_link2 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1835, SUB_1836, ADD_1838, 0.145067, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1852, MUL_1854, ADD_1857, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1871, MUL_1873, ADD_1876, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1885, NEGATE_1889, SUB_1900, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_1909, NEGATE_1913, SUB_1924, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
        }  // (1179, 1179)
        if (/*panda_link5 vs. panda_rightfinger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2399, ADD_2400, ADD_2401, 0.173531, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2420, ADD_2421, ADD_2422, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2441, ADD_2442, ADD_2443, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2468, SUB_2469, SUB_2470, 0.06, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2498, ADD_2499, ADD_2500, 0.05, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2531, ADD_2532, ADD_2533, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2564, ADD_2565, ADD_2566, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2597, ADD_2598, ADD_2599, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2630, ADD_2631, ADD_2632, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2669, ADD_2670, ADD_2671, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2708, ADD_2709, ADD_2710, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2747, ADD_2748, ADD_2749, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_2786, ADD_2787, ADD_2788, 0.025, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
        }  // (1179, 1179)
        if (/*panda_rightfinger*/ sphere_environment_in_collision(
            environment, ADD_4052, ADD_4053, ADD_4054, 0.031022))
        {
            if (sphere_environment_in_collision(environment, ADD_4082, ADD_4083, ADD_4084, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4112, ADD_4113, ADD_4114, 0.012))
            {
                return false;
            }
        }  // (1179, 1179)
        set_attachment_pose(environment, ADD_969, ADD_970, ADD_971, SUB_1039, ADD_1050, SUB_1063, ADD_1074);
        if (/*attachment vs. panda_link0*/ attachment_sphere_collision<decltype(q[0])>(
            environment, 0.0, 0.0, 0.05, 0.08))
        {
            return false;
        }  // (1179, 1180)
        if (/*attachment vs. panda_link1*/ attachment_sphere_collision<decltype(q[0])>(
            environment, ADD_1636, SUB_1637, 0.2598976, 0.144259))
        {
            if (attachment_sphere_collision<decltype(q[0])>(environment, MUL_1650, NEGATE_1654, 0.333, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, MUL_1674, NEGATE_1678, 0.333, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, 0.0, 0.0, 0.213, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, 0.0, 0.0, 0.163, 0.06))
            {
                return false;
            }
        }  // (1180, 1180)
        if (/*attachment vs. panda_link2*/ attachment_sphere_collision<decltype(q[0])>(
            environment, ADD_1835, SUB_1836, ADD_1838, 0.145067))
        {
            if (attachment_sphere_collision<decltype(q[0])>(environment, MUL_1852, MUL_1854, ADD_1857, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, MUL_1871, MUL_1873, ADD_1876, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(
                    environment, MUL_1885, NEGATE_1889, SUB_1900, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(
                    environment, MUL_1909, NEGATE_1913, SUB_1924, 0.06))
            {
                return false;
            }
        }  // (1180, 1180)
        if (/*attachment vs. panda_link5*/ attachment_sphere_collision<decltype(q[0])>(
            environment, ADD_2399, ADD_2400, ADD_2401, 0.173531))
        {
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2420, ADD_2421, ADD_2422, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2441, ADD_2442, ADD_2443, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, SUB_2468, SUB_2469, SUB_2470, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2498, ADD_2499, ADD_2500, 0.05))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2531, ADD_2532, ADD_2533, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2564, ADD_2565, ADD_2566, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2597, ADD_2598, ADD_2599, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2630, ADD_2631, ADD_2632, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2669, ADD_2670, ADD_2671, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2708, ADD_2709, ADD_2710, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2747, ADD_2748, ADD_2749, 0.025))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_2786, ADD_2787, ADD_2788, 0.025))
            {
                return false;
            }
        }  // (1180, 1180)
        if (attachment_environment_collision(environment))
        {
            return false;
        }  // (1180, 1180)
        return true;
    }

    inline auto eefk(const std::array<float, 7> &q) noexcept -> std::array<float, 7>
    {
        auto INPUT_6 = q[6];
        auto DIV_842 = INPUT_6 * 0.5;
        auto SIN_843 = std::sin(DIV_842);
        auto COS_849 = std::cos(DIV_842);
        auto INPUT_5 = q[5];
        auto DIV_697 = INPUT_5 * 0.5;
        auto SIN_698 = std::sin(DIV_697);
        auto COS_704 = std::cos(DIV_697);
        auto INPUT_4 = q[4];
        auto DIV_568 = INPUT_4 * 0.5;
        auto SIN_569 = std::sin(DIV_568);
        auto COS_575 = std::cos(DIV_568);
        auto INPUT_3 = q[3];
        auto DIV_407 = INPUT_3 * 0.5;
        auto SIN_408 = std::sin(DIV_407);
        auto COS_414 = std::cos(DIV_407);
        auto INPUT_2 = q[2];
        auto DIV_262 = INPUT_2 * 0.5;
        auto SIN_263 = std::sin(DIV_262);
        auto COS_269 = std::cos(DIV_262);
        auto INPUT_1 = q[1];
        auto DIV_117 = INPUT_1 * 0.5;
        auto SIN_118 = std::sin(DIV_117);
        auto COS_124 = std::cos(DIV_117);
        auto INPUT_0 = q[0];
        auto DIV_8 = INPUT_0 * 0.5;
        auto SIN_9 = std::sin(DIV_8);
        auto COS_15 = std::cos(DIV_8);
        auto MUL_46 = COS_15 * 0.7071068;
        auto MUL_126 = MUL_46 * COS_124;
        auto MUL_140 = MUL_46 * SIN_118;
        auto MUL_62 = SIN_9 * 0.7071068;
        auto MUL_143 = MUL_62 * COS_124;
        auto SUB_138 = MUL_140 - MUL_143;
        auto ADD_144 = MUL_140 + MUL_143;
        auto MUL_128 = MUL_62 * SIN_118;
        auto SUB_149 = MUL_126 - MUL_128;
        auto ADD_130 = MUL_126 + MUL_128;
        auto MUL_182 = SUB_149 * 0.7071068;
        auto MUL_198 = ADD_144 * 0.7071068;
        auto MUL_224 = ADD_144 * 0.316;
        auto MUL_235 = SUB_149 * MUL_224;
        auto MUL_196 = SUB_138 * 0.7071068;
        auto SUB_209 = MUL_198 - MUL_196;
        auto ADD_199 = MUL_196 + MUL_198;
        auto MUL_284 = SUB_209 * COS_269;
        auto MUL_289 = SUB_209 * SIN_263;
        auto MUL_278 = ADD_199 * COS_269;
        auto MUL_272 = ADD_199 * SIN_263;
        auto MUL_184 = ADD_130 * 0.7071068;
        auto SUB_186 = MUL_182 - MUL_184;
        auto ADD_215 = MUL_182 + MUL_184;
        auto MUL_286 = ADD_215 * COS_269;
        auto SUB_290 = MUL_286 - MUL_289;
        auto MUL_351 = SUB_290 * 0.7071068;
        auto MUL_281 = ADD_215 * SIN_263;
        auto ADD_285 = MUL_281 + MUL_284;
        auto MUL_348 = ADD_285 * 0.7071068;
        auto MUL_371 = ADD_285 * 0.0825;
        auto MUL_376 = ADD_285 * MUL_371;
        auto MUL_271 = SUB_186 * COS_269;
        auto ADD_273 = MUL_271 + MUL_272;
        auto MUL_353 = ADD_273 * 0.7071068;
        auto SUB_354 = MUL_351 - MUL_353;
        auto ADD_326 = MUL_351 + MUL_353;
        auto MUL_431 = SUB_354 * COS_414;
        auto MUL_426 = SUB_354 * SIN_408;
        auto MUL_416 = ADD_326 * COS_414;
        auto MUL_421 = ADD_326 * SIN_408;
        auto MUL_276 = SUB_186 * SIN_263;
        auto SUB_279 = MUL_278 - MUL_276;
        auto MUL_345 = SUB_279 * 0.7071068;
        auto SUB_349 = MUL_348 - MUL_345;
        auto ADD_339 = MUL_345 + MUL_348;
        auto MUL_429 = SUB_349 * COS_414;
        auto ADD_430 = MUL_426 + MUL_429;
        auto MUL_486 = ADD_430 * 0.7071068;
        auto MUL_514 = ADD_430 * 0.384;
        auto MUL_527 = ADD_430 * 0.0825;
        auto MUL_533 = ADD_430 * MUL_527;
        auto MUL_434 = SUB_349 * SIN_408;
        auto SUB_435 = MUL_431 - MUL_434;
        auto MUL_529 = SUB_435 * MUL_514;
        auto MUL_469 = SUB_435 * 0.7071068;
        auto MUL_423 = ADD_339 * COS_414;
        auto SUB_424 = MUL_423 - MUL_421;
        auto MUL_483 = SUB_424 * 0.7071068;
        auto SUB_488 = MUL_483 - MUL_486;
        auto ADD_499 = MUL_483 + MUL_486;
        auto MUL_590 = ADD_499 * COS_575;
        auto MUL_595 = ADD_499 * SIN_569;
        auto MUL_584 = SUB_488 * COS_575;
        auto MUL_578 = SUB_488 * SIN_569;
        auto MUL_520 = SUB_424 * 0.0825;
        auto MUL_417 = ADD_339 * SIN_408;
        auto ADD_418 = MUL_416 + MUL_417;
        auto MUL_472 = ADD_418 * 0.7071068;
        auto SUB_473 = MUL_472 - MUL_469;
        auto ADD_506 = MUL_469 + MUL_472;
        auto MUL_592 = ADD_506 * COS_575;
        auto SUB_596 = MUL_592 - MUL_595;
        auto MUL_657 = SUB_596 * 0.7071068;
        auto MUL_587 = ADD_506 * SIN_569;
        auto ADD_591 = MUL_587 + MUL_590;
        auto MUL_654 = ADD_591 * 0.7071068;
        auto MUL_577 = SUB_473 * COS_575;
        auto ADD_579 = MUL_577 + MUL_578;
        auto MUL_659 = ADD_579 * 0.7071068;
        auto SUB_660 = MUL_657 - MUL_659;
        auto ADD_632 = MUL_657 + MUL_659;
        auto MUL_716 = SUB_660 * SIN_698;
        auto MUL_721 = SUB_660 * COS_704;
        auto MUL_711 = ADD_632 * SIN_698;
        auto MUL_706 = ADD_632 * COS_704;
        auto MUL_582 = SUB_473 * SIN_569;
        auto SUB_585 = MUL_584 - MUL_582;
        auto MUL_651 = SUB_585 * 0.7071068;
        auto SUB_655 = MUL_654 - MUL_651;
        auto ADD_645 = MUL_651 + MUL_654;
        auto MUL_724 = SUB_655 * SIN_698;
        auto SUB_725 = MUL_721 - MUL_724;
        auto MUL_758 = SUB_725 * 0.7071068;
        auto MUL_719 = SUB_655 * COS_704;
        auto ADD_720 = MUL_716 + MUL_719;
        auto MUL_773 = ADD_720 * 0.7071068;
        auto MUL_806 = ADD_720 * 0.088;
        auto MUL_811 = ADD_720 * MUL_806;
        auto MUL_707 = ADD_645 * SIN_698;
        auto ADD_708 = MUL_706 + MUL_707;
        auto MUL_760 = ADD_708 * 0.7071068;
        auto SUB_789 = MUL_758 - MUL_760;
        auto ADD_761 = MUL_758 + MUL_760;
        auto MUL_866 = SUB_789 * COS_849;
        auto MUL_861 = SUB_789 * SIN_843;
        auto MUL_851 = ADD_761 * COS_849;
        auto MUL_856 = ADD_761 * SIN_843;
        auto MUL_713 = ADD_645 * COS_704;
        auto SUB_714 = MUL_713 - MUL_711;
        auto MUL_771 = SUB_714 * 0.7071068;
        auto SUB_784 = MUL_773 - MUL_771;
        auto ADD_774 = MUL_771 + MUL_773;
        auto MUL_864 = SUB_784 * COS_849;
        auto ADD_865 = MUL_861 + MUL_864;
        auto MUL_869 = SUB_784 * SIN_843;
        auto SUB_870 = MUL_866 - MUL_869;
        auto MUL_858 = ADD_774 * COS_849;
        auto SUB_859 = MUL_858 - MUL_856;
        auto MUL_931 = SUB_859 * 0.107;
        auto MUL_942 = SUB_870 * MUL_931;
        auto MUL_852 = ADD_774 * SIN_843;
        auto ADD_853 = MUL_851 + MUL_852;
        auto MUL_939 = ADD_853 * 0.107;
        auto MUL_944 = ADD_865 * MUL_939;
        auto ADD_945 = MUL_942 + MUL_944;
        auto MUL_947 = ADD_945 * 2.0;
        auto MUL_801 = SUB_714 * 0.088;
        auto MUL_809 = SUB_714 * MUL_801;
        auto ADD_813 = MUL_809 + MUL_811;
        auto MUL_816 = ADD_813 * 2.0;
        auto SUB_819 = 0.088 - MUL_816;
        auto MUL_517 = ADD_418 * 0.384;
        auto ADD_522 = MUL_517 + MUL_520;
        auto MUL_531 = SUB_424 * ADD_522;
        auto SUB_532 = MUL_531 - MUL_529;
        auto ADD_534 = SUB_532 + MUL_533;
        auto MUL_536 = ADD_534 * 2.0;
        auto SUB_539 = MUL_536 - 0.0825;
        auto MUL_366 = SUB_279 * 0.0825;
        auto MUL_374 = SUB_279 * MUL_366;
        auto ADD_378 = MUL_374 + MUL_376;
        auto MUL_381 = ADD_378 * 2.0;
        auto SUB_384 = 0.0825 - MUL_381;
        auto MUL_228 = ADD_130 * 0.316;
        auto MUL_236 = SUB_138 * MUL_228;
        auto ADD_237 = MUL_235 + MUL_236;
        auto MUL_240 = ADD_237 * 2.0;
        auto ADD_403 = MUL_240 + SUB_384;
        auto ADD_564 = ADD_403 + SUB_539;
        auto ADD_838 = ADD_564 + SUB_819;
        auto ADD_969 = ADD_838 + MUL_947;
        auto MUL_950 = SUB_870 * MUL_939;
        auto MUL_953 = ADD_865 * MUL_931;
        auto SUB_954 = MUL_953 - MUL_950;
        auto MUL_956 = SUB_954 * 2.0;
        auto MUL_821 = SUB_725 * MUL_806;
        auto MUL_822 = ADD_708 * MUL_801;
        auto ADD_824 = MUL_821 + MUL_822;
        auto MUL_827 = ADD_824 * 2.0;
        auto MUL_541 = SUB_435 * MUL_527;
        auto MUL_546 = ADD_430 * MUL_514;
        auto MUL_543 = ADD_418 * ADD_522;
        auto ADD_544 = MUL_541 + MUL_543;
        auto ADD_548 = ADD_544 + MUL_546;
        auto MUL_551 = ADD_548 * 2.0;
        auto SUB_554 = 0.384 - MUL_551;
        auto MUL_386 = SUB_290 * MUL_371;
        auto MUL_387 = ADD_273 * MUL_366;
        auto ADD_389 = MUL_386 + MUL_387;
        auto MUL_392 = ADD_389 * 2.0;
        auto MUL_246 = ADD_144 * MUL_224;
        auto MUL_244 = ADD_130 * MUL_228;
        auto ADD_247 = MUL_244 + MUL_246;
        auto MUL_249 = ADD_247 * 2.0;
        auto SUB_252 = MUL_249 - 0.316;
        auto ADD_404 = SUB_252 + MUL_392;
        auto ADD_565 = ADD_404 + SUB_554;
        auto ADD_839 = ADD_565 + MUL_827;
        auto ADD_970 = ADD_839 + MUL_956;
        auto MUL_961 = SUB_859 * MUL_931;
        auto MUL_959 = ADD_853 * MUL_939;
        auto ADD_962 = MUL_959 + MUL_961;
        auto MUL_965 = ADD_962 * 2.0;
        auto SUB_968 = 0.107 - MUL_965;
        auto MUL_829 = SUB_725 * MUL_801;
        auto MUL_831 = ADD_708 * MUL_806;
        auto SUB_833 = MUL_831 - MUL_829;
        auto MUL_836 = SUB_833 * 2.0;
        auto MUL_555 = SUB_435 * ADD_522;
        auto MUL_558 = SUB_424 * MUL_514;
        auto MUL_556 = ADD_418 * MUL_527;
        auto SUB_557 = MUL_555 - MUL_556;
        auto ADD_560 = SUB_557 + MUL_558;
        auto MUL_562 = ADD_560 * 2.0;
        auto MUL_394 = SUB_290 * MUL_366;
        auto MUL_396 = ADD_273 * MUL_371;
        auto SUB_398 = MUL_396 - MUL_394;
        auto MUL_401 = SUB_398 * 2.0;
        auto MUL_253 = SUB_149 * MUL_228;
        auto MUL_255 = SUB_138 * MUL_224;
        auto SUB_256 = MUL_253 - MUL_255;
        auto MUL_258 = SUB_256 * 2.0;
        auto ADD_260 = 0.333 + MUL_258;
        auto ADD_405 = ADD_260 + MUL_401;
        auto ADD_566 = ADD_405 + MUL_562;
        auto ADD_840 = ADD_566 + MUL_836;
        auto ADD_971 = ADD_840 + SUB_968;
        auto MUL_1034 = ADD_853 * 0.9238795;
        auto MUL_1037 = SUB_859 * 0.3826834;
        auto SUB_1039 = MUL_1034 - MUL_1037;
        auto MUL_1049 = SUB_859 * 0.9238795;
        auto MUL_1046 = ADD_853 * 0.3826834;
        auto ADD_1050 = MUL_1046 + MUL_1049;
        auto MUL_1055 = SUB_870 * 0.3826834;
        auto MUL_1062 = ADD_865 * 0.9238795;
        auto SUB_1063 = MUL_1062 - MUL_1055;
        auto MUL_1065 = SUB_870 * 0.9238795;
        auto MUL_1072 = ADD_865 * 0.3826834;
        auto ADD_1074 = MUL_1065 + MUL_1072;
        return {ADD_969, ADD_970, ADD_971, SUB_1039, ADD_1050, SUB_1063, ADD_1074};
    }
}  // namespace vamp::robots::panda

// NOLINTEND(*-magic-numbers)
