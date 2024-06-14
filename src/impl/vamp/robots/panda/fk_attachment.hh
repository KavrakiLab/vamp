#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::panda_attachment
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
        // ignore collision of static link
        // if (/*panda_link0*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.05, 0.08))
        // {
        //     return false;
        // }  // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_14 = INPUT_0 * 0.5;
        auto SIN_15 = DIV_14.sin();
        auto COS_21 = DIV_14.cos();
        auto MUL_2328 = COS_21 * SIN_15;
        auto MUL_2347 = MUL_2328 * 2.0;
        auto MUL_2378 = MUL_2347 * 0.039;
        auto MUL_2327 = SIN_15 * SIN_15;
        auto MUL_2337 = MUL_2327 * 2.0;
        auto SUB_2340 = 1.0 - MUL_2337;
        auto MUL_2367 = SUB_2340 * 0.001;
        auto SUB_2394 = MUL_2378 - MUL_2367;
        auto MUL_2381 = SUB_2340 * 0.039;
        auto MUL_2371 = MUL_2347 * 0.001;
        auto ADD_2395 = MUL_2371 + MUL_2381;
        auto NEGATE_2396 = -ADD_2395;
        auto MUL_2409 = MUL_2347 * 0.08;
        auto MUL_2412 = SUB_2340 * 0.08;
        auto NEGATE_2413 = -MUL_2412;
        auto MUL_2433 = MUL_2347 * 0.03;
        auto MUL_2436 = SUB_2340 * 0.03;
        auto NEGATE_2437 = -MUL_2436;
        if (/*panda_link1*/ sphere_environment_in_collision(environment, SUB_2394, NEGATE_2396, 0.248, 0.154))
        {
            if (sphere_environment_in_collision(environment, MUL_2409, NEGATE_2413, 0.333, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_2433, NEGATE_2437, 0.333, 0.06))
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
        auto MUL_80 = COS_21 * 0.7071068;
        auto MUL_78 = SIN_15 * 0.7071068;
        auto INPUT_1 = q[1];
        auto DIV_123 = INPUT_1 * 0.5;
        auto SIN_124 = DIV_123.sin();
        auto COS_130 = DIV_123.cos();
        auto MUL_149 = MUL_78 * COS_130;
        auto MUL_134 = MUL_78 * SIN_124;
        auto MUL_132 = MUL_80 * COS_130;
        auto SUB_155 = MUL_132 - MUL_134;
        auto ADD_136 = MUL_132 + MUL_134;
        auto MUL_146 = MUL_80 * SIN_124;
        auto SUB_144 = MUL_146 - MUL_149;
        auto ADD_150 = MUL_146 + MUL_149;
        auto MUL_2509 = SUB_155 * ADD_150;
        auto MUL_2510 = SUB_155 * SUB_144;
        auto MUL_2516 = ADD_136 * ADD_150;
        auto SUB_2549 = MUL_2510 - MUL_2516;
        auto MUL_2551 = SUB_2549 * 2.0;
        auto MUL_2580 = MUL_2551 * 0.04;
        auto MUL_2514 = ADD_136 * SUB_144;
        auto ADD_2534 = MUL_2514 + MUL_2509;
        auto MUL_2537 = ADD_2534 * 2.0;
        auto MUL_2570 = MUL_2537 * 0.085;
        auto ADD_2585 = MUL_2570 + MUL_2580;
        auto MUL_2512 = SUB_155 * ADD_136;
        auto MUL_2508 = ADD_150 * ADD_150;
        auto MUL_2518 = SUB_144 * ADD_150;
        auto ADD_2552 = MUL_2518 + MUL_2512;
        auto MUL_2554 = ADD_2552 * 2.0;
        auto MUL_2582 = MUL_2554 * 0.04;
        auto MUL_2511 = ADD_136 * ADD_136;
        auto ADD_2539 = MUL_2508 + MUL_2511;
        auto MUL_2542 = ADD_2539 * 2.0;
        auto SUB_2545 = 1.0 - MUL_2542;
        auto MUL_2573 = SUB_2545 * 0.085;
        auto SUB_2586 = MUL_2582 - MUL_2573;
        auto SUB_2546 = MUL_2518 - MUL_2512;
        auto MUL_2548 = SUB_2546 * 2.0;
        auto MUL_2577 = MUL_2548 * 0.085;
        auto MUL_2507 = SUB_144 * SUB_144;
        auto ADD_2555 = MUL_2507 + MUL_2511;
        auto MUL_2558 = ADD_2555 * 2.0;
        auto SUB_2561 = 1.0 - MUL_2558;
        auto MUL_2584 = SUB_2561 * 0.04;
        auto SUB_2587 = MUL_2584 - MUL_2577;
        auto ADD_2588 = 0.333 + SUB_2587;
        auto MUL_2602 = MUL_2551 * 0.03;
        auto MUL_2604 = MUL_2554 * 0.03;
        auto MUL_2606 = SUB_2561 * 0.03;
        auto ADD_2607 = 0.333 + MUL_2606;
        auto MUL_2621 = MUL_2551 * 0.08;
        auto MUL_2623 = MUL_2554 * 0.08;
        auto MUL_2625 = SUB_2561 * 0.08;
        auto ADD_2626 = 0.333 + MUL_2625;
        auto MUL_2635 = MUL_2537 * 0.12;
        auto MUL_2638 = SUB_2545 * 0.12;
        auto NEGATE_2639 = -MUL_2638;
        auto MUL_2642 = MUL_2548 * 0.12;
        auto SUB_2650 = 0.333 - MUL_2642;
        auto MUL_2659 = MUL_2537 * 0.17;
        auto MUL_2662 = SUB_2545 * 0.17;
        auto NEGATE_2663 = -MUL_2662;
        auto MUL_2666 = MUL_2548 * 0.17;
        auto SUB_2674 = 0.333 - MUL_2666;
        if (/*panda_link2*/ sphere_environment_in_collision(environment, ADD_2585, SUB_2586, ADD_2588, 0.154))
        {
            if (sphere_environment_in_collision(environment, MUL_2602, MUL_2604, ADD_2607, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_2621, MUL_2623, ADD_2626, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_2635, NEGATE_2639, SUB_2650, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, MUL_2659, NEGATE_2663, SUB_2674, 0.06))
            {
                return false;
            }
        }  // (22, 87)
        auto MUL_188 = SUB_155 * 0.7071068;
        auto MUL_204 = ADD_150 * 0.7071068;
        auto MUL_202 = SUB_144 * 0.7071068;
        auto SUB_215 = MUL_204 - MUL_202;
        auto ADD_205 = MUL_202 + MUL_204;
        auto MUL_190 = ADD_136 * 0.7071068;
        auto SUB_192 = MUL_188 - MUL_190;
        auto ADD_221 = MUL_188 + MUL_190;
        auto MUL_230 = ADD_150 * 0.316;
        auto MUL_241 = SUB_155 * MUL_230;
        auto MUL_234 = ADD_136 * 0.316;
        auto MUL_242 = SUB_144 * MUL_234;
        auto ADD_243 = MUL_241 + MUL_242;
        auto MUL_246 = ADD_243 * 2.0;
        auto INPUT_2 = q[2];
        auto DIV_268 = INPUT_2 * 0.5;
        auto SIN_269 = DIV_268.sin();
        auto COS_275 = DIV_268.cos();
        auto MUL_292 = ADD_221 * COS_275;
        auto MUL_287 = ADD_221 * SIN_269;
        auto MUL_290 = SUB_215 * COS_275;
        auto ADD_291 = MUL_287 + MUL_290;
        auto MUL_2686 = ADD_291 * ADD_291;
        auto MUL_295 = SUB_215 * SIN_269;
        auto SUB_296 = MUL_292 - MUL_295;
        auto MUL_2687 = SUB_296 * ADD_291;
        auto MUL_277 = SUB_192 * COS_275;
        auto MUL_282 = SUB_192 * SIN_269;
        auto MUL_284 = ADD_205 * COS_275;
        auto SUB_285 = MUL_284 - MUL_282;
        auto MUL_2688 = SUB_296 * SUB_285;
        auto MUL_2685 = SUB_285 * SUB_285;
        auto ADD_2694 = MUL_2685 + MUL_2686;
        auto MUL_2697 = ADD_2694 * 2.0;
        auto SUB_2700 = 1.0 - MUL_2697;
        auto MUL_2734 = SUB_2700 * 0.039;
        auto MUL_278 = ADD_205 * SIN_269;
        auto ADD_279 = MUL_277 + MUL_278;
        auto MUL_2692 = ADD_279 * ADD_291;
        auto ADD_2720 = MUL_2692 + MUL_2688;
        auto MUL_2722 = ADD_2720 * 2.0;
        auto MUL_2747 = MUL_2722 * 0.052;
        auto MUL_2691 = ADD_279 * SUB_285;
        auto SUB_2707 = MUL_2691 - MUL_2687;
        auto MUL_2709 = SUB_2707 * 2.0;
        auto MUL_2740 = MUL_2709 * 0.028;
        auto ADD_2757 = MUL_2734 + MUL_2740;
        auto SUB_2760 = ADD_2757 - MUL_2747;
        auto ADD_2763 = MUL_246 + SUB_2760;
        auto ADD_2701 = MUL_2691 + MUL_2687;
        auto MUL_2703 = ADD_2701 * 2.0;
        auto MUL_2736 = MUL_2703 * 0.039;
        auto MUL_2690 = SUB_296 * ADD_279;
        auto MUL_2693 = SUB_285 * ADD_291;
        auto SUB_2723 = MUL_2693 - MUL_2690;
        auto MUL_2725 = SUB_2723 * 2.0;
        auto MUL_2751 = MUL_2725 * 0.052;
        auto MUL_2689 = ADD_279 * ADD_279;
        auto ADD_2710 = MUL_2686 + MUL_2689;
        auto MUL_2713 = ADD_2710 * 2.0;
        auto SUB_2716 = 1.0 - MUL_2713;
        auto MUL_2742 = SUB_2716 * 0.028;
        auto ADD_2758 = MUL_2736 + MUL_2742;
        auto SUB_2761 = ADD_2758 - MUL_2751;
        auto MUL_252 = ADD_150 * MUL_230;
        auto MUL_250 = ADD_136 * MUL_234;
        auto ADD_253 = MUL_250 + MUL_252;
        auto MUL_255 = ADD_253 * 2.0;
        auto SUB_258 = MUL_255 - 0.316;
        auto ADD_2764 = SUB_258 + SUB_2761;
        auto SUB_2704 = MUL_2692 - MUL_2688;
        auto ADD_2717 = MUL_2693 + MUL_2690;
        auto ADD_2726 = MUL_2685 + MUL_2689;
        auto MUL_2729 = ADD_2726 * 2.0;
        auto SUB_2732 = 1.0 - MUL_2729;
        auto MUL_2755 = SUB_2732 * 0.052;
        auto MUL_2719 = ADD_2717 * 2.0;
        auto MUL_2744 = MUL_2719 * 0.028;
        auto MUL_2706 = SUB_2704 * 2.0;
        auto MUL_2738 = MUL_2706 * 0.039;
        auto ADD_2759 = MUL_2738 + MUL_2744;
        auto SUB_2762 = ADD_2759 - MUL_2755;
        auto MUL_259 = SUB_155 * MUL_234;
        auto MUL_261 = SUB_144 * MUL_230;
        auto SUB_262 = MUL_259 - MUL_261;
        auto MUL_264 = SUB_262 * 2.0;
        auto ADD_266 = 0.333 + MUL_264;
        auto ADD_2765 = ADD_266 + SUB_2762;
        auto MUL_2780 = MUL_2722 * 0.1;
        auto SUB_2790 = MUL_246 - MUL_2780;
        auto MUL_2784 = MUL_2725 * 0.1;
        auto SUB_2791 = SUB_258 - MUL_2784;
        auto MUL_2788 = SUB_2732 * 0.1;
        auto SUB_2792 = ADD_266 - MUL_2788;
        auto MUL_2807 = MUL_2722 * 0.06;
        auto SUB_2817 = MUL_246 - MUL_2807;
        auto MUL_2811 = MUL_2725 * 0.06;
        auto SUB_2818 = SUB_258 - MUL_2811;
        auto MUL_2815 = SUB_2732 * 0.06;
        auto SUB_2819 = ADD_266 - MUL_2815;
        auto MUL_2827 = MUL_2709 * 0.06;
        auto MUL_2821 = SUB_2700 * 0.08;
        auto ADD_2838 = MUL_2821 + MUL_2827;
        auto ADD_2841 = MUL_246 + ADD_2838;
        auto MUL_2829 = SUB_2716 * 0.06;
        auto MUL_2823 = MUL_2703 * 0.08;
        auto ADD_2839 = MUL_2823 + MUL_2829;
        auto ADD_2842 = SUB_258 + ADD_2839;
        auto MUL_2831 = MUL_2719 * 0.06;
        auto MUL_2825 = MUL_2706 * 0.08;
        auto ADD_2840 = MUL_2825 + MUL_2831;
        auto ADD_2843 = ADD_266 + ADD_2840;
        auto MUL_2851 = MUL_2709 * 0.02;
        auto ADD_2862 = MUL_2821 + MUL_2851;
        auto ADD_2865 = MUL_246 + ADD_2862;
        auto MUL_2853 = SUB_2716 * 0.02;
        auto ADD_2863 = MUL_2823 + MUL_2853;
        auto ADD_2866 = SUB_258 + ADD_2863;
        auto MUL_2855 = MUL_2719 * 0.02;
        auto ADD_2864 = MUL_2825 + MUL_2855;
        auto ADD_2867 = ADD_266 + ADD_2864;
        if (/*panda_link3*/ sphere_environment_in_collision(environment, ADD_2763, ADD_2764, ADD_2765, 0.128))
        {
            if (sphere_environment_in_collision(environment, SUB_2790, SUB_2791, SUB_2792, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2817, SUB_2818, SUB_2819, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2841, ADD_2842, ADD_2843, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2865, ADD_2866, ADD_2867, 0.055))
            {
                return false;
            }
        }  // (87, 208)
        auto MUL_329 = SUB_296 * 0.7071068;
        auto MUL_344 = ADD_291 * 0.7071068;
        auto MUL_342 = SUB_285 * 0.7071068;
        auto SUB_355 = MUL_344 - MUL_342;
        auto ADD_345 = MUL_342 + MUL_344;
        auto MUL_331 = ADD_279 * 0.7071068;
        auto SUB_360 = MUL_329 - MUL_331;
        auto ADD_332 = MUL_329 + MUL_331;
        auto MUL_377 = ADD_291 * 0.0825;
        auto MUL_382 = ADD_291 * MUL_377;
        auto MUL_372 = SUB_285 * 0.0825;
        auto MUL_380 = SUB_285 * MUL_372;
        auto ADD_384 = MUL_380 + MUL_382;
        auto MUL_387 = ADD_384 * 2.0;
        auto SUB_390 = 0.0825 - MUL_387;
        auto ADD_409 = MUL_246 + SUB_390;
        auto INPUT_3 = q[3];
        auto DIV_413 = INPUT_3 * 0.5;
        auto SIN_414 = DIV_413.sin();
        auto COS_420 = DIV_413.cos();
        auto MUL_437 = SUB_360 * COS_420;
        auto MUL_432 = SUB_360 * SIN_414;
        auto MUL_435 = SUB_355 * COS_420;
        auto ADD_436 = MUL_432 + MUL_435;
        auto MUL_2879 = ADD_436 * ADD_436;
        auto MUL_440 = SUB_355 * SIN_414;
        auto SUB_441 = MUL_437 - MUL_440;
        auto MUL_2880 = SUB_441 * ADD_436;
        auto MUL_422 = ADD_332 * COS_420;
        auto MUL_427 = ADD_332 * SIN_414;
        auto MUL_429 = ADD_345 * COS_420;
        auto SUB_430 = MUL_429 - MUL_427;
        auto MUL_2881 = SUB_441 * SUB_430;
        auto MUL_2878 = SUB_430 * SUB_430;
        auto ADD_2887 = MUL_2878 + MUL_2879;
        auto MUL_2890 = ADD_2887 * 2.0;
        auto SUB_2893 = 1.0 - MUL_2890;
        auto MUL_2928 = SUB_2893 * 0.042;
        auto MUL_423 = ADD_345 * SIN_414;
        auto ADD_424 = MUL_422 + MUL_423;
        auto MUL_2885 = ADD_424 * ADD_436;
        auto ADD_2913 = MUL_2885 + MUL_2881;
        auto MUL_2915 = ADD_2913 * 2.0;
        auto MUL_2945 = MUL_2915 * 0.029;
        auto MUL_2884 = ADD_424 * SUB_430;
        auto SUB_2900 = MUL_2884 - MUL_2880;
        auto MUL_2902 = SUB_2900 * 2.0;
        auto MUL_2939 = MUL_2902 * 0.049;
        auto SUB_2950 = MUL_2939 - MUL_2928;
        auto ADD_2953 = SUB_2950 + MUL_2945;
        auto ADD_2956 = ADD_409 + ADD_2953;
        auto ADD_2894 = MUL_2884 + MUL_2880;
        auto MUL_2896 = ADD_2894 * 2.0;
        auto MUL_2932 = MUL_2896 * 0.042;
        auto MUL_2883 = SUB_441 * ADD_424;
        auto MUL_2886 = SUB_430 * ADD_436;
        auto SUB_2916 = MUL_2886 - MUL_2883;
        auto MUL_2918 = SUB_2916 * 2.0;
        auto MUL_2947 = MUL_2918 * 0.029;
        auto MUL_2882 = ADD_424 * ADD_424;
        auto ADD_2903 = MUL_2879 + MUL_2882;
        auto MUL_2906 = ADD_2903 * 2.0;
        auto SUB_2909 = 1.0 - MUL_2906;
        auto MUL_2941 = SUB_2909 * 0.049;
        auto SUB_2951 = MUL_2941 - MUL_2932;
        auto ADD_2954 = SUB_2951 + MUL_2947;
        auto MUL_392 = SUB_296 * MUL_377;
        auto MUL_393 = ADD_279 * MUL_372;
        auto ADD_395 = MUL_392 + MUL_393;
        auto MUL_398 = ADD_395 * 2.0;
        auto ADD_410 = SUB_258 + MUL_398;
        auto ADD_2957 = ADD_410 + ADD_2954;
        auto SUB_2897 = MUL_2885 - MUL_2881;
        auto ADD_2910 = MUL_2886 + MUL_2883;
        auto ADD_2919 = MUL_2878 + MUL_2882;
        auto MUL_2922 = ADD_2919 * 2.0;
        auto SUB_2925 = 1.0 - MUL_2922;
        auto MUL_2949 = SUB_2925 * 0.029;
        auto MUL_2912 = ADD_2910 * 2.0;
        auto MUL_2943 = MUL_2912 * 0.049;
        auto MUL_2899 = SUB_2897 * 2.0;
        auto MUL_2936 = MUL_2899 * 0.042;
        auto SUB_2952 = MUL_2943 - MUL_2936;
        auto ADD_2955 = SUB_2952 + MUL_2949;
        auto MUL_400 = SUB_296 * MUL_372;
        auto MUL_402 = ADD_279 * MUL_377;
        auto SUB_404 = MUL_402 - MUL_400;
        auto MUL_407 = SUB_404 * 2.0;
        auto ADD_411 = ADD_266 + MUL_407;
        auto ADD_2958 = ADD_411 + ADD_2955;
        auto MUL_2961 = SUB_2893 * 0.08;
        auto MUL_2972 = MUL_2902 * 0.095;
        auto SUB_2983 = MUL_2972 - MUL_2961;
        auto ADD_2986 = ADD_409 + SUB_2983;
        auto MUL_2974 = SUB_2909 * 0.095;
        auto MUL_2965 = MUL_2896 * 0.08;
        auto SUB_2984 = MUL_2974 - MUL_2965;
        auto ADD_2987 = ADD_410 + SUB_2984;
        auto MUL_2976 = MUL_2912 * 0.095;
        auto MUL_2969 = MUL_2899 * 0.08;
        auto SUB_2985 = MUL_2976 - MUL_2969;
        auto ADD_2988 = ADD_411 + SUB_2985;
        auto MUL_3002 = MUL_2915 * 0.02;
        auto ADD_3007 = ADD_409 + MUL_3002;
        auto MUL_3004 = MUL_2918 * 0.02;
        auto ADD_3008 = ADD_410 + MUL_3004;
        auto MUL_3006 = SUB_2925 * 0.02;
        auto ADD_3009 = ADD_411 + MUL_3006;
        auto MUL_3023 = MUL_2915 * 0.06;
        auto ADD_3028 = ADD_409 + MUL_3023;
        auto MUL_3025 = MUL_2918 * 0.06;
        auto ADD_3029 = ADD_410 + MUL_3025;
        auto MUL_3027 = SUB_2925 * 0.06;
        auto ADD_3030 = ADD_411 + MUL_3027;
        auto MUL_3044 = MUL_2902 * 0.06;
        auto SUB_3055 = MUL_3044 - MUL_2961;
        auto ADD_3058 = ADD_409 + SUB_3055;
        auto MUL_3046 = SUB_2909 * 0.06;
        auto SUB_3056 = MUL_3046 - MUL_2965;
        auto ADD_3059 = ADD_410 + SUB_3056;
        auto MUL_3048 = MUL_2912 * 0.06;
        auto SUB_3057 = MUL_3048 - MUL_2969;
        auto ADD_3060 = ADD_411 + SUB_3057;
        if (/*panda_link4*/ sphere_environment_in_collision(environment, ADD_2956, ADD_2957, ADD_2958, 0.126))
        {
            if (sphere_environment_in_collision(environment, ADD_2986, ADD_2987, ADD_2988, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3007, ADD_3008, ADD_3009, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3028, ADD_3029, ADD_3030, 0.055))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3058, ADD_3059, ADD_3060, 0.055))
            {
                return false;
            }
        }  // (208, 331)
        auto MUL_475 = SUB_441 * 0.7071068;
        auto MUL_492 = ADD_436 * 0.7071068;
        auto MUL_533 = ADD_436 * 0.0825;
        auto MUL_539 = ADD_436 * MUL_533;
        auto MUL_489 = SUB_430 * 0.7071068;
        auto SUB_494 = MUL_489 - MUL_492;
        auto ADD_505 = MUL_489 + MUL_492;
        auto MUL_526 = SUB_430 * 0.0825;
        auto MUL_478 = ADD_424 * 0.7071068;
        auto SUB_479 = MUL_478 - MUL_475;
        auto ADD_512 = MUL_475 + MUL_478;
        auto MUL_520 = ADD_436 * 0.384;
        auto MUL_535 = SUB_441 * MUL_520;
        auto MUL_523 = ADD_424 * 0.384;
        auto ADD_528 = MUL_523 + MUL_526;
        auto MUL_537 = SUB_430 * ADD_528;
        auto SUB_538 = MUL_537 - MUL_535;
        auto ADD_540 = SUB_538 + MUL_539;
        auto MUL_542 = ADD_540 * 2.0;
        auto SUB_545 = MUL_542 - 0.0825;
        auto ADD_570 = ADD_409 + SUB_545;
        auto INPUT_4 = q[4];
        auto DIV_574 = INPUT_4 * 0.5;
        auto SIN_575 = DIV_574.sin();
        auto COS_581 = DIV_574.cos();
        auto MUL_598 = ADD_512 * COS_581;
        auto MUL_593 = ADD_512 * SIN_575;
        auto MUL_596 = ADD_505 * COS_581;
        auto ADD_597 = MUL_593 + MUL_596;
        auto MUL_3072 = ADD_597 * ADD_597;
        auto MUL_601 = ADD_505 * SIN_575;
        auto SUB_602 = MUL_598 - MUL_601;
        auto MUL_3073 = SUB_602 * ADD_597;
        auto MUL_583 = SUB_479 * COS_581;
        auto MUL_588 = SUB_479 * SIN_575;
        auto MUL_590 = SUB_494 * COS_581;
        auto SUB_591 = MUL_590 - MUL_588;
        auto MUL_3074 = SUB_602 * SUB_591;
        auto MUL_3071 = SUB_591 * SUB_591;
        auto ADD_3080 = MUL_3071 + MUL_3072;
        auto MUL_3083 = ADD_3080 * 2.0;
        auto SUB_3086 = 1.0 - MUL_3083;
        auto MUL_3121 = SUB_3086 * 0.001;
        auto MUL_584 = SUB_494 * SIN_575;
        auto ADD_585 = MUL_583 + MUL_584;
        auto MUL_3078 = ADD_585 * ADD_597;
        auto ADD_3106 = MUL_3078 + MUL_3074;
        auto MUL_3108 = ADD_3106 * 2.0;
        auto MUL_3139 = MUL_3108 * 0.11;
        auto MUL_3077 = ADD_585 * SUB_591;
        auto SUB_3093 = MUL_3077 - MUL_3073;
        auto MUL_3095 = SUB_3093 * 2.0;
        auto MUL_3132 = MUL_3095 * 0.037;
        auto SUB_3149 = MUL_3132 - MUL_3121;
        auto SUB_3152 = SUB_3149 - MUL_3139;
        auto ADD_3155 = ADD_570 + SUB_3152;
        auto ADD_3087 = MUL_3077 + MUL_3073;
        auto MUL_3089 = ADD_3087 * 2.0;
        auto MUL_3125 = MUL_3089 * 0.001;
        auto MUL_3076 = SUB_602 * ADD_585;
        auto MUL_3079 = SUB_591 * ADD_597;
        auto SUB_3109 = MUL_3079 - MUL_3076;
        auto MUL_3111 = SUB_3109 * 2.0;
        auto MUL_3143 = MUL_3111 * 0.11;
        auto MUL_3075 = ADD_585 * ADD_585;
        auto ADD_3096 = MUL_3072 + MUL_3075;
        auto MUL_3099 = ADD_3096 * 2.0;
        auto SUB_3102 = 1.0 - MUL_3099;
        auto MUL_3134 = SUB_3102 * 0.037;
        auto SUB_3150 = MUL_3134 - MUL_3125;
        auto SUB_3153 = SUB_3150 - MUL_3143;
        auto MUL_547 = SUB_441 * MUL_533;
        auto MUL_552 = ADD_436 * MUL_520;
        auto MUL_549 = ADD_424 * ADD_528;
        auto ADD_550 = MUL_547 + MUL_549;
        auto ADD_554 = ADD_550 + MUL_552;
        auto MUL_557 = ADD_554 * 2.0;
        auto SUB_560 = 0.384 - MUL_557;
        auto ADD_571 = ADD_410 + SUB_560;
        auto ADD_3156 = ADD_571 + SUB_3153;
        auto SUB_3090 = MUL_3078 - MUL_3074;
        auto ADD_3103 = MUL_3079 + MUL_3076;
        auto ADD_3112 = MUL_3071 + MUL_3075;
        auto MUL_3115 = ADD_3112 * 2.0;
        auto SUB_3118 = 1.0 - MUL_3115;
        auto MUL_3147 = SUB_3118 * 0.11;
        auto MUL_3105 = ADD_3103 * 2.0;
        auto MUL_3136 = MUL_3105 * 0.037;
        auto MUL_3092 = SUB_3090 * 2.0;
        auto MUL_3129 = MUL_3092 * 0.001;
        auto SUB_3151 = MUL_3136 - MUL_3129;
        auto SUB_3154 = SUB_3151 - MUL_3147;
        auto MUL_561 = SUB_441 * ADD_528;
        auto MUL_564 = SUB_430 * MUL_520;
        auto MUL_562 = ADD_424 * MUL_533;
        auto SUB_563 = MUL_561 - MUL_562;
        auto ADD_566 = SUB_563 + MUL_564;
        auto MUL_568 = ADD_566 * 2.0;
        auto ADD_572 = ADD_411 + MUL_568;
        auto ADD_3157 = ADD_572 + SUB_3154;
        auto MUL_3165 = MUL_3095 * 0.055;
        auto ADD_3176 = ADD_570 + MUL_3165;
        auto MUL_3167 = SUB_3102 * 0.055;
        auto ADD_3177 = ADD_571 + MUL_3167;
        auto MUL_3169 = MUL_3105 * 0.055;
        auto ADD_3178 = ADD_572 + MUL_3169;
        auto MUL_3186 = MUL_3095 * 0.075;
        auto ADD_3197 = ADD_570 + MUL_3186;
        auto MUL_3188 = SUB_3102 * 0.075;
        auto ADD_3198 = ADD_571 + MUL_3188;
        auto MUL_3190 = MUL_3105 * 0.075;
        auto ADD_3199 = ADD_572 + MUL_3190;
        auto MUL_3214 = MUL_3108 * 0.22;
        auto SUB_3224 = ADD_570 - MUL_3214;
        auto MUL_3218 = MUL_3111 * 0.22;
        auto SUB_3225 = ADD_571 - MUL_3218;
        auto MUL_3222 = SUB_3118 * 0.22;
        auto SUB_3226 = ADD_572 - MUL_3222;
        auto MUL_3241 = MUL_3108 * 0.18;
        auto MUL_3234 = MUL_3095 * 0.05;
        auto SUB_3251 = MUL_3234 - MUL_3241;
        auto ADD_3254 = ADD_570 + SUB_3251;
        auto MUL_3245 = MUL_3111 * 0.18;
        auto MUL_3236 = SUB_3102 * 0.05;
        auto SUB_3252 = MUL_3236 - MUL_3245;
        auto ADD_3255 = ADD_571 + SUB_3252;
        auto MUL_3249 = SUB_3118 * 0.18;
        auto MUL_3238 = MUL_3105 * 0.05;
        auto SUB_3253 = MUL_3238 - MUL_3249;
        auto ADD_3256 = ADD_572 + SUB_3253;
        auto MUL_3264 = MUL_3095 * 0.08;
        auto MUL_3271 = MUL_3108 * 0.14;
        auto MUL_3258 = SUB_3086 * 0.01;
        auto ADD_3281 = MUL_3258 + MUL_3264;
        auto SUB_3284 = ADD_3281 - MUL_3271;
        auto ADD_3287 = ADD_570 + SUB_3284;
        auto MUL_3275 = MUL_3111 * 0.14;
        auto MUL_3266 = SUB_3102 * 0.08;
        auto MUL_3260 = MUL_3089 * 0.01;
        auto ADD_3282 = MUL_3260 + MUL_3266;
        auto SUB_3285 = ADD_3282 - MUL_3275;
        auto ADD_3288 = ADD_571 + SUB_3285;
        auto MUL_3279 = SUB_3118 * 0.14;
        auto MUL_3268 = MUL_3105 * 0.08;
        auto MUL_3262 = MUL_3092 * 0.01;
        auto ADD_3283 = MUL_3262 + MUL_3268;
        auto SUB_3286 = ADD_3283 - MUL_3279;
        auto ADD_3289 = ADD_572 + SUB_3286;
        auto MUL_3297 = MUL_3095 * 0.085;
        auto ADD_3314 = MUL_3258 + MUL_3297;
        auto SUB_3317 = ADD_3314 - MUL_3139;
        auto ADD_3320 = ADD_570 + SUB_3317;
        auto MUL_3299 = SUB_3102 * 0.085;
        auto ADD_3315 = MUL_3260 + MUL_3299;
        auto SUB_3318 = ADD_3315 - MUL_3143;
        auto ADD_3321 = ADD_571 + SUB_3318;
        auto MUL_3301 = MUL_3105 * 0.085;
        auto ADD_3316 = MUL_3262 + MUL_3301;
        auto SUB_3319 = ADD_3316 - MUL_3147;
        auto ADD_3322 = ADD_572 + SUB_3319;
        auto MUL_3337 = MUL_3108 * 0.08;
        auto MUL_3330 = MUL_3095 * 0.09;
        auto ADD_3347 = MUL_3258 + MUL_3330;
        auto SUB_3350 = ADD_3347 - MUL_3337;
        auto ADD_3353 = ADD_570 + SUB_3350;
        auto MUL_3341 = MUL_3111 * 0.08;
        auto MUL_3332 = SUB_3102 * 0.09;
        auto ADD_3348 = MUL_3260 + MUL_3332;
        auto SUB_3351 = ADD_3348 - MUL_3341;
        auto ADD_3354 = ADD_571 + SUB_3351;
        auto MUL_3345 = SUB_3118 * 0.08;
        auto MUL_3334 = MUL_3105 * 0.09;
        auto ADD_3349 = MUL_3262 + MUL_3334;
        auto SUB_3352 = ADD_3349 - MUL_3345;
        auto ADD_3355 = ADD_572 + SUB_3352;
        auto MUL_3370 = MUL_3108 * 0.05;
        auto MUL_3363 = MUL_3095 * 0.095;
        auto ADD_3380 = MUL_3258 + MUL_3363;
        auto SUB_3383 = ADD_3380 - MUL_3370;
        auto ADD_3386 = ADD_570 + SUB_3383;
        auto MUL_3374 = MUL_3111 * 0.05;
        auto MUL_3365 = SUB_3102 * 0.095;
        auto ADD_3381 = MUL_3260 + MUL_3365;
        auto SUB_3384 = ADD_3381 - MUL_3374;
        auto ADD_3387 = ADD_571 + SUB_3384;
        auto MUL_3378 = SUB_3118 * 0.05;
        auto MUL_3367 = MUL_3105 * 0.095;
        auto ADD_3382 = MUL_3262 + MUL_3367;
        auto SUB_3385 = ADD_3382 - MUL_3378;
        auto ADD_3388 = ADD_572 + SUB_3385;
        auto SUB_3419 = MUL_3264 - MUL_3258;
        auto SUB_3422 = SUB_3419 - MUL_3271;
        auto ADD_3425 = ADD_570 + SUB_3422;
        auto SUB_3420 = MUL_3266 - MUL_3260;
        auto SUB_3423 = SUB_3420 - MUL_3275;
        auto ADD_3426 = ADD_571 + SUB_3423;
        auto SUB_3421 = MUL_3268 - MUL_3262;
        auto SUB_3424 = SUB_3421 - MUL_3279;
        auto ADD_3427 = ADD_572 + SUB_3424;
        auto SUB_3458 = MUL_3297 - MUL_3258;
        auto SUB_3461 = SUB_3458 - MUL_3139;
        auto ADD_3464 = ADD_570 + SUB_3461;
        auto SUB_3459 = MUL_3299 - MUL_3260;
        auto SUB_3462 = SUB_3459 - MUL_3143;
        auto ADD_3465 = ADD_571 + SUB_3462;
        auto SUB_3460 = MUL_3301 - MUL_3262;
        auto SUB_3463 = SUB_3460 - MUL_3147;
        auto ADD_3466 = ADD_572 + SUB_3463;
        auto SUB_3497 = MUL_3330 - MUL_3258;
        auto SUB_3500 = SUB_3497 - MUL_3337;
        auto ADD_3503 = ADD_570 + SUB_3500;
        auto SUB_3498 = MUL_3332 - MUL_3260;
        auto SUB_3501 = SUB_3498 - MUL_3341;
        auto ADD_3504 = ADD_571 + SUB_3501;
        auto SUB_3499 = MUL_3334 - MUL_3262;
        auto SUB_3502 = SUB_3499 - MUL_3345;
        auto ADD_3505 = ADD_572 + SUB_3502;
        auto SUB_3536 = MUL_3363 - MUL_3258;
        auto SUB_3539 = SUB_3536 - MUL_3370;
        auto ADD_3542 = ADD_570 + SUB_3539;
        auto SUB_3537 = MUL_3365 - MUL_3260;
        auto SUB_3540 = SUB_3537 - MUL_3374;
        auto ADD_3543 = ADD_571 + SUB_3540;
        auto SUB_3538 = MUL_3367 - MUL_3262;
        auto SUB_3541 = SUB_3538 - MUL_3378;
        auto ADD_3544 = ADD_572 + SUB_3541;
        if (/*panda_link0 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_3155, ADD_3156, ADD_3157, 0.176))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
        }  // (331, 557)
        if (/*panda_link1 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_3155, ADD_3156, ADD_3157, 0.176))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        if (/*panda_link2 vs. panda_link5*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_3155, ADD_3156, ADD_3157, 0.176))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        if (/*panda_link5*/ sphere_environment_in_collision(environment, ADD_3155, ADD_3156, ADD_3157, 0.176))
        {
            if (sphere_environment_in_collision(environment, ADD_3176, ADD_3177, ADD_3178, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3197, ADD_3198, ADD_3199, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3224, SUB_3225, SUB_3226, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3254, ADD_3255, ADD_3256, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3287, ADD_3288, ADD_3289, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3320, ADD_3321, ADD_3322, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3353, ADD_3354, ADD_3355, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3386, ADD_3387, ADD_3388, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3425, ADD_3426, ADD_3427, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3464, ADD_3465, ADD_3466, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3503, ADD_3504, ADD_3505, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3542, ADD_3543, ADD_3544, 0.025))
            {
                return false;
            }
        }  // (557, 557)
        auto MUL_663 = SUB_602 * 0.7071068;
        auto MUL_660 = ADD_597 * 0.7071068;
        auto MUL_657 = SUB_591 * 0.7071068;
        auto SUB_661 = MUL_660 - MUL_657;
        auto ADD_651 = MUL_657 + MUL_660;
        auto MUL_665 = ADD_585 * 0.7071068;
        auto SUB_666 = MUL_663 - MUL_665;
        auto ADD_638 = MUL_663 + MUL_665;
        auto INPUT_5 = q[5];
        auto DIV_703 = INPUT_5 * 0.5;
        auto SIN_704 = DIV_703.sin();
        auto COS_710 = DIV_703.cos();
        auto MUL_727 = SUB_666 * COS_710;
        auto MUL_722 = SUB_666 * SIN_704;
        auto MUL_725 = SUB_661 * COS_710;
        auto ADD_726 = MUL_722 + MUL_725;
        auto MUL_3572 = ADD_726 * ADD_726;
        auto MUL_730 = SUB_661 * SIN_704;
        auto SUB_731 = MUL_727 - MUL_730;
        auto MUL_3573 = SUB_731 * ADD_726;
        auto MUL_712 = ADD_638 * COS_710;
        auto MUL_717 = ADD_638 * SIN_704;
        auto MUL_719 = ADD_651 * COS_710;
        auto SUB_720 = MUL_719 - MUL_717;
        auto MUL_3571 = SUB_720 * SUB_720;
        auto ADD_3580 = MUL_3571 + MUL_3572;
        auto MUL_3583 = ADD_3580 * 2.0;
        auto SUB_3586 = 1.0 - MUL_3583;
        auto MUL_3620 = SUB_3586 * 0.042;
        auto MUL_713 = ADD_651 * SIN_704;
        auto ADD_714 = MUL_712 + MUL_713;
        auto MUL_3577 = ADD_714 * SUB_720;
        auto SUB_3593 = MUL_3577 - MUL_3573;
        auto MUL_3595 = SUB_3593 * 2.0;
        auto MUL_3626 = MUL_3595 * 0.014;
        auto ADD_3637 = MUL_3620 + MUL_3626;
        auto ADD_3640 = ADD_570 + ADD_3637;
        auto ADD_3587 = MUL_3577 + MUL_3573;
        auto MUL_3589 = ADD_3587 * 2.0;
        auto MUL_3622 = MUL_3589 * 0.042;
        auto MUL_3575 = ADD_714 * ADD_714;
        auto ADD_3596 = MUL_3572 + MUL_3575;
        auto MUL_3599 = ADD_3596 * 2.0;
        auto SUB_3602 = 1.0 - MUL_3599;
        auto MUL_3628 = SUB_3602 * 0.014;
        auto ADD_3638 = MUL_3622 + MUL_3628;
        auto ADD_3641 = ADD_571 + ADD_3638;
        auto MUL_3574 = SUB_731 * SUB_720;
        auto MUL_3576 = SUB_731 * ADD_714;
        auto MUL_3579 = SUB_720 * ADD_726;
        auto ADD_3603 = MUL_3579 + MUL_3576;
        auto MUL_3578 = ADD_714 * ADD_726;
        auto SUB_3590 = MUL_3578 - MUL_3574;
        auto MUL_3605 = ADD_3603 * 2.0;
        auto MUL_3630 = MUL_3605 * 0.014;
        auto MUL_3592 = SUB_3590 * 2.0;
        auto MUL_3624 = MUL_3592 * 0.042;
        auto ADD_3639 = MUL_3624 + MUL_3630;
        auto ADD_3642 = ADD_572 + ADD_3639;
        auto MUL_3662 = SUB_3586 * 0.08;
        auto MUL_3669 = MUL_3595 * 0.01;
        auto SUB_3685 = MUL_3662 - MUL_3669;
        auto ADD_3688 = ADD_570 + SUB_3685;
        auto MUL_3664 = MUL_3589 * 0.08;
        auto MUL_3673 = SUB_3602 * 0.01;
        auto SUB_3686 = MUL_3664 - MUL_3673;
        auto ADD_3689 = ADD_571 + SUB_3686;
        auto MUL_3677 = MUL_3605 * 0.01;
        auto MUL_3666 = MUL_3592 * 0.08;
        auto SUB_3687 = MUL_3666 - MUL_3677;
        auto ADD_3690 = ADD_572 + SUB_3687;
        auto MUL_3698 = MUL_3595 * 0.035;
        auto ADD_3709 = MUL_3662 + MUL_3698;
        auto ADD_3712 = ADD_570 + ADD_3709;
        auto MUL_3700 = SUB_3602 * 0.035;
        auto ADD_3710 = MUL_3664 + MUL_3700;
        auto ADD_3713 = ADD_571 + ADD_3710;
        auto MUL_3702 = MUL_3605 * 0.035;
        auto ADD_3711 = MUL_3666 + MUL_3702;
        auto ADD_3714 = ADD_572 + ADD_3711;
        if (/*panda_link0 vs. panda_link6*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_3640, ADD_3641, ADD_3642, 0.095))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_570, ADD_571, ADD_572, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3688, ADD_3689, ADD_3690, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3712, ADD_3713, ADD_3714, 0.052))
            {
                return false;
            }
        }  // (557, 637)
        if (/*panda_link1 vs. panda_link6*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_3640, ADD_3641, ADD_3642, 0.095))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_570, ADD_571, ADD_572, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3688, ADD_3689, ADD_3690, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3712, ADD_3713, ADD_3714, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_570, ADD_571, ADD_572, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3688, ADD_3689, ADD_3690, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3712, ADD_3713, ADD_3714, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_570, ADD_571, ADD_572, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3688, ADD_3689, ADD_3690, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3712, ADD_3713, ADD_3714, 0.052))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_570, ADD_571, ADD_572, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3688, ADD_3689, ADD_3690, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3712, ADD_3713, ADD_3714, 0.052))
            {
                return false;
            }
        }  // (637, 637)
        if (/*panda_link6*/ sphere_environment_in_collision(environment, ADD_3640, ADD_3641, ADD_3642, 0.095))
        {
            if (sphere_environment_in_collision(environment, ADD_570, ADD_571, ADD_572, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3688, ADD_3689, ADD_3690, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3712, ADD_3713, ADD_3714, 0.052))
            {
                return false;
            }
        }  // (637, 637)
        auto MUL_764 = SUB_731 * 0.7071068;
        auto MUL_779 = ADD_726 * 0.7071068;
        auto MUL_777 = SUB_720 * 0.7071068;
        auto SUB_790 = MUL_779 - MUL_777;
        auto ADD_780 = MUL_777 + MUL_779;
        auto MUL_766 = ADD_714 * 0.7071068;
        auto SUB_795 = MUL_764 - MUL_766;
        auto ADD_767 = MUL_764 + MUL_766;
        auto MUL_812 = ADD_726 * 0.088;
        auto MUL_817 = ADD_726 * MUL_812;
        auto MUL_807 = SUB_720 * 0.088;
        auto MUL_815 = SUB_720 * MUL_807;
        auto ADD_819 = MUL_815 + MUL_817;
        auto MUL_822 = ADD_819 * 2.0;
        auto SUB_825 = 0.088 - MUL_822;
        auto ADD_844 = ADD_570 + SUB_825;
        auto INPUT_6 = q[6];
        auto DIV_848 = INPUT_6 * 0.5;
        auto SIN_849 = DIV_848.sin();
        auto COS_855 = DIV_848.cos();
        auto MUL_872 = SUB_795 * COS_855;
        auto MUL_867 = SUB_795 * SIN_849;
        auto MUL_870 = SUB_790 * COS_855;
        auto ADD_871 = MUL_867 + MUL_870;
        auto MUL_3724 = ADD_871 * ADD_871;
        auto MUL_875 = SUB_790 * SIN_849;
        auto SUB_876 = MUL_872 - MUL_875;
        auto MUL_3725 = SUB_876 * ADD_871;
        auto MUL_857 = ADD_767 * COS_855;
        auto MUL_862 = ADD_767 * SIN_849;
        auto MUL_864 = ADD_780 * COS_855;
        auto SUB_865 = MUL_864 - MUL_862;
        auto MUL_3726 = SUB_876 * SUB_865;
        auto MUL_3723 = SUB_865 * SUB_865;
        auto ADD_3732 = MUL_3723 + MUL_3724;
        auto MUL_858 = ADD_780 * SIN_849;
        auto ADD_859 = MUL_857 + MUL_858;
        auto MUL_3730 = ADD_859 * ADD_871;
        auto ADD_3758 = MUL_3730 + MUL_3726;
        auto MUL_3729 = ADD_859 * SUB_865;
        auto SUB_3745 = MUL_3729 - MUL_3725;
        auto MUL_3760 = ADD_3758 * 2.0;
        auto MUL_3784 = MUL_3760 * 0.075;
        auto MUL_3747 = SUB_3745 * 2.0;
        auto MUL_3778 = MUL_3747 * 0.015;
        auto MUL_3735 = ADD_3732 * 2.0;
        auto SUB_3738 = 1.0 - MUL_3735;
        auto MUL_3772 = SUB_3738 * 0.015;
        auto ADD_3789 = MUL_3772 + MUL_3778;
        auto ADD_3792 = ADD_3789 + MUL_3784;
        auto ADD_3795 = ADD_844 + ADD_3792;
        auto ADD_3739 = MUL_3729 + MUL_3725;
        auto MUL_3728 = SUB_876 * ADD_859;
        auto MUL_3731 = SUB_865 * ADD_871;
        auto SUB_3761 = MUL_3731 - MUL_3728;
        auto MUL_3727 = ADD_859 * ADD_859;
        auto ADD_3748 = MUL_3724 + MUL_3727;
        auto MUL_827 = SUB_731 * MUL_812;
        auto MUL_828 = ADD_714 * MUL_807;
        auto ADD_830 = MUL_827 + MUL_828;
        auto MUL_833 = ADD_830 * 2.0;
        auto ADD_845 = ADD_571 + MUL_833;
        auto MUL_3763 = SUB_3761 * 2.0;
        auto MUL_3786 = MUL_3763 * 0.075;
        auto MUL_3751 = ADD_3748 * 2.0;
        auto SUB_3754 = 1.0 - MUL_3751;
        auto MUL_3780 = SUB_3754 * 0.015;
        auto MUL_3741 = ADD_3739 * 2.0;
        auto MUL_3774 = MUL_3741 * 0.015;
        auto ADD_3790 = MUL_3774 + MUL_3780;
        auto ADD_3793 = ADD_3790 + MUL_3786;
        auto ADD_3796 = ADD_845 + ADD_3793;
        auto SUB_3742 = MUL_3730 - MUL_3726;
        auto ADD_3755 = MUL_3731 + MUL_3728;
        auto ADD_3764 = MUL_3723 + MUL_3727;
        auto MUL_835 = SUB_731 * MUL_807;
        auto MUL_837 = ADD_714 * MUL_812;
        auto SUB_839 = MUL_837 - MUL_835;
        auto MUL_842 = SUB_839 * 2.0;
        auto ADD_846 = ADD_572 + MUL_842;
        auto MUL_3767 = ADD_3764 * 2.0;
        auto SUB_3770 = 1.0 - MUL_3767;
        auto MUL_3788 = SUB_3770 * 0.075;
        auto MUL_3757 = ADD_3755 * 2.0;
        auto MUL_3782 = MUL_3757 * 0.015;
        auto MUL_3744 = SUB_3742 * 2.0;
        auto MUL_3776 = MUL_3744 * 0.015;
        auto ADD_3791 = MUL_3776 + MUL_3782;
        auto ADD_3794 = ADD_3791 + MUL_3788;
        auto ADD_3797 = ADD_846 + ADD_3794;
        auto MUL_3811 = MUL_3760 * 0.07;
        auto ADD_3816 = ADD_844 + MUL_3811;
        auto MUL_3813 = MUL_3763 * 0.07;
        auto ADD_3817 = ADD_845 + MUL_3813;
        auto MUL_3815 = SUB_3770 * 0.07;
        auto ADD_3818 = ADD_846 + MUL_3815;
        auto MUL_3832 = MUL_3760 * 0.08;
        auto MUL_3826 = MUL_3747 * 0.04;
        auto MUL_3820 = SUB_3738 * 0.02;
        auto ADD_3837 = MUL_3820 + MUL_3826;
        auto ADD_3840 = ADD_3837 + MUL_3832;
        auto ADD_3843 = ADD_844 + ADD_3840;
        auto MUL_3834 = MUL_3763 * 0.08;
        auto MUL_3828 = SUB_3754 * 0.04;
        auto MUL_3822 = MUL_3741 * 0.02;
        auto ADD_3838 = MUL_3822 + MUL_3828;
        auto ADD_3841 = ADD_3838 + MUL_3834;
        auto ADD_3844 = ADD_845 + ADD_3841;
        auto MUL_3836 = SUB_3770 * 0.08;
        auto MUL_3830 = MUL_3757 * 0.04;
        auto MUL_3824 = MUL_3744 * 0.02;
        auto ADD_3839 = MUL_3824 + MUL_3830;
        auto ADD_3842 = ADD_3839 + MUL_3836;
        auto ADD_3845 = ADD_846 + ADD_3842;
        auto MUL_3853 = MUL_3747 * 0.02;
        auto MUL_3847 = SUB_3738 * 0.04;
        auto ADD_3864 = MUL_3847 + MUL_3853;
        auto ADD_3867 = ADD_3864 + MUL_3832;
        auto ADD_3870 = ADD_844 + ADD_3867;
        auto MUL_3855 = SUB_3754 * 0.02;
        auto MUL_3849 = MUL_3741 * 0.04;
        auto ADD_3865 = MUL_3849 + MUL_3855;
        auto ADD_3868 = ADD_3865 + MUL_3834;
        auto ADD_3871 = ADD_845 + ADD_3868;
        auto MUL_3857 = MUL_3757 * 0.02;
        auto MUL_3851 = MUL_3744 * 0.04;
        auto ADD_3866 = MUL_3851 + MUL_3857;
        auto ADD_3869 = ADD_3866 + MUL_3836;
        auto ADD_3872 = ADD_846 + ADD_3869;
        auto MUL_3886 = MUL_3760 * 0.085;
        auto MUL_3880 = MUL_3747 * 0.06;
        auto ADD_3891 = MUL_3847 + MUL_3880;
        auto ADD_3894 = ADD_3891 + MUL_3886;
        auto ADD_3897 = ADD_844 + ADD_3894;
        auto MUL_3888 = MUL_3763 * 0.085;
        auto MUL_3882 = SUB_3754 * 0.06;
        auto ADD_3892 = MUL_3849 + MUL_3882;
        auto ADD_3895 = ADD_3892 + MUL_3888;
        auto ADD_3898 = ADD_845 + ADD_3895;
        auto MUL_3890 = SUB_3770 * 0.085;
        auto MUL_3884 = MUL_3757 * 0.06;
        auto ADD_3893 = MUL_3851 + MUL_3884;
        auto ADD_3896 = ADD_3893 + MUL_3890;
        auto ADD_3899 = ADD_846 + ADD_3896;
        auto MUL_3901 = SUB_3738 * 0.06;
        auto ADD_3918 = MUL_3901 + MUL_3826;
        auto ADD_3921 = ADD_3918 + MUL_3886;
        auto ADD_3924 = ADD_844 + ADD_3921;
        auto MUL_3903 = MUL_3741 * 0.06;
        auto ADD_3919 = MUL_3903 + MUL_3828;
        auto ADD_3922 = ADD_3919 + MUL_3888;
        auto ADD_3925 = ADD_845 + ADD_3922;
        auto MUL_3905 = MUL_3744 * 0.06;
        auto ADD_3920 = MUL_3905 + MUL_3830;
        auto ADD_3923 = ADD_3920 + MUL_3890;
        auto ADD_3926 = ADD_846 + ADD_3923;
        if (/*panda_link0 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_3795, ADD_3796, ADD_3797, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
        }  // (637, 793)
        if (/*panda_link1 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_3795, ADD_3796, ADD_3797, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link2 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_3795, ADD_3796, ADD_3797, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link5 vs. panda_link7*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_3795, ADD_3796, ADD_3797, 0.072))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        if (/*panda_link7*/ sphere_environment_in_collision(environment, ADD_3795, ADD_3796, ADD_3797, 0.072))
        {
            if (sphere_environment_in_collision(environment, ADD_3816, ADD_3817, ADD_3818, 0.05))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3843, ADD_3844, ADD_3845, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3870, ADD_3871, ADD_3872, 0.025))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3897, ADD_3898, ADD_3899, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3924, ADD_3925, ADD_3926, 0.02))
            {
                return false;
            }
        }  // (793, 793)
        auto MUL_1066 = SUB_876 * 0.7071068;
        auto MUL_1072 = ADD_871 * 0.7071068;
        auto SUB_1073 = MUL_1066 - MUL_1072;
        auto ADD_1064 = MUL_1066 + MUL_1072;
        auto MUL_3989 = SUB_1073 * ADD_1064;
        auto MUL_1052 = SUB_865 * 0.7071068;
        auto MUL_1049 = ADD_859 * 0.7071068;
        auto SUB_1053 = MUL_1052 - MUL_1049;
        auto ADD_1043 = MUL_1049 + MUL_1052;
        auto MUL_3990 = SUB_1073 * SUB_1053;
        auto MUL_3994 = ADD_1043 * ADD_1064;
        auto ADD_4022 = MUL_3994 + MUL_3990;
        auto MUL_3993 = ADD_1043 * SUB_1053;
        auto SUB_4009 = MUL_3993 - MUL_3989;
        auto MUL_4024 = ADD_4022 * 2.0;
        auto MUL_4011 = SUB_4009 * 2.0;
        auto MUL_937 = SUB_865 * 0.107;
        auto MUL_948 = SUB_876 * MUL_937;
        auto MUL_945 = ADD_859 * 0.107;
        auto MUL_950 = ADD_871 * MUL_945;
        auto ADD_951 = MUL_948 + MUL_950;
        auto MUL_953 = ADD_951 * 2.0;
        auto ADD_975 = ADD_844 + MUL_953;
        auto MUL_4072 = MUL_4024 * 0.0436958;
        auto MUL_4066 = MUL_4011 * 1e-06;
        auto ADD_4077 = MUL_4066 + MUL_4072;
        auto ADD_4080 = ADD_975 + ADD_4077;
        auto MUL_3992 = SUB_1073 * ADD_1043;
        auto MUL_3988 = ADD_1064 * ADD_1064;
        auto MUL_3995 = SUB_1053 * ADD_1064;
        auto SUB_4025 = MUL_3995 - MUL_3992;
        auto MUL_3991 = ADD_1043 * ADD_1043;
        auto ADD_4012 = MUL_3988 + MUL_3991;
        auto MUL_956 = SUB_876 * MUL_945;
        auto MUL_959 = ADD_871 * MUL_937;
        auto SUB_960 = MUL_959 - MUL_956;
        auto MUL_962 = SUB_960 * 2.0;
        auto ADD_976 = ADD_845 + MUL_962;
        auto MUL_4027 = SUB_4025 * 2.0;
        auto MUL_4074 = MUL_4027 * 0.0436958;
        auto MUL_4015 = ADD_4012 * 2.0;
        auto SUB_4018 = 1.0 - MUL_4015;
        auto MUL_4068 = SUB_4018 * 1e-06;
        auto ADD_4078 = MUL_4068 + MUL_4074;
        auto ADD_4081 = ADD_976 + ADD_4078;
        auto ADD_4019 = MUL_3995 + MUL_3992;
        auto MUL_3987 = SUB_1053 * SUB_1053;
        auto ADD_4028 = MUL_3987 + MUL_3991;
        auto MUL_967 = SUB_865 * MUL_937;
        auto MUL_965 = ADD_859 * MUL_945;
        auto ADD_968 = MUL_965 + MUL_967;
        auto MUL_971 = ADD_968 * 2.0;
        auto SUB_974 = 0.107 - MUL_971;
        auto ADD_977 = ADD_846 + SUB_974;
        if (environment.attachments)
        {
            if (set_attachment_pose_hack(
                    environment, ADD_975, ADD_976, ADD_977, ADD_1043, SUB_1053, ADD_1064, SUB_1073))
            {
                return false;
            }
            if (attachment_environment_collision(environment))
            {
                return false;
            }
            if (/*attachment vs. panda_link0*/ attachment_sphere_collision<decltype(q[0])>(
                environment, 0.0, 0.0, 0.05, 0.08))
            {
                if (attachment_sphere_collision<decltype(q[0])>(environment, 0.0, 0.0, 0.05, 0.08))
                {
                    return false;
                }
            }
            if (/*attachment vs. panda_link1*/ attachment_sphere_collision<decltype(q[0])>(
                environment, SUB_2394, NEGATE_2396, 0.248, 0.154))
            {
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, MUL_2409, NEGATE_2413, 0.333, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, MUL_2433, NEGATE_2437, 0.333, 0.06))
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
            }
            if (/*attachment vs. panda_link2*/ attachment_sphere_collision<decltype(q[0])>(
                environment, ADD_2585, SUB_2586, ADD_2588, 0.154))
            {
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, MUL_2602, MUL_2604, ADD_2607, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, MUL_2621, MUL_2623, ADD_2626, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, MUL_2635, NEGATE_2639, SUB_2650, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, MUL_2659, NEGATE_2663, SUB_2674, 0.06))
                {
                    return false;
                }
            }
            if (/*attachment vs. panda_link5*/ attachment_sphere_collision<decltype(q[0])>(
                environment, ADD_3155, ADD_3156, ADD_3157, 0.176))
            {
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3176, ADD_3177, ADD_3178, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3197, ADD_3198, ADD_3199, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, SUB_3224, SUB_3225, SUB_3226, 0.06))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3254, ADD_3255, ADD_3256, 0.05))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3287, ADD_3288, ADD_3289, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3320, ADD_3321, ADD_3322, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3353, ADD_3354, ADD_3355, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3386, ADD_3387, ADD_3388, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3425, ADD_3426, ADD_3427, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3464, ADD_3465, ADD_3466, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3503, ADD_3504, ADD_3505, 0.025))
                {
                    return false;
                }
                if (attachment_sphere_collision<decltype(q[0])>(
                        environment, ADD_3542, ADD_3543, ADD_3544, 0.025))
                {
                    return false;
                }
            }
        }  // (793, 847)
        auto MUL_4031 = ADD_4028 * 2.0;
        auto SUB_4034 = 1.0 - MUL_4031;
        auto MUL_4076 = SUB_4034 * 0.0436958;
        auto MUL_4021 = ADD_4019 * 2.0;
        auto MUL_4070 = MUL_4021 * 1e-06;
        auto ADD_4079 = MUL_4070 + MUL_4076;
        auto ADD_4082 = ADD_977 + ADD_4079;
        if (/*panda_link0 vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4080, ADD_4081, ADD_4082, 0.054561))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
        }  // (847, 854)
        if (/*panda_link1 vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4080, ADD_4081, ADD_4082, 0.054561))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
        }  // (854, 854)
        if (/*panda_link2 vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4080, ADD_4081, ADD_4082, 0.054561))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
        }  // (854, 854)
        if (/*panda_link5 vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4080, ADD_4081, ADD_4082, 0.054561))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4080, ADD_4081, ADD_4082, 0.054561))
            {
                return false;
            }
        }  // (854, 854)
        if (/*robotiq_85_base_link*/ sphere_environment_in_collision(
            environment, ADD_4080, ADD_4081, ADD_4082, 0.054561))
        {
            return false;
        }  // (854, 854)
        auto MUL_1193 = SUB_1053 * 0.062792;
        auto MUL_1206 = SUB_1073 * MUL_1193;
        auto MUL_1202 = ADD_1043 * 0.062792;
        auto MUL_1204 = ADD_1064 * 0.0306011;
        auto SUB_1205 = MUL_1202 - MUL_1204;
        auto MUL_1210 = ADD_1064 * SUB_1205;
        auto MUL_1199 = SUB_1053 * 0.0306011;
        auto MUL_1207 = SUB_1053 * MUL_1199;
        auto SUB_1209 = MUL_1206 - MUL_1207;
        auto ADD_1211 = SUB_1209 + MUL_1210;
        auto MUL_1213 = ADD_1211 * 2.0;
        auto ADD_1215 = MUL_1213 + 0.0306011;
        auto ADD_1240 = ADD_975 + ADD_1215;
        auto INPUT_7 = q[7];
        auto DIV_1244 = INPUT_7 * 0.5;
        auto SIN_1245 = DIV_1244.sin();
        auto COS_1254 = DIV_1244.cos();
        auto MUL_1250 = SIN_1245 * 1.0;
        auto MUL_1261 = SUB_1073 * MUL_1250;
        auto MUL_1273 = SUB_1073 * COS_1254;
        auto MUL_1258 = ADD_1064 * MUL_1250;
        auto MUL_1271 = ADD_1064 * COS_1254;
        auto MUL_1275 = SUB_1053 * MUL_1250;
        auto ADD_1277 = MUL_1273 + MUL_1275;
        auto MUL_1264 = SUB_1053 * COS_1254;
        auto SUB_1265 = MUL_1264 - MUL_1261;
        auto MUL_4088 = ADD_1277 * SUB_1265;
        auto MUL_4085 = SUB_1265 * SUB_1265;
        auto MUL_1268 = ADD_1043 * MUL_1250;
        auto SUB_1272 = MUL_1271 - MUL_1268;
        auto MUL_4086 = SUB_1272 * SUB_1272;
        auto ADD_4094 = MUL_4085 + MUL_4086;
        auto MUL_1256 = ADD_1043 * COS_1254;
        auto ADD_1260 = MUL_1256 + MUL_1258;
        auto MUL_4092 = ADD_1260 * SUB_1272;
        auto ADD_4120 = MUL_4092 + MUL_4088;
        auto MUL_4122 = ADD_4120 * 2.0;
        auto MUL_4170 = MUL_4122 * 0.0032636;
        auto MUL_4097 = ADD_4094 * 2.0;
        auto SUB_4100 = 1.0 - MUL_4097;
        auto MUL_4158 = SUB_4100 * 0.0154672;
        auto ADD_4175 = MUL_4158 + MUL_4170;
        auto ADD_4178 = ADD_1240 + ADD_4175;
        auto MUL_4087 = ADD_1277 * SUB_1272;
        auto MUL_4090 = ADD_1277 * ADD_1260;
        auto MUL_4093 = SUB_1265 * SUB_1272;
        auto SUB_4123 = MUL_4093 - MUL_4090;
        auto MUL_4091 = ADD_1260 * SUB_1265;
        auto ADD_4101 = MUL_4091 + MUL_4087;
        auto MUL_1217 = SUB_1073 * SUB_1205;
        auto MUL_1222 = ADD_1064 * MUL_1193;
        auto MUL_1219 = ADD_1043 * MUL_1199;
        auto SUB_1221 = MUL_1219 - MUL_1217;
        auto ADD_1223 = SUB_1221 + MUL_1222;
        auto MUL_1225 = ADD_1223 * 2.0;
        auto ADD_1241 = ADD_976 + MUL_1225;
        auto MUL_4125 = SUB_4123 * 2.0;
        auto MUL_4172 = MUL_4125 * 0.0032636;
        auto MUL_4103 = ADD_4101 * 2.0;
        auto MUL_4160 = MUL_4103 * 0.0154672;
        auto ADD_4176 = MUL_4160 + MUL_4172;
        auto ADD_4179 = ADD_1241 + ADD_4176;
        auto SUB_4104 = MUL_4092 - MUL_4088;
        auto MUL_4089 = ADD_1260 * ADD_1260;
        auto ADD_4126 = MUL_4085 + MUL_4089;
        auto MUL_1227 = SUB_1073 * MUL_1199;
        auto MUL_1232 = SUB_1053 * MUL_1193;
        auto MUL_1229 = ADD_1043 * SUB_1205;
        auto ADD_1230 = MUL_1227 + MUL_1229;
        auto ADD_1233 = ADD_1230 + MUL_1232;
        auto MUL_1236 = ADD_1233 * 2.0;
        auto SUB_1239 = 0.062792 - MUL_1236;
        auto ADD_1242 = ADD_977 + SUB_1239;
        auto MUL_4129 = ADD_4126 * 2.0;
        auto SUB_4132 = 1.0 - MUL_4129;
        auto MUL_4174 = SUB_4132 * 0.0032636;
        auto MUL_4106 = SUB_4104 * 2.0;
        auto MUL_4162 = MUL_4106 * 0.0154672;
        auto ADD_4177 = MUL_4162 + MUL_4174;
        auto ADD_4180 = ADD_1242 + ADD_4177;
        if (/*left_outer_knuckle*/ sphere_environment_in_collision(
            environment, ADD_4178, ADD_4179, ADD_4180, 0.022718))
        {
            return false;
        }  // (854, 934)
        if (/*panda_link0 vs. left_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
        }  // (934, 934)
        if (/*panda_link1 vs. left_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4178, ADD_4179, ADD_4180, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
        }  // (934, 934)
        if (/*panda_link2 vs. left_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4178, ADD_4179, ADD_4180, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
        }  // (934, 934)
        if (/*panda_link5 vs. left_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4178, ADD_4179, ADD_4180, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4178, ADD_4179, ADD_4180, 0.022718))
            {
                return false;
            }
        }  // (934, 934)
        auto MUL_4220 = ADD_4120 * 2.0;
        auto MUL_4195 = ADD_4094 * 2.0;
        auto SUB_4198 = 1.0 - MUL_4195;
        auto MUL_4232 = SUB_4198 * 0.001;
        auto MUL_1354 = SUB_1272 * 0.031691;
        auto MUL_1347 = SUB_1265 * 0.031691;
        auto MUL_1359 = SUB_1265 * MUL_1347;
        auto MUL_4244 = MUL_4220 * 0.024;
        auto ADD_4249 = MUL_4232 + MUL_4244;
        auto MUL_1340 = SUB_1265 * 0.001934;
        auto MUL_1357 = ADD_1277 * MUL_1340;
        auto ADD_1361 = MUL_1357 + MUL_1359;
        auto MUL_1351 = ADD_1260 * 0.001934;
        auto ADD_1355 = MUL_1351 + MUL_1354;
        auto MUL_1363 = SUB_1272 * ADD_1355;
        auto ADD_1365 = ADD_1361 + MUL_1363;
        auto MUL_1368 = ADD_1365 * 2.0;
        auto SUB_1371 = 0.031691 - MUL_1368;
        auto ADD_1396 = ADD_1240 + SUB_1371;
        auto ADD_4252 = ADD_1396 + ADD_4249;
        auto MUL_1373 = ADD_1277 * ADD_1355;
        auto MUL_1377 = SUB_1272 * MUL_1340;
        auto MUL_1374 = ADD_1260 * MUL_1347;
        auto ADD_1376 = MUL_1373 + MUL_1374;
        auto SUB_1379 = ADD_1376 - MUL_1377;
        auto MUL_1381 = SUB_1379 * 2.0;
        auto ADD_1397 = ADD_1241 + MUL_1381;
        auto MUL_4223 = SUB_4123 * 2.0;
        auto MUL_4246 = MUL_4223 * 0.024;
        auto MUL_4201 = ADD_4101 * 2.0;
        auto MUL_4234 = MUL_4201 * 0.001;
        auto ADD_4250 = MUL_4234 + MUL_4246;
        auto ADD_4253 = ADD_1397 + ADD_4250;
        auto MUL_1383 = ADD_1277 * MUL_1347;
        auto MUL_1388 = SUB_1265 * MUL_1340;
        auto MUL_1385 = ADD_1260 * ADD_1355;
        auto SUB_1387 = MUL_1385 - MUL_1383;
        auto ADD_1390 = SUB_1387 + MUL_1388;
        auto MUL_1392 = ADD_1390 * 2.0;
        auto SUB_1395 = MUL_1392 - 0.001934;
        auto ADD_1398 = ADD_1242 + SUB_1395;
        auto MUL_4227 = ADD_4126 * 2.0;
        auto SUB_4230 = 1.0 - MUL_4227;
        auto MUL_4248 = SUB_4230 * 0.024;
        auto MUL_4204 = SUB_4104 * 2.0;
        auto MUL_4236 = MUL_4204 * 0.001;
        auto ADD_4251 = MUL_4236 + MUL_4248;
        auto ADD_4254 = ADD_1398 + ADD_4251;
        auto SUB_4205 = MUL_4091 - MUL_4087;
        auto MUL_4207 = SUB_4205 * 2.0;
        auto MUL_4268 = MUL_4220 * 0.0414929;
        auto MUL_4256 = SUB_4198 * 0.0025323;
        auto MUL_4262 = MUL_4207 * 0.000302;
        auto ADD_4273 = MUL_4256 + MUL_4262;
        auto ADD_4276 = ADD_4273 + MUL_4268;
        auto ADD_4279 = ADD_1396 + ADD_4276;
        auto ADD_4208 = MUL_4086 + MUL_4089;
        auto MUL_4270 = MUL_4223 * 0.0414929;
        auto MUL_4211 = ADD_4208 * 2.0;
        auto SUB_4214 = 1.0 - MUL_4211;
        auto MUL_4264 = SUB_4214 * 0.000302;
        auto MUL_4258 = MUL_4201 * 0.0025323;
        auto ADD_4274 = MUL_4258 + MUL_4264;
        auto ADD_4277 = ADD_4274 + MUL_4270;
        auto ADD_4280 = ADD_1397 + ADD_4277;
        auto ADD_4215 = MUL_4093 + MUL_4090;
        auto MUL_4272 = SUB_4230 * 0.0414929;
        auto MUL_4217 = ADD_4215 * 2.0;
        auto MUL_4266 = MUL_4217 * 0.000302;
        auto MUL_4260 = MUL_4204 * 0.0025323;
        auto ADD_4275 = MUL_4260 + MUL_4266;
        auto ADD_4278 = ADD_4275 + MUL_4272;
        auto ADD_4281 = ADD_1398 + ADD_4278;
        auto MUL_4295 = MUL_4220 * 0.0060249;
        auto MUL_4283 = SUB_4198 * 0.0003683;
        auto MUL_4289 = MUL_4207 * 0.000299;
        auto ADD_4300 = MUL_4283 + MUL_4289;
        auto ADD_4303 = ADD_4300 + MUL_4295;
        auto ADD_4306 = ADD_1396 + ADD_4303;
        auto MUL_4297 = MUL_4223 * 0.0060249;
        auto MUL_4291 = SUB_4214 * 0.000299;
        auto MUL_4285 = MUL_4201 * 0.0003683;
        auto ADD_4301 = MUL_4285 + MUL_4291;
        auto ADD_4304 = ADD_4301 + MUL_4297;
        auto ADD_4307 = ADD_1397 + ADD_4304;
        auto MUL_4299 = SUB_4230 * 0.0060249;
        auto MUL_4293 = MUL_4217 * 0.000299;
        auto MUL_4287 = MUL_4204 * 0.0003683;
        auto ADD_4302 = MUL_4287 + MUL_4293;
        auto ADD_4305 = ADD_4302 + MUL_4299;
        auto ADD_4308 = ADD_1398 + ADD_4305;
        auto MUL_4328 = MUL_4220 * 0.0238049;
        auto MUL_4310 = SUB_4198 * 0.0014533;
        auto MUL_4317 = MUL_4207 * 0.000343;
        auto SUB_4333 = MUL_4310 - MUL_4317;
        auto ADD_4336 = SUB_4333 + MUL_4328;
        auto ADD_4339 = ADD_1396 + ADD_4336;
        auto MUL_4330 = MUL_4223 * 0.0238049;
        auto MUL_4321 = SUB_4214 * 0.000343;
        auto MUL_4312 = MUL_4201 * 0.0014533;
        auto SUB_4334 = MUL_4312 - MUL_4321;
        auto ADD_4337 = SUB_4334 + MUL_4330;
        auto ADD_4340 = ADD_1397 + ADD_4337;
        auto MUL_4332 = SUB_4230 * 0.0238049;
        auto MUL_4325 = MUL_4217 * 0.000343;
        auto MUL_4314 = MUL_4204 * 0.0014533;
        auto SUB_4335 = MUL_4314 - MUL_4325;
        auto ADD_4338 = SUB_4335 + MUL_4332;
        auto ADD_4341 = ADD_1398 + ADD_4338;
        if (/*left_outer_finger*/ sphere_environment_in_collision(
            environment, ADD_4252, ADD_4253, ADD_4254, 0.033))
        {
            if (sphere_environment_in_collision(environment, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
        }  // (934, 1043)
        if (/*panda_link0 vs. left_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4252, ADD_4253, ADD_4254, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
        }  // (1043, 1043)
        if (/*panda_link1 vs. left_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4252, ADD_4253, ADD_4254, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
        }  // (1043, 1043)
        if (/*panda_link2 vs. left_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4252, ADD_4253, ADD_4254, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
        }  // (1043, 1043)
        if (/*panda_link5 vs. left_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4252, ADD_4253, ADD_4254, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4279, ADD_4280, ADD_4281, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4306, ADD_4307, ADD_4308, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4339, ADD_4340, ADD_4341, 0.015747))
            {
                return false;
            }
        }  // (1043, 1043)
        auto MUL_1487 = SUB_1053 * 0.0693075;
        auto MUL_1500 = SUB_1073 * MUL_1487;
        auto MUL_1496 = ADD_1043 * 0.0693075;
        auto MUL_1498 = ADD_1064 * 0.0127;
        auto SUB_1499 = MUL_1496 - MUL_1498;
        auto MUL_1504 = ADD_1064 * SUB_1499;
        auto MUL_1493 = SUB_1053 * 0.0127;
        auto MUL_1501 = SUB_1053 * MUL_1493;
        auto SUB_1503 = MUL_1500 - MUL_1501;
        auto ADD_1505 = SUB_1503 + MUL_1504;
        auto MUL_1507 = ADD_1505 * 2.0;
        auto ADD_1509 = MUL_1507 + 0.0127;
        auto ADD_1534 = ADD_975 + ADD_1509;
        auto INPUT_8 = q[8];
        auto DIV_1538 = INPUT_8 * 0.5;
        auto SIN_1539 = DIV_1538.sin();
        auto COS_1548 = DIV_1538.cos();
        auto MUL_1544 = SIN_1539 * 1.0;
        auto MUL_1555 = SUB_1073 * MUL_1544;
        auto MUL_1567 = SUB_1073 * COS_1548;
        auto MUL_1552 = ADD_1064 * MUL_1544;
        auto MUL_1565 = ADD_1064 * COS_1548;
        auto MUL_1569 = SUB_1053 * MUL_1544;
        auto ADD_1571 = MUL_1567 + MUL_1569;
        auto MUL_1558 = SUB_1053 * COS_1548;
        auto SUB_1559 = MUL_1558 - MUL_1555;
        auto MUL_4353 = ADD_1571 * SUB_1559;
        auto MUL_4350 = SUB_1559 * SUB_1559;
        auto MUL_1562 = ADD_1043 * MUL_1544;
        auto SUB_1566 = MUL_1565 - MUL_1562;
        auto MUL_4351 = SUB_1566 * SUB_1566;
        auto ADD_4359 = MUL_4350 + MUL_4351;
        auto MUL_1550 = ADD_1043 * COS_1548;
        auto ADD_1554 = MUL_1550 + MUL_1552;
        auto MUL_4357 = ADD_1554 * SUB_1566;
        auto ADD_4385 = MUL_4357 + MUL_4353;
        auto MUL_4362 = ADD_4359 * 2.0;
        auto SUB_4365 = 1.0 - MUL_4362;
        auto MUL_4399 = SUB_4365 * 0.017;
        auto MUL_4387 = ADD_4385 * 2.0;
        auto MUL_4411 = MUL_4387 * 0.022;
        auto ADD_4416 = MUL_4399 + MUL_4411;
        auto ADD_4419 = ADD_1534 + ADD_4416;
        auto MUL_4352 = ADD_1571 * SUB_1566;
        auto MUL_4355 = ADD_1571 * ADD_1554;
        auto MUL_4358 = SUB_1559 * SUB_1566;
        auto SUB_4388 = MUL_4358 - MUL_4355;
        auto MUL_4356 = ADD_1554 * SUB_1559;
        auto ADD_4366 = MUL_4356 + MUL_4352;
        auto MUL_1511 = SUB_1073 * SUB_1499;
        auto MUL_1516 = ADD_1064 * MUL_1487;
        auto MUL_1513 = ADD_1043 * MUL_1493;
        auto SUB_1515 = MUL_1513 - MUL_1511;
        auto ADD_1517 = SUB_1515 + MUL_1516;
        auto MUL_1519 = ADD_1517 * 2.0;
        auto ADD_1535 = ADD_976 + MUL_1519;
        auto MUL_4368 = ADD_4366 * 2.0;
        auto MUL_4401 = MUL_4368 * 0.017;
        auto MUL_4390 = SUB_4388 * 2.0;
        auto MUL_4413 = MUL_4390 * 0.022;
        auto ADD_4417 = MUL_4401 + MUL_4413;
        auto ADD_4420 = ADD_1535 + ADD_4417;
        auto SUB_4369 = MUL_4357 - MUL_4353;
        auto MUL_4354 = ADD_1554 * ADD_1554;
        auto ADD_4391 = MUL_4350 + MUL_4354;
        auto MUL_1521 = SUB_1073 * MUL_1493;
        auto MUL_1526 = SUB_1053 * MUL_1487;
        auto MUL_1523 = ADD_1043 * SUB_1499;
        auto ADD_1524 = MUL_1521 + MUL_1523;
        auto ADD_1527 = ADD_1524 + MUL_1526;
        auto MUL_1530 = ADD_1527 * 2.0;
        auto SUB_1533 = 0.0693075 - MUL_1530;
        auto ADD_1536 = ADD_977 + SUB_1533;
        auto MUL_4371 = SUB_4369 * 2.0;
        auto MUL_4394 = ADD_4391 * 2.0;
        auto SUB_4397 = 1.0 - MUL_4394;
        auto MUL_4415 = SUB_4397 * 0.022;
        auto MUL_4403 = MUL_4371 * 0.017;
        auto ADD_4418 = MUL_4403 + MUL_4415;
        auto ADD_4421 = ADD_1536 + ADD_4418;
        auto SUB_4372 = MUL_4356 - MUL_4352;
        auto MUL_4374 = SUB_4372 * 2.0;
        auto MUL_4435 = MUL_4387 * 0.0355595;
        auto MUL_4423 = SUB_4365 * 0.0270317;
        auto MUL_4429 = MUL_4374 * 0.000714;
        auto ADD_4440 = MUL_4423 + MUL_4429;
        auto ADD_4443 = ADD_4440 + MUL_4435;
        auto ADD_4446 = ADD_1534 + ADD_4443;
        auto ADD_4375 = MUL_4351 + MUL_4354;
        auto MUL_4425 = MUL_4368 * 0.0270317;
        auto MUL_4437 = MUL_4390 * 0.0355595;
        auto MUL_4378 = ADD_4375 * 2.0;
        auto SUB_4381 = 1.0 - MUL_4378;
        auto MUL_4431 = SUB_4381 * 0.000714;
        auto ADD_4441 = MUL_4425 + MUL_4431;
        auto ADD_4444 = ADD_4441 + MUL_4437;
        auto ADD_4447 = ADD_1535 + ADD_4444;
        auto ADD_4382 = MUL_4358 + MUL_4355;
        auto MUL_4439 = SUB_4397 * 0.0355595;
        auto MUL_4384 = ADD_4382 * 2.0;
        auto MUL_4433 = MUL_4384 * 0.000714;
        auto MUL_4427 = MUL_4371 * 0.0270317;
        auto ADD_4442 = MUL_4427 + MUL_4433;
        auto ADD_4445 = ADD_4442 + MUL_4439;
        auto ADD_4448 = ADD_1536 + ADD_4445;
        auto MUL_4462 = MUL_4387 * 0.0107115;
        auto MUL_4450 = SUB_4365 * 0.0081427;
        auto MUL_4456 = MUL_4374 * 0.007858;
        auto ADD_4467 = MUL_4450 + MUL_4456;
        auto ADD_4470 = ADD_4467 + MUL_4462;
        auto ADD_4473 = ADD_1534 + ADD_4470;
        auto MUL_4452 = MUL_4368 * 0.0081427;
        auto MUL_4464 = MUL_4390 * 0.0107115;
        auto MUL_4458 = SUB_4381 * 0.007858;
        auto ADD_4468 = MUL_4452 + MUL_4458;
        auto ADD_4471 = ADD_4468 + MUL_4464;
        auto ADD_4474 = ADD_1535 + ADD_4471;
        auto MUL_4466 = SUB_4397 * 0.0107115;
        auto MUL_4460 = MUL_4384 * 0.007858;
        auto MUL_4454 = MUL_4371 * 0.0081427;
        auto ADD_4469 = MUL_4454 + MUL_4460;
        auto ADD_4472 = ADD_4469 + MUL_4466;
        auto ADD_4475 = ADD_1536 + ADD_4472;
        auto MUL_4495 = MUL_4387 * 0.0118005;
        auto MUL_4484 = MUL_4374 * 0.010721;
        auto MUL_4477 = SUB_4365 * 0.0089687;
        auto SUB_4500 = MUL_4477 - MUL_4484;
        auto ADD_4503 = SUB_4500 + MUL_4495;
        auto ADD_4506 = ADD_1534 + ADD_4503;
        auto MUL_4479 = MUL_4368 * 0.0089687;
        auto MUL_4497 = MUL_4390 * 0.0118005;
        auto MUL_4488 = SUB_4381 * 0.010721;
        auto SUB_4501 = MUL_4479 - MUL_4488;
        auto ADD_4504 = SUB_4501 + MUL_4497;
        auto ADD_4507 = ADD_1535 + ADD_4504;
        auto MUL_4499 = SUB_4397 * 0.0118005;
        auto MUL_4492 = MUL_4384 * 0.010721;
        auto MUL_4481 = MUL_4371 * 0.0089687;
        auto SUB_4502 = MUL_4481 - MUL_4492;
        auto ADD_4505 = SUB_4502 + MUL_4499;
        auto ADD_4508 = ADD_1536 + ADD_4505;
        if (/*left_inner_knuckle*/ sphere_environment_in_collision(
            environment, ADD_4419, ADD_4420, ADD_4421, 0.037))
        {
            if (sphere_environment_in_collision(environment, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
        }  // (1043, 1184)
        if (/*panda_link0 vs. left_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4419, ADD_4420, ADD_4421, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
        }  // (1184, 1184)
        if (/*panda_link1 vs. left_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4419, ADD_4420, ADD_4421, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
        }  // (1184, 1184)
        if (/*panda_link2 vs. left_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4419, ADD_4420, ADD_4421, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
        }  // (1184, 1184)
        if (/*panda_link5 vs. left_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4419, ADD_4420, ADD_4421, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4446, ADD_4447, ADD_4448, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4473, ADD_4474, ADD_4475, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4506, ADD_4507, ADD_4508, 0.020571))
            {
                return false;
            }
        }  // (1184, 1184)
        auto MUL_1633 = SUB_1559 * 0.045497;
        auto MUL_1646 = ADD_1571 * MUL_1633;
        auto MUL_1642 = ADD_1554 * 0.045497;
        auto MUL_1644 = SUB_1566 * 0.0345853;
        auto SUB_1645 = MUL_1642 - MUL_1644;
        auto MUL_1650 = SUB_1566 * SUB_1645;
        auto MUL_1639 = SUB_1559 * 0.0345853;
        auto MUL_1647 = SUB_1559 * MUL_1639;
        auto SUB_1649 = MUL_1646 - MUL_1647;
        auto ADD_1651 = SUB_1649 + MUL_1650;
        auto MUL_1653 = ADD_1651 * 2.0;
        auto ADD_1655 = MUL_1653 + 0.0345853;
        auto ADD_1680 = ADD_1534 + ADD_1655;
        auto INPUT_9 = q[9];
        auto DIV_1684 = INPUT_9 * 0.5;
        auto SIN_1685 = DIV_1684.sin();
        auto COS_1694 = DIV_1684.cos();
        auto MUL_1690 = SIN_1685 * 1.0;
        auto MUL_1701 = ADD_1571 * MUL_1690;
        auto MUL_1713 = ADD_1571 * COS_1694;
        auto MUL_1698 = SUB_1566 * MUL_1690;
        auto MUL_1711 = SUB_1566 * COS_1694;
        auto MUL_1715 = SUB_1559 * MUL_1690;
        auto ADD_1717 = MUL_1713 + MUL_1715;
        auto MUL_1704 = SUB_1559 * COS_1694;
        auto SUB_1705 = MUL_1704 - MUL_1701;
        auto MUL_4520 = ADD_1717 * SUB_1705;
        auto MUL_4517 = SUB_1705 * SUB_1705;
        auto MUL_1708 = ADD_1554 * MUL_1690;
        auto SUB_1712 = MUL_1711 - MUL_1708;
        auto MUL_4518 = SUB_1712 * SUB_1712;
        auto ADD_4526 = MUL_4517 + MUL_4518;
        auto MUL_1696 = ADD_1554 * COS_1694;
        auto ADD_1700 = MUL_1696 + MUL_1698;
        auto MUL_4524 = ADD_1700 * SUB_1712;
        auto ADD_4552 = MUL_4524 + MUL_4520;
        auto MUL_4554 = ADD_4552 * 2.0;
        auto MUL_4578 = MUL_4554 * 0.014;
        auto MUL_4529 = ADD_4526 * 2.0;
        auto SUB_4532 = 1.0 - MUL_4529;
        auto MUL_4566 = SUB_4532 * 0.004;
        auto ADD_4583 = MUL_4566 + MUL_4578;
        auto ADD_4586 = ADD_1680 + ADD_4583;
        auto MUL_4519 = ADD_1717 * SUB_1712;
        auto MUL_4522 = ADD_1717 * ADD_1700;
        auto MUL_4525 = SUB_1705 * SUB_1712;
        auto SUB_4555 = MUL_4525 - MUL_4522;
        auto MUL_4523 = ADD_1700 * SUB_1705;
        auto ADD_4533 = MUL_4523 + MUL_4519;
        auto MUL_1657 = ADD_1571 * SUB_1645;
        auto MUL_1662 = SUB_1566 * MUL_1633;
        auto MUL_1659 = ADD_1554 * MUL_1639;
        auto SUB_1661 = MUL_1659 - MUL_1657;
        auto ADD_1663 = SUB_1661 + MUL_1662;
        auto MUL_1665 = ADD_1663 * 2.0;
        auto ADD_1681 = ADD_1535 + MUL_1665;
        auto MUL_4557 = SUB_4555 * 2.0;
        auto MUL_4580 = MUL_4557 * 0.014;
        auto MUL_4535 = ADD_4533 * 2.0;
        auto MUL_4568 = MUL_4535 * 0.004;
        auto ADD_4584 = MUL_4568 + MUL_4580;
        auto ADD_4587 = ADD_1681 + ADD_4584;
        auto SUB_4536 = MUL_4524 - MUL_4520;
        auto MUL_4521 = ADD_1700 * ADD_1700;
        auto ADD_4558 = MUL_4517 + MUL_4521;
        auto MUL_1667 = ADD_1571 * MUL_1639;
        auto MUL_1672 = SUB_1559 * MUL_1633;
        auto MUL_1669 = ADD_1554 * SUB_1645;
        auto ADD_1670 = MUL_1667 + MUL_1669;
        auto ADD_1673 = ADD_1670 + MUL_1672;
        auto MUL_1676 = ADD_1673 * 2.0;
        auto SUB_1679 = 0.045497 - MUL_1676;
        auto ADD_1682 = ADD_1536 + SUB_1679;
        auto MUL_4561 = ADD_4558 * 2.0;
        auto SUB_4564 = 1.0 - MUL_4561;
        auto MUL_4582 = SUB_4564 * 0.014;
        auto MUL_4538 = SUB_4536 * 2.0;
        auto MUL_4570 = MUL_4538 * 0.004;
        auto ADD_4585 = MUL_4570 + MUL_4582;
        auto ADD_4588 = ADD_1682 + ADD_4585;
        auto MUL_4590 = SUB_4532 * 0.0066238;
        auto MUL_4602 = MUL_4554 * 0.0024046;
        auto ADD_4607 = MUL_4590 + MUL_4602;
        auto ADD_4610 = ADD_1680 + ADD_4607;
        auto MUL_4604 = MUL_4557 * 0.0024046;
        auto MUL_4592 = MUL_4535 * 0.0066238;
        auto ADD_4608 = MUL_4592 + MUL_4604;
        auto ADD_4611 = ADD_1681 + ADD_4608;
        auto MUL_4606 = SUB_4564 * 0.0024046;
        auto MUL_4594 = MUL_4538 * 0.0066238;
        auto ADD_4609 = MUL_4594 + MUL_4606;
        auto ADD_4612 = ADD_1682 + ADD_4609;
        auto SUB_4539 = MUL_4523 - MUL_4519;
        auto MUL_4541 = SUB_4539 * 2.0;
        auto MUL_4620 = MUL_4541 * 1e-06;
        auto MUL_4626 = MUL_4554 * 0.0267306;
        auto MUL_4614 = SUB_4532 * 0.0011118;
        auto ADD_4631 = MUL_4614 + MUL_4620;
        auto ADD_4634 = ADD_4631 + MUL_4626;
        auto ADD_4637 = ADD_1680 + ADD_4634;
        auto ADD_4542 = MUL_4518 + MUL_4521;
        auto MUL_4628 = MUL_4557 * 0.0267306;
        auto MUL_4545 = ADD_4542 * 2.0;
        auto SUB_4548 = 1.0 - MUL_4545;
        auto MUL_4622 = SUB_4548 * 1e-06;
        auto MUL_4616 = MUL_4535 * 0.0011118;
        auto ADD_4632 = MUL_4616 + MUL_4622;
        auto ADD_4635 = ADD_4632 + MUL_4628;
        auto ADD_4638 = ADD_1681 + ADD_4635;
        auto ADD_4549 = MUL_4525 + MUL_4522;
        auto MUL_4630 = SUB_4564 * 0.0267306;
        auto MUL_4551 = ADD_4549 * 2.0;
        auto MUL_4624 = MUL_4551 * 1e-06;
        auto MUL_4618 = MUL_4538 * 0.0011118;
        auto ADD_4633 = MUL_4618 + MUL_4624;
        auto ADD_4636 = ADD_4633 + MUL_4630;
        auto ADD_4639 = ADD_1682 + ADD_4636;
        if (/*left_inner_finger*/ sphere_environment_in_collision(
            environment, ADD_4586, ADD_4587, ADD_4588, 0.032))
        {
            if (sphere_environment_in_collision(environment, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
        }  // (1184, 1301)
        if (/*panda_link0 vs. left_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4586, ADD_4587, ADD_4588, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
        }  // (1301, 1301)
        if (/*panda_link1 vs. left_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4586, ADD_4587, ADD_4588, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
        }  // (1301, 1301)
        if (/*panda_link2 vs. left_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4586, ADD_4587, ADD_4588, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
        }  // (1301, 1301)
        if (/*panda_link5 vs. left_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4586, ADD_4587, ADD_4588, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4610, ADD_4611, ADD_4612, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4637, ADD_4638, ADD_4639, 0.019047))
            {
                return false;
            }
        }  // (1301, 1301)
        auto ADD_1799 = MUL_1500 + MUL_1501;
        auto ADD_1796 = MUL_1496 + MUL_1498;
        auto MUL_1800 = ADD_1064 * ADD_1796;
        auto ADD_1801 = ADD_1799 + MUL_1800;
        auto MUL_1803 = ADD_1801 * 2.0;
        auto SUB_1806 = MUL_1803 - 0.0127;
        auto ADD_1827 = ADD_975 + SUB_1806;
        auto INPUT_10 = q[10];
        auto DIV_1831 = INPUT_10 * 0.5;
        auto SIN_1832 = DIV_1831.sin();
        auto COS_1838 = DIV_1831.cos();
        auto MUL_1855 = SUB_1073 * COS_1838;
        auto MUL_1842 = SUB_1073 * SIN_1832;
        auto MUL_1857 = ADD_1064 * COS_1838;
        auto MUL_1844 = ADD_1064 * SIN_1832;
        auto MUL_1840 = SUB_1053 * COS_1838;
        auto SUB_1843 = MUL_1840 - MUL_1842;
        auto MUL_1853 = SUB_1053 * SIN_1832;
        auto ADD_1856 = MUL_1853 + MUL_1855;
        auto MUL_4647 = ADD_1856 * ADD_1856;
        auto MUL_4655 = SUB_1843 * ADD_1856;
        auto MUL_1847 = ADD_1043 * COS_1838;
        auto ADD_1849 = MUL_1844 + MUL_1847;
        auto MUL_4646 = ADD_1849 * ADD_1849;
        auto ADD_4658 = MUL_4646 + MUL_4647;
        auto MUL_1860 = ADD_1043 * SIN_1832;
        auto SUB_1862 = MUL_1860 - MUL_1857;
        auto MUL_4649 = SUB_1862 * ADD_1849;
        auto SUB_4686 = MUL_4655 - MUL_4649;
        auto MUL_4688 = SUB_4686 * 2.0;
        auto MUL_4714 = MUL_4688 * 0.022;
        auto MUL_4661 = ADD_4658 * 2.0;
        auto SUB_4664 = 1.0 - MUL_4661;
        auto MUL_4702 = SUB_4664 * 0.017;
        auto ADD_4720 = MUL_4702 + MUL_4714;
        auto ADD_4723 = ADD_1827 + ADD_4720;
        auto MUL_4648 = SUB_1862 * ADD_1856;
        auto MUL_4652 = SUB_1862 * SUB_1843;
        auto MUL_4656 = ADD_1849 * ADD_1856;
        auto ADD_4689 = MUL_4656 + MUL_4652;
        auto MUL_4653 = SUB_1843 * ADD_1849;
        auto SUB_4665 = MUL_4648 - MUL_4653;
        auto MUL_1808 = SUB_1073 * ADD_1796;
        auto ADD_1811 = MUL_1808 + MUL_1513;
        auto SUB_1814 = MUL_1516 - ADD_1811;
        auto MUL_1816 = SUB_1814 * 2.0;
        auto ADD_1828 = ADD_976 + MUL_1816;
        auto MUL_4692 = ADD_4689 * 2.0;
        auto MUL_4716 = MUL_4692 * 0.022;
        auto MUL_4667 = SUB_4665 * 2.0;
        auto MUL_4704 = MUL_4667 * 0.017;
        auto SUB_4721 = MUL_4704 - MUL_4716;
        auto ADD_4724 = ADD_1828 + SUB_4721;
        auto ADD_4668 = MUL_4655 + MUL_4649;
        auto MUL_4651 = SUB_1843 * SUB_1843;
        auto ADD_4694 = MUL_4646 + MUL_4651;
        auto MUL_1819 = ADD_1043 * ADD_1796;
        auto SUB_1820 = MUL_1521 - MUL_1819;
        auto SUB_1822 = SUB_1820 - MUL_1526;
        auto MUL_1824 = SUB_1822 * 2.0;
        auto ADD_1826 = MUL_1824 + 0.0693075;
        auto ADD_1829 = ADD_977 + ADD_1826;
        auto MUL_4697 = ADD_4694 * 2.0;
        auto SUB_4700 = 1.0 - MUL_4697;
        auto MUL_4719 = SUB_4700 * 0.022;
        auto MUL_4670 = ADD_4668 * 2.0;
        auto MUL_4706 = MUL_4670 * 0.017;
        auto ADD_4722 = MUL_4706 + MUL_4719;
        auto ADD_4725 = ADD_1829 + ADD_4722;
        auto ADD_4671 = MUL_4653 + MUL_4648;
        auto MUL_4740 = MUL_4688 * 0.0355595;
        auto MUL_4674 = ADD_4671 * 2.0;
        auto MUL_4733 = MUL_4674 * 0.000714;
        auto MUL_4727 = SUB_4664 * 0.0270317;
        auto SUB_4746 = MUL_4727 - MUL_4733;
        auto ADD_4749 = SUB_4746 + MUL_4740;
        auto ADD_4752 = ADD_1827 + ADD_4749;
        auto ADD_4676 = MUL_4647 + MUL_4651;
        auto MUL_4742 = MUL_4692 * 0.0355595;
        auto MUL_4679 = ADD_4676 * 2.0;
        auto SUB_4682 = 1.0 - MUL_4679;
        auto MUL_4736 = SUB_4682 * 0.000714;
        auto MUL_4729 = MUL_4667 * 0.0270317;
        auto ADD_4747 = MUL_4729 + MUL_4736;
        auto SUB_4750 = ADD_4747 - MUL_4742;
        auto ADD_4753 = ADD_1828 + SUB_4750;
        auto SUB_4683 = MUL_4652 - MUL_4656;
        auto MUL_4745 = SUB_4700 * 0.0355595;
        auto MUL_4685 = SUB_4683 * 2.0;
        auto MUL_4738 = MUL_4685 * 0.000714;
        auto MUL_4731 = MUL_4670 * 0.0270317;
        auto ADD_4748 = MUL_4731 + MUL_4738;
        auto ADD_4751 = ADD_4748 + MUL_4745;
        auto ADD_4754 = ADD_1829 + ADD_4751;
        auto MUL_4769 = MUL_4688 * 0.0107115;
        auto MUL_4762 = MUL_4674 * 0.007858;
        auto MUL_4756 = SUB_4664 * 0.0081427;
        auto SUB_4775 = MUL_4756 - MUL_4762;
        auto ADD_4778 = SUB_4775 + MUL_4769;
        auto ADD_4781 = ADD_1827 + ADD_4778;
        auto MUL_4771 = MUL_4692 * 0.0107115;
        auto MUL_4765 = SUB_4682 * 0.007858;
        auto MUL_4758 = MUL_4667 * 0.0081427;
        auto ADD_4776 = MUL_4758 + MUL_4765;
        auto SUB_4779 = ADD_4776 - MUL_4771;
        auto ADD_4782 = ADD_1828 + SUB_4779;
        auto MUL_4774 = SUB_4700 * 0.0107115;
        auto MUL_4767 = MUL_4685 * 0.007858;
        auto MUL_4760 = MUL_4670 * 0.0081427;
        auto ADD_4777 = MUL_4760 + MUL_4767;
        auto ADD_4780 = ADD_4777 + MUL_4774;
        auto ADD_4783 = ADD_1829 + ADD_4780;
        auto MUL_4802 = MUL_4688 * 0.0118005;
        auto MUL_4792 = MUL_4674 * 0.010721;
        auto MUL_4785 = SUB_4664 * 0.0089687;
        auto ADD_4808 = MUL_4785 + MUL_4792;
        auto ADD_4811 = ADD_4808 + MUL_4802;
        auto ADD_4814 = ADD_1827 + ADD_4811;
        auto MUL_4804 = MUL_4692 * 0.0118005;
        auto MUL_4795 = SUB_4682 * 0.010721;
        auto MUL_4787 = MUL_4667 * 0.0089687;
        auto SUB_4809 = MUL_4787 - MUL_4795;
        auto SUB_4812 = SUB_4809 - MUL_4804;
        auto ADD_4815 = ADD_1828 + SUB_4812;
        auto MUL_4807 = SUB_4700 * 0.0118005;
        auto MUL_4799 = MUL_4685 * 0.010721;
        auto MUL_4789 = MUL_4670 * 0.0089687;
        auto SUB_4810 = MUL_4789 - MUL_4799;
        auto ADD_4813 = SUB_4810 + MUL_4807;
        auto ADD_4816 = ADD_1829 + ADD_4813;
        if (/*panda_link0 vs. right_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4723, ADD_4724, ADD_4725, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
        }  // (1301, 1431)
        if (/*panda_link1 vs. right_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4723, ADD_4724, ADD_4725, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
        }  // (1431, 1431)
        if (/*panda_link2 vs. right_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4723, ADD_4724, ADD_4725, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
        }  // (1431, 1431)
        if (/*panda_link5 vs. right_inner_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4723, ADD_4724, ADD_4725, 0.037))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
        }  // (1431, 1431)
        if (/*right_inner_knuckle*/ sphere_environment_in_collision(
            environment, ADD_4723, ADD_4724, ADD_4725, 0.037))
        {
            if (sphere_environment_in_collision(environment, ADD_4752, ADD_4753, ADD_4754, 0.020628))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4781, ADD_4782, ADD_4783, 0.020638))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4814, ADD_4815, ADD_4816, 0.020571))
            {
                return false;
            }
        }  // (1431, 1431)
        auto MUL_1923 = ADD_1849 * 0.0458574;
        auto MUL_1937 = SUB_1862 * MUL_1923;
        auto MUL_1933 = SUB_1843 * 0.0458574;
        auto MUL_1935 = ADD_1856 * 0.034106;
        auto SUB_1936 = MUL_1933 - MUL_1935;
        auto MUL_1943 = ADD_1856 * SUB_1936;
        auto MUL_1930 = ADD_1849 * 0.034106;
        auto MUL_1939 = ADD_1849 * MUL_1930;
        auto ADD_1941 = MUL_1937 + MUL_1939;
        auto SUB_1944 = MUL_1943 - ADD_1941;
        auto MUL_1946 = SUB_1944 * 2.0;
        auto ADD_1948 = MUL_1946 + 0.034106;
        auto ADD_1972 = ADD_1827 + ADD_1948;
        auto INPUT_11 = q[11];
        auto DIV_1976 = INPUT_11 * 0.5;
        auto SIN_1977 = DIV_1976.sin();
        auto COS_1983 = DIV_1976.cos();
        auto MUL_2000 = SUB_1862 * COS_1983;
        auto MUL_1989 = SUB_1862 * SIN_1977;
        auto MUL_1998 = ADD_1856 * COS_1983;
        auto MUL_1987 = ADD_1856 * SIN_1977;
        auto MUL_1991 = ADD_1849 * COS_1983;
        auto SUB_1993 = MUL_1989 - MUL_1991;
        auto MUL_4825 = SUB_1993 * SUB_1993;
        auto MUL_2002 = ADD_1849 * SIN_1977;
        auto ADD_2004 = MUL_2000 + MUL_2002;
        auto MUL_4828 = ADD_2004 * SUB_1993;
        auto MUL_1985 = SUB_1843 * COS_1983;
        auto SUB_1988 = MUL_1985 - MUL_1987;
        auto MUL_1996 = SUB_1843 * SIN_1977;
        auto ADD_1999 = MUL_1996 + MUL_1998;
        auto MUL_4826 = ADD_1999 * ADD_1999;
        auto ADD_4834 = MUL_4825 + MUL_4826;
        auto MUL_4832 = SUB_1988 * ADD_1999;
        auto ADD_4860 = MUL_4832 + MUL_4828;
        auto MUL_4862 = ADD_4860 * 2.0;
        auto MUL_4886 = MUL_4862 * 0.014;
        auto MUL_4837 = ADD_4834 * 2.0;
        auto SUB_4840 = 1.0 - MUL_4837;
        auto MUL_4874 = SUB_4840 * 0.004;
        auto ADD_4891 = MUL_4874 + MUL_4886;
        auto ADD_4894 = ADD_1972 + ADD_4891;
        auto MUL_4827 = ADD_2004 * ADD_1999;
        auto MUL_4830 = ADD_2004 * SUB_1988;
        auto MUL_4833 = SUB_1993 * ADD_1999;
        auto SUB_4863 = MUL_4833 - MUL_4830;
        auto MUL_4831 = SUB_1988 * SUB_1993;
        auto ADD_4841 = MUL_4831 + MUL_4827;
        auto MUL_1950 = SUB_1862 * SUB_1936;
        auto MUL_1955 = ADD_1856 * MUL_1923;
        auto MUL_1952 = SUB_1843 * MUL_1930;
        auto ADD_1953 = MUL_1950 + MUL_1952;
        auto ADD_1957 = ADD_1953 + MUL_1955;
        auto MUL_1960 = ADD_1957 * 2.0;
        auto SUB_1973 = ADD_1828 - MUL_1960;
        auto MUL_4865 = SUB_4863 * 2.0;
        auto MUL_4888 = MUL_4865 * 0.014;
        auto MUL_4843 = ADD_4841 * 2.0;
        auto MUL_4876 = MUL_4843 * 0.004;
        auto ADD_4892 = MUL_4876 + MUL_4888;
        auto ADD_4895 = SUB_1973 + ADD_4892;
        auto SUB_4844 = MUL_4832 - MUL_4828;
        auto MUL_4829 = SUB_1988 * SUB_1988;
        auto ADD_4866 = MUL_4825 + MUL_4829;
        auto MUL_1963 = SUB_1862 * MUL_1930;
        auto MUL_1966 = ADD_1849 * MUL_1923;
        auto MUL_1964 = SUB_1843 * SUB_1936;
        auto SUB_1965 = MUL_1963 - MUL_1964;
        auto SUB_1967 = SUB_1965 - MUL_1966;
        auto MUL_1969 = SUB_1967 * 2.0;
        auto ADD_1971 = MUL_1969 + 0.0458574;
        auto ADD_1974 = ADD_1829 + ADD_1971;
        auto MUL_4869 = ADD_4866 * 2.0;
        auto SUB_4872 = 1.0 - MUL_4869;
        auto MUL_4890 = SUB_4872 * 0.014;
        auto MUL_4846 = SUB_4844 * 2.0;
        auto MUL_4878 = MUL_4846 * 0.004;
        auto ADD_4893 = MUL_4878 + MUL_4890;
        auto ADD_4896 = ADD_1974 + ADD_4893;
        auto MUL_4910 = MUL_4862 * 0.0024046;
        auto MUL_4898 = SUB_4840 * 0.0066238;
        auto ADD_4915 = MUL_4898 + MUL_4910;
        auto ADD_4918 = ADD_1972 + ADD_4915;
        auto MUL_4912 = MUL_4865 * 0.0024046;
        auto MUL_4900 = MUL_4843 * 0.0066238;
        auto ADD_4916 = MUL_4900 + MUL_4912;
        auto ADD_4919 = SUB_1973 + ADD_4916;
        auto MUL_4914 = SUB_4872 * 0.0024046;
        auto MUL_4902 = MUL_4846 * 0.0066238;
        auto ADD_4917 = MUL_4902 + MUL_4914;
        auto ADD_4920 = ADD_1974 + ADD_4917;
        auto SUB_4847 = MUL_4831 - MUL_4827;
        auto MUL_4934 = MUL_4862 * 0.0267306;
        auto MUL_4849 = SUB_4847 * 2.0;
        auto MUL_4928 = MUL_4849 * 1e-06;
        auto MUL_4922 = SUB_4840 * 0.0011118;
        auto ADD_4939 = MUL_4922 + MUL_4928;
        auto ADD_4942 = ADD_4939 + MUL_4934;
        auto ADD_4945 = ADD_1972 + ADD_4942;
        auto ADD_4850 = MUL_4826 + MUL_4829;
        auto MUL_4936 = MUL_4865 * 0.0267306;
        auto MUL_4853 = ADD_4850 * 2.0;
        auto SUB_4856 = 1.0 - MUL_4853;
        auto MUL_4930 = SUB_4856 * 1e-06;
        auto MUL_4924 = MUL_4843 * 0.0011118;
        auto ADD_4940 = MUL_4924 + MUL_4930;
        auto ADD_4943 = ADD_4940 + MUL_4936;
        auto ADD_4946 = SUB_1973 + ADD_4943;
        auto ADD_4857 = MUL_4833 + MUL_4830;
        auto MUL_4938 = SUB_4872 * 0.0267306;
        auto MUL_4859 = ADD_4857 * 2.0;
        auto MUL_4932 = MUL_4859 * 1e-06;
        auto MUL_4926 = MUL_4846 * 0.0011118;
        auto ADD_4941 = MUL_4926 + MUL_4932;
        auto ADD_4944 = ADD_4941 + MUL_4938;
        auto ADD_4947 = ADD_1974 + ADD_4944;
        if (/*panda_link0 vs. right_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_4894, ADD_4895, ADD_4896, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
        }  // (1431, 1547)
        if (/*panda_link1 vs. right_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_4894, ADD_4895, ADD_4896, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
        }  // (1547, 1547)
        if (/*panda_link2 vs. right_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_4894, ADD_4895, ADD_4896, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
        }  // (1547, 1547)
        if (/*panda_link5 vs. right_inner_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_4894, ADD_4895, ADD_4896, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
        }  // (1547, 1547)
        if (/*right_inner_finger*/ sphere_environment_in_collision(
            environment, ADD_4894, ADD_4895, ADD_4896, 0.032))
        {
            if (sphere_environment_in_collision(environment, ADD_4918, ADD_4919, ADD_4920, 0.020134))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4945, ADD_4946, ADD_4947, 0.019047))
            {
                return false;
            }
        }  // (1547, 1547)
        auto ADD_2086 = MUL_1206 + MUL_1207;
        auto ADD_2083 = MUL_1202 + MUL_1204;
        auto MUL_2087 = ADD_1064 * ADD_2083;
        auto ADD_2088 = ADD_2086 + MUL_2087;
        auto MUL_2090 = ADD_2088 * 2.0;
        auto SUB_2093 = MUL_2090 - 0.0306011;
        auto ADD_2114 = ADD_975 + SUB_2093;
        auto INPUT_12 = q[12];
        auto DIV_2118 = INPUT_12 * 0.5;
        auto SIN_2119 = DIV_2118.sin();
        auto COS_2125 = DIV_2118.cos();
        auto MUL_2142 = SUB_1073 * COS_2125;
        auto MUL_2129 = SUB_1073 * SIN_2119;
        auto MUL_2144 = ADD_1064 * COS_2125;
        auto MUL_2131 = ADD_1064 * SIN_2119;
        auto MUL_2127 = SUB_1053 * COS_2125;
        auto SUB_2130 = MUL_2127 - MUL_2129;
        auto MUL_2140 = SUB_1053 * SIN_2119;
        auto ADD_2143 = MUL_2140 + MUL_2142;
        auto MUL_4955 = ADD_2143 * ADD_2143;
        auto MUL_4963 = SUB_2130 * ADD_2143;
        auto MUL_2134 = ADD_1043 * COS_2125;
        auto ADD_2136 = MUL_2131 + MUL_2134;
        auto MUL_4954 = ADD_2136 * ADD_2136;
        auto ADD_4966 = MUL_4954 + MUL_4955;
        auto MUL_2147 = ADD_1043 * SIN_2119;
        auto SUB_2149 = MUL_2147 - MUL_2144;
        auto MUL_4957 = SUB_2149 * ADD_2136;
        auto SUB_4994 = MUL_4963 - MUL_4957;
        auto MUL_4996 = SUB_4994 * 2.0;
        auto MUL_5047 = MUL_4996 * 0.0032636;
        auto MUL_4969 = ADD_4966 * 2.0;
        auto SUB_4972 = 1.0 - MUL_4969;
        auto MUL_5035 = SUB_4972 * 0.0154672;
        auto ADD_5053 = MUL_5035 + MUL_5047;
        auto ADD_5056 = ADD_2114 + ADD_5053;
        auto MUL_4956 = SUB_2149 * ADD_2143;
        auto MUL_4960 = SUB_2149 * SUB_2130;
        auto MUL_4964 = ADD_2136 * ADD_2143;
        auto ADD_4997 = MUL_4964 + MUL_4960;
        auto MUL_4961 = SUB_2130 * ADD_2136;
        auto SUB_4973 = MUL_4956 - MUL_4961;
        auto MUL_2095 = SUB_1073 * ADD_2083;
        auto ADD_2098 = MUL_2095 + MUL_1219;
        auto SUB_2101 = MUL_1222 - ADD_2098;
        auto MUL_2103 = SUB_2101 * 2.0;
        auto ADD_2115 = ADD_976 + MUL_2103;
        auto MUL_5000 = ADD_4997 * 2.0;
        auto MUL_5049 = MUL_5000 * 0.0032636;
        auto MUL_4975 = SUB_4973 * 2.0;
        auto MUL_5037 = MUL_4975 * 0.0154672;
        auto SUB_5054 = MUL_5037 - MUL_5049;
        auto ADD_5057 = ADD_2115 + SUB_5054;
        auto ADD_4976 = MUL_4963 + MUL_4957;
        auto MUL_4959 = SUB_2130 * SUB_2130;
        auto ADD_5002 = MUL_4954 + MUL_4959;
        auto MUL_2106 = ADD_1043 * ADD_2083;
        auto SUB_2107 = MUL_1227 - MUL_2106;
        auto SUB_2109 = SUB_2107 - MUL_1232;
        auto MUL_2111 = SUB_2109 * 2.0;
        auto ADD_2113 = MUL_2111 + 0.062792;
        auto ADD_2116 = ADD_977 + ADD_2113;
        auto MUL_5005 = ADD_5002 * 2.0;
        auto SUB_5008 = 1.0 - MUL_5005;
        auto MUL_5052 = SUB_5008 * 0.0032636;
        auto MUL_4978 = ADD_4976 * 2.0;
        auto MUL_5039 = MUL_4978 * 0.0154672;
        auto ADD_5055 = MUL_5039 + MUL_5052;
        auto ADD_5058 = ADD_2116 + ADD_5055;
        if (/*panda_link0 vs. right_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_5056, ADD_5057, ADD_5058, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
        }  // (1547, 1616)
        if (/*panda_link1 vs. right_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_5056, ADD_5057, ADD_5058, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
        }  // (1616, 1616)
        if (/*panda_link2 vs. right_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_5056, ADD_5057, ADD_5058, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
        }  // (1616, 1616)
        if (/*panda_link5 vs. right_outer_knuckle*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_5056, ADD_5057, ADD_5058, 0.022718))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_5056, ADD_5057, ADD_5058, 0.022718))
            {
                return false;
            }
        }  // (1616, 1616)
        if (/*right_outer_knuckle*/ sphere_environment_in_collision(
            environment, ADD_5056, ADD_5057, ADD_5058, 0.022718))
        {
            return false;
        }  // (1616, 1616)
        auto MUL_5103 = SUB_4994 * 2.0;
        auto MUL_5129 = MUL_5103 * 0.024;
        auto MUL_5076 = ADD_4966 * 2.0;
        auto SUB_5079 = 1.0 - MUL_5076;
        auto MUL_5117 = SUB_5079 * 0.001;
        auto ADD_5135 = MUL_5117 + MUL_5129;
        auto MUL_2224 = ADD_2143 * 0.0317096;
        auto MUL_2217 = ADD_2136 * 0.0317096;
        auto MUL_2228 = ADD_2136 * MUL_2217;
        auto MUL_2211 = ADD_2136 * 0.0016014;
        auto MUL_2227 = SUB_2149 * MUL_2211;
        auto SUB_2230 = MUL_2227 - MUL_2228;
        auto MUL_2221 = SUB_2130 * 0.0016014;
        auto ADD_2225 = MUL_2221 + MUL_2224;
        auto MUL_2231 = ADD_2143 * ADD_2225;
        auto SUB_2233 = SUB_2230 - MUL_2231;
        auto MUL_2235 = SUB_2233 * 2.0;
        auto ADD_2237 = MUL_2235 + 0.0317096;
        auto ADD_2259 = ADD_2114 + ADD_2237;
        auto ADD_5138 = ADD_2259 + ADD_5135;
        auto MUL_2239 = SUB_2149 * ADD_2225;
        auto MUL_2242 = ADD_2143 * MUL_2211;
        auto MUL_2240 = SUB_2130 * MUL_2217;
        auto SUB_2241 = MUL_2239 - MUL_2240;
        auto ADD_2243 = SUB_2241 + MUL_2242;
        auto MUL_2245 = ADD_2243 * 2.0;
        auto ADD_2260 = ADD_2115 + MUL_2245;
        auto MUL_5107 = ADD_4997 * 2.0;
        auto MUL_5131 = MUL_5107 * 0.024;
        auto MUL_5082 = SUB_4973 * 2.0;
        auto MUL_5119 = MUL_5082 * 0.001;
        auto SUB_5136 = MUL_5119 - MUL_5131;
        auto ADD_5139 = ADD_2260 + SUB_5136;
        auto MUL_2247 = SUB_2149 * MUL_2217;
        auto MUL_2251 = ADD_2136 * MUL_2211;
        auto MUL_2248 = SUB_2130 * ADD_2225;
        auto ADD_2250 = MUL_2247 + MUL_2248;
        auto ADD_2253 = ADD_2250 + MUL_2251;
        auto MUL_2255 = ADD_2253 * 2.0;
        auto SUB_2258 = MUL_2255 - 0.0016014;
        auto ADD_2261 = ADD_2116 + SUB_2258;
        auto MUL_5112 = ADD_5002 * 2.0;
        auto SUB_5115 = 1.0 - MUL_5112;
        auto MUL_5134 = SUB_5115 * 0.024;
        auto MUL_5085 = ADD_4976 * 2.0;
        auto MUL_5121 = MUL_5085 * 0.001;
        auto ADD_5137 = MUL_5121 + MUL_5134;
        auto ADD_5140 = ADD_2261 + ADD_5137;
        auto ADD_5086 = MUL_4961 + MUL_4956;
        auto MUL_5155 = MUL_5103 * 0.0414929;
        auto MUL_5089 = ADD_5086 * 2.0;
        auto MUL_5148 = MUL_5089 * 0.000302;
        auto MUL_5142 = SUB_5079 * 0.0025323;
        auto SUB_5161 = MUL_5142 - MUL_5148;
        auto ADD_5164 = SUB_5161 + MUL_5155;
        auto ADD_5167 = ADD_2259 + ADD_5164;
        auto ADD_5091 = MUL_4955 + MUL_4959;
        auto MUL_5157 = MUL_5107 * 0.0414929;
        auto MUL_5094 = ADD_5091 * 2.0;
        auto SUB_5097 = 1.0 - MUL_5094;
        auto MUL_5151 = SUB_5097 * 0.000302;
        auto MUL_5144 = MUL_5082 * 0.0025323;
        auto ADD_5162 = MUL_5144 + MUL_5151;
        auto SUB_5165 = ADD_5162 - MUL_5157;
        auto ADD_5168 = ADD_2260 + SUB_5165;
        auto SUB_5098 = MUL_4960 - MUL_4964;
        auto MUL_5160 = SUB_5115 * 0.0414929;
        auto MUL_5100 = SUB_5098 * 2.0;
        auto MUL_5153 = MUL_5100 * 0.000302;
        auto MUL_5146 = MUL_5085 * 0.0025323;
        auto ADD_5163 = MUL_5146 + MUL_5153;
        auto ADD_5166 = ADD_5163 + MUL_5160;
        auto ADD_5169 = ADD_2261 + ADD_5166;
        auto MUL_5184 = MUL_5103 * 0.0060249;
        auto MUL_5177 = MUL_5089 * 0.000299;
        auto MUL_5171 = SUB_5079 * 0.0003683;
        auto SUB_5190 = MUL_5171 - MUL_5177;
        auto ADD_5193 = SUB_5190 + MUL_5184;
        auto ADD_5196 = ADD_2259 + ADD_5193;
        auto MUL_5186 = MUL_5107 * 0.0060249;
        auto MUL_5180 = SUB_5097 * 0.000299;
        auto MUL_5173 = MUL_5082 * 0.0003683;
        auto ADD_5191 = MUL_5173 + MUL_5180;
        auto SUB_5194 = ADD_5191 - MUL_5186;
        auto ADD_5197 = ADD_2260 + SUB_5194;
        auto MUL_5189 = SUB_5115 * 0.0060249;
        auto MUL_5182 = MUL_5100 * 0.000299;
        auto MUL_5175 = MUL_5085 * 0.0003683;
        auto ADD_5192 = MUL_5175 + MUL_5182;
        auto ADD_5195 = ADD_5192 + MUL_5189;
        auto ADD_5198 = ADD_2261 + ADD_5195;
        auto MUL_5217 = MUL_5103 * 0.0238049;
        auto MUL_5207 = MUL_5089 * 0.000343;
        auto MUL_5200 = SUB_5079 * 0.0014533;
        auto ADD_5223 = MUL_5200 + MUL_5207;
        auto ADD_5226 = ADD_5223 + MUL_5217;
        auto ADD_5229 = ADD_2259 + ADD_5226;
        auto MUL_5219 = MUL_5107 * 0.0238049;
        auto MUL_5210 = SUB_5097 * 0.000343;
        auto MUL_5202 = MUL_5082 * 0.0014533;
        auto SUB_5224 = MUL_5202 - MUL_5210;
        auto SUB_5227 = SUB_5224 - MUL_5219;
        auto ADD_5230 = ADD_2260 + SUB_5227;
        auto MUL_5222 = SUB_5115 * 0.0238049;
        auto MUL_5214 = MUL_5100 * 0.000343;
        auto MUL_5204 = MUL_5085 * 0.0014533;
        auto SUB_5225 = MUL_5204 - MUL_5214;
        auto ADD_5228 = SUB_5225 + MUL_5222;
        auto ADD_5231 = ADD_2261 + ADD_5228;
        if (/*panda_link0 vs. right_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.05, 0.08, ADD_5138, ADD_5139, ADD_5140, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.05, 0.08, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
        }  // (1616, 1725)
        if (/*panda_link1 vs. right_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2394, NEGATE_2396, 0.248, 0.154, ADD_5138, ADD_5139, ADD_5140, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2409, NEGATE_2413, 0.333, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2433, NEGATE_2437, 0.333, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.213, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.163, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
        }  // (1725, 1725)
        if (/*panda_link2 vs. right_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2585, SUB_2586, ADD_2588, 0.154, ADD_5138, ADD_5139, ADD_5140, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2602, MUL_2604, ADD_2607, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2621, MUL_2623, ADD_2626, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2635, NEGATE_2639, SUB_2650, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    MUL_2659, NEGATE_2663, SUB_2674, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
        }  // (1725, 1725)
        if (/*panda_link5 vs. right_outer_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3155, ADD_3156, ADD_3157, 0.176, ADD_5138, ADD_5139, ADD_5140, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3176, ADD_3177, ADD_3178, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3197, ADD_3198, ADD_3199, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3224, SUB_3225, SUB_3226, 0.06, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3254, ADD_3255, ADD_3256, 0.05, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3287, ADD_3288, ADD_3289, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3320, ADD_3321, ADD_3322, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3353, ADD_3354, ADD_3355, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3386, ADD_3387, ADD_3388, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3425, ADD_3426, ADD_3427, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3464, ADD_3465, ADD_3466, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3503, ADD_3504, ADD_3505, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3542, ADD_3543, ADD_3544, 0.025, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
        }  // (1725, 1725)
        if (/*right_outer_finger*/ sphere_environment_in_collision(
            environment, ADD_5138, ADD_5139, ADD_5140, 0.033))
        {
            if (sphere_environment_in_collision(environment, ADD_5167, ADD_5168, ADD_5169, 0.015696))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_5196, ADD_5197, ADD_5198, 0.015752))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_5229, ADD_5230, ADD_5231, 0.015747))
            {
                return false;
            }
        }
        return true;
    }
}  // namespace vamp::robots::panda_attachment

// NOLINTEND(*-magic-numbers)
