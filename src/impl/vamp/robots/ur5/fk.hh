#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::ur5
{
    using Configuration = FloatVector<6>;
    template <std::size_t block_width>
    using ConfigurationBlock = FloatVector<block_width, 6>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, 6> s_m_a{
        6.2831853,
        6.2831853,
        6.2831853,
        6.2831853,
        6.2831853,
        6.2831853};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 6> s_a_a{
        -3.14159265,
        -3.14159265,
        -3.14159265,
        -3.14159265,
        -3.14159265,
        -3.14159265};
    static const Configuration s_m(s_m_a);
    static const Configuration s_a(s_a_a);

    inline void scale_configuration(Configuration &q) noexcept
    {
        q = q * s_m + s_a;
    }

    template <std::size_t block_width>
    inline void scale_configuration_block(ConfigurationBlock<block_width> &q) noexcept
    {
        q[0] = -3.14159265 + (q[0] * 6.2831853);
        q[1] = -3.14159265 + (q[1] * 6.2831853);
        q[2] = -3.14159265 + (q[2] * 6.2831853);
        q[3] = -3.14159265 + (q[3] * 6.2831853);
        q[4] = -3.14159265 + (q[4] * 6.2831853);
        q[5] = -3.14159265 + (q[5] * 6.2831853);
    }

    alignas(Configuration::S::Alignment) constexpr std::array<float, 6> d_m_a{
        0.15915494327375637,
        0.15915494327375637,
        0.15915494327375637,
        0.15915494327375637,
        0.15915494327375637,
        0.15915494327375637};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 6> d_a_a{
        -3.14159265,
        -3.14159265,
        -3.14159265,
        -3.14159265,
        -3.14159265,
        -3.14159265};
    static const Configuration d_m(d_m_a);
    static const Configuration d_a(d_a_a);

    inline void descale_configuration(Configuration &q) noexcept
    {
        q = (q - d_a) * d_m;
    }

    template <std::size_t block_width>
    inline void descale_configuration_block(ConfigurationBlock<block_width> &q) noexcept
    {
        q[0] = 0.15915494327375637 * (q[0] - -3.14159265);
        q[1] = 0.15915494327375637 * (q[1] - -3.14159265);
        q[2] = 0.15915494327375637 * (q[2] - -3.14159265);
        q[3] = 0.15915494327375637 * (q[3] - -3.14159265);
        q[4] = 0.15915494327375637 * (q[4] - -3.14159265);
        q[5] = 0.15915494327375637 * (q[5] - -3.14159265);
    }

    inline static constexpr auto space_measure() noexcept -> float
    {
        return 700852.7173113511;
    }

    constexpr auto n_spheres = 36;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
        out.r[0] = 0.08;      // (0, 0)
        out.r[1] = 0.08;      // (0, 0)
        out.r[2] = 0.08;      // (0, 0)
        out.r[3] = 0.08;      // (0, 0)
        out.r[4] = 0.08;      // (0, 0)
        out.r[5] = 0.08;      // (0, 0)
        out.r[6] = 0.08;      // (0, 0)
        out.r[7] = 0.08;      // (0, 0)
        out.r[8] = 0.06;      // (0, 0)
        out.r[9] = 0.06;      // (0, 0)
        out.r[10] = 0.06;     // (0, 0)
        out.r[11] = 0.06;     // (0, 0)
        out.r[12] = 0.04;     // (0, 0)
        out.r[13] = 0.04;     // (0, 0)
        out.r[14] = 0.04;     // (0, 0)
        out.r[15] = 0.04;     // (0, 0)
        out.r[16] = 0.04;     // (0, 0)
        out.r[17] = 0.04;     // (0, 0)
        out.r[18] = 0.04;     // (0, 0)
        out.r[19] = 0.04;     // (0, 0)
        out.r[20] = 0.04;     // (0, 0)
        out.r[21] = 0.04;     // (0, 0)
        out.r[22] = 0.02;     // (0, 0)
        out.r[23] = 0.015;    // (0, 0)
        out.r[24] = 0.015;    // (0, 0)
        out.r[25] = 0.015;    // (0, 0)
        out.r[26] = 0.02;     // (0, 0)
        out.r[27] = 0.02;     // (0, 0)
        out.r[28] = 0.02;     // (0, 0)
        out.r[29] = 0.02;     // (0, 0)
        out.r[30] = 0.02;     // (0, 0)
        out.r[31] = 0.02;     // (0, 0)
        out.r[32] = 0.02;     // (0, 0)
        out.r[33] = 0.015;    // (0, 0)
        out.r[34] = 0.015;    // (0, 0)
        out.r[35] = 0.015;    // (0, 0)
        out.x[0] = 0.0;       // (0, 0)
        out.x[1] = 0.0;       // (0, 0)
        out.y[0] = 0.0;       // (0, 0)
        out.y[1] = 0.0;       // (0, 0)
        out.z[0] = 0.9144;    // (0, 0)
        out.z[1] = 1.003559;  // (0, 0)
        out.z[6] = 1.003559;  // (0, 0)
        auto INPUT_1 = q[1];
        auto DIV_123 = INPUT_1 * 0.5;
        auto SIN_124 = DIV_123.sin();
        auto COS_130 = DIV_123.cos();
        auto INPUT_0 = q[0];
        auto DIV_7 = INPUT_0 * 0.5;
        auto SIN_8 = DIV_7.sin();
        auto COS_14 = DIV_7.cos();
        auto MUL_41 = COS_14 * 0.7073883;
        auto MUL_38 = COS_14 * 0.7068252;
        auto MUL_32 = SIN_8 * 0.7073883;
        auto ADD_39 = MUL_32 + MUL_38;
        auto MUL_56 = ADD_39 * 0.7071068;
        auto MUL_147 = MUL_56 * COS_130;
        auto MUL_144 = MUL_56 * SIN_124;
        auto SUB_148 = MUL_147 - MUL_144;
        auto ADD_136 = MUL_147 + MUL_144;
        auto MUL_2778 = ADD_136 * SUB_148;
        auto MUL_85 = ADD_39 * 0.13585;
        auto MUL_47 = SIN_8 * 0.7068252;
        auto SUB_48 = MUL_41 - MUL_47;
        auto MUL_95 = SUB_48 * MUL_85;
        auto MUL_100 = MUL_95 * 2.0;
        auto MUL_59 = SUB_48 * 0.7071068;
        auto MUL_140 = MUL_59 * COS_130;
        auto MUL_138 = MUL_59 * SIN_124;
        auto SUB_152 = MUL_140 - MUL_138;
        auto ADD_141 = MUL_138 + MUL_140;
        auto MUL_2772 = SUB_152 * ADD_141;
        auto SUB_2811 = MUL_2772 - MUL_2778;
        auto MUL_2813 = SUB_2811 * 2.0;
        auto MUL_2837 = MUL_2813 * 0.105;
        auto SUB_2842 = MUL_2837 - MUL_100;
        out.x[2] = SUB_2842;  // (0, 33)
        auto MUL_2858 = MUL_2813 * 0.21;
        auto SUB_2863 = MUL_2858 - MUL_100;
        out.x[3] = SUB_2863;  // (33, 35)
        auto MUL_2879 = MUL_2813 * 0.315;
        auto SUB_2884 = MUL_2879 - MUL_100;
        out.x[4] = SUB_2884;  // (35, 37)
        auto MUL_2900 = MUL_2813 * 0.42;
        auto SUB_2905 = MUL_2900 - MUL_100;
        out.x[5] = SUB_2905;  // (37, 39)
        auto NEGATE_101 = -MUL_100;
        out.x[6] = NEGATE_101;  // (39, 40)
        auto MUL_214 = ADD_141 * 0.425;
        auto MUL_226 = ADD_136 * 0.425;
        auto MUL_233 = SUB_148 * MUL_226;
        auto MUL_217 = SUB_148 * 0.1197;
        auto ADD_219 = MUL_214 + MUL_217;
        auto MUL_230 = SUB_152 * ADD_219;
        auto MUL_222 = ADD_136 * 0.1197;
        auto MUL_231 = ADD_141 * MUL_222;
        auto ADD_232 = MUL_230 + MUL_231;
        auto SUB_235 = ADD_232 - MUL_233;
        auto MUL_237 = SUB_235 * 2.0;
        auto SUB_260 = MUL_237 - MUL_100;
        out.x[7] = SUB_260;  // (40, 52)
        auto INPUT_2 = q[2];
        auto DIV_264 = INPUT_2 * 0.5;
        auto SIN_265 = DIV_264.sin();
        auto COS_271 = DIV_264.cos();
        auto MUL_290 = SUB_152 * COS_271;
        auto MUL_279 = SUB_152 * SIN_265;
        auto MUL_288 = SUB_148 * COS_271;
        auto MUL_276 = SUB_148 * SIN_265;
        auto MUL_281 = ADD_141 * COS_271;
        auto ADD_282 = MUL_279 + MUL_281;
        auto MUL_292 = ADD_141 * SIN_265;
        auto SUB_293 = MUL_290 - MUL_292;
        auto MUL_2929 = SUB_293 * ADD_282;
        auto MUL_273 = ADD_136 * COS_271;
        auto ADD_277 = MUL_273 + MUL_276;
        auto MUL_285 = ADD_136 * SIN_265;
        auto SUB_289 = MUL_288 - MUL_285;
        auto MUL_2935 = ADD_277 * SUB_289;
        auto SUB_2968 = MUL_2929 - MUL_2935;
        auto MUL_2970 = SUB_2968 * 2.0;
        auto MUL_3012 = MUL_2970 * 0.0975;
        auto ADD_3017 = SUB_260 + MUL_3012;
        out.x[8] = ADD_3017;  // (52, 74)
        auto MUL_3033 = MUL_2970 * 0.195;
        auto ADD_3038 = SUB_260 + MUL_3033;
        out.x[9] = ADD_3038;  // (74, 76)
        auto MUL_3054 = MUL_2970 * 0.2925;
        auto ADD_3059 = SUB_260 + MUL_3054;
        out.x[10] = ADD_3059;  // (76, 78)
        auto MUL_3075 = MUL_2970 * 0.39;
        auto ADD_3080 = SUB_260 + MUL_3075;
        out.x[11] = ADD_3080;  // (78, 80)
        auto MUL_338 = SUB_293 * 0.7071068;
        auto MUL_334 = SUB_289 * 0.7071068;
        auto MUL_342 = ADD_282 * 0.7071068;
        auto SUB_362 = MUL_338 - MUL_342;
        auto ADD_343 = MUL_338 + MUL_342;
        auto MUL_329 = ADD_277 * 0.7071068;
        auto SUB_355 = MUL_334 - MUL_329;
        auto ADD_335 = MUL_329 + MUL_334;
        auto MUL_366 = ADD_282 * 0.39225;
        auto MUL_378 = SUB_293 * MUL_366;
        auto MUL_374 = ADD_277 * 0.39225;
        auto MUL_380 = SUB_289 * MUL_374;
        auto SUB_382 = MUL_378 - MUL_380;
        auto MUL_384 = SUB_382 * 2.0;
        auto ADD_405 = SUB_260 + MUL_384;
        auto INPUT_3 = q[3];
        auto DIV_409 = INPUT_3 * 0.5;
        auto SIN_410 = DIV_409.sin();
        auto COS_416 = DIV_409.cos();
        auto MUL_435 = SUB_362 * COS_416;
        auto MUL_424 = SUB_362 * SIN_410;
        auto MUL_433 = SUB_355 * COS_416;
        auto MUL_421 = SUB_355 * SIN_410;
        auto MUL_426 = ADD_343 * COS_416;
        auto ADD_427 = MUL_424 + MUL_426;
        auto MUL_437 = ADD_343 * SIN_410;
        auto SUB_438 = MUL_435 - MUL_437;
        auto MUL_3086 = SUB_438 * ADD_427;
        auto MUL_418 = ADD_335 * COS_416;
        auto ADD_422 = MUL_418 + MUL_421;
        auto MUL_3090 = ADD_422 * ADD_427;
        auto MUL_430 = ADD_335 * SIN_410;
        auto SUB_434 = MUL_433 - MUL_430;
        auto MUL_3085 = SUB_438 * SUB_434;
        auto ADD_3110 = MUL_3090 + MUL_3085;
        auto MUL_3113 = ADD_3110 * 2.0;
        auto MUL_3145 = MUL_3113 * 0.09;
        auto MUL_3092 = ADD_422 * SUB_434;
        auto SUB_3125 = MUL_3086 - MUL_3092;
        auto MUL_3127 = SUB_3125 * 2.0;
        auto MUL_3152 = MUL_3127 * 0.03;
        auto SUB_3157 = MUL_3152 - MUL_3145;
        auto ADD_3160 = ADD_405 + SUB_3157;
        out.x[12] = ADD_3160;  // (80, 123)
        auto ADD_3188 = MUL_3145 + MUL_3152;
        auto SUB_3192 = ADD_405 - ADD_3188;
        out.x[13] = SUB_3192;  // (123, 125)
        auto SUB_3214 = ADD_405 - MUL_3145;
        out.x[14] = SUB_3214;  // (125, 126)
        auto MUL_502 = SUB_434 * 0.093;
        auto MUL_513 = SUB_438 * MUL_502;
        auto MUL_505 = ADD_422 * 0.093;
        auto MUL_515 = ADD_427 * MUL_505;
        auto ADD_517 = MUL_513 + MUL_515;
        auto MUL_521 = ADD_517 * 2.0;
        auto SUB_546 = ADD_405 - MUL_521;
        auto INPUT_4 = q[4];
        auto DIV_550 = INPUT_4 * 0.5;
        auto SIN_551 = DIV_550.sin();
        auto COS_557 = DIV_550.cos();
        auto MUL_575 = SUB_438 * COS_557;
        auto MUL_570 = SUB_438 * SIN_551;
        auto MUL_573 = SUB_434 * COS_557;
        auto ADD_574 = MUL_570 + MUL_573;
        auto MUL_578 = SUB_434 * SIN_551;
        auto SUB_579 = MUL_575 - MUL_578;
        auto MUL_3219 = SUB_579 * ADD_574;
        auto MUL_567 = ADD_427 * COS_557;
        auto MUL_561 = ADD_427 * SIN_551;
        auto MUL_559 = ADD_422 * COS_557;
        auto SUB_562 = MUL_561 - MUL_559;
        auto MUL_3224 = SUB_562 * ADD_574;
        auto MUL_565 = ADD_422 * SIN_551;
        auto ADD_568 = MUL_565 + MUL_567;
        auto MUL_3220 = SUB_579 * ADD_568;
        auto ADD_3252 = MUL_3224 + MUL_3220;
        auto MUL_3254 = ADD_3252 * 2.0;
        auto MUL_3278 = MUL_3254 * 0.09;
        auto MUL_3223 = SUB_562 * ADD_568;
        auto SUB_3239 = MUL_3223 - MUL_3219;
        auto MUL_3241 = SUB_3239 * 2.0;
        auto MUL_3272 = MUL_3241 * 0.03;
        auto ADD_3283 = MUL_3272 + MUL_3278;
        auto ADD_3286 = SUB_546 + ADD_3283;
        out.x[15] = ADD_3286;  // (126, 161)
        auto SUB_3313 = MUL_3278 - MUL_3272;
        auto ADD_3316 = SUB_546 + SUB_3313;
        out.x[16] = ADD_3316;  // (161, 163)
        auto ADD_3337 = SUB_546 + MUL_3278;
        out.x[17] = ADD_3337;  // (163, 164)
        auto MUL_648 = SUB_562 * 0.09465;
        auto MUL_653 = ADD_574 * MUL_648;
        auto MUL_640 = ADD_568 * 0.09465;
        auto MUL_651 = SUB_579 * MUL_640;
        auto ADD_654 = MUL_651 + MUL_653;
        auto MUL_656 = ADD_654 * 2.0;
        auto ADD_678 = SUB_546 + MUL_656;
        auto INPUT_5 = q[5];
        auto DIV_682 = INPUT_5 * 0.5;
        auto SIN_683 = DIV_682.sin();
        auto COS_689 = DIV_682.cos();
        auto MUL_705 = SUB_579 * COS_689;
        auto MUL_695 = SUB_579 * SIN_683;
        auto MUL_703 = ADD_574 * COS_689;
        auto MUL_693 = ADD_574 * SIN_683;
        auto MUL_691 = SUB_562 * COS_689;
        auto SUB_694 = MUL_691 - MUL_693;
        auto MUL_701 = SUB_562 * SIN_683;
        auto ADD_704 = MUL_701 + MUL_703;
        auto MUL_697 = ADD_568 * COS_689;
        auto ADD_698 = MUL_695 + MUL_697;
        auto MUL_3346 = SUB_694 * ADD_698;
        auto MUL_707 = ADD_568 * SIN_683;
        auto SUB_708 = MUL_705 - MUL_707;
        auto MUL_3342 = SUB_708 * ADD_704;
        auto SUB_3362 = MUL_3346 - MUL_3342;
        auto MUL_3364 = SUB_3362 * 2.0;
        auto MUL_3395 = MUL_3364 * 0.06;
        auto ADD_3406 = ADD_678 + MUL_3395;
        out.x[18] = ADD_3406;  // (164, 193)
        auto MUL_770 = SUB_708 * 0.7071068;
        auto MUL_776 = ADD_704 * 0.7071068;
        auto SUB_777 = MUL_770 - MUL_776;
        auto ADD_768 = MUL_770 + MUL_776;
        auto MUL_1064 = SUB_777 * 0.7073883;
        auto MUL_1054 = SUB_777 * 0.7068252;
        auto MUL_1061 = ADD_768 * 0.7073883;
        auto SUB_1062 = MUL_1061 - MUL_1054;
        auto MUL_1071 = ADD_768 * 0.7068252;
        auto ADD_1073 = MUL_1064 + MUL_1071;
        auto MUL_3507 = ADD_1073 * SUB_1062;
        auto MUL_756 = ADD_698 * 0.7071068;
        auto MUL_753 = SUB_694 * 0.7071068;
        auto SUB_757 = MUL_756 - MUL_753;
        auto ADD_747 = MUL_753 + MUL_756;
        auto MUL_1048 = SUB_757 * 0.7073883;
        auto MUL_1036 = SUB_757 * 0.7068252;
        auto MUL_1033 = ADD_747 * 0.7073883;
        auto SUB_1038 = MUL_1033 - MUL_1036;
        auto MUL_1045 = ADD_747 * 0.7068252;
        auto ADD_1049 = MUL_1045 + MUL_1048;
        auto MUL_3511 = SUB_1038 * ADD_1049;
        auto SUB_3527 = MUL_3511 - MUL_3507;
        auto MUL_3529 = SUB_3527 * 2.0;
        auto MUL_781 = ADD_704 * 0.0823;
        auto MUL_791 = SUB_708 * MUL_781;
        auto MUL_784 = SUB_694 * 0.0823;
        auto MUL_793 = ADD_698 * MUL_784;
        auto SUB_794 = MUL_793 - MUL_791;
        auto MUL_797 = SUB_794 * 2.0;
        auto ADD_820 = ADD_678 + MUL_797;
        auto MUL_1086 = ADD_768 * 0.035;
        auto MUL_1091 = ADD_768 * MUL_1086;
        auto MUL_1081 = SUB_757 * 0.035;
        auto MUL_1089 = SUB_757 * MUL_1081;
        auto ADD_1093 = MUL_1089 + MUL_1091;
        auto MUL_1096 = ADD_1093 * 2.0;
        auto SUB_1099 = 0.035 - MUL_1096;
        auto ADD_1118 = ADD_820 + SUB_1099;
        auto MUL_3561 = MUL_3529 * 0.02;
        auto SUB_3577 = ADD_1118 - MUL_3561;
        out.x[19] = SUB_3577;  // (193, 234)
        auto MUL_1472 = ADD_1073 * 0.7073883;
        auto MUL_1440 = ADD_1073 * 0.7068252;
        auto MUL_1469 = SUB_1062 * 0.7073883;
        auto MUL_1457 = SUB_1062 * 0.7068252;
        auto MUL_1443 = SUB_1038 * 0.7073883;
        auto SUB_1444 = MUL_1443 - MUL_1440;
        auto MUL_1475 = SUB_1038 * 0.7068252;
        auto ADD_1477 = MUL_1472 + MUL_1475;
        auto MUL_1454 = ADD_1049 * 0.7073883;
        auto SUB_1459 = MUL_1454 - MUL_1457;
        auto MUL_3679 = ADD_1477 * SUB_1459;
        auto MUL_1466 = ADD_1049 * 0.7068252;
        auto ADD_1470 = MUL_1466 + MUL_1469;
        auto MUL_3683 = SUB_1444 * ADD_1470;
        auto ADD_3711 = MUL_3683 + MUL_3679;
        auto MUL_3713 = ADD_3711 * 2.0;
        auto MUL_3737 = MUL_3713 * 0.06;
        auto MUL_1346 = SUB_1038 * 0.0375;
        auto MUL_1351 = SUB_1062 * MUL_1346;
        auto MUL_1338 = ADD_1049 * 0.0375;
        auto MUL_1349 = ADD_1073 * MUL_1338;
        auto ADD_1352 = MUL_1349 + MUL_1351;
        auto MUL_1354 = ADD_1352 * 2.0;
        auto ADD_1376 = ADD_1118 + MUL_1354;
        auto MUL_1494 = SUB_1038 * 0.037;
        auto MUL_1501 = SUB_1062 * MUL_1494;
        auto MUL_1484 = ADD_1049 * 0.037;
        auto MUL_1498 = ADD_1073 * MUL_1484;
        auto ADD_1503 = MUL_1498 + MUL_1501;
        auto MUL_1506 = ADD_1503 * 2.0;
        auto SUB_1529 = ADD_1376 - MUL_1506;
        auto ADD_3742 = SUB_1529 + MUL_3737;
        out.x[20] = ADD_3742;  // (234, 266)
        auto MUL_3758 = MUL_3713 * 0.02;
        auto ADD_3763 = SUB_1529 + MUL_3758;
        out.x[21] = ADD_3763;  // (266, 268)
        auto MUL_1620 = SUB_1459 * 0.062792;
        auto MUL_1633 = ADD_1477 * MUL_1620;
        auto MUL_1629 = SUB_1444 * 0.062792;
        auto MUL_1631 = ADD_1470 * 0.0306011;
        auto SUB_1632 = MUL_1629 - MUL_1631;
        auto MUL_1637 = ADD_1470 * SUB_1632;
        auto MUL_1626 = SUB_1459 * 0.0306011;
        auto MUL_1634 = SUB_1459 * MUL_1626;
        auto SUB_1636 = MUL_1633 - MUL_1634;
        auto ADD_1638 = SUB_1636 + MUL_1637;
        auto MUL_1640 = ADD_1638 * 2.0;
        auto ADD_1642 = MUL_1640 + 0.0306011;
        auto ADD_1667 = SUB_1529 + ADD_1642;
        out.x[22] = ADD_1667;  // (268, 281)
        auto MUL_3869 = ADD_3711 * 2.0;
        auto MUL_3893 = MUL_3869 * 0.04;
        auto MUL_1773 = ADD_1470 * 0.031691;
        auto MUL_1766 = SUB_1459 * 0.031691;
        auto MUL_1778 = SUB_1459 * MUL_1766;
        auto MUL_1759 = SUB_1459 * 0.001934;
        auto MUL_1776 = ADD_1477 * MUL_1759;
        auto ADD_1780 = MUL_1776 + MUL_1778;
        auto MUL_1770 = SUB_1444 * 0.001934;
        auto ADD_1774 = MUL_1770 + MUL_1773;
        auto MUL_1782 = ADD_1470 * ADD_1774;
        auto ADD_1784 = ADD_1780 + MUL_1782;
        auto MUL_1787 = ADD_1784 * 2.0;
        auto SUB_1790 = 0.031691 - MUL_1787;
        auto ADD_1815 = ADD_1667 + SUB_1790;
        out.x[24] = ADD_1815;  // (281, 296)
        auto ADD_3898 = ADD_1815 + MUL_3893;
        out.x[23] = ADD_3898;  // (296, 297)
        auto MUL_3932 = MUL_3869 * 0.02;
        auto ADD_3937 = ADD_1815 + MUL_3932;
        out.x[25] = ADD_3937;  // (297, 299)
        auto MUL_3977 = ADD_3711 * 2.0;
        auto MUL_4001 = MUL_3977 * 0.02;
        auto MUL_3941 = ADD_1470 * ADD_1470;
        auto MUL_3940 = SUB_1459 * SUB_1459;
        auto ADD_3949 = MUL_3940 + MUL_3941;
        auto MUL_3952 = ADD_3949 * 2.0;
        auto SUB_3955 = 1.0 - MUL_3952;
        auto MUL_3989 = SUB_3955 * 0.02;
        auto ADD_4006 = MUL_3989 + MUL_4001;
        auto MUL_1906 = SUB_1459 * 0.0693075;
        auto MUL_1919 = ADD_1477 * MUL_1906;
        auto MUL_1915 = SUB_1444 * 0.0693075;
        auto MUL_1917 = ADD_1470 * 0.0127;
        auto SUB_1918 = MUL_1915 - MUL_1917;
        auto MUL_1923 = ADD_1470 * SUB_1918;
        auto MUL_1912 = SUB_1459 * 0.0127;
        auto MUL_1920 = SUB_1459 * MUL_1912;
        auto SUB_1922 = MUL_1919 - MUL_1920;
        auto ADD_1924 = SUB_1922 + MUL_1923;
        auto MUL_1926 = ADD_1924 * 2.0;
        auto ADD_1928 = MUL_1926 + 0.0127;
        auto ADD_1953 = SUB_1529 + ADD_1928;
        auto ADD_4009 = ADD_1953 + ADD_4006;
        out.x[26] = ADD_4009;  // (299, 322)
        auto MUL_4049 = ADD_3711 * 2.0;
        auto MUL_2044 = SUB_1459 * 0.045497;
        auto MUL_2057 = ADD_1477 * MUL_2044;
        auto MUL_2053 = SUB_1444 * 0.045497;
        auto MUL_2055 = ADD_1470 * 0.0345853;
        auto SUB_2056 = MUL_2053 - MUL_2055;
        auto MUL_2061 = ADD_1470 * SUB_2056;
        auto MUL_2050 = SUB_1459 * 0.0345853;
        auto MUL_2058 = SUB_1459 * MUL_2050;
        auto SUB_2060 = MUL_2057 - MUL_2058;
        auto ADD_2062 = SUB_2060 + MUL_2061;
        auto MUL_2064 = ADD_2062 * 2.0;
        auto ADD_2066 = MUL_2064 + 0.0345853;
        auto ADD_2091 = ADD_1953 + ADD_2066;
        out.x[28] = ADD_2091;  // (322, 336)
        auto MUL_4073 = MUL_4049 * 0.025;
        auto ADD_4078 = ADD_2091 + MUL_4073;
        out.x[27] = ADD_4078;  // (336, 338)
        auto ADD_2202 = MUL_1919 + MUL_1920;
        auto ADD_2199 = MUL_1915 + MUL_1917;
        auto MUL_4100 = ADD_1477 * ADD_1477;
        auto MUL_2203 = ADD_1470 * ADD_2199;
        auto ADD_2204 = ADD_2202 + MUL_2203;
        auto MUL_2206 = ADD_2204 * 2.0;
        auto SUB_2209 = MUL_2206 - 0.0127;
        auto ADD_2230 = SUB_1529 + SUB_2209;
        auto MUL_4103 = ADD_1470 * SUB_1444;
        auto MUL_4109 = SUB_1459 * ADD_1477;
        auto ADD_4142 = MUL_4109 + MUL_4103;
        auto MUL_4144 = ADD_4142 * 2.0;
        auto MUL_4169 = MUL_4144 * 0.02;
        auto MUL_4099 = SUB_1444 * SUB_1444;
        auto ADD_4112 = MUL_4099 + MUL_4100;
        auto MUL_4115 = ADD_4112 * 2.0;
        auto SUB_4118 = 1.0 - MUL_4115;
        auto MUL_4156 = SUB_4118 * 0.02;
        auto ADD_4174 = MUL_4156 + MUL_4169;
        auto ADD_4177 = ADD_2230 + ADD_4174;
        out.x[29] = ADD_4177;  // (338, 358)
        auto MUL_4225 = ADD_4142 * 2.0;
        auto MUL_4249 = MUL_4225 * 0.025;
        auto MUL_2330 = SUB_1459 * 0.0458574;
        auto MUL_2320 = SUB_1444 * 0.0458574;
        auto MUL_2334 = ADD_1470 * MUL_2320;
        auto MUL_2332 = ADD_1477 * 0.034106;
        auto SUB_2333 = MUL_2330 - MUL_2332;
        auto MUL_2338 = ADD_1477 * SUB_2333;
        auto MUL_2327 = SUB_1444 * 0.034106;
        auto MUL_2335 = SUB_1444 * MUL_2327;
        auto SUB_2337 = MUL_2334 - MUL_2335;
        auto ADD_2339 = SUB_2337 + MUL_2338;
        auto MUL_2341 = ADD_2339 * 2.0;
        auto ADD_2343 = MUL_2341 + 0.034106;
        auto ADD_2366 = ADD_2230 + ADD_2343;
        out.x[31] = ADD_2366;  // (358, 373)
        auto ADD_4254 = ADD_2366 + MUL_4249;
        out.x[30] = ADD_4254;  // (373, 374)
        auto ADD_2476 = MUL_1633 + MUL_1634;
        auto ADD_2473 = MUL_1629 + MUL_1631;
        auto MUL_2477 = ADD_1470 * ADD_2473;
        auto ADD_2478 = ADD_2476 + MUL_2477;
        auto MUL_2480 = ADD_2478 * 2.0;
        auto SUB_2483 = MUL_2480 - 0.0306011;
        auto ADD_2504 = SUB_1529 + SUB_2483;
        out.x[32] = ADD_2504;  // (374, 381)
        auto MUL_4394 = ADD_4142 * 2.0;
        auto MUL_4418 = MUL_4394 * 0.04;
        auto MUL_2608 = ADD_1477 * 0.0317096;
        auto MUL_2601 = SUB_1444 * 0.0317096;
        auto MUL_2613 = SUB_1444 * MUL_2601;
        auto MUL_2605 = SUB_1459 * 0.0016014;
        auto ADD_2609 = MUL_2605 + MUL_2608;
        auto MUL_2617 = ADD_1477 * ADD_2609;
        auto MUL_2595 = SUB_1444 * 0.0016014;
        auto MUL_2611 = ADD_1470 * MUL_2595;
        auto ADD_2615 = MUL_2611 + MUL_2613;
        auto ADD_2619 = ADD_2615 + MUL_2617;
        auto MUL_2622 = ADD_2619 * 2.0;
        auto SUB_2625 = 0.0317096 - MUL_2622;
        auto ADD_2649 = ADD_2504 + SUB_2625;
        out.x[34] = ADD_2649;  // (381, 396)
        auto ADD_4423 = ADD_2649 + MUL_4418;
        out.x[33] = ADD_4423;  // (396, 397)
        auto MUL_4457 = MUL_4394 * 0.02;
        auto ADD_4462 = ADD_2649 + MUL_4457;
        out.x[35] = ADD_4462;  // (397, 399)
        auto MUL_2774 = SUB_152 * ADD_136;
        auto MUL_2780 = ADD_141 * SUB_148;
        auto ADD_2814 = MUL_2780 + MUL_2774;
        auto MUL_2816 = ADD_2814 * 2.0;
        auto MUL_2839 = MUL_2816 * 0.105;
        auto MUL_106 = ADD_39 * MUL_85;
        auto MUL_109 = MUL_106 * 2.0;
        auto SUB_112 = 0.13585 - MUL_109;
        out.y[6] = SUB_112;  // (399, 407)
        auto ADD_2843 = SUB_112 + MUL_2839;
        out.y[2] = ADD_2843;  // (407, 408)
        auto MUL_2860 = MUL_2816 * 0.21;
        auto ADD_2864 = SUB_112 + MUL_2860;
        out.y[3] = ADD_2864;  // (408, 410)
        auto MUL_2881 = MUL_2816 * 0.315;
        auto ADD_2885 = SUB_112 + MUL_2881;
        out.y[4] = ADD_2885;  // (410, 412)
        auto MUL_2902 = MUL_2816 * 0.42;
        auto ADD_2906 = SUB_112 + MUL_2902;
        out.y[5] = ADD_2906;  // (412, 414)
        auto MUL_240 = SUB_152 * MUL_226;
        auto MUL_244 = SUB_148 * ADD_219;
        auto MUL_241 = ADD_136 * MUL_222;
        auto ADD_243 = MUL_240 + MUL_241;
        auto ADD_245 = ADD_243 + MUL_244;
        auto MUL_247 = ADD_245 * 2.0;
        auto SUB_250 = MUL_247 - 0.1197;
        auto ADD_261 = SUB_112 + SUB_250;
        out.y[7] = ADD_261;  // (414, 422)
        auto MUL_2931 = SUB_293 * ADD_277;
        auto MUL_2937 = ADD_282 * SUB_289;
        auto ADD_2971 = MUL_2937 + MUL_2931;
        auto MUL_2973 = ADD_2971 * 2.0;
        auto MUL_3014 = MUL_2973 * 0.0975;
        auto ADD_3018 = ADD_261 + MUL_3014;
        out.y[8] = ADD_3018;  // (422, 428)
        auto MUL_3035 = MUL_2973 * 0.195;
        auto ADD_3039 = ADD_261 + MUL_3035;
        out.y[9] = ADD_3039;  // (428, 430)
        auto MUL_3056 = MUL_2973 * 0.2925;
        auto ADD_3060 = ADD_261 + MUL_3056;
        out.y[10] = ADD_3060;  // (430, 432)
        auto MUL_3077 = MUL_2973 * 0.39;
        auto ADD_3081 = ADD_261 + MUL_3077;
        out.y[11] = ADD_3081;  // (432, 434)
        auto MUL_3088 = SUB_438 * ADD_422;
        auto MUL_3084 = SUB_434 * SUB_434;
        auto MUL_3094 = ADD_427 * SUB_434;
        auto ADD_3128 = MUL_3094 + MUL_3088;
        auto MUL_3130 = ADD_3128 * 2.0;
        auto MUL_3154 = MUL_3130 * 0.03;
        auto MUL_3087 = ADD_422 * ADD_422;
        auto ADD_3115 = MUL_3084 + MUL_3087;
        auto MUL_3118 = ADD_3115 * 2.0;
        auto SUB_3121 = 1.0 - MUL_3118;
        auto MUL_3148 = SUB_3121 * 0.09;
        auto ADD_3158 = MUL_3148 + MUL_3154;
        auto MUL_387 = SUB_293 * MUL_374;
        auto MUL_389 = SUB_289 * MUL_366;
        auto ADD_390 = MUL_387 + MUL_389;
        auto MUL_392 = ADD_390 * 2.0;
        auto ADD_406 = ADD_261 + MUL_392;
        auto ADD_3161 = ADD_406 + ADD_3158;
        out.y[12] = ADD_3161;  // (434, 452)
        auto SUB_3190 = MUL_3148 - MUL_3154;
        auto ADD_3193 = ADD_406 + SUB_3190;
        out.y[13] = ADD_3193;  // (452, 454)
        auto ADD_3215 = ADD_406 + MUL_3148;
        out.y[14] = ADD_3215;  // (454, 455)
        auto MUL_3222 = SUB_579 * SUB_562;
        auto MUL_3218 = ADD_574 * ADD_574;
        auto MUL_3221 = SUB_562 * SUB_562;
        auto ADD_3242 = MUL_3218 + MUL_3221;
        auto MUL_3245 = ADD_3242 * 2.0;
        auto SUB_3248 = 1.0 - MUL_3245;
        auto MUL_3274 = SUB_3248 * 0.03;
        auto MUL_3225 = ADD_568 * ADD_574;
        auto SUB_3255 = MUL_3225 - MUL_3222;
        auto MUL_3257 = SUB_3255 * 2.0;
        auto MUL_3280 = MUL_3257 * 0.09;
        auto ADD_3284 = MUL_3274 + MUL_3280;
        auto MUL_528 = SUB_434 * MUL_502;
        auto MUL_526 = ADD_422 * MUL_505;
        auto ADD_530 = MUL_526 + MUL_528;
        auto MUL_533 = ADD_530 * 2.0;
        auto SUB_536 = 0.093 - MUL_533;
        auto ADD_547 = ADD_406 + SUB_536;
        auto ADD_3287 = ADD_547 + ADD_3284;
        out.y[15] = ADD_3287;  // (455, 474)
        auto SUB_3314 = MUL_3280 - MUL_3274;
        auto ADD_3317 = ADD_547 + SUB_3314;
        out.y[16] = ADD_3317;  // (474, 476)
        auto ADD_3338 = ADD_547 + MUL_3280;
        out.y[17] = ADD_3338;  // (476, 477)
        auto MUL_3341 = ADD_704 * ADD_704;
        auto MUL_3344 = SUB_694 * SUB_694;
        auto ADD_3365 = MUL_3341 + MUL_3344;
        auto MUL_3368 = ADD_3365 * 2.0;
        auto SUB_3371 = 1.0 - MUL_3368;
        auto MUL_3397 = SUB_3371 * 0.06;
        auto MUL_659 = SUB_579 * MUL_648;
        auto MUL_662 = ADD_574 * MUL_640;
        auto SUB_663 = MUL_662 - MUL_659;
        auto MUL_665 = SUB_663 * 2.0;
        auto ADD_679 = ADD_547 + MUL_665;
        auto ADD_3407 = ADD_679 + MUL_3397;
        out.y[18] = ADD_3407;  // (477, 489)
        auto MUL_3506 = SUB_1062 * SUB_1062;
        auto MUL_3509 = SUB_1038 * SUB_1038;
        auto ADD_3530 = MUL_3506 + MUL_3509;
        auto MUL_3533 = ADD_3530 * 2.0;
        auto SUB_3536 = 1.0 - MUL_3533;
        auto MUL_3565 = SUB_3536 * 0.02;
        auto MUL_1101 = SUB_777 * MUL_1086;
        auto MUL_1102 = ADD_747 * MUL_1081;
        auto ADD_1104 = MUL_1101 + MUL_1102;
        auto MUL_1107 = ADD_1104 * 2.0;
        auto MUL_803 = ADD_704 * MUL_781;
        auto MUL_801 = SUB_694 * MUL_784;
        auto ADD_805 = MUL_801 + MUL_803;
        auto MUL_808 = ADD_805 * 2.0;
        auto SUB_811 = 0.0823 - MUL_808;
        auto ADD_821 = ADD_679 + SUB_811;
        auto ADD_1119 = ADD_821 + MUL_1107;
        auto SUB_3578 = ADD_1119 - MUL_3565;
        out.y[19] = SUB_3578;  // (489, 507)
        auto MUL_3681 = ADD_1477 * SUB_1444;
        auto MUL_3684 = SUB_1459 * ADD_1470;
        auto SUB_3714 = MUL_3684 - MUL_3681;
        auto MUL_3716 = SUB_3714 * 2.0;
        auto MUL_3739 = MUL_3716 * 0.06;
        auto MUL_1510 = ADD_1073 * MUL_1494;
        auto MUL_1357 = ADD_1073 * MUL_1346;
        auto MUL_1512 = SUB_1062 * MUL_1484;
        auto SUB_1514 = MUL_1510 - MUL_1512;
        auto MUL_1516 = SUB_1514 * 2.0;
        auto MUL_1360 = SUB_1062 * MUL_1338;
        auto SUB_1361 = MUL_1360 - MUL_1357;
        auto MUL_1363 = SUB_1361 * 2.0;
        auto ADD_1377 = ADD_1119 + MUL_1363;
        auto ADD_1530 = ADD_1377 + MUL_1516;
        auto ADD_3743 = ADD_1530 + MUL_3739;
        out.y[20] = ADD_3743;  // (507, 523)
        auto MUL_3760 = MUL_3716 * 0.02;
        auto ADD_3764 = ADD_1530 + MUL_3760;
        out.y[21] = ADD_3764;  // (523, 525)
        auto MUL_1644 = ADD_1477 * SUB_1632;
        auto MUL_1649 = ADD_1470 * MUL_1620;
        auto MUL_1646 = SUB_1444 * MUL_1626;
        auto SUB_1648 = MUL_1646 - MUL_1644;
        auto ADD_1650 = SUB_1648 + MUL_1649;
        auto MUL_1652 = ADD_1650 * 2.0;
        auto ADD_1668 = ADD_1530 + MUL_1652;
        out.y[22] = ADD_1668;  // (525, 532)
        auto MUL_3872 = SUB_3714 * 2.0;
        auto MUL_3895 = MUL_3872 * 0.04;
        auto MUL_1792 = ADD_1477 * ADD_1774;
        auto MUL_1796 = ADD_1470 * MUL_1759;
        auto MUL_1793 = SUB_1444 * MUL_1766;
        auto ADD_1795 = MUL_1792 + MUL_1793;
        auto SUB_1798 = ADD_1795 - MUL_1796;
        auto MUL_1800 = SUB_1798 * 2.0;
        auto ADD_1816 = ADD_1668 + MUL_1800;
        out.y[24] = ADD_1816;  // (532, 541)
        auto ADD_3899 = ADD_1816 + MUL_3895;
        out.y[23] = ADD_3899;  // (541, 542)
        auto MUL_3934 = MUL_3872 * 0.02;
        auto ADD_3938 = ADD_1816 + MUL_3934;
        out.y[25] = ADD_3938;  // (542, 544)
        auto MUL_3980 = SUB_3714 * 2.0;
        auto MUL_4003 = MUL_3980 * 0.02;
        auto MUL_1930 = ADD_1477 * SUB_1918;
        auto MUL_3942 = ADD_1477 * ADD_1470;
        auto MUL_1935 = ADD_1470 * MUL_1906;
        auto MUL_1932 = SUB_1444 * MUL_1912;
        auto SUB_1934 = MUL_1932 - MUL_1930;
        auto ADD_1936 = SUB_1934 + MUL_1935;
        auto MUL_1938 = ADD_1936 * 2.0;
        auto ADD_1954 = ADD_1530 + MUL_1938;
        auto MUL_3946 = SUB_1444 * SUB_1459;
        auto ADD_3956 = MUL_3946 + MUL_3942;
        auto MUL_3958 = ADD_3956 * 2.0;
        auto MUL_3991 = MUL_3958 * 0.02;
        auto ADD_4007 = MUL_3991 + MUL_4003;
        auto ADD_4010 = ADD_1954 + ADD_4007;
        out.y[26] = ADD_4010;  // (544, 560)
        auto MUL_4052 = SUB_3714 * 2.0;
        auto MUL_4075 = MUL_4052 * 0.025;
        auto MUL_2068 = ADD_1477 * SUB_2056;
        auto MUL_2073 = ADD_1470 * MUL_2044;
        auto MUL_2070 = SUB_1444 * MUL_2050;
        auto SUB_2072 = MUL_2070 - MUL_2068;
        auto ADD_2074 = SUB_2072 + MUL_2073;
        auto MUL_2076 = ADD_2074 * 2.0;
        auto ADD_2092 = ADD_1954 + MUL_2076;
        out.y[28] = ADD_2092;  // (560, 569)
        auto ADD_4079 = ADD_2092 + MUL_4075;
        out.y[27] = ADD_4079;  // (569, 570)
        auto MUL_2211 = ADD_1477 * ADD_2199;
        auto ADD_2214 = MUL_2211 + MUL_1932;
        auto SUB_2217 = MUL_1935 - ADD_2214;
        auto MUL_2219 = SUB_2217 * 2.0;
        auto ADD_2231 = ADD_1530 + MUL_2219;
        auto MUL_4101 = ADD_1470 * ADD_1477;
        auto MUL_4105 = ADD_1470 * SUB_1459;
        auto MUL_4107 = SUB_1459 * SUB_1444;
        auto ADD_4119 = MUL_4107 + MUL_4101;
        auto MUL_4122 = ADD_4119 * 2.0;
        auto MUL_4158 = MUL_4122 * 0.02;
        auto MUL_4110 = SUB_1444 * ADD_1477;
        auto SUB_4145 = MUL_4105 - MUL_4110;
        auto MUL_4147 = SUB_4145 * 2.0;
        auto MUL_4171 = MUL_4147 * 0.02;
        auto SUB_4175 = MUL_4171 - MUL_4158;
        auto ADD_4178 = ADD_2231 + SUB_4175;
        out.y[29] = ADD_4178;  // (570, 587)
        auto MUL_4228 = SUB_4145 * 2.0;
        auto MUL_4251 = MUL_4228 * 0.025;
        auto MUL_2347 = ADD_1477 * MUL_2320;
        auto MUL_2344 = ADD_1470 * SUB_2333;
        auto MUL_2345 = SUB_1459 * MUL_2327;
        auto SUB_2346 = MUL_2344 - MUL_2345;
        auto SUB_2349 = SUB_2346 - MUL_2347;
        auto MUL_2351 = SUB_2349 * 2.0;
        auto ADD_2367 = ADD_2231 + MUL_2351;
        out.y[31] = ADD_2367;  // (587, 596)
        auto ADD_4255 = ADD_2367 + MUL_4251;
        out.y[30] = ADD_4255;  // (596, 597)
        auto MUL_2485 = ADD_1477 * ADD_2473;
        auto ADD_2488 = MUL_2485 + MUL_1646;
        auto SUB_2491 = MUL_1649 - ADD_2488;
        auto MUL_2493 = SUB_2491 * 2.0;
        auto ADD_2505 = ADD_1530 + MUL_2493;
        out.y[32] = ADD_2505;  // (597, 602)
        auto MUL_4397 = SUB_4145 * 2.0;
        auto MUL_4420 = MUL_4397 * 0.04;
        auto MUL_2631 = ADD_1477 * MUL_2595;
        auto MUL_2626 = ADD_1470 * ADD_2609;
        auto MUL_2628 = SUB_1459 * MUL_2601;
        auto ADD_2629 = MUL_2626 + MUL_2628;
        auto SUB_2632 = MUL_2631 - ADD_2629;
        auto MUL_2634 = SUB_2632 * 2.0;
        auto ADD_2650 = ADD_2505 + MUL_2634;
        out.y[34] = ADD_2650;  // (602, 611)
        auto ADD_4424 = ADD_2650 + MUL_4420;
        out.y[33] = ADD_4424;  // (611, 612)
        auto MUL_4459 = MUL_4397 * 0.02;
        auto ADD_4463 = ADD_2650 + MUL_4459;
        out.y[35] = ADD_4463;  // (612, 614)
        auto MUL_2769 = ADD_141 * ADD_141;
        auto MUL_2773 = ADD_136 * ADD_136;
        auto ADD_2817 = MUL_2769 + MUL_2773;
        auto MUL_2820 = ADD_2817 * 2.0;
        auto SUB_2823 = 1.0 - MUL_2820;
        auto MUL_2841 = SUB_2823 * 0.105;
        auto ADD_2844 = 1.003559 + MUL_2841;
        out.z[2] = ADD_2844;  // (614, 621)
        auto MUL_2862 = SUB_2823 * 0.21;
        auto ADD_2865 = 1.003559 + MUL_2862;
        out.z[3] = ADD_2865;  // (621, 623)
        auto MUL_2883 = SUB_2823 * 0.315;
        auto ADD_2886 = 1.003559 + MUL_2883;
        out.z[4] = ADD_2886;  // (623, 625)
        auto MUL_2904 = SUB_2823 * 0.42;
        auto ADD_2907 = 1.003559 + MUL_2904;
        out.z[5] = ADD_2907;  // (625, 627)
        auto MUL_251 = SUB_152 * MUL_222;
        auto MUL_254 = ADD_141 * ADD_219;
        auto MUL_252 = ADD_136 * MUL_226;
        auto SUB_253 = MUL_251 - MUL_252;
        auto SUB_255 = SUB_253 - MUL_254;
        auto MUL_257 = SUB_255 * 2.0;
        auto ADD_259 = MUL_257 + 0.425;
        auto ADD_262 = 1.003559 + ADD_259;
        out.z[7] = ADD_262;  // (627, 635)
        auto MUL_2926 = ADD_282 * ADD_282;
        auto MUL_2930 = ADD_277 * ADD_277;
        auto ADD_2974 = MUL_2926 + MUL_2930;
        auto MUL_2977 = ADD_2974 * 2.0;
        auto SUB_2980 = 1.0 - MUL_2977;
        auto MUL_3016 = SUB_2980 * 0.0975;
        auto ADD_3019 = ADD_262 + MUL_3016;
        out.z[8] = ADD_3019;  // (635, 642)
        auto MUL_3037 = SUB_2980 * 0.195;
        auto ADD_3040 = ADD_262 + MUL_3037;
        out.z[9] = ADD_3040;  // (642, 644)
        auto MUL_3058 = SUB_2980 * 0.2925;
        auto ADD_3061 = ADD_262 + MUL_3058;
        out.z[10] = ADD_3061;  // (644, 646)
        auto MUL_3079 = SUB_2980 * 0.39;
        auto ADD_3082 = ADD_262 + MUL_3079;
        out.z[11] = ADD_3082;  // (646, 648)
        auto SUB_3122 = MUL_3094 - MUL_3088;
        auto MUL_3124 = SUB_3122 * 2.0;
        auto MUL_3150 = MUL_3124 * 0.09;
        auto MUL_3083 = ADD_427 * ADD_427;
        auto ADD_3131 = MUL_3083 + MUL_3087;
        auto MUL_3134 = ADD_3131 * 2.0;
        auto SUB_3137 = 1.0 - MUL_3134;
        auto MUL_3156 = SUB_3137 * 0.03;
        auto ADD_3159 = MUL_3150 + MUL_3156;
        auto MUL_397 = ADD_282 * MUL_366;
        auto MUL_395 = ADD_277 * MUL_374;
        auto ADD_398 = MUL_395 + MUL_397;
        auto MUL_401 = ADD_398 * 2.0;
        auto SUB_404 = 0.39225 - MUL_401;
        auto ADD_407 = ADD_262 + SUB_404;
        auto ADD_3162 = ADD_407 + ADD_3159;
        out.z[12] = ADD_3162;  // (648, 664)
        auto SUB_3191 = MUL_3150 - MUL_3156;
        auto ADD_3194 = ADD_407 + SUB_3191;
        out.z[13] = ADD_3194;  // (664, 666)
        auto ADD_3216 = ADD_407 + MUL_3150;
        out.z[14] = ADD_3216;  // (666, 667)
        auto ADD_3249 = MUL_3225 + MUL_3222;
        auto MUL_3251 = ADD_3249 * 2.0;
        auto MUL_3276 = MUL_3251 * 0.03;
        auto MUL_3217 = ADD_568 * ADD_568;
        auto ADD_3258 = MUL_3217 + MUL_3221;
        auto MUL_3261 = ADD_3258 * 2.0;
        auto SUB_3264 = 1.0 - MUL_3261;
        auto MUL_3282 = SUB_3264 * 0.09;
        auto ADD_3285 = MUL_3276 + MUL_3282;
        auto MUL_537 = SUB_438 * MUL_505;
        auto MUL_540 = ADD_427 * MUL_502;
        auto SUB_542 = MUL_540 - MUL_537;
        auto MUL_544 = SUB_542 * 2.0;
        auto ADD_548 = ADD_407 + MUL_544;
        auto ADD_3288 = ADD_548 + ADD_3285;
        out.z[15] = ADD_3288;  // (667, 682)
        auto SUB_3315 = MUL_3282 - MUL_3276;
        auto ADD_3318 = ADD_548 + SUB_3315;
        out.z[16] = ADD_3318;  // (682, 684)
        auto ADD_3339 = ADD_548 + MUL_3282;
        out.z[17] = ADD_3339;  // (684, 685)
        auto MUL_3345 = SUB_708 * SUB_694;
        auto MUL_3348 = ADD_698 * ADD_704;
        auto ADD_3372 = MUL_3348 + MUL_3345;
        auto MUL_3374 = ADD_3372 * 2.0;
        auto MUL_3399 = MUL_3374 * 0.06;
        auto MUL_668 = SUB_562 * MUL_648;
        auto MUL_670 = ADD_568 * MUL_640;
        auto ADD_671 = MUL_668 + MUL_670;
        auto MUL_674 = ADD_671 * 2.0;
        auto SUB_677 = 0.09465 - MUL_674;
        auto ADD_680 = ADD_548 + SUB_677;
        auto ADD_3408 = ADD_680 + MUL_3399;
        out.z[18] = ADD_3408;  // (685, 697)
        auto MUL_3510 = ADD_1073 * SUB_1038;
        auto MUL_3513 = ADD_1049 * SUB_1062;
        auto ADD_3537 = MUL_3513 + MUL_3510;
        auto MUL_3539 = ADD_3537 * 2.0;
        auto MUL_3569 = MUL_3539 * 0.02;
        auto MUL_1109 = SUB_777 * MUL_1081;
        auto MUL_1111 = ADD_747 * MUL_1086;
        auto SUB_1113 = MUL_1111 - MUL_1109;
        auto MUL_1116 = SUB_1113 * 2.0;
        auto MUL_812 = SUB_708 * MUL_784;
        auto MUL_814 = ADD_698 * MUL_781;
        auto ADD_816 = MUL_812 + MUL_814;
        auto MUL_818 = ADD_816 * 2.0;
        auto ADD_822 = ADD_680 + MUL_818;
        auto ADD_1120 = ADD_822 + MUL_1116;
        auto SUB_3579 = ADD_1120 - MUL_3569;
        out.z[19] = SUB_3579;  // (697, 713)
        auto ADD_3717 = MUL_3940 + MUL_4099;
        auto MUL_3720 = ADD_3717 * 2.0;
        auto SUB_3723 = 1.0 - MUL_3720;
        auto MUL_3741 = SUB_3723 * 0.06;
        auto MUL_1519 = SUB_1038 * MUL_1494;
        auto MUL_1366 = SUB_1038 * MUL_1346;
        auto MUL_1521 = ADD_1049 * MUL_1484;
        auto ADD_1523 = MUL_1519 + MUL_1521;
        auto MUL_1525 = ADD_1523 * 2.0;
        auto SUB_1528 = MUL_1525 - 0.037;
        auto MUL_1368 = ADD_1049 * MUL_1338;
        auto ADD_1369 = MUL_1366 + MUL_1368;
        auto MUL_1372 = ADD_1369 * 2.0;
        auto SUB_1375 = 0.0375 - MUL_1372;
        auto ADD_1378 = ADD_1120 + SUB_1375;
        auto ADD_1531 = ADD_1378 + SUB_1528;
        auto ADD_3744 = ADD_1531 + MUL_3741;
        out.z[20] = ADD_3744;  // (713, 730)
        auto MUL_3762 = SUB_3723 * 0.02;
        auto ADD_3765 = ADD_1531 + MUL_3762;
        out.z[21] = ADD_3765;  // (730, 732)
        auto MUL_1654 = ADD_1477 * MUL_1626;
        auto MUL_1659 = SUB_1459 * MUL_1620;
        auto MUL_1656 = SUB_1444 * SUB_1632;
        auto ADD_1657 = MUL_1654 + MUL_1656;
        auto ADD_1660 = ADD_1657 + MUL_1659;
        auto MUL_1663 = ADD_1660 * 2.0;
        auto SUB_1666 = 0.062792 - MUL_1663;
        auto ADD_1669 = ADD_1531 + SUB_1666;
        out.z[22] = ADD_1669;  // (732, 740)
        auto MUL_3876 = ADD_3717 * 2.0;
        auto SUB_3879 = 1.0 - MUL_3876;
        auto MUL_3897 = SUB_3879 * 0.04;
        auto MUL_1802 = ADD_1477 * MUL_1766;
        auto MUL_1807 = SUB_1459 * MUL_1759;
        auto MUL_1804 = SUB_1444 * ADD_1774;
        auto SUB_1806 = MUL_1804 - MUL_1802;
        auto ADD_1809 = SUB_1806 + MUL_1807;
        auto MUL_1811 = ADD_1809 * 2.0;
        auto SUB_1814 = MUL_1811 - 0.001934;
        auto ADD_1817 = ADD_1669 + SUB_1814;
        out.z[24] = ADD_1817;  // (740, 751)
        auto ADD_3900 = ADD_1817 + MUL_3897;
        out.z[23] = ADD_3900;  // (751, 752)
        auto MUL_3936 = SUB_3879 * 0.02;
        auto ADD_3939 = ADD_1817 + MUL_3936;
        out.z[25] = ADD_3939;  // (752, 754)
        auto SUB_3959 = MUL_3683 - MUL_3679;
        auto MUL_3984 = ADD_3717 * 2.0;
        auto SUB_3987 = 1.0 - MUL_3984;
        auto MUL_4005 = SUB_3987 * 0.02;
        auto MUL_3961 = SUB_3959 * 2.0;
        auto MUL_3993 = MUL_3961 * 0.02;
        auto ADD_4008 = MUL_3993 + MUL_4005;
        auto MUL_1940 = ADD_1477 * MUL_1912;
        auto MUL_1945 = SUB_1459 * MUL_1906;
        auto MUL_1942 = SUB_1444 * SUB_1918;
        auto ADD_1943 = MUL_1940 + MUL_1942;
        auto ADD_1946 = ADD_1943 + MUL_1945;
        auto MUL_1949 = ADD_1946 * 2.0;
        auto SUB_1952 = 0.0693075 - MUL_1949;
        auto ADD_1955 = ADD_1531 + SUB_1952;
        auto ADD_4011 = ADD_1955 + ADD_4008;
        out.z[26] = ADD_4011;  // (754, 770)
        auto MUL_4056 = ADD_3717 * 2.0;
        auto SUB_4059 = 1.0 - MUL_4056;
        auto MUL_4077 = SUB_4059 * 0.025;
        auto MUL_2078 = ADD_1477 * MUL_2050;
        auto MUL_2083 = SUB_1459 * MUL_2044;
        auto MUL_2080 = SUB_1444 * SUB_2056;
        auto ADD_2081 = MUL_2078 + MUL_2080;
        auto ADD_2084 = ADD_2081 + MUL_2083;
        auto MUL_2087 = ADD_2084 * 2.0;
        auto SUB_2090 = 0.045497 - MUL_2087;
        auto ADD_2093 = ADD_1955 + SUB_2090;
        out.z[28] = ADD_2093;  // (770, 781)
        auto ADD_4080 = ADD_2093 + MUL_4077;
        out.z[27] = ADD_4080;  // (781, 782)
        auto SUB_4124 = MUL_4109 - MUL_4103;
        auto ADD_4148 = MUL_4099 + MUL_3940;
        auto MUL_4126 = SUB_4124 * 2.0;
        auto MUL_4161 = MUL_4126 * 0.02;
        auto MUL_4151 = ADD_4148 * 2.0;
        auto SUB_4154 = 1.0 - MUL_4151;
        auto MUL_4173 = SUB_4154 * 0.02;
        auto ADD_4176 = MUL_4161 + MUL_4173;
        auto MUL_2222 = SUB_1444 * ADD_2199;
        auto SUB_2223 = MUL_1940 - MUL_2222;
        auto SUB_2225 = SUB_2223 - MUL_1945;
        auto MUL_2227 = SUB_2225 * 2.0;
        auto ADD_2229 = MUL_2227 + 0.0693075;
        auto ADD_2232 = ADD_1531 + ADD_2229;
        auto ADD_4179 = ADD_2232 + ADD_4176;
        out.z[29] = ADD_4179;  // (782, 797)
        auto MUL_4232 = ADD_4148 * 2.0;
        auto SUB_4235 = 1.0 - MUL_4232;
        auto MUL_4253 = SUB_4235 * 0.025;
        auto MUL_2353 = ADD_1470 * MUL_2327;
        auto MUL_2355 = SUB_1459 * SUB_2333;
        auto ADD_2356 = MUL_2353 + MUL_2355;
        auto MUL_2358 = SUB_1444 * MUL_2320;
        auto ADD_2359 = ADD_2356 + MUL_2358;
        auto MUL_2362 = ADD_2359 * 2.0;
        auto SUB_2365 = 0.0458574 - MUL_2362;
        auto ADD_2368 = ADD_2232 + SUB_2365;
        out.z[31] = ADD_2368;  // (797, 808)
        auto ADD_4256 = ADD_2368 + MUL_4253;
        out.z[30] = ADD_4256;  // (808, 809)
        auto MUL_2496 = SUB_1444 * ADD_2473;
        auto SUB_2497 = MUL_1654 - MUL_2496;
        auto SUB_2499 = SUB_2497 - MUL_1659;
        auto MUL_2501 = SUB_2499 * 2.0;
        auto ADD_2503 = MUL_2501 + 0.062792;
        auto ADD_2506 = ADD_1531 + ADD_2503;
        out.z[32] = ADD_2506;  // (809, 815)
        auto MUL_4401 = ADD_4148 * 2.0;
        auto SUB_4404 = 1.0 - MUL_4401;
        auto MUL_4422 = SUB_4404 * 0.04;
        auto MUL_2636 = ADD_1470 * MUL_2601;
        auto MUL_2638 = SUB_1459 * ADD_2609;
        auto SUB_2640 = MUL_2638 - MUL_2636;
        auto MUL_2641 = SUB_1444 * MUL_2595;
        auto ADD_2643 = SUB_2640 + MUL_2641;
        auto MUL_2645 = ADD_2643 * 2.0;
        auto SUB_2648 = MUL_2645 - 0.0016014;
        auto ADD_2651 = ADD_2506 + SUB_2648;
        out.z[34] = ADD_2651;  // (815, 826)
        auto ADD_4425 = ADD_2651 + MUL_4422;
        out.z[33] = ADD_4425;  // (826, 827)
        auto MUL_4461 = SUB_4404 * 0.02;
        auto ADD_4464 = ADD_2651 + MUL_4461;
        out.z[35] = ADD_4464;  // (827, 829)
    }

#define RETURN_FAILURE                                                                                       \
    std::cout << __LINE__ << std::endl;                                                                      \
    return false;

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        // if (/*base_link*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.9144, 0.08))
        //{
        //     return false;
        // }  // (0, 0)
        // if (/*shoulder_link*/ sphere_environment_in_collision(environment, 0.0, 0.0, 1.003559, 0.08))
        //{
        //     return false;
        // }  // (0, 0)
        auto INPUT_1 = q[1];
        auto DIV_123 = INPUT_1 * 0.5;
        auto SIN_124 = DIV_123.sin();
        auto COS_130 = DIV_123.cos();
        auto INPUT_0 = q[0];
        auto DIV_7 = INPUT_0 * 0.5;
        auto SIN_8 = DIV_7.sin();
        auto COS_14 = DIV_7.cos();
        auto MUL_41 = COS_14 * 0.7073883;
        auto MUL_38 = COS_14 * 0.7068252;
        auto MUL_32 = SIN_8 * 0.7073883;
        auto ADD_39 = MUL_32 + MUL_38;
        auto MUL_56 = ADD_39 * 0.7071068;
        auto MUL_147 = MUL_56 * COS_130;
        auto MUL_144 = MUL_56 * SIN_124;
        auto SUB_148 = MUL_147 - MUL_144;
        auto ADD_136 = MUL_147 + MUL_144;
        auto MUL_2805 = ADD_136 * SUB_148;
        auto MUL_85 = ADD_39 * 0.13585;
        auto MUL_47 = SIN_8 * 0.7068252;
        auto SUB_48 = MUL_41 - MUL_47;
        auto MUL_95 = SUB_48 * MUL_85;
        auto MUL_100 = MUL_95 * 2.0;
        auto MUL_59 = SUB_48 * 0.7071068;
        auto MUL_140 = MUL_59 * COS_130;
        auto MUL_138 = MUL_59 * SIN_124;
        auto SUB_152 = MUL_140 - MUL_138;
        auto ADD_141 = MUL_138 + MUL_140;
        auto MUL_2799 = SUB_152 * ADD_141;
        auto SUB_2838 = MUL_2799 - MUL_2805;
        auto MUL_2840 = SUB_2838 * 2.0;
        auto MUL_2864 = MUL_2840 * 0.21;
        auto SUB_2869 = MUL_2864 - MUL_100;
        auto MUL_2801 = SUB_152 * ADD_136;
        auto MUL_2807 = ADD_141 * SUB_148;
        auto ADD_2841 = MUL_2807 + MUL_2801;
        auto MUL_2843 = ADD_2841 * 2.0;
        auto MUL_2866 = MUL_2843 * 0.21;
        auto MUL_106 = ADD_39 * MUL_85;
        auto MUL_109 = MUL_106 * 2.0;
        auto SUB_112 = 0.13585 - MUL_109;
        auto ADD_2870 = SUB_112 + MUL_2866;
        auto MUL_2796 = ADD_141 * ADD_141;
        auto MUL_2800 = ADD_136 * ADD_136;
        auto ADD_2844 = MUL_2796 + MUL_2800;
        auto MUL_2847 = ADD_2844 * 2.0;
        auto SUB_2850 = 1.0 - MUL_2847;
        auto MUL_2868 = SUB_2850 * 0.21;
        auto ADD_2871 = 1.003559 + MUL_2868;
        auto MUL_2885 = MUL_2840 * 0.105;
        auto SUB_2890 = MUL_2885 - MUL_100;
        auto MUL_2887 = MUL_2843 * 0.105;
        auto ADD_2891 = SUB_112 + MUL_2887;
        auto MUL_2889 = SUB_2850 * 0.105;
        auto ADD_2892 = 1.003559 + MUL_2889;
        auto MUL_2927 = MUL_2840 * 0.315;
        auto SUB_2932 = MUL_2927 - MUL_100;
        auto MUL_2929 = MUL_2843 * 0.315;
        auto ADD_2933 = SUB_112 + MUL_2929;
        auto MUL_2931 = SUB_2850 * 0.315;
        auto ADD_2934 = 1.003559 + MUL_2931;
        auto MUL_2948 = MUL_2840 * 0.42;
        auto SUB_2953 = MUL_2948 - MUL_100;
        auto MUL_2950 = MUL_2843 * 0.42;
        auto ADD_2954 = SUB_112 + MUL_2950;
        auto MUL_2952 = SUB_2850 * 0.42;
        auto ADD_2955 = 1.003559 + MUL_2952;
        auto NEGATE_101 = -MUL_100;
        if (/*base_link vs. upper_arm_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, SUB_2869, ADD_2870, ADD_2871, 0.29))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2890, ADD_2891, ADD_2892, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2869, ADD_2870, ADD_2871, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2932, ADD_2933, ADD_2934, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2953, ADD_2954, ADD_2955, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, NEGATE_101, SUB_112, 1.003559, 0.08))
            {
                return false;
            }
        }  // (0, 68)
        if (/*upper_arm_link*/ sphere_environment_in_collision(
            environment, SUB_2869, ADD_2870, ADD_2871, 0.29))
        {
            if (sphere_environment_in_collision(environment, SUB_2890, ADD_2891, ADD_2892, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2869, ADD_2870, ADD_2871, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2932, ADD_2933, ADD_2934, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2953, ADD_2954, ADD_2955, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, NEGATE_101, SUB_112, 1.003559, 0.08))
            {
                return false;
            }
        }  // (68, 68)
        auto MUL_214 = ADD_141 * 0.425;
        auto MUL_226 = ADD_136 * 0.425;
        auto MUL_233 = SUB_148 * MUL_226;
        auto MUL_217 = SUB_148 * 0.1197;
        auto ADD_219 = MUL_214 + MUL_217;
        auto MUL_230 = SUB_152 * ADD_219;
        auto MUL_222 = ADD_136 * 0.1197;
        auto MUL_231 = ADD_141 * MUL_222;
        auto ADD_232 = MUL_230 + MUL_231;
        auto SUB_235 = ADD_232 - MUL_233;
        auto MUL_237 = SUB_235 * 2.0;
        auto SUB_260 = MUL_237 - MUL_100;
        auto INPUT_2 = q[2];
        auto DIV_264 = INPUT_2 * 0.5;
        auto SIN_265 = DIV_264.sin();
        auto COS_271 = DIV_264.cos();
        auto MUL_290 = SUB_152 * COS_271;
        auto MUL_279 = SUB_152 * SIN_265;
        auto MUL_288 = SUB_148 * COS_271;
        auto MUL_276 = SUB_148 * SIN_265;
        auto MUL_281 = ADD_141 * COS_271;
        auto ADD_282 = MUL_279 + MUL_281;
        auto MUL_2986 = ADD_282 * ADD_282;
        auto MUL_292 = ADD_141 * SIN_265;
        auto SUB_293 = MUL_290 - MUL_292;
        auto MUL_2989 = SUB_293 * ADD_282;
        auto MUL_273 = ADD_136 * COS_271;
        auto ADD_277 = MUL_273 + MUL_276;
        auto MUL_2993 = ADD_277 * ADD_282;
        auto MUL_285 = ADD_136 * SIN_265;
        auto SUB_289 = MUL_288 - MUL_285;
        auto MUL_2988 = SUB_293 * SUB_289;
        auto ADD_3013 = MUL_2993 + MUL_2988;
        auto MUL_3016 = ADD_3013 * 2.0;
        auto MUL_3049 = MUL_3016 * 0.003;
        auto MUL_2987 = SUB_289 * SUB_289;
        auto ADD_2998 = MUL_2986 + MUL_2987;
        auto MUL_3001 = ADD_2998 * 2.0;
        auto SUB_3004 = 1.0 - MUL_3001;
        auto MUL_3042 = SUB_3004 * 0.002;
        auto SUB_3061 = MUL_3042 - MUL_3049;
        auto MUL_2995 = ADD_277 * SUB_289;
        auto SUB_3028 = MUL_2989 - MUL_2995;
        auto MUL_3030 = SUB_3028 * 2.0;
        auto MUL_3056 = MUL_3030 * 0.185;
        auto ADD_3064 = SUB_3061 + MUL_3056;
        auto ADD_3067 = SUB_260 + ADD_3064;
        auto SUB_3005 = MUL_2988 - MUL_2993;
        auto MUL_3007 = SUB_3005 * 2.0;
        auto MUL_3044 = MUL_3007 * 0.002;
        auto MUL_2991 = SUB_293 * ADD_277;
        auto MUL_2997 = ADD_282 * SUB_289;
        auto ADD_3031 = MUL_2997 + MUL_2991;
        auto MUL_3033 = ADD_3031 * 2.0;
        auto MUL_3058 = MUL_3033 * 0.185;
        auto MUL_2990 = ADD_277 * ADD_277;
        auto ADD_3018 = MUL_2987 + MUL_2990;
        auto MUL_3021 = ADD_3018 * 2.0;
        auto SUB_3024 = 1.0 - MUL_3021;
        auto MUL_3052 = SUB_3024 * 0.003;
        auto ADD_3062 = MUL_3044 + MUL_3052;
        auto ADD_3065 = ADD_3062 + MUL_3058;
        auto MUL_240 = SUB_152 * MUL_226;
        auto MUL_244 = SUB_148 * ADD_219;
        auto MUL_241 = ADD_136 * MUL_222;
        auto ADD_243 = MUL_240 + MUL_241;
        auto ADD_245 = ADD_243 + MUL_244;
        auto MUL_247 = ADD_245 * 2.0;
        auto SUB_250 = MUL_247 - 0.1197;
        auto ADD_261 = SUB_112 + SUB_250;
        auto ADD_3068 = ADD_261 + ADD_3065;
        auto SUB_3025 = MUL_2997 - MUL_2991;
        auto ADD_3008 = MUL_2995 + MUL_2989;
        auto ADD_3034 = MUL_2986 + MUL_2990;
        auto MUL_3037 = ADD_3034 * 2.0;
        auto SUB_3040 = 1.0 - MUL_3037;
        auto MUL_3060 = SUB_3040 * 0.185;
        auto MUL_3027 = SUB_3025 * 2.0;
        auto MUL_3054 = MUL_3027 * 0.003;
        auto MUL_3011 = ADD_3008 * 2.0;
        auto MUL_3046 = MUL_3011 * 0.002;
        auto SUB_3063 = MUL_3054 - MUL_3046;
        auto ADD_3066 = SUB_3063 + MUL_3060;
        auto MUL_251 = SUB_152 * MUL_222;
        auto MUL_254 = ADD_141 * ADD_219;
        auto MUL_252 = ADD_136 * MUL_226;
        auto SUB_253 = MUL_251 - MUL_252;
        auto SUB_255 = SUB_253 - MUL_254;
        auto MUL_257 = SUB_255 * 2.0;
        auto ADD_259 = MUL_257 + 0.425;
        auto ADD_262 = 1.003559 + ADD_259;
        auto ADD_3069 = ADD_262 + ADD_3066;
        auto MUL_3101 = MUL_3030 * 0.0975;
        auto ADD_3106 = SUB_260 + MUL_3101;
        auto MUL_3103 = MUL_3033 * 0.0975;
        auto ADD_3107 = ADD_261 + MUL_3103;
        auto MUL_3105 = SUB_3040 * 0.0975;
        auto ADD_3108 = ADD_262 + MUL_3105;
        auto MUL_3122 = MUL_3030 * 0.195;
        auto ADD_3127 = SUB_260 + MUL_3122;
        auto MUL_3124 = MUL_3033 * 0.195;
        auto ADD_3128 = ADD_261 + MUL_3124;
        auto MUL_3126 = SUB_3040 * 0.195;
        auto ADD_3129 = ADD_262 + MUL_3126;
        auto MUL_3143 = MUL_3030 * 0.2925;
        auto ADD_3148 = SUB_260 + MUL_3143;
        auto MUL_3145 = MUL_3033 * 0.2925;
        auto ADD_3149 = ADD_261 + MUL_3145;
        auto MUL_3147 = SUB_3040 * 0.2925;
        auto ADD_3150 = ADD_262 + MUL_3147;
        auto MUL_3164 = MUL_3030 * 0.39;
        auto ADD_3169 = SUB_260 + MUL_3164;
        auto MUL_3166 = MUL_3033 * 0.39;
        auto ADD_3170 = ADD_261 + MUL_3166;
        auto MUL_3168 = SUB_3040 * 0.39;
        auto ADD_3171 = ADD_262 + MUL_3168;
        if (/*base_link vs. forearm_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (68, 184)
        if (/*forearm_link*/ sphere_environment_in_collision(
            environment, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (sphere_environment_in_collision(environment, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (184, 184)
        if (/*shoulder_link vs. forearm_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (184, 184)
        auto MUL_338 = SUB_293 * 0.7071068;
        auto MUL_334 = SUB_289 * 0.7071068;
        auto MUL_342 = ADD_282 * 0.7071068;
        auto SUB_362 = MUL_338 - MUL_342;
        auto ADD_343 = MUL_338 + MUL_342;
        auto MUL_329 = ADD_277 * 0.7071068;
        auto SUB_355 = MUL_334 - MUL_329;
        auto ADD_335 = MUL_329 + MUL_334;
        auto MUL_366 = ADD_282 * 0.39225;
        auto MUL_378 = SUB_293 * MUL_366;
        auto MUL_374 = ADD_277 * 0.39225;
        auto MUL_380 = SUB_289 * MUL_374;
        auto SUB_382 = MUL_378 - MUL_380;
        auto MUL_384 = SUB_382 * 2.0;
        auto ADD_405 = SUB_260 + MUL_384;
        auto INPUT_3 = q[3];
        auto DIV_409 = INPUT_3 * 0.5;
        auto SIN_410 = DIV_409.sin();
        auto COS_416 = DIV_409.cos();
        auto MUL_435 = SUB_362 * COS_416;
        auto MUL_424 = SUB_362 * SIN_410;
        auto MUL_433 = SUB_355 * COS_416;
        auto MUL_421 = SUB_355 * SIN_410;
        auto MUL_426 = ADD_343 * COS_416;
        auto ADD_427 = MUL_424 + MUL_426;
        auto MUL_437 = ADD_343 * SIN_410;
        auto SUB_438 = MUL_435 - MUL_437;
        auto MUL_418 = ADD_335 * COS_416;
        auto ADD_422 = MUL_418 + MUL_421;
        auto MUL_3191 = ADD_422 * ADD_427;
        auto MUL_430 = ADD_335 * SIN_410;
        auto SUB_434 = MUL_433 - MUL_430;
        auto MUL_3186 = SUB_438 * SUB_434;
        auto ADD_3211 = MUL_3191 + MUL_3186;
        auto MUL_3214 = ADD_3211 * 2.0;
        auto MUL_3246 = MUL_3214 * 0.09;
        auto SUB_3258 = ADD_405 - MUL_3246;
        auto MUL_3185 = SUB_434 * SUB_434;
        auto MUL_3188 = ADD_422 * ADD_422;
        auto ADD_3216 = MUL_3185 + MUL_3188;
        auto MUL_3219 = ADD_3216 * 2.0;
        auto SUB_3222 = 1.0 - MUL_3219;
        auto MUL_3249 = SUB_3222 * 0.09;
        auto MUL_387 = SUB_293 * MUL_374;
        auto MUL_389 = SUB_289 * MUL_366;
        auto ADD_390 = MUL_387 + MUL_389;
        auto MUL_392 = ADD_390 * 2.0;
        auto ADD_406 = ADD_261 + MUL_392;
        auto ADD_3259 = ADD_406 + MUL_3249;
        auto MUL_3189 = SUB_438 * ADD_422;
        auto MUL_3195 = ADD_427 * SUB_434;
        auto SUB_3223 = MUL_3195 - MUL_3189;
        auto MUL_3225 = SUB_3223 * 2.0;
        auto MUL_3251 = MUL_3225 * 0.09;
        auto MUL_397 = ADD_282 * MUL_366;
        auto MUL_395 = ADD_277 * MUL_374;
        auto ADD_398 = MUL_395 + MUL_397;
        auto MUL_401 = ADD_398 * 2.0;
        auto SUB_404 = 0.39225 - MUL_401;
        auto ADD_407 = ADD_262 + SUB_404;
        auto ADD_3260 = ADD_407 + MUL_3251;
        auto MUL_3187 = SUB_438 * ADD_427;
        auto MUL_3193 = ADD_422 * SUB_434;
        auto SUB_3226 = MUL_3187 - MUL_3193;
        auto MUL_3228 = SUB_3226 * 2.0;
        auto MUL_3275 = MUL_3228 * 0.03;
        auto SUB_3280 = MUL_3275 - MUL_3246;
        auto ADD_3283 = ADD_405 + SUB_3280;
        auto ADD_3229 = MUL_3195 + MUL_3189;
        auto MUL_3231 = ADD_3229 * 2.0;
        auto MUL_3277 = MUL_3231 * 0.03;
        auto ADD_3281 = MUL_3249 + MUL_3277;
        auto ADD_3284 = ADD_406 + ADD_3281;
        auto MUL_3184 = ADD_427 * ADD_427;
        auto ADD_3232 = MUL_3184 + MUL_3188;
        auto MUL_3235 = ADD_3232 * 2.0;
        auto SUB_3238 = 1.0 - MUL_3235;
        auto MUL_3279 = SUB_3238 * 0.03;
        auto ADD_3282 = MUL_3251 + MUL_3279;
        auto ADD_3285 = ADD_407 + ADD_3282;
        auto ADD_3311 = MUL_3246 + MUL_3275;
        auto SUB_3315 = ADD_405 - ADD_3311;
        auto SUB_3313 = MUL_3249 - MUL_3277;
        auto ADD_3316 = ADD_406 + SUB_3313;
        auto SUB_3314 = MUL_3251 - MUL_3279;
        auto ADD_3317 = ADD_407 + SUB_3314;
        if (/*base_link vs. wrist_1_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (184, 270)
        if (/*shoulder_link vs. wrist_1_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (270, 270)
        if (/*upper_arm_link vs. wrist_1_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (270, 270)
        if (/*wrist_1_link*/ sphere_environment_in_collision(environment, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_environment_in_collision(environment, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (270, 270)
        auto MUL_502 = SUB_434 * 0.093;
        auto MUL_513 = SUB_438 * MUL_502;
        auto MUL_505 = ADD_422 * 0.093;
        auto MUL_515 = ADD_427 * MUL_505;
        auto ADD_517 = MUL_513 + MUL_515;
        auto MUL_521 = ADD_517 * 2.0;
        auto SUB_546 = ADD_405 - MUL_521;
        auto INPUT_4 = q[4];
        auto DIV_550 = INPUT_4 * 0.5;
        auto SIN_551 = DIV_550.sin();
        auto COS_557 = DIV_550.cos();
        auto MUL_575 = SUB_438 * COS_557;
        auto MUL_570 = SUB_438 * SIN_551;
        auto MUL_573 = SUB_434 * COS_557;
        auto ADD_574 = MUL_570 + MUL_573;
        auto MUL_578 = SUB_434 * SIN_551;
        auto SUB_579 = MUL_575 - MUL_578;
        auto MUL_567 = ADD_427 * COS_557;
        auto MUL_561 = ADD_427 * SIN_551;
        auto MUL_559 = ADD_422 * COS_557;
        auto SUB_562 = MUL_561 - MUL_559;
        auto MUL_3355 = SUB_562 * ADD_574;
        auto MUL_565 = ADD_422 * SIN_551;
        auto ADD_568 = MUL_565 + MUL_567;
        auto MUL_3351 = SUB_579 * ADD_568;
        auto ADD_3383 = MUL_3355 + MUL_3351;
        auto MUL_3385 = ADD_3383 * 2.0;
        auto MUL_3409 = MUL_3385 * 0.09;
        auto ADD_3414 = SUB_546 + MUL_3409;
        auto MUL_3353 = SUB_579 * SUB_562;
        auto MUL_3356 = ADD_568 * ADD_574;
        auto SUB_3386 = MUL_3356 - MUL_3353;
        auto MUL_3388 = SUB_3386 * 2.0;
        auto MUL_3411 = MUL_3388 * 0.09;
        auto MUL_528 = SUB_434 * MUL_502;
        auto MUL_526 = ADD_422 * MUL_505;
        auto ADD_530 = MUL_526 + MUL_528;
        auto MUL_533 = ADD_530 * 2.0;
        auto SUB_536 = 0.093 - MUL_533;
        auto ADD_547 = ADD_406 + SUB_536;
        auto ADD_3415 = ADD_547 + MUL_3411;
        auto MUL_3352 = SUB_562 * SUB_562;
        auto MUL_3348 = ADD_568 * ADD_568;
        auto ADD_3389 = MUL_3348 + MUL_3352;
        auto MUL_3392 = ADD_3389 * 2.0;
        auto SUB_3395 = 1.0 - MUL_3392;
        auto MUL_3413 = SUB_3395 * 0.09;
        auto MUL_537 = SUB_438 * MUL_505;
        auto MUL_540 = ADD_427 * MUL_502;
        auto SUB_542 = MUL_540 - MUL_537;
        auto MUL_544 = SUB_542 * 2.0;
        auto ADD_548 = ADD_407 + MUL_544;
        auto ADD_3416 = ADD_548 + MUL_3413;
        auto MUL_3350 = SUB_579 * ADD_574;
        auto MUL_3354 = SUB_562 * ADD_568;
        auto SUB_3370 = MUL_3354 - MUL_3350;
        auto MUL_3372 = SUB_3370 * 2.0;
        auto MUL_3424 = MUL_3372 * 0.03;
        auto ADD_3435 = MUL_3424 + MUL_3409;
        auto ADD_3438 = SUB_546 + ADD_3435;
        auto MUL_3349 = ADD_574 * ADD_574;
        auto ADD_3373 = MUL_3349 + MUL_3352;
        auto MUL_3376 = ADD_3373 * 2.0;
        auto SUB_3379 = 1.0 - MUL_3376;
        auto MUL_3426 = SUB_3379 * 0.03;
        auto ADD_3436 = MUL_3426 + MUL_3411;
        auto ADD_3439 = ADD_547 + ADD_3436;
        auto ADD_3380 = MUL_3356 + MUL_3353;
        auto MUL_3382 = ADD_3380 * 2.0;
        auto MUL_3428 = MUL_3382 * 0.03;
        auto ADD_3437 = MUL_3428 + MUL_3413;
        auto ADD_3440 = ADD_548 + ADD_3437;
        auto SUB_3465 = MUL_3409 - MUL_3424;
        auto ADD_3468 = SUB_546 + SUB_3465;
        auto SUB_3466 = MUL_3411 - MUL_3426;
        auto ADD_3469 = ADD_547 + SUB_3466;
        auto SUB_3467 = MUL_3413 - MUL_3428;
        auto ADD_3470 = ADD_548 + SUB_3467;
        if (/*base_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (270, 348)
        if (/*shoulder_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        if (/*upper_arm_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        if (/*wrist_2_link*/ sphere_environment_in_collision(environment, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_environment_in_collision(environment, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        auto MUL_648 = SUB_562 * 0.09465;
        auto MUL_653 = ADD_574 * MUL_648;
        auto MUL_640 = ADD_568 * 0.09465;
        auto MUL_651 = SUB_579 * MUL_640;
        auto ADD_654 = MUL_651 + MUL_653;
        auto MUL_656 = ADD_654 * 2.0;
        auto ADD_678 = SUB_546 + MUL_656;
        auto INPUT_5 = q[5];
        auto DIV_682 = INPUT_5 * 0.5;
        auto SIN_683 = DIV_682.sin();
        auto COS_689 = DIV_682.cos();
        auto MUL_705 = SUB_579 * COS_689;
        auto MUL_695 = SUB_579 * SIN_683;
        auto MUL_703 = ADD_574 * COS_689;
        auto MUL_693 = ADD_574 * SIN_683;
        auto MUL_691 = SUB_562 * COS_689;
        auto SUB_694 = MUL_691 - MUL_693;
        auto MUL_701 = SUB_562 * SIN_683;
        auto ADD_704 = MUL_701 + MUL_703;
        auto MUL_697 = ADD_568 * COS_689;
        auto ADD_698 = MUL_695 + MUL_697;
        auto MUL_3506 = SUB_694 * ADD_698;
        auto MUL_707 = ADD_568 * SIN_683;
        auto SUB_708 = MUL_705 - MUL_707;
        auto MUL_3502 = SUB_708 * ADD_704;
        auto SUB_3522 = MUL_3506 - MUL_3502;
        auto MUL_3524 = SUB_3522 * 2.0;
        auto MUL_3576 = MUL_3524 * 0.06;
        auto ADD_3587 = ADD_678 + MUL_3576;
        auto MUL_3501 = ADD_704 * ADD_704;
        auto MUL_3504 = SUB_694 * SUB_694;
        auto ADD_3525 = MUL_3501 + MUL_3504;
        auto MUL_3528 = ADD_3525 * 2.0;
        auto SUB_3531 = 1.0 - MUL_3528;
        auto MUL_3578 = SUB_3531 * 0.06;
        auto MUL_659 = SUB_579 * MUL_648;
        auto MUL_662 = ADD_574 * MUL_640;
        auto SUB_663 = MUL_662 - MUL_659;
        auto MUL_665 = SUB_663 * 2.0;
        auto ADD_679 = ADD_547 + MUL_665;
        auto ADD_3588 = ADD_679 + MUL_3578;
        auto MUL_3505 = SUB_708 * SUB_694;
        auto MUL_3508 = ADD_698 * ADD_704;
        auto ADD_3532 = MUL_3508 + MUL_3505;
        auto MUL_3534 = ADD_3532 * 2.0;
        auto MUL_3580 = MUL_3534 * 0.06;
        auto MUL_668 = SUB_562 * MUL_648;
        auto MUL_670 = ADD_568 * MUL_640;
        auto ADD_671 = MUL_668 + MUL_670;
        auto MUL_674 = ADD_671 * 2.0;
        auto SUB_677 = 0.09465 - MUL_674;
        auto ADD_680 = ADD_548 + SUB_677;
        auto ADD_3589 = ADD_680 + MUL_3580;
        if (/*wrist_3_link*/ sphere_environment_in_collision(environment, ADD_3587, ADD_3588, ADD_3589, 0.04))
        {
            return false;
        }  // (348, 401)
        auto MUL_770 = SUB_708 * 0.7071068;
        auto MUL_776 = ADD_704 * 0.7071068;
        auto SUB_777 = MUL_770 - MUL_776;
        auto ADD_768 = MUL_770 + MUL_776;
        auto MUL_1064 = SUB_777 * 0.7073883;
        auto MUL_1054 = SUB_777 * 0.7068252;
        auto MUL_1061 = ADD_768 * 0.7073883;
        auto SUB_1062 = MUL_1061 - MUL_1054;
        auto MUL_1071 = ADD_768 * 0.7068252;
        auto ADD_1073 = MUL_1064 + MUL_1071;
        auto MUL_3690 = ADD_1073 * SUB_1062;
        auto MUL_756 = ADD_698 * 0.7071068;
        auto MUL_753 = SUB_694 * 0.7071068;
        auto SUB_757 = MUL_756 - MUL_753;
        auto ADD_747 = MUL_753 + MUL_756;
        auto MUL_1048 = SUB_757 * 0.7073883;
        auto MUL_1036 = SUB_757 * 0.7068252;
        auto MUL_1033 = ADD_747 * 0.7073883;
        auto SUB_1038 = MUL_1033 - MUL_1036;
        auto MUL_1045 = ADD_747 * 0.7068252;
        auto ADD_1049 = MUL_1045 + MUL_1048;
        auto MUL_3694 = SUB_1038 * ADD_1049;
        auto SUB_3710 = MUL_3694 - MUL_3690;
        auto MUL_3712 = SUB_3710 * 2.0;
        auto MUL_781 = ADD_704 * 0.0823;
        auto MUL_791 = SUB_708 * MUL_781;
        auto MUL_784 = SUB_694 * 0.0823;
        auto MUL_793 = ADD_698 * MUL_784;
        auto SUB_794 = MUL_793 - MUL_791;
        auto MUL_797 = SUB_794 * 2.0;
        auto ADD_820 = ADD_678 + MUL_797;
        auto MUL_1086 = ADD_768 * 0.035;
        auto MUL_1091 = ADD_768 * MUL_1086;
        auto MUL_1081 = SUB_757 * 0.035;
        auto MUL_1089 = SUB_757 * MUL_1081;
        auto ADD_1093 = MUL_1089 + MUL_1091;
        auto MUL_1096 = ADD_1093 * 2.0;
        auto SUB_1099 = 0.035 - MUL_1096;
        auto ADD_1118 = ADD_820 + SUB_1099;
        auto MUL_3771 = MUL_3712 * 0.02;
        auto SUB_3787 = ADD_1118 - MUL_3771;
        auto MUL_3689 = SUB_1062 * SUB_1062;
        auto MUL_3692 = SUB_1038 * SUB_1038;
        auto ADD_3713 = MUL_3689 + MUL_3692;
        auto MUL_3716 = ADD_3713 * 2.0;
        auto SUB_3719 = 1.0 - MUL_3716;
        auto MUL_3775 = SUB_3719 * 0.02;
        auto MUL_1101 = SUB_777 * MUL_1086;
        auto MUL_1102 = ADD_747 * MUL_1081;
        auto ADD_1104 = MUL_1101 + MUL_1102;
        auto MUL_1107 = ADD_1104 * 2.0;
        auto MUL_803 = ADD_704 * MUL_781;
        auto MUL_801 = SUB_694 * MUL_784;
        auto ADD_805 = MUL_801 + MUL_803;
        auto MUL_808 = ADD_805 * 2.0;
        auto SUB_811 = 0.0823 - MUL_808;
        auto ADD_821 = ADD_679 + SUB_811;
        auto ADD_1119 = ADD_821 + MUL_1107;
        auto SUB_3788 = ADD_1119 - MUL_3775;
        auto MUL_3693 = ADD_1073 * SUB_1038;
        auto MUL_3696 = ADD_1049 * SUB_1062;
        auto ADD_3720 = MUL_3696 + MUL_3693;
        auto MUL_3722 = ADD_3720 * 2.0;
        auto MUL_3779 = MUL_3722 * 0.02;
        auto MUL_1109 = SUB_777 * MUL_1081;
        auto MUL_1111 = ADD_747 * MUL_1086;
        auto SUB_1113 = MUL_1111 - MUL_1109;
        auto MUL_1116 = SUB_1113 * 2.0;
        auto MUL_812 = SUB_708 * MUL_784;
        auto MUL_814 = ADD_698 * MUL_781;
        auto ADD_816 = MUL_812 + MUL_814;
        auto MUL_818 = ADD_816 * 2.0;
        auto ADD_822 = ADD_680 + MUL_818;
        auto ADD_1120 = ADD_822 + MUL_1116;
        auto SUB_3789 = ADD_1120 - MUL_3779;
        if (/*fts_robotside*/ sphere_environment_in_collision(
            environment, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            return false;
        }  // (401, 476)
        auto MUL_1472 = ADD_1073 * 0.7073883;
        auto MUL_1440 = ADD_1073 * 0.7068252;
        auto MUL_1469 = SUB_1062 * 0.7073883;
        auto MUL_1457 = SUB_1062 * 0.7068252;
        auto MUL_1443 = SUB_1038 * 0.7073883;
        auto SUB_1444 = MUL_1443 - MUL_1440;
        auto MUL_1475 = SUB_1038 * 0.7068252;
        auto ADD_1477 = MUL_1472 + MUL_1475;
        auto MUL_1454 = ADD_1049 * 0.7073883;
        auto SUB_1459 = MUL_1454 - MUL_1457;
        auto MUL_3891 = ADD_1477 * SUB_1459;
        auto MUL_1466 = ADD_1049 * 0.7068252;
        auto ADD_1470 = MUL_1466 + MUL_1469;
        auto MUL_3895 = SUB_1444 * ADD_1470;
        auto ADD_3923 = MUL_3895 + MUL_3891;
        auto MUL_3925 = ADD_3923 * 2.0;
        auto MUL_3949 = MUL_3925 * 0.04;
        auto MUL_1346 = SUB_1038 * 0.0375;
        auto MUL_1351 = SUB_1062 * MUL_1346;
        auto MUL_1338 = ADD_1049 * 0.0375;
        auto MUL_1349 = ADD_1073 * MUL_1338;
        auto ADD_1352 = MUL_1349 + MUL_1351;
        auto MUL_1354 = ADD_1352 * 2.0;
        auto ADD_1376 = ADD_1118 + MUL_1354;
        auto MUL_1494 = SUB_1038 * 0.037;
        auto MUL_1501 = SUB_1062 * MUL_1494;
        auto MUL_1484 = ADD_1049 * 0.037;
        auto MUL_1498 = ADD_1073 * MUL_1484;
        auto ADD_1503 = MUL_1498 + MUL_1501;
        auto MUL_1506 = ADD_1503 * 2.0;
        auto SUB_1529 = ADD_1376 - MUL_1506;
        auto ADD_3954 = SUB_1529 + MUL_3949;
        auto MUL_3893 = ADD_1477 * SUB_1444;
        auto MUL_3896 = SUB_1459 * ADD_1470;
        auto SUB_3926 = MUL_3896 - MUL_3893;
        auto MUL_3928 = SUB_3926 * 2.0;
        auto MUL_3951 = MUL_3928 * 0.04;
        auto MUL_1510 = ADD_1073 * MUL_1494;
        auto MUL_1357 = ADD_1073 * MUL_1346;
        auto MUL_1512 = SUB_1062 * MUL_1484;
        auto SUB_1514 = MUL_1510 - MUL_1512;
        auto MUL_1516 = SUB_1514 * 2.0;
        auto MUL_1360 = SUB_1062 * MUL_1338;
        auto SUB_1361 = MUL_1360 - MUL_1357;
        auto MUL_1363 = SUB_1361 * 2.0;
        auto ADD_1377 = ADD_1119 + MUL_1363;
        auto ADD_1530 = ADD_1377 + MUL_1516;
        auto ADD_3955 = ADD_1530 + MUL_3951;
        auto MUL_3888 = SUB_1459 * SUB_1459;
        auto MUL_3892 = SUB_1444 * SUB_1444;
        auto ADD_3929 = MUL_3888 + MUL_3892;
        auto MUL_3932 = ADD_3929 * 2.0;
        auto SUB_3935 = 1.0 - MUL_3932;
        auto MUL_3953 = SUB_3935 * 0.04;
        auto MUL_1519 = SUB_1038 * MUL_1494;
        auto MUL_1366 = SUB_1038 * MUL_1346;
        auto MUL_1521 = ADD_1049 * MUL_1484;
        auto ADD_1523 = MUL_1519 + MUL_1521;
        auto MUL_1525 = ADD_1523 * 2.0;
        auto SUB_1528 = MUL_1525 - 0.037;
        auto MUL_1368 = ADD_1049 * MUL_1338;
        auto ADD_1369 = MUL_1366 + MUL_1368;
        auto MUL_1372 = ADD_1369 * 2.0;
        auto SUB_1375 = 0.0375 - MUL_1372;
        auto ADD_1378 = ADD_1120 + SUB_1375;
        auto ADD_1531 = ADD_1378 + SUB_1528;
        auto ADD_3956 = ADD_1531 + MUL_3953;
        auto MUL_3970 = MUL_3925 * 0.06;
        auto ADD_3975 = SUB_1529 + MUL_3970;
        auto MUL_3972 = MUL_3928 * 0.06;
        auto ADD_3976 = ADD_1530 + MUL_3972;
        auto MUL_3974 = SUB_3935 * 0.06;
        auto ADD_3977 = ADD_1531 + MUL_3974;
        auto MUL_3991 = MUL_3925 * 0.02;
        auto ADD_3996 = SUB_1529 + MUL_3991;
        auto MUL_3993 = MUL_3928 * 0.02;
        auto ADD_3997 = ADD_1530 + MUL_3993;
        auto MUL_3995 = SUB_3935 * 0.02;
        auto ADD_3998 = ADD_1531 + MUL_3995;
        if (/*base_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (476, 555)
        if (/*robotiq_85_base_link*/ sphere_environment_in_collision(
            environment, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_environment_in_collision(environment, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        if (/*shoulder_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        if (/*upper_arm_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        auto MUL_1620 = SUB_1459 * 0.062792;
        auto MUL_1633 = ADD_1477 * MUL_1620;
        auto MUL_1629 = SUB_1444 * 0.062792;
        auto MUL_1631 = ADD_1470 * 0.0306011;
        auto SUB_1632 = MUL_1629 - MUL_1631;
        auto MUL_1637 = ADD_1470 * SUB_1632;
        auto MUL_1626 = SUB_1459 * 0.0306011;
        auto MUL_1634 = SUB_1459 * MUL_1626;
        auto SUB_1636 = MUL_1633 - MUL_1634;
        auto ADD_1638 = SUB_1636 + MUL_1637;
        auto MUL_1640 = ADD_1638 * 2.0;
        auto ADD_1642 = MUL_1640 + 0.0306011;
        auto ADD_1667 = SUB_1529 + ADD_1642;
        auto MUL_1644 = ADD_1477 * SUB_1632;
        auto MUL_1649 = ADD_1470 * MUL_1620;
        auto MUL_1646 = SUB_1444 * MUL_1626;
        auto SUB_1648 = MUL_1646 - MUL_1644;
        auto ADD_1650 = SUB_1648 + MUL_1649;
        auto MUL_1652 = ADD_1650 * 2.0;
        auto ADD_1668 = ADD_1530 + MUL_1652;
        auto MUL_1654 = ADD_1477 * MUL_1626;
        auto MUL_1659 = SUB_1459 * MUL_1620;
        auto MUL_1656 = SUB_1444 * SUB_1632;
        auto ADD_1657 = MUL_1654 + MUL_1656;
        auto ADD_1660 = ADD_1657 + MUL_1659;
        auto MUL_1663 = ADD_1660 * 2.0;
        auto SUB_1666 = 0.062792 - MUL_1663;
        auto ADD_1669 = ADD_1531 + SUB_1666;
        if (/*base_link vs. robotiq_85_left_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }  // (555, 583)
        if (/*robotiq_85_left_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            return false;
        }  // (583, 583)
        if (/*shoulder_link vs. robotiq_85_left_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }  // (583, 583)
        if (/*upper_arm_link vs. robotiq_85_left_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }  // (583, 583)
        auto MUL_4128 = ADD_3923 * 2.0;
        auto MUL_4152 = MUL_4128 * 0.02;
        auto MUL_1773 = ADD_1470 * 0.031691;
        auto MUL_1766 = SUB_1459 * 0.031691;
        auto MUL_1778 = SUB_1459 * MUL_1766;
        auto MUL_1759 = SUB_1459 * 0.001934;
        auto MUL_1776 = ADD_1477 * MUL_1759;
        auto ADD_1780 = MUL_1776 + MUL_1778;
        auto MUL_1770 = SUB_1444 * 0.001934;
        auto ADD_1774 = MUL_1770 + MUL_1773;
        auto MUL_1782 = ADD_1470 * ADD_1774;
        auto ADD_1784 = ADD_1780 + MUL_1782;
        auto MUL_1787 = ADD_1784 * 2.0;
        auto SUB_1790 = 0.031691 - MUL_1787;
        auto ADD_1815 = ADD_1667 + SUB_1790;
        auto ADD_4157 = ADD_1815 + MUL_4152;
        auto MUL_4131 = SUB_3926 * 2.0;
        auto MUL_4154 = MUL_4131 * 0.02;
        auto MUL_1792 = ADD_1477 * ADD_1774;
        auto MUL_1796 = ADD_1470 * MUL_1759;
        auto MUL_1793 = SUB_1444 * MUL_1766;
        auto ADD_1795 = MUL_1792 + MUL_1793;
        auto SUB_1798 = ADD_1795 - MUL_1796;
        auto MUL_1800 = SUB_1798 * 2.0;
        auto ADD_1816 = ADD_1668 + MUL_1800;
        auto ADD_4158 = ADD_1816 + MUL_4154;
        auto MUL_4135 = ADD_3929 * 2.0;
        auto SUB_4138 = 1.0 - MUL_4135;
        auto MUL_4156 = SUB_4138 * 0.02;
        auto MUL_1802 = ADD_1477 * MUL_1766;
        auto MUL_1807 = SUB_1459 * MUL_1759;
        auto MUL_1804 = SUB_1444 * ADD_1774;
        auto SUB_1806 = MUL_1804 - MUL_1802;
        auto ADD_1809 = SUB_1806 + MUL_1807;
        auto MUL_1811 = ADD_1809 * 2.0;
        auto SUB_1814 = MUL_1811 - 0.001934;
        auto ADD_1817 = ADD_1669 + SUB_1814;
        auto ADD_4159 = ADD_1817 + MUL_4156;
        auto MUL_4173 = MUL_4128 * 0.04;
        auto ADD_4178 = ADD_1815 + MUL_4173;
        auto MUL_4175 = MUL_4131 * 0.04;
        auto ADD_4179 = ADD_1816 + MUL_4175;
        auto MUL_4177 = SUB_4138 * 0.04;
        auto ADD_4180 = ADD_1817 + MUL_4177;
        if (/*base_link vs. robotiq_85_left_finger_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }  // (583, 627)
        if (/*robotiq_85_left_finger_link*/ sphere_environment_in_collision(
            environment, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_environment_in_collision(environment, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }  // (627, 627)
        if (/*shoulder_link vs. robotiq_85_left_finger_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }  // (627, 627)
        if (/*upper_arm_link vs. robotiq_85_left_finger_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }  // (627, 627)
        auto MUL_4265 = ADD_3923 * 2.0;
        auto MUL_4313 = MUL_4265 * 0.02;
        auto MUL_4229 = ADD_1470 * ADD_1470;
        auto ADD_4237 = MUL_3888 + MUL_4229;
        auto MUL_4240 = ADD_4237 * 2.0;
        auto SUB_4243 = 1.0 - MUL_4240;
        auto MUL_4301 = SUB_4243 * 0.02;
        auto ADD_4318 = MUL_4301 + MUL_4313;
        auto MUL_1906 = SUB_1459 * 0.0693075;
        auto MUL_1919 = ADD_1477 * MUL_1906;
        auto MUL_1915 = SUB_1444 * 0.0693075;
        auto MUL_1917 = ADD_1470 * 0.0127;
        auto SUB_1918 = MUL_1915 - MUL_1917;
        auto MUL_1923 = ADD_1470 * SUB_1918;
        auto MUL_1912 = SUB_1459 * 0.0127;
        auto MUL_1920 = SUB_1459 * MUL_1912;
        auto SUB_1922 = MUL_1919 - MUL_1920;
        auto ADD_1924 = SUB_1922 + MUL_1923;
        auto MUL_1926 = ADD_1924 * 2.0;
        auto ADD_1928 = MUL_1926 + 0.0127;
        auto ADD_1953 = SUB_1529 + ADD_1928;
        auto ADD_4321 = ADD_1953 + ADD_4318;
        auto MUL_4268 = SUB_3926 * 2.0;
        auto MUL_4315 = MUL_4268 * 0.02;
        auto MUL_1930 = ADD_1477 * SUB_1918;
        auto MUL_4230 = ADD_1477 * ADD_1470;
        auto MUL_1935 = ADD_1470 * MUL_1906;
        auto MUL_1932 = SUB_1444 * MUL_1912;
        auto SUB_1934 = MUL_1932 - MUL_1930;
        auto ADD_1936 = SUB_1934 + MUL_1935;
        auto MUL_1938 = ADD_1936 * 2.0;
        auto ADD_1954 = ADD_1530 + MUL_1938;
        auto MUL_4234 = SUB_1444 * SUB_1459;
        auto ADD_4244 = MUL_4234 + MUL_4230;
        auto MUL_4246 = ADD_4244 * 2.0;
        auto MUL_4303 = MUL_4246 * 0.02;
        auto ADD_4319 = MUL_4303 + MUL_4315;
        auto ADD_4322 = ADD_1954 + ADD_4319;
        auto SUB_4247 = MUL_3895 - MUL_3891;
        auto MUL_4272 = ADD_3929 * 2.0;
        auto SUB_4275 = 1.0 - MUL_4272;
        auto MUL_4317 = SUB_4275 * 0.02;
        auto MUL_4249 = SUB_4247 * 2.0;
        auto MUL_4305 = MUL_4249 * 0.02;
        auto ADD_4320 = MUL_4305 + MUL_4317;
        auto MUL_1940 = ADD_1477 * MUL_1912;
        auto MUL_1945 = SUB_1459 * MUL_1906;
        auto MUL_1942 = SUB_1444 * SUB_1918;
        auto ADD_1943 = MUL_1940 + MUL_1942;
        auto ADD_1946 = ADD_1943 + MUL_1945;
        auto MUL_1949 = ADD_1946 * 2.0;
        auto SUB_1952 = 0.0693075 - MUL_1949;
        auto ADD_1955 = ADD_1531 + SUB_1952;
        auto ADD_4323 = ADD_1955 + ADD_4320;
        if (/*base_link vs. robotiq_85_left_inner_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }  // (627, 681)
        if (/*robotiq_85_left_inner_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            return false;
        }  // (681, 681)
        if (/*shoulder_link vs. robotiq_85_left_inner_knuckle_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(0.0, 0.0, 1.003559, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }  // (681, 681)
        if (/*upper_arm_link vs. robotiq_85_left_inner_knuckle_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }  // (681, 681)
        auto MUL_4363 = ADD_3923 * 2.0;
        auto MUL_2044 = SUB_1459 * 0.045497;
        auto MUL_2057 = ADD_1477 * MUL_2044;
        auto MUL_2053 = SUB_1444 * 0.045497;
        auto MUL_2055 = ADD_1470 * 0.0345853;
        auto SUB_2056 = MUL_2053 - MUL_2055;
        auto MUL_2061 = ADD_1470 * SUB_2056;
        auto MUL_2050 = SUB_1459 * 0.0345853;
        auto MUL_2058 = SUB_1459 * MUL_2050;
        auto SUB_2060 = MUL_2057 - MUL_2058;
        auto ADD_2062 = SUB_2060 + MUL_2061;
        auto MUL_2064 = ADD_2062 * 2.0;
        auto ADD_2066 = MUL_2064 + 0.0345853;
        auto ADD_2091 = ADD_1953 + ADD_2066;
        auto MUL_4387 = MUL_4363 * 0.013;
        auto ADD_4392 = ADD_2091 + MUL_4387;
        auto MUL_4366 = SUB_3926 * 2.0;
        auto MUL_4389 = MUL_4366 * 0.013;
        auto MUL_2068 = ADD_1477 * SUB_2056;
        auto MUL_2073 = ADD_1470 * MUL_2044;
        auto MUL_2070 = SUB_1444 * MUL_2050;
        auto SUB_2072 = MUL_2070 - MUL_2068;
        auto ADD_2074 = SUB_2072 + MUL_2073;
        auto MUL_2076 = ADD_2074 * 2.0;
        auto ADD_2092 = ADD_1954 + MUL_2076;
        auto ADD_4393 = ADD_2092 + MUL_4389;
        auto MUL_4370 = ADD_3929 * 2.0;
        auto SUB_4373 = 1.0 - MUL_4370;
        auto MUL_4391 = SUB_4373 * 0.013;
        auto MUL_2078 = ADD_1477 * MUL_2050;
        auto MUL_2083 = SUB_1459 * MUL_2044;
        auto MUL_2080 = SUB_1444 * SUB_2056;
        auto ADD_2081 = MUL_2078 + MUL_2080;
        auto ADD_2084 = ADD_2081 + MUL_2083;
        auto MUL_2087 = ADD_2084 * 2.0;
        auto SUB_2090 = 0.045497 - MUL_2087;
        auto ADD_2093 = ADD_1955 + SUB_2090;
        auto ADD_4394 = ADD_2093 + MUL_4391;
        auto MUL_4408 = MUL_4363 * 0.025;
        auto ADD_4413 = ADD_2091 + MUL_4408;
        auto MUL_4410 = MUL_4366 * 0.025;
        auto ADD_4414 = ADD_2092 + MUL_4410;
        auto MUL_4412 = SUB_4373 * 0.025;
        auto ADD_4415 = ADD_2093 + MUL_4412;
        if (/*base_link vs. robotiq_85_left_finger_tip_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }  // (681, 725)
        if (/*robotiq_85_left_finger_tip_link*/ sphere_environment_in_collision(
            environment, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_environment_in_collision(environment, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }  // (725, 725)
        if (/*shoulder_link vs. robotiq_85_left_finger_tip_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(0.0, 0.0, 1.003559, 0.08, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }  // (725, 725)
        if (/*upper_arm_link vs. robotiq_85_left_finger_tip_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }  // (725, 725)
        auto ADD_2202 = MUL_1919 + MUL_1920;
        auto ADD_2199 = MUL_1915 + MUL_1917;
        auto MUL_4441 = ADD_1477 * ADD_1477;
        auto ADD_4453 = MUL_3892 + MUL_4441;
        auto MUL_4456 = ADD_4453 * 2.0;
        auto SUB_4459 = 1.0 - MUL_4456;
        auto MUL_4522 = SUB_4459 * 0.02;
        auto MUL_2203 = ADD_1470 * ADD_2199;
        auto ADD_2204 = ADD_2202 + MUL_2203;
        auto MUL_2206 = ADD_2204 * 2.0;
        auto SUB_2209 = MUL_2206 - 0.0127;
        auto ADD_2230 = SUB_1529 + SUB_2209;
        auto MUL_4444 = ADD_1470 * SUB_1444;
        auto MUL_4450 = SUB_1459 * ADD_1477;
        auto ADD_4483 = MUL_4450 + MUL_4444;
        auto MUL_4485 = ADD_4483 * 2.0;
        auto MUL_4535 = MUL_4485 * 0.02;
        auto ADD_4540 = MUL_4522 + MUL_4535;
        auto ADD_4543 = ADD_2230 + ADD_4540;
        auto MUL_2211 = ADD_1477 * ADD_2199;
        auto ADD_2214 = MUL_2211 + MUL_1932;
        auto SUB_2217 = MUL_1935 - ADD_2214;
        auto MUL_2219 = SUB_2217 * 2.0;
        auto ADD_2231 = ADD_1530 + MUL_2219;
        auto MUL_4442 = ADD_1470 * ADD_1477;
        auto MUL_4446 = ADD_1470 * SUB_1459;
        auto MUL_4448 = SUB_1459 * SUB_1444;
        auto ADD_4460 = MUL_4448 + MUL_4442;
        auto MUL_4463 = ADD_4460 * 2.0;
        auto MUL_4524 = MUL_4463 * 0.02;
        auto MUL_4451 = SUB_1444 * ADD_1477;
        auto SUB_4486 = MUL_4446 - MUL_4451;
        auto MUL_4488 = SUB_4486 * 2.0;
        auto MUL_4537 = MUL_4488 * 0.02;
        auto SUB_4541 = MUL_4537 - MUL_4524;
        auto ADD_4544 = ADD_2231 + SUB_4541;
        auto SUB_4465 = MUL_4450 - MUL_4444;
        auto ADD_4489 = MUL_3892 + MUL_3888;
        auto MUL_4492 = ADD_4489 * 2.0;
        auto SUB_4495 = 1.0 - MUL_4492;
        auto MUL_4539 = SUB_4495 * 0.02;
        auto MUL_4467 = SUB_4465 * 2.0;
        auto MUL_4527 = MUL_4467 * 0.02;
        auto ADD_4542 = MUL_4527 + MUL_4539;
        auto MUL_2222 = SUB_1444 * ADD_2199;
        auto SUB_2223 = MUL_1940 - MUL_2222;
        auto SUB_2225 = SUB_2223 - MUL_1945;
        auto MUL_2227 = SUB_2225 * 2.0;
        auto ADD_2229 = MUL_2227 + 0.0693075;
        auto ADD_2232 = ADD_1531 + ADD_2229;
        auto ADD_4545 = ADD_2232 + ADD_4542;
        if (/*base_link vs. robotiq_85_right_inner_knuckle_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(0.0, 0.0, 0.9144, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }  // (725, 776)
        if (/*robotiq_85_right_inner_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            return false;
        }  // (776, 776)
        if (/*shoulder_link vs. robotiq_85_right_inner_knuckle_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(0.0, 0.0, 1.003559, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }  // (776, 776)
        if (/*upper_arm_link vs. robotiq_85_right_inner_knuckle_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }  // (776, 776)
        auto MUL_4593 = ADD_4483 * 2.0;
        auto MUL_4617 = MUL_4593 * 0.013;
        auto MUL_2330 = SUB_1459 * 0.0458574;
        auto MUL_2320 = SUB_1444 * 0.0458574;
        auto MUL_2334 = ADD_1470 * MUL_2320;
        auto MUL_2332 = ADD_1477 * 0.034106;
        auto SUB_2333 = MUL_2330 - MUL_2332;
        auto MUL_2338 = ADD_1477 * SUB_2333;
        auto MUL_2327 = SUB_1444 * 0.034106;
        auto MUL_2335 = SUB_1444 * MUL_2327;
        auto SUB_2337 = MUL_2334 - MUL_2335;
        auto ADD_2339 = SUB_2337 + MUL_2338;
        auto MUL_2341 = ADD_2339 * 2.0;
        auto ADD_2343 = MUL_2341 + 0.034106;
        auto ADD_2366 = ADD_2230 + ADD_2343;
        auto ADD_4622 = ADD_2366 + MUL_4617;
        auto MUL_4596 = SUB_4486 * 2.0;
        auto MUL_4619 = MUL_4596 * 0.013;
        auto MUL_2347 = ADD_1477 * MUL_2320;
        auto MUL_2344 = ADD_1470 * SUB_2333;
        auto MUL_2345 = SUB_1459 * MUL_2327;
        auto SUB_2346 = MUL_2344 - MUL_2345;
        auto SUB_2349 = SUB_2346 - MUL_2347;
        auto MUL_2351 = SUB_2349 * 2.0;
        auto ADD_2367 = ADD_2231 + MUL_2351;
        auto ADD_4623 = ADD_2367 + MUL_4619;
        auto MUL_4600 = ADD_4489 * 2.0;
        auto SUB_4603 = 1.0 - MUL_4600;
        auto MUL_4621 = SUB_4603 * 0.013;
        auto MUL_2353 = ADD_1470 * MUL_2327;
        auto MUL_2355 = SUB_1459 * SUB_2333;
        auto ADD_2356 = MUL_2353 + MUL_2355;
        auto MUL_2358 = SUB_1444 * MUL_2320;
        auto ADD_2359 = ADD_2356 + MUL_2358;
        auto MUL_2362 = ADD_2359 * 2.0;
        auto SUB_2365 = 0.0458574 - MUL_2362;
        auto ADD_2368 = ADD_2232 + SUB_2365;
        auto ADD_4624 = ADD_2368 + MUL_4621;
        auto MUL_4638 = MUL_4593 * 0.025;
        auto ADD_4643 = ADD_2366 + MUL_4638;
        auto MUL_4640 = MUL_4596 * 0.025;
        auto ADD_4644 = ADD_2367 + MUL_4640;
        auto MUL_4642 = SUB_4603 * 0.025;
        auto ADD_4645 = ADD_2368 + MUL_4642;
        if (/*base_link vs. robotiq_85_right_finger_tip_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }  // (776, 820)
        if (/*robotiq_85_right_finger_tip_link*/ sphere_environment_in_collision(
            environment, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_environment_in_collision(environment, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }  // (820, 820)
        if (/*shoulder_link vs. robotiq_85_right_finger_tip_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(0.0, 0.0, 1.003559, 0.08, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }  // (820, 820)
        if (/*upper_arm_link vs. robotiq_85_right_finger_tip_link*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }  // (820, 820)
        auto ADD_2476 = MUL_1633 + MUL_1634;
        auto ADD_2473 = MUL_1629 + MUL_1631;
        auto MUL_2477 = ADD_1470 * ADD_2473;
        auto ADD_2478 = ADD_2476 + MUL_2477;
        auto MUL_2480 = ADD_2478 * 2.0;
        auto SUB_2483 = MUL_2480 - 0.0306011;
        auto ADD_2504 = SUB_1529 + SUB_2483;
        auto MUL_2485 = ADD_1477 * ADD_2473;
        auto ADD_2488 = MUL_2485 + MUL_1646;
        auto SUB_2491 = MUL_1649 - ADD_2488;
        auto MUL_2493 = SUB_2491 * 2.0;
        auto ADD_2505 = ADD_1530 + MUL_2493;
        auto MUL_2496 = SUB_1444 * ADD_2473;
        auto SUB_2497 = MUL_1654 - MUL_2496;
        auto SUB_2499 = SUB_2497 - MUL_1659;
        auto MUL_2501 = SUB_2499 * 2.0;
        auto ADD_2503 = MUL_2501 + 0.062792;
        auto ADD_2506 = ADD_1531 + ADD_2503;
        if (/*base_link vs. robotiq_85_right_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }  // (820, 838)
        if (/*robotiq_85_right_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            return false;
        }  // (838, 838)
        if (/*shoulder_link vs. robotiq_85_right_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }  // (838, 838)
        if (/*upper_arm_link vs. robotiq_85_right_knuckle_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }  // (838, 838)
        auto MUL_4809 = ADD_4483 * 2.0;
        auto MUL_4833 = MUL_4809 * 0.02;
        auto MUL_2608 = ADD_1477 * 0.0317096;
        auto MUL_2601 = SUB_1444 * 0.0317096;
        auto MUL_2613 = SUB_1444 * MUL_2601;
        auto MUL_2605 = SUB_1459 * 0.0016014;
        auto ADD_2609 = MUL_2605 + MUL_2608;
        auto MUL_2617 = ADD_1477 * ADD_2609;
        auto MUL_2595 = SUB_1444 * 0.0016014;
        auto MUL_2611 = ADD_1470 * MUL_2595;
        auto ADD_2615 = MUL_2611 + MUL_2613;
        auto ADD_2619 = ADD_2615 + MUL_2617;
        auto MUL_2622 = ADD_2619 * 2.0;
        auto SUB_2625 = 0.0317096 - MUL_2622;
        auto ADD_2649 = ADD_2504 + SUB_2625;
        auto ADD_4838 = ADD_2649 + MUL_4833;
        auto MUL_4812 = SUB_4486 * 2.0;
        auto MUL_4835 = MUL_4812 * 0.02;
        auto MUL_2631 = ADD_1477 * MUL_2595;
        auto MUL_2626 = ADD_1470 * ADD_2609;
        auto MUL_2628 = SUB_1459 * MUL_2601;
        auto ADD_2629 = MUL_2626 + MUL_2628;
        auto SUB_2632 = MUL_2631 - ADD_2629;
        auto MUL_2634 = SUB_2632 * 2.0;
        auto ADD_2650 = ADD_2505 + MUL_2634;
        auto ADD_4839 = ADD_2650 + MUL_4835;
        auto MUL_4816 = ADD_4489 * 2.0;
        auto SUB_4819 = 1.0 - MUL_4816;
        auto MUL_4837 = SUB_4819 * 0.02;
        auto MUL_2636 = ADD_1470 * MUL_2601;
        auto MUL_2638 = SUB_1459 * ADD_2609;
        auto SUB_2640 = MUL_2638 - MUL_2636;
        auto MUL_2641 = SUB_1444 * MUL_2595;
        auto ADD_2643 = SUB_2640 + MUL_2641;
        auto MUL_2645 = ADD_2643 * 2.0;
        auto SUB_2648 = MUL_2645 - 0.0016014;
        auto ADD_2651 = ADD_2506 + SUB_2648;
        auto ADD_4840 = ADD_2651 + MUL_4837;
        auto MUL_4854 = MUL_4809 * 0.04;
        auto ADD_4859 = ADD_2649 + MUL_4854;
        auto MUL_4856 = MUL_4812 * 0.04;
        auto ADD_4860 = ADD_2650 + MUL_4856;
        auto MUL_4858 = SUB_4819 * 0.04;
        auto ADD_4861 = ADD_2651 + MUL_4858;
        if (/*robotiq_85_right_finger_link*/ sphere_environment_in_collision(
            environment, ADD_4838, ADD_4839, ADD_4840, 0.035))
        {
            if (sphere_environment_in_collision(environment, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
        }  // (838, 882)

        return true;
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk_attachment(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        if (/*base_link*/ sphere_environment_in_collision(environment, 0.0, 0.0, 0.9144, 0.08))
        {
            return false;
        }  // (0, 0)
        if (/*shoulder_link*/ sphere_environment_in_collision(environment, 0.0, 0.0, 1.003559, 0.08))
        {
            return false;
        }  // (0, 0)
        auto INPUT_1 = q[1];
        auto DIV_123 = INPUT_1 * 0.5;
        auto SIN_124 = DIV_123.sin();
        auto COS_130 = DIV_123.cos();
        auto INPUT_0 = q[0];
        auto DIV_7 = INPUT_0 * 0.5;
        auto SIN_8 = DIV_7.sin();
        auto COS_14 = DIV_7.cos();
        auto MUL_41 = COS_14 * 0.7073883;
        auto MUL_38 = COS_14 * 0.7068252;
        auto MUL_32 = SIN_8 * 0.7073883;
        auto ADD_39 = MUL_32 + MUL_38;
        auto MUL_56 = ADD_39 * 0.7071068;
        auto MUL_85 = ADD_39 * 0.13585;
        auto MUL_47 = SIN_8 * 0.7068252;
        auto SUB_48 = MUL_41 - MUL_47;
        auto MUL_95 = SUB_48 * MUL_85;
        auto MUL_59 = SUB_48 * 0.7071068;
        auto MUL_100 = MUL_95 * 2.0;
        auto MUL_140 = MUL_59 * COS_130;
        auto MUL_138 = MUL_59 * SIN_124;
        auto SUB_152 = MUL_140 - MUL_138;
        auto ADD_141 = MUL_138 + MUL_140;
        auto MUL_2799 = SUB_152 * ADD_141;
        auto MUL_147 = MUL_56 * COS_130;
        auto MUL_144 = MUL_56 * SIN_124;
        auto SUB_148 = MUL_147 - MUL_144;
        auto ADD_136 = MUL_147 + MUL_144;
        auto MUL_2805 = ADD_136 * SUB_148;
        auto SUB_2838 = MUL_2799 - MUL_2805;
        auto MUL_2840 = SUB_2838 * 2.0;
        auto MUL_2864 = MUL_2840 * 0.21;
        auto SUB_2869 = MUL_2864 - MUL_100;
        auto MUL_106 = ADD_39 * MUL_85;
        auto MUL_2801 = SUB_152 * ADD_136;
        auto MUL_2807 = ADD_141 * SUB_148;
        auto ADD_2841 = MUL_2807 + MUL_2801;
        auto MUL_2843 = ADD_2841 * 2.0;
        auto MUL_2866 = MUL_2843 * 0.21;
        auto MUL_109 = MUL_106 * 2.0;
        auto SUB_112 = 0.13585 - MUL_109;
        auto ADD_2870 = SUB_112 + MUL_2866;
        auto MUL_2796 = ADD_141 * ADD_141;
        auto MUL_2800 = ADD_136 * ADD_136;
        auto ADD_2844 = MUL_2796 + MUL_2800;
        auto MUL_2847 = ADD_2844 * 2.0;
        auto SUB_2850 = 1.0 - MUL_2847;
        auto MUL_2868 = SUB_2850 * 0.21;
        auto ADD_2871 = 1.003559 + MUL_2868;
        auto MUL_2885 = MUL_2840 * 0.105;
        auto SUB_2890 = MUL_2885 - MUL_100;
        auto MUL_2887 = MUL_2843 * 0.105;
        auto ADD_2891 = SUB_112 + MUL_2887;
        auto MUL_2889 = SUB_2850 * 0.105;
        auto ADD_2892 = 1.003559 + MUL_2889;
        auto MUL_2927 = MUL_2840 * 0.315;
        auto SUB_2932 = MUL_2927 - MUL_100;
        auto MUL_2929 = MUL_2843 * 0.315;
        auto ADD_2933 = SUB_112 + MUL_2929;
        auto MUL_2931 = SUB_2850 * 0.315;
        auto ADD_2934 = 1.003559 + MUL_2931;
        auto MUL_2948 = MUL_2840 * 0.42;
        auto SUB_2953 = MUL_2948 - MUL_100;
        auto MUL_2950 = MUL_2843 * 0.42;
        auto ADD_2954 = SUB_112 + MUL_2950;
        auto MUL_2952 = SUB_2850 * 0.42;
        auto ADD_2955 = 1.003559 + MUL_2952;
        auto NEGATE_101 = -MUL_100;
        if (/*base_link vs. upper_arm_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, SUB_2869, ADD_2870, ADD_2871, 0.29))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2890, ADD_2891, ADD_2892, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2869, ADD_2870, ADD_2871, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2932, ADD_2933, ADD_2934, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_2953, ADD_2954, ADD_2955, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, NEGATE_101, SUB_112, 1.003559, 0.08))
            {
                return false;
            }
        }  // (0, 68)
        if (/*upper_arm_link*/ sphere_environment_in_collision(
            environment, SUB_2869, ADD_2870, ADD_2871, 0.29))
        {
            if (sphere_environment_in_collision(environment, SUB_2890, ADD_2891, ADD_2892, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2869, ADD_2870, ADD_2871, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2932, ADD_2933, ADD_2934, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_2953, ADD_2954, ADD_2955, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, NEGATE_101, SUB_112, 1.003559, 0.08))
            {
                return false;
            }
        }  // (68, 68)
        auto MUL_214 = ADD_141 * 0.425;
        auto MUL_226 = ADD_136 * 0.425;
        auto MUL_233 = SUB_148 * MUL_226;
        auto MUL_217 = SUB_148 * 0.1197;
        auto ADD_219 = MUL_214 + MUL_217;
        auto MUL_230 = SUB_152 * ADD_219;
        auto MUL_222 = ADD_136 * 0.1197;
        auto MUL_231 = ADD_141 * MUL_222;
        auto ADD_232 = MUL_230 + MUL_231;
        auto SUB_235 = ADD_232 - MUL_233;
        auto MUL_237 = SUB_235 * 2.0;
        auto SUB_260 = MUL_237 - MUL_100;
        auto INPUT_2 = q[2];
        auto DIV_264 = INPUT_2 * 0.5;
        auto SIN_265 = DIV_264.sin();
        auto COS_271 = DIV_264.cos();
        auto MUL_290 = SUB_152 * COS_271;
        auto MUL_279 = SUB_152 * SIN_265;
        auto MUL_288 = SUB_148 * COS_271;
        auto MUL_276 = SUB_148 * SIN_265;
        auto MUL_281 = ADD_141 * COS_271;
        auto ADD_282 = MUL_279 + MUL_281;
        auto MUL_2986 = ADD_282 * ADD_282;
        auto MUL_292 = ADD_141 * SIN_265;
        auto SUB_293 = MUL_290 - MUL_292;
        auto MUL_2989 = SUB_293 * ADD_282;
        auto MUL_273 = ADD_136 * COS_271;
        auto ADD_277 = MUL_273 + MUL_276;
        auto MUL_2993 = ADD_277 * ADD_282;
        auto MUL_285 = ADD_136 * SIN_265;
        auto SUB_289 = MUL_288 - MUL_285;
        auto MUL_2988 = SUB_293 * SUB_289;
        auto ADD_3013 = MUL_2993 + MUL_2988;
        auto MUL_3016 = ADD_3013 * 2.0;
        auto MUL_3049 = MUL_3016 * 0.003;
        auto MUL_2987 = SUB_289 * SUB_289;
        auto ADD_2998 = MUL_2986 + MUL_2987;
        auto MUL_3001 = ADD_2998 * 2.0;
        auto SUB_3004 = 1.0 - MUL_3001;
        auto MUL_3042 = SUB_3004 * 0.002;
        auto SUB_3061 = MUL_3042 - MUL_3049;
        auto MUL_2995 = ADD_277 * SUB_289;
        auto SUB_3028 = MUL_2989 - MUL_2995;
        auto MUL_3030 = SUB_3028 * 2.0;
        auto MUL_3056 = MUL_3030 * 0.185;
        auto ADD_3064 = SUB_3061 + MUL_3056;
        auto ADD_3067 = SUB_260 + ADD_3064;
        auto SUB_3005 = MUL_2988 - MUL_2993;
        auto MUL_3007 = SUB_3005 * 2.0;
        auto MUL_3044 = MUL_3007 * 0.002;
        auto MUL_2991 = SUB_293 * ADD_277;
        auto MUL_2997 = ADD_282 * SUB_289;
        auto ADD_3031 = MUL_2997 + MUL_2991;
        auto MUL_3033 = ADD_3031 * 2.0;
        auto MUL_3058 = MUL_3033 * 0.185;
        auto MUL_2990 = ADD_277 * ADD_277;
        auto ADD_3018 = MUL_2987 + MUL_2990;
        auto MUL_3021 = ADD_3018 * 2.0;
        auto SUB_3024 = 1.0 - MUL_3021;
        auto MUL_3052 = SUB_3024 * 0.003;
        auto ADD_3062 = MUL_3044 + MUL_3052;
        auto ADD_3065 = ADD_3062 + MUL_3058;
        auto MUL_240 = SUB_152 * MUL_226;
        auto MUL_244 = SUB_148 * ADD_219;
        auto MUL_241 = ADD_136 * MUL_222;
        auto ADD_243 = MUL_240 + MUL_241;
        auto ADD_245 = ADD_243 + MUL_244;
        auto MUL_247 = ADD_245 * 2.0;
        auto SUB_250 = MUL_247 - 0.1197;
        auto ADD_261 = SUB_112 + SUB_250;
        auto ADD_3068 = ADD_261 + ADD_3065;
        auto SUB_3025 = MUL_2997 - MUL_2991;
        auto ADD_3008 = MUL_2995 + MUL_2989;
        auto ADD_3034 = MUL_2986 + MUL_2990;
        auto MUL_3037 = ADD_3034 * 2.0;
        auto SUB_3040 = 1.0 - MUL_3037;
        auto MUL_3060 = SUB_3040 * 0.185;
        auto MUL_3027 = SUB_3025 * 2.0;
        auto MUL_3054 = MUL_3027 * 0.003;
        auto MUL_3011 = ADD_3008 * 2.0;
        auto MUL_3046 = MUL_3011 * 0.002;
        auto SUB_3063 = MUL_3054 - MUL_3046;
        auto ADD_3066 = SUB_3063 + MUL_3060;
        auto MUL_251 = SUB_152 * MUL_222;
        auto MUL_254 = ADD_141 * ADD_219;
        auto MUL_252 = ADD_136 * MUL_226;
        auto SUB_253 = MUL_251 - MUL_252;
        auto SUB_255 = SUB_253 - MUL_254;
        auto MUL_257 = SUB_255 * 2.0;
        auto ADD_259 = MUL_257 + 0.425;
        auto ADD_262 = 1.003559 + ADD_259;
        auto ADD_3069 = ADD_262 + ADD_3066;
        auto MUL_3101 = MUL_3030 * 0.0975;
        auto ADD_3106 = SUB_260 + MUL_3101;
        auto MUL_3103 = MUL_3033 * 0.0975;
        auto ADD_3107 = ADD_261 + MUL_3103;
        auto MUL_3105 = SUB_3040 * 0.0975;
        auto ADD_3108 = ADD_262 + MUL_3105;
        auto MUL_3122 = MUL_3030 * 0.195;
        auto ADD_3127 = SUB_260 + MUL_3122;
        auto MUL_3124 = MUL_3033 * 0.195;
        auto ADD_3128 = ADD_261 + MUL_3124;
        auto MUL_3126 = SUB_3040 * 0.195;
        auto ADD_3129 = ADD_262 + MUL_3126;
        auto MUL_3143 = MUL_3030 * 0.2925;
        auto ADD_3148 = SUB_260 + MUL_3143;
        auto MUL_3145 = MUL_3033 * 0.2925;
        auto ADD_3149 = ADD_261 + MUL_3145;
        auto MUL_3147 = SUB_3040 * 0.2925;
        auto ADD_3150 = ADD_262 + MUL_3147;
        auto MUL_3164 = MUL_3030 * 0.39;
        auto ADD_3169 = SUB_260 + MUL_3164;
        auto MUL_3166 = MUL_3033 * 0.39;
        auto ADD_3170 = ADD_261 + MUL_3166;
        auto MUL_3168 = SUB_3040 * 0.39;
        auto ADD_3171 = ADD_262 + MUL_3168;
        if (/*base_link vs. forearm_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (68, 184)
        if (/*forearm_link*/ sphere_environment_in_collision(
            environment, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (sphere_environment_in_collision(environment, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (184, 184)
        if (/*shoulder_link vs. forearm_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (184, 184)
        auto MUL_338 = SUB_293 * 0.7071068;
        auto MUL_334 = SUB_289 * 0.7071068;
        auto MUL_342 = ADD_282 * 0.7071068;
        auto SUB_362 = MUL_338 - MUL_342;
        auto ADD_343 = MUL_338 + MUL_342;
        auto MUL_329 = ADD_277 * 0.7071068;
        auto SUB_355 = MUL_334 - MUL_329;
        auto ADD_335 = MUL_329 + MUL_334;
        auto MUL_366 = ADD_282 * 0.39225;
        auto MUL_378 = SUB_293 * MUL_366;
        auto MUL_374 = ADD_277 * 0.39225;
        auto MUL_380 = SUB_289 * MUL_374;
        auto SUB_382 = MUL_378 - MUL_380;
        auto MUL_384 = SUB_382 * 2.0;
        auto ADD_405 = SUB_260 + MUL_384;
        auto INPUT_3 = q[3];
        auto DIV_409 = INPUT_3 * 0.5;
        auto SIN_410 = DIV_409.sin();
        auto COS_416 = DIV_409.cos();
        auto MUL_435 = SUB_362 * COS_416;
        auto MUL_424 = SUB_362 * SIN_410;
        auto MUL_433 = SUB_355 * COS_416;
        auto MUL_421 = SUB_355 * SIN_410;
        auto MUL_426 = ADD_343 * COS_416;
        auto ADD_427 = MUL_424 + MUL_426;
        auto MUL_437 = ADD_343 * SIN_410;
        auto SUB_438 = MUL_435 - MUL_437;
        auto MUL_418 = ADD_335 * COS_416;
        auto ADD_422 = MUL_418 + MUL_421;
        auto MUL_3191 = ADD_422 * ADD_427;
        auto MUL_430 = ADD_335 * SIN_410;
        auto SUB_434 = MUL_433 - MUL_430;
        auto MUL_3186 = SUB_438 * SUB_434;
        auto ADD_3211 = MUL_3191 + MUL_3186;
        auto MUL_3214 = ADD_3211 * 2.0;
        auto MUL_3246 = MUL_3214 * 0.09;
        auto SUB_3258 = ADD_405 - MUL_3246;
        auto MUL_3185 = SUB_434 * SUB_434;
        auto MUL_3188 = ADD_422 * ADD_422;
        auto ADD_3216 = MUL_3185 + MUL_3188;
        auto MUL_3219 = ADD_3216 * 2.0;
        auto SUB_3222 = 1.0 - MUL_3219;
        auto MUL_3249 = SUB_3222 * 0.09;
        auto MUL_387 = SUB_293 * MUL_374;
        auto MUL_389 = SUB_289 * MUL_366;
        auto ADD_390 = MUL_387 + MUL_389;
        auto MUL_392 = ADD_390 * 2.0;
        auto ADD_406 = ADD_261 + MUL_392;
        auto ADD_3259 = ADD_406 + MUL_3249;
        auto MUL_3189 = SUB_438 * ADD_422;
        auto MUL_3195 = ADD_427 * SUB_434;
        auto SUB_3223 = MUL_3195 - MUL_3189;
        auto MUL_3225 = SUB_3223 * 2.0;
        auto MUL_3251 = MUL_3225 * 0.09;
        auto MUL_397 = ADD_282 * MUL_366;
        auto MUL_395 = ADD_277 * MUL_374;
        auto ADD_398 = MUL_395 + MUL_397;
        auto MUL_401 = ADD_398 * 2.0;
        auto SUB_404 = 0.39225 - MUL_401;
        auto ADD_407 = ADD_262 + SUB_404;
        auto ADD_3260 = ADD_407 + MUL_3251;
        auto MUL_3187 = SUB_438 * ADD_427;
        auto MUL_3193 = ADD_422 * SUB_434;
        auto SUB_3226 = MUL_3187 - MUL_3193;
        auto MUL_3228 = SUB_3226 * 2.0;
        auto MUL_3275 = MUL_3228 * 0.03;
        auto SUB_3280 = MUL_3275 - MUL_3246;
        auto ADD_3283 = ADD_405 + SUB_3280;
        auto ADD_3229 = MUL_3195 + MUL_3189;
        auto MUL_3231 = ADD_3229 * 2.0;
        auto MUL_3277 = MUL_3231 * 0.03;
        auto ADD_3281 = MUL_3249 + MUL_3277;
        auto ADD_3284 = ADD_406 + ADD_3281;
        auto MUL_3184 = ADD_427 * ADD_427;
        auto ADD_3232 = MUL_3184 + MUL_3188;
        auto MUL_3235 = ADD_3232 * 2.0;
        auto SUB_3238 = 1.0 - MUL_3235;
        auto MUL_3279 = SUB_3238 * 0.03;
        auto ADD_3282 = MUL_3251 + MUL_3279;
        auto ADD_3285 = ADD_407 + ADD_3282;
        auto ADD_3311 = MUL_3246 + MUL_3275;
        auto SUB_3315 = ADD_405 - ADD_3311;
        auto SUB_3313 = MUL_3249 - MUL_3277;
        auto ADD_3316 = ADD_406 + SUB_3313;
        auto SUB_3314 = MUL_3251 - MUL_3279;
        auto ADD_3317 = ADD_407 + SUB_3314;
        if (/*base_link vs. wrist_1_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (184, 270)
        if (/*shoulder_link vs. wrist_1_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (270, 270)
        if (/*upper_arm_link vs. wrist_1_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (270, 270)
        if (/*wrist_1_link*/ sphere_environment_in_collision(environment, SUB_3258, ADD_3259, ADD_3260, 0.07))
        {
            if (sphere_environment_in_collision(environment, ADD_3283, ADD_3284, ADD_3285, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3315, ADD_3316, ADD_3317, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_3258, ADD_3259, ADD_3260, 0.04))
            {
                return false;
            }
        }  // (270, 270)
        auto MUL_502 = SUB_434 * 0.093;
        auto MUL_513 = SUB_438 * MUL_502;
        auto MUL_505 = ADD_422 * 0.093;
        auto MUL_515 = ADD_427 * MUL_505;
        auto ADD_517 = MUL_513 + MUL_515;
        auto MUL_521 = ADD_517 * 2.0;
        auto SUB_546 = ADD_405 - MUL_521;
        auto INPUT_4 = q[4];
        auto DIV_550 = INPUT_4 * 0.5;
        auto SIN_551 = DIV_550.sin();
        auto COS_557 = DIV_550.cos();
        auto MUL_575 = SUB_438 * COS_557;
        auto MUL_570 = SUB_438 * SIN_551;
        auto MUL_573 = SUB_434 * COS_557;
        auto ADD_574 = MUL_570 + MUL_573;
        auto MUL_578 = SUB_434 * SIN_551;
        auto SUB_579 = MUL_575 - MUL_578;
        auto MUL_567 = ADD_427 * COS_557;
        auto MUL_561 = ADD_427 * SIN_551;
        auto MUL_559 = ADD_422 * COS_557;
        auto SUB_562 = MUL_561 - MUL_559;
        auto MUL_3355 = SUB_562 * ADD_574;
        auto MUL_565 = ADD_422 * SIN_551;
        auto ADD_568 = MUL_565 + MUL_567;
        auto MUL_3351 = SUB_579 * ADD_568;
        auto ADD_3383 = MUL_3355 + MUL_3351;
        auto MUL_3385 = ADD_3383 * 2.0;
        auto MUL_3409 = MUL_3385 * 0.09;
        auto ADD_3414 = SUB_546 + MUL_3409;
        auto MUL_3353 = SUB_579 * SUB_562;
        auto MUL_3356 = ADD_568 * ADD_574;
        auto SUB_3386 = MUL_3356 - MUL_3353;
        auto MUL_3388 = SUB_3386 * 2.0;
        auto MUL_3411 = MUL_3388 * 0.09;
        auto MUL_528 = SUB_434 * MUL_502;
        auto MUL_526 = ADD_422 * MUL_505;
        auto ADD_530 = MUL_526 + MUL_528;
        auto MUL_533 = ADD_530 * 2.0;
        auto SUB_536 = 0.093 - MUL_533;
        auto ADD_547 = ADD_406 + SUB_536;
        auto ADD_3415 = ADD_547 + MUL_3411;
        auto MUL_3352 = SUB_562 * SUB_562;
        auto MUL_3348 = ADD_568 * ADD_568;
        auto ADD_3389 = MUL_3348 + MUL_3352;
        auto MUL_3392 = ADD_3389 * 2.0;
        auto SUB_3395 = 1.0 - MUL_3392;
        auto MUL_3413 = SUB_3395 * 0.09;
        auto MUL_537 = SUB_438 * MUL_505;
        auto MUL_540 = ADD_427 * MUL_502;
        auto SUB_542 = MUL_540 - MUL_537;
        auto MUL_544 = SUB_542 * 2.0;
        auto ADD_548 = ADD_407 + MUL_544;
        auto ADD_3416 = ADD_548 + MUL_3413;
        auto MUL_3350 = SUB_579 * ADD_574;
        auto MUL_3354 = SUB_562 * ADD_568;
        auto SUB_3370 = MUL_3354 - MUL_3350;
        auto MUL_3372 = SUB_3370 * 2.0;
        auto MUL_3424 = MUL_3372 * 0.03;
        auto ADD_3435 = MUL_3424 + MUL_3409;
        auto ADD_3438 = SUB_546 + ADD_3435;
        auto MUL_3349 = ADD_574 * ADD_574;
        auto ADD_3373 = MUL_3349 + MUL_3352;
        auto MUL_3376 = ADD_3373 * 2.0;
        auto SUB_3379 = 1.0 - MUL_3376;
        auto MUL_3426 = SUB_3379 * 0.03;
        auto ADD_3436 = MUL_3426 + MUL_3411;
        auto ADD_3439 = ADD_547 + ADD_3436;
        auto ADD_3380 = MUL_3356 + MUL_3353;
        auto MUL_3382 = ADD_3380 * 2.0;
        auto MUL_3428 = MUL_3382 * 0.03;
        auto ADD_3437 = MUL_3428 + MUL_3413;
        auto ADD_3440 = ADD_548 + ADD_3437;
        auto SUB_3465 = MUL_3409 - MUL_3424;
        auto ADD_3468 = SUB_546 + SUB_3465;
        auto SUB_3466 = MUL_3411 - MUL_3426;
        auto ADD_3469 = ADD_547 + SUB_3466;
        auto SUB_3467 = MUL_3413 - MUL_3428;
        auto ADD_3470 = ADD_548 + SUB_3467;
        if (/*base_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (270, 348)
        if (/*forearm_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        if (/*shoulder_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        if (/*upper_arm_link vs. wrist_2_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        if (/*wrist_2_link*/ sphere_environment_in_collision(environment, ADD_3414, ADD_3415, ADD_3416, 0.07))
        {
            if (sphere_environment_in_collision(environment, ADD_3438, ADD_3439, ADD_3440, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3468, ADD_3469, ADD_3470, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3414, ADD_3415, ADD_3416, 0.04))
            {
                return false;
            }
        }  // (348, 348)
        auto MUL_648 = SUB_562 * 0.09465;
        auto MUL_653 = ADD_574 * MUL_648;
        auto MUL_640 = ADD_568 * 0.09465;
        auto MUL_651 = SUB_579 * MUL_640;
        auto ADD_654 = MUL_651 + MUL_653;
        auto MUL_656 = ADD_654 * 2.0;
        auto ADD_678 = SUB_546 + MUL_656;
        auto INPUT_5 = q[5];
        auto DIV_682 = INPUT_5 * 0.5;
        auto SIN_683 = DIV_682.sin();
        auto COS_689 = DIV_682.cos();
        auto MUL_705 = SUB_579 * COS_689;
        auto MUL_695 = SUB_579 * SIN_683;
        auto MUL_703 = ADD_574 * COS_689;
        auto MUL_693 = ADD_574 * SIN_683;
        auto MUL_691 = SUB_562 * COS_689;
        auto SUB_694 = MUL_691 - MUL_693;
        auto MUL_701 = SUB_562 * SIN_683;
        auto ADD_704 = MUL_701 + MUL_703;
        auto MUL_697 = ADD_568 * COS_689;
        auto ADD_698 = MUL_695 + MUL_697;
        auto MUL_3506 = SUB_694 * ADD_698;
        auto MUL_707 = ADD_568 * SIN_683;
        auto SUB_708 = MUL_705 - MUL_707;
        auto MUL_3502 = SUB_708 * ADD_704;
        auto SUB_3522 = MUL_3506 - MUL_3502;
        auto MUL_3524 = SUB_3522 * 2.0;
        auto MUL_3576 = MUL_3524 * 0.06;
        auto ADD_3587 = ADD_678 + MUL_3576;
        auto MUL_3501 = ADD_704 * ADD_704;
        auto MUL_3504 = SUB_694 * SUB_694;
        auto ADD_3525 = MUL_3501 + MUL_3504;
        auto MUL_3528 = ADD_3525 * 2.0;
        auto SUB_3531 = 1.0 - MUL_3528;
        auto MUL_3578 = SUB_3531 * 0.06;
        auto MUL_659 = SUB_579 * MUL_648;
        auto MUL_662 = ADD_574 * MUL_640;
        auto SUB_663 = MUL_662 - MUL_659;
        auto MUL_665 = SUB_663 * 2.0;
        auto ADD_679 = ADD_547 + MUL_665;
        auto ADD_3588 = ADD_679 + MUL_3578;
        auto MUL_3505 = SUB_708 * SUB_694;
        auto MUL_3508 = ADD_698 * ADD_704;
        auto ADD_3532 = MUL_3508 + MUL_3505;
        auto MUL_3534 = ADD_3532 * 2.0;
        auto MUL_3580 = MUL_3534 * 0.06;
        auto MUL_668 = SUB_562 * MUL_648;
        auto MUL_670 = ADD_568 * MUL_640;
        auto ADD_671 = MUL_668 + MUL_670;
        auto MUL_674 = ADD_671 * 2.0;
        auto SUB_677 = 0.09465 - MUL_674;
        auto ADD_680 = ADD_548 + SUB_677;
        auto ADD_3589 = ADD_680 + MUL_3580;
        if (/*base_link vs. wrist_3_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
        }  // (348, 401)
        if (/*forearm_link vs. wrist_3_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_3587, ADD_3588, ADD_3589, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
        }  // (401, 401)
        if (/*shoulder_link vs. wrist_3_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
        }  // (401, 401)
        if (/*upper_arm_link vs. wrist_3_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_3587, ADD_3588, ADD_3589, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3587, ADD_3588, ADD_3589, 0.04))
            {
                return false;
            }
        }  // (401, 401)
        if (/*wrist_3_link*/ sphere_environment_in_collision(environment, ADD_3587, ADD_3588, ADD_3589, 0.04))
        {
            return false;
        }  // (401, 401)
        auto MUL_770 = SUB_708 * 0.7071068;
        auto MUL_776 = ADD_704 * 0.7071068;
        auto SUB_777 = MUL_770 - MUL_776;
        auto ADD_768 = MUL_770 + MUL_776;
        auto MUL_1064 = SUB_777 * 0.7073883;
        auto MUL_1054 = SUB_777 * 0.7068252;
        auto MUL_1061 = ADD_768 * 0.7073883;
        auto SUB_1062 = MUL_1061 - MUL_1054;
        auto MUL_1071 = ADD_768 * 0.7068252;
        auto ADD_1073 = MUL_1064 + MUL_1071;
        auto MUL_3690 = ADD_1073 * SUB_1062;
        auto MUL_756 = ADD_698 * 0.7071068;
        auto MUL_753 = SUB_694 * 0.7071068;
        auto SUB_757 = MUL_756 - MUL_753;
        auto ADD_747 = MUL_753 + MUL_756;
        auto MUL_1048 = SUB_757 * 0.7073883;
        auto MUL_1036 = SUB_757 * 0.7068252;
        auto MUL_1033 = ADD_747 * 0.7073883;
        auto SUB_1038 = MUL_1033 - MUL_1036;
        auto MUL_1045 = ADD_747 * 0.7068252;
        auto ADD_1049 = MUL_1045 + MUL_1048;
        auto MUL_3694 = SUB_1038 * ADD_1049;
        auto SUB_3710 = MUL_3694 - MUL_3690;
        auto MUL_3712 = SUB_3710 * 2.0;
        auto MUL_781 = ADD_704 * 0.0823;
        auto MUL_791 = SUB_708 * MUL_781;
        auto MUL_784 = SUB_694 * 0.0823;
        auto MUL_793 = ADD_698 * MUL_784;
        auto SUB_794 = MUL_793 - MUL_791;
        auto MUL_797 = SUB_794 * 2.0;
        auto ADD_820 = ADD_678 + MUL_797;
        auto MUL_1086 = ADD_768 * 0.035;
        auto MUL_1091 = ADD_768 * MUL_1086;
        auto MUL_1081 = SUB_757 * 0.035;
        auto MUL_1089 = SUB_757 * MUL_1081;
        auto ADD_1093 = MUL_1089 + MUL_1091;
        auto MUL_1096 = ADD_1093 * 2.0;
        auto SUB_1099 = 0.035 - MUL_1096;
        auto ADD_1118 = ADD_820 + SUB_1099;
        auto MUL_3771 = MUL_3712 * 0.02;
        auto SUB_3787 = ADD_1118 - MUL_3771;
        auto MUL_3689 = SUB_1062 * SUB_1062;
        auto MUL_3692 = SUB_1038 * SUB_1038;
        auto ADD_3713 = MUL_3689 + MUL_3692;
        auto MUL_3716 = ADD_3713 * 2.0;
        auto SUB_3719 = 1.0 - MUL_3716;
        auto MUL_3775 = SUB_3719 * 0.02;
        auto MUL_1101 = SUB_777 * MUL_1086;
        auto MUL_1102 = ADD_747 * MUL_1081;
        auto ADD_1104 = MUL_1101 + MUL_1102;
        auto MUL_1107 = ADD_1104 * 2.0;
        auto MUL_803 = ADD_704 * MUL_781;
        auto MUL_801 = SUB_694 * MUL_784;
        auto ADD_805 = MUL_801 + MUL_803;
        auto MUL_808 = ADD_805 * 2.0;
        auto SUB_811 = 0.0823 - MUL_808;
        auto ADD_821 = ADD_679 + SUB_811;
        auto ADD_1119 = ADD_821 + MUL_1107;
        auto SUB_3788 = ADD_1119 - MUL_3775;
        auto MUL_3693 = ADD_1073 * SUB_1038;
        auto MUL_3696 = ADD_1049 * SUB_1062;
        auto ADD_3720 = MUL_3696 + MUL_3693;
        auto MUL_3722 = ADD_3720 * 2.0;
        auto MUL_3779 = MUL_3722 * 0.02;
        auto MUL_1109 = SUB_777 * MUL_1081;
        auto MUL_1111 = ADD_747 * MUL_1086;
        auto SUB_1113 = MUL_1111 - MUL_1109;
        auto MUL_1116 = SUB_1113 * 2.0;
        auto MUL_812 = SUB_708 * MUL_784;
        auto MUL_814 = ADD_698 * MUL_781;
        auto ADD_816 = MUL_812 + MUL_814;
        auto MUL_818 = ADD_816 * 2.0;
        auto ADD_822 = ADD_680 + MUL_818;
        auto ADD_1120 = ADD_822 + MUL_1116;
        auto SUB_3789 = ADD_1120 - MUL_3779;
        if (/*base_link vs. fts_robotside*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
        }  // (401, 476)
        if (/*forearm_link vs. fts_robotside*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3067, ADD_3068, ADD_3069, 0.265, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
        }  // (476, 476)
        if (/*fts_robotside*/ sphere_environment_in_collision(
            environment, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            return false;
        }  // (476, 476)
        if (/*shoulder_link vs. fts_robotside*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
        }  // (476, 476)
        if (/*upper_arm_link vs. fts_robotside*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
        }  // (476, 476)
        if (/*wrist_1_link vs. fts_robotside*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_3258, ADD_3259, ADD_3260, 0.07, SUB_3787, SUB_3788, SUB_3789, 0.04))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3283, ADD_3284, ADD_3285, 0.04, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3315, ADD_3316, ADD_3317, 0.04, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_3258, ADD_3259, ADD_3260, 0.04, SUB_3787, SUB_3788, SUB_3789, 0.04))
            {
                return false;
            }
        }  // (476, 476)
        auto MUL_1472 = ADD_1073 * 0.7073883;
        auto MUL_1440 = ADD_1073 * 0.7068252;
        auto MUL_1469 = SUB_1062 * 0.7073883;
        auto MUL_1457 = SUB_1062 * 0.7068252;
        auto MUL_1443 = SUB_1038 * 0.7073883;
        auto SUB_1444 = MUL_1443 - MUL_1440;
        auto MUL_1475 = SUB_1038 * 0.7068252;
        auto ADD_1477 = MUL_1472 + MUL_1475;
        auto MUL_1454 = ADD_1049 * 0.7073883;
        auto SUB_1459 = MUL_1454 - MUL_1457;
        auto MUL_3891 = ADD_1477 * SUB_1459;
        auto MUL_1466 = ADD_1049 * 0.7068252;
        auto ADD_1470 = MUL_1466 + MUL_1469;
        auto MUL_3895 = SUB_1444 * ADD_1470;
        auto ADD_3923 = MUL_3895 + MUL_3891;
        auto MUL_3925 = ADD_3923 * 2.0;
        auto MUL_3949 = MUL_3925 * 0.04;
        auto MUL_1346 = SUB_1038 * 0.0375;
        auto MUL_1351 = SUB_1062 * MUL_1346;
        auto MUL_1338 = ADD_1049 * 0.0375;
        auto MUL_1349 = ADD_1073 * MUL_1338;
        auto ADD_1352 = MUL_1349 + MUL_1351;
        auto MUL_1354 = ADD_1352 * 2.0;
        auto ADD_1376 = ADD_1118 + MUL_1354;
        auto MUL_1494 = SUB_1038 * 0.037;
        auto MUL_1501 = SUB_1062 * MUL_1494;
        auto MUL_1484 = ADD_1049 * 0.037;
        auto MUL_1498 = ADD_1073 * MUL_1484;
        auto ADD_1503 = MUL_1498 + MUL_1501;
        auto MUL_1506 = ADD_1503 * 2.0;
        auto SUB_1529 = ADD_1376 - MUL_1506;
        auto ADD_3954 = SUB_1529 + MUL_3949;
        auto MUL_3893 = ADD_1477 * SUB_1444;
        auto MUL_3896 = SUB_1459 * ADD_1470;
        auto SUB_3926 = MUL_3896 - MUL_3893;
        auto MUL_3928 = SUB_3926 * 2.0;
        auto MUL_3951 = MUL_3928 * 0.04;
        auto MUL_1510 = ADD_1073 * MUL_1494;
        auto MUL_1357 = ADD_1073 * MUL_1346;
        auto MUL_1512 = SUB_1062 * MUL_1484;
        auto SUB_1514 = MUL_1510 - MUL_1512;
        auto MUL_1516 = SUB_1514 * 2.0;
        auto MUL_1360 = SUB_1062 * MUL_1338;
        auto SUB_1361 = MUL_1360 - MUL_1357;
        auto MUL_1363 = SUB_1361 * 2.0;
        auto ADD_1377 = ADD_1119 + MUL_1363;
        auto ADD_1530 = ADD_1377 + MUL_1516;
        auto ADD_3955 = ADD_1530 + MUL_3951;
        auto MUL_3888 = SUB_1459 * SUB_1459;
        auto MUL_3892 = SUB_1444 * SUB_1444;
        auto ADD_3929 = MUL_3888 + MUL_3892;
        auto MUL_3932 = ADD_3929 * 2.0;
        auto SUB_3935 = 1.0 - MUL_3932;
        auto MUL_3953 = SUB_3935 * 0.04;
        auto MUL_1519 = SUB_1038 * MUL_1494;
        auto MUL_1366 = SUB_1038 * MUL_1346;
        auto MUL_1521 = ADD_1049 * MUL_1484;
        auto ADD_1523 = MUL_1519 + MUL_1521;
        auto MUL_1525 = ADD_1523 * 2.0;
        auto SUB_1528 = MUL_1525 - 0.037;
        auto MUL_1368 = ADD_1049 * MUL_1338;
        auto ADD_1369 = MUL_1366 + MUL_1368;
        auto MUL_1372 = ADD_1369 * 2.0;
        auto SUB_1375 = 0.0375 - MUL_1372;
        auto ADD_1378 = ADD_1120 + SUB_1375;
        auto ADD_1531 = ADD_1378 + SUB_1528;
        auto ADD_3956 = ADD_1531 + MUL_3953;
        auto MUL_3970 = MUL_3925 * 0.06;
        auto ADD_3975 = SUB_1529 + MUL_3970;
        auto MUL_3972 = MUL_3928 * 0.06;
        auto ADD_3976 = ADD_1530 + MUL_3972;
        auto MUL_3974 = SUB_3935 * 0.06;
        auto ADD_3977 = ADD_1531 + MUL_3974;
        auto MUL_3991 = MUL_3925 * 0.02;
        auto ADD_3996 = SUB_1529 + MUL_3991;
        auto MUL_3993 = MUL_3928 * 0.02;
        auto ADD_3997 = ADD_1530 + MUL_3993;
        auto MUL_3995 = SUB_3935 * 0.02;
        auto ADD_3998 = ADD_1531 + MUL_3995;
        if (/*base_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 0.9144, 0.08, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (476, 555)
        if (/*forearm_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        if (/*robotiq_85_base_link*/ sphere_environment_in_collision(
            environment, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_environment_in_collision(environment, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        if (/*shoulder_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, 1.003559, 0.08, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        if (/*upper_arm_link vs. robotiq_85_base_link*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_3954, ADD_3955, ADD_3956, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3975, ADD_3976, ADD_3977, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_3996, ADD_3997, ADD_3998, 0.04))
            {
                return false;
            }
        }  // (555, 555)
        auto MUL_1620 = SUB_1459 * 0.062792;
        auto MUL_1633 = ADD_1477 * MUL_1620;
        auto MUL_1629 = SUB_1444 * 0.062792;
        auto MUL_1631 = ADD_1470 * 0.0306011;
        auto SUB_1632 = MUL_1629 - MUL_1631;
        auto MUL_1637 = ADD_1470 * SUB_1632;
        auto MUL_1626 = SUB_1459 * 0.0306011;
        auto MUL_1634 = SUB_1459 * MUL_1626;
        auto SUB_1636 = MUL_1633 - MUL_1634;
        auto ADD_1638 = SUB_1636 + MUL_1637;
        auto MUL_1640 = ADD_1638 * 2.0;
        auto ADD_1642 = MUL_1640 + 0.0306011;
        auto ADD_1667 = SUB_1529 + ADD_1642;
        auto MUL_1644 = ADD_1477 * SUB_1632;
        auto MUL_1649 = ADD_1470 * MUL_1620;
        auto MUL_1646 = SUB_1444 * MUL_1626;
        auto SUB_1648 = MUL_1646 - MUL_1644;
        auto ADD_1650 = SUB_1648 + MUL_1649;
        auto MUL_1652 = ADD_1650 * 2.0;
        auto ADD_1668 = ADD_1530 + MUL_1652;
        auto MUL_1654 = ADD_1477 * MUL_1626;
        auto MUL_1659 = SUB_1459 * MUL_1620;
        auto MUL_1656 = SUB_1444 * SUB_1632;
        auto ADD_1657 = MUL_1654 + MUL_1656;
        auto ADD_1660 = ADD_1657 + MUL_1659;
        auto MUL_1663 = ADD_1660 * 2.0;
        auto SUB_1666 = 0.062792 - MUL_1663;
        auto ADD_1669 = ADD_1531 + SUB_1666;
        if (/*base_link vs. robotiq_85_left_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }   // (555, 583)
        if (/*forearm_link vs. robotiq_85_left_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }  // (583, 583)
        if (/*robotiq_85_left_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            return false;
        }   // (583, 583)
        if (/*shoulder_link vs. robotiq_85_left_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }   // (583, 583)
        if (/*upper_arm_link vs. robotiq_85_left_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_1667, ADD_1668, ADD_1669, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_1667, ADD_1668, ADD_1669, 0.02))
            {
                return false;
            }
        }  // (583, 583)
        auto MUL_4128 = ADD_3923 * 2.0;
        auto MUL_4152 = MUL_4128 * 0.02;
        auto MUL_1773 = ADD_1470 * 0.031691;
        auto MUL_1766 = SUB_1459 * 0.031691;
        auto MUL_1778 = SUB_1459 * MUL_1766;
        auto MUL_1759 = SUB_1459 * 0.001934;
        auto MUL_1776 = ADD_1477 * MUL_1759;
        auto ADD_1780 = MUL_1776 + MUL_1778;
        auto MUL_1770 = SUB_1444 * 0.001934;
        auto ADD_1774 = MUL_1770 + MUL_1773;
        auto MUL_1782 = ADD_1470 * ADD_1774;
        auto ADD_1784 = ADD_1780 + MUL_1782;
        auto MUL_1787 = ADD_1784 * 2.0;
        auto SUB_1790 = 0.031691 - MUL_1787;
        auto ADD_1815 = ADD_1667 + SUB_1790;
        auto ADD_4157 = ADD_1815 + MUL_4152;
        auto MUL_4131 = SUB_3926 * 2.0;
        auto MUL_4154 = MUL_4131 * 0.02;
        auto MUL_1792 = ADD_1477 * ADD_1774;
        auto MUL_1796 = ADD_1470 * MUL_1759;
        auto MUL_1793 = SUB_1444 * MUL_1766;
        auto ADD_1795 = MUL_1792 + MUL_1793;
        auto SUB_1798 = ADD_1795 - MUL_1796;
        auto MUL_1800 = SUB_1798 * 2.0;
        auto ADD_1816 = ADD_1668 + MUL_1800;
        auto ADD_4158 = ADD_1816 + MUL_4154;
        auto MUL_4135 = ADD_3929 * 2.0;
        auto SUB_4138 = 1.0 - MUL_4135;
        auto MUL_4156 = SUB_4138 * 0.02;
        auto MUL_1802 = ADD_1477 * MUL_1766;
        auto MUL_1807 = SUB_1459 * MUL_1759;
        auto MUL_1804 = SUB_1444 * ADD_1774;
        auto SUB_1806 = MUL_1804 - MUL_1802;
        auto ADD_1809 = SUB_1806 + MUL_1807;
        auto MUL_1811 = ADD_1809 * 2.0;
        auto SUB_1814 = MUL_1811 - 0.001934;
        auto ADD_1817 = ADD_1669 + SUB_1814;
        auto ADD_4159 = ADD_1817 + MUL_4156;
        auto MUL_4173 = MUL_4128 * 0.04;
        auto ADD_4178 = ADD_1815 + MUL_4173;
        auto MUL_4175 = MUL_4131 * 0.04;
        auto ADD_4179 = ADD_1816 + MUL_4175;
        auto MUL_4177 = SUB_4138 * 0.04;
        auto ADD_4180 = ADD_1817 + MUL_4177;
        if (/*base_link vs. robotiq_85_left_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }   // (583, 627)
        if (/*forearm_link vs. robotiq_85_left_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }  // (627, 627)
        if (/*robotiq_85_left_finger_link*/ sphere_environment_in_collision(
            environment, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_environment_in_collision(environment, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }   // (627, 627)
        if (/*shoulder_link vs. robotiq_85_left_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }   // (627, 627)
        if (/*upper_arm_link vs. robotiq_85_left_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4157, ADD_4158, ADD_4159, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4178, ADD_4179, ADD_4180, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_1815, ADD_1816, ADD_1817, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4157, ADD_4158, ADD_4159, 0.015))
            {
                return false;
            }
        }  // (627, 627)
        auto MUL_4265 = ADD_3923 * 2.0;
        auto MUL_4313 = MUL_4265 * 0.02;
        auto MUL_4229 = ADD_1470 * ADD_1470;
        auto ADD_4237 = MUL_3888 + MUL_4229;
        auto MUL_4240 = ADD_4237 * 2.0;
        auto SUB_4243 = 1.0 - MUL_4240;
        auto MUL_4301 = SUB_4243 * 0.02;
        auto ADD_4318 = MUL_4301 + MUL_4313;
        auto MUL_1906 = SUB_1459 * 0.0693075;
        auto MUL_1919 = ADD_1477 * MUL_1906;
        auto MUL_1915 = SUB_1444 * 0.0693075;
        auto MUL_1917 = ADD_1470 * 0.0127;
        auto SUB_1918 = MUL_1915 - MUL_1917;
        auto MUL_1923 = ADD_1470 * SUB_1918;
        auto MUL_1912 = SUB_1459 * 0.0127;
        auto MUL_1920 = SUB_1459 * MUL_1912;
        auto SUB_1922 = MUL_1919 - MUL_1920;
        auto ADD_1924 = SUB_1922 + MUL_1923;
        auto MUL_1926 = ADD_1924 * 2.0;
        auto ADD_1928 = MUL_1926 + 0.0127;
        auto ADD_1953 = SUB_1529 + ADD_1928;
        auto ADD_4321 = ADD_1953 + ADD_4318;
        auto MUL_4268 = SUB_3926 * 2.0;
        auto MUL_4315 = MUL_4268 * 0.02;
        auto MUL_1930 = ADD_1477 * SUB_1918;
        auto MUL_4230 = ADD_1477 * ADD_1470;
        auto MUL_1935 = ADD_1470 * MUL_1906;
        auto MUL_1932 = SUB_1444 * MUL_1912;
        auto SUB_1934 = MUL_1932 - MUL_1930;
        auto ADD_1936 = SUB_1934 + MUL_1935;
        auto MUL_1938 = ADD_1936 * 2.0;
        auto ADD_1954 = ADD_1530 + MUL_1938;
        auto MUL_4234 = SUB_1444 * SUB_1459;
        auto ADD_4244 = MUL_4234 + MUL_4230;
        auto MUL_4246 = ADD_4244 * 2.0;
        auto MUL_4303 = MUL_4246 * 0.02;
        auto ADD_4319 = MUL_4303 + MUL_4315;
        auto ADD_4322 = ADD_1954 + ADD_4319;
        auto SUB_4247 = MUL_3895 - MUL_3891;
        auto MUL_4272 = ADD_3929 * 2.0;
        auto SUB_4275 = 1.0 - MUL_4272;
        auto MUL_4317 = SUB_4275 * 0.02;
        auto MUL_4249 = SUB_4247 * 2.0;
        auto MUL_4305 = MUL_4249 * 0.02;
        auto ADD_4320 = MUL_4305 + MUL_4317;
        auto MUL_1940 = ADD_1477 * MUL_1912;
        auto MUL_1945 = SUB_1459 * MUL_1906;
        auto MUL_1942 = SUB_1444 * SUB_1918;
        auto ADD_1943 = MUL_1940 + MUL_1942;
        auto ADD_1946 = ADD_1943 + MUL_1945;
        auto MUL_1949 = ADD_1946 * 2.0;
        auto SUB_1952 = 0.0693075 - MUL_1949;
        auto ADD_1955 = ADD_1531 + SUB_1952;
        auto ADD_4323 = ADD_1955 + ADD_4320;
        if (/*base_link vs. robotiq_85_left_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }   // (627, 681)
        if (/*forearm_link vs. robotiq_85_left_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }  // (681, 681)
        if (/*robotiq_85_left_inner_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            return false;
        }   // (681, 681)
        if (/*shoulder_link vs. robotiq_85_left_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }   // (681, 681)
        if (/*upper_arm_link vs. robotiq_85_left_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4321, ADD_4322, ADD_4323, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4321, ADD_4322, ADD_4323, 0.02))
            {
                return false;
            }
        }  // (681, 681)
        auto MUL_4363 = ADD_3923 * 2.0;
        auto MUL_2044 = SUB_1459 * 0.045497;
        auto MUL_2057 = ADD_1477 * MUL_2044;
        auto MUL_2053 = SUB_1444 * 0.045497;
        auto MUL_2055 = ADD_1470 * 0.0345853;
        auto SUB_2056 = MUL_2053 - MUL_2055;
        auto MUL_2061 = ADD_1470 * SUB_2056;
        auto MUL_2050 = SUB_1459 * 0.0345853;
        auto MUL_2058 = SUB_1459 * MUL_2050;
        auto SUB_2060 = MUL_2057 - MUL_2058;
        auto ADD_2062 = SUB_2060 + MUL_2061;
        auto MUL_2064 = ADD_2062 * 2.0;
        auto ADD_2066 = MUL_2064 + 0.0345853;
        auto ADD_2091 = ADD_1953 + ADD_2066;
        auto MUL_4387 = MUL_4363 * 0.013;
        auto ADD_4392 = ADD_2091 + MUL_4387;
        auto MUL_4366 = SUB_3926 * 2.0;
        auto MUL_4389 = MUL_4366 * 0.013;
        auto MUL_2068 = ADD_1477 * SUB_2056;
        auto MUL_2073 = ADD_1470 * MUL_2044;
        auto MUL_2070 = SUB_1444 * MUL_2050;
        auto SUB_2072 = MUL_2070 - MUL_2068;
        auto ADD_2074 = SUB_2072 + MUL_2073;
        auto MUL_2076 = ADD_2074 * 2.0;
        auto ADD_2092 = ADD_1954 + MUL_2076;
        auto ADD_4393 = ADD_2092 + MUL_4389;
        auto MUL_4370 = ADD_3929 * 2.0;
        auto SUB_4373 = 1.0 - MUL_4370;
        auto MUL_4391 = SUB_4373 * 0.013;
        auto MUL_2078 = ADD_1477 * MUL_2050;
        auto MUL_2083 = SUB_1459 * MUL_2044;
        auto MUL_2080 = SUB_1444 * SUB_2056;
        auto ADD_2081 = MUL_2078 + MUL_2080;
        auto ADD_2084 = ADD_2081 + MUL_2083;
        auto MUL_2087 = ADD_2084 * 2.0;
        auto SUB_2090 = 0.045497 - MUL_2087;
        auto ADD_2093 = ADD_1955 + SUB_2090;
        auto ADD_4394 = ADD_2093 + MUL_4391;
        auto MUL_4408 = MUL_4363 * 0.025;
        auto ADD_4413 = ADD_2091 + MUL_4408;
        auto MUL_4410 = MUL_4366 * 0.025;
        auto ADD_4414 = ADD_2092 + MUL_4410;
        auto MUL_4412 = SUB_4373 * 0.025;
        auto ADD_4415 = ADD_2093 + MUL_4412;
        if (/*base_link vs. robotiq_85_left_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }   // (681, 725)
        if (/*forearm_link vs. robotiq_85_left_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }  // (725, 725)
        if (/*robotiq_85_left_finger_tip_link*/ sphere_environment_in_collision(
            environment, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_environment_in_collision(environment, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }   // (725, 725)
        if (/*shoulder_link vs. robotiq_85_left_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }   // (725, 725)
        if (/*upper_arm_link vs. robotiq_85_left_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4392, ADD_4393, ADD_4394, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4413, ADD_4414, ADD_4415, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2091, ADD_2092, ADD_2093, 0.02))
            {
                return false;
            }
        }  // (725, 725)
        auto ADD_2202 = MUL_1919 + MUL_1920;
        auto ADD_2199 = MUL_1915 + MUL_1917;
        auto MUL_4441 = ADD_1477 * ADD_1477;
        auto ADD_4453 = MUL_3892 + MUL_4441;
        auto MUL_4456 = ADD_4453 * 2.0;
        auto SUB_4459 = 1.0 - MUL_4456;
        auto MUL_4522 = SUB_4459 * 0.02;
        auto MUL_2203 = ADD_1470 * ADD_2199;
        auto ADD_2204 = ADD_2202 + MUL_2203;
        auto MUL_2206 = ADD_2204 * 2.0;
        auto SUB_2209 = MUL_2206 - 0.0127;
        auto ADD_2230 = SUB_1529 + SUB_2209;
        auto MUL_4444 = ADD_1470 * SUB_1444;
        auto MUL_4450 = SUB_1459 * ADD_1477;
        auto ADD_4483 = MUL_4450 + MUL_4444;
        auto MUL_4485 = ADD_4483 * 2.0;
        auto MUL_4535 = MUL_4485 * 0.02;
        auto ADD_4540 = MUL_4522 + MUL_4535;
        auto ADD_4543 = ADD_2230 + ADD_4540;
        auto MUL_2211 = ADD_1477 * ADD_2199;
        auto ADD_2214 = MUL_2211 + MUL_1932;
        auto SUB_2217 = MUL_1935 - ADD_2214;
        auto MUL_2219 = SUB_2217 * 2.0;
        auto ADD_2231 = ADD_1530 + MUL_2219;
        auto MUL_4442 = ADD_1470 * ADD_1477;
        auto MUL_4446 = ADD_1470 * SUB_1459;
        auto MUL_4448 = SUB_1459 * SUB_1444;
        auto ADD_4460 = MUL_4448 + MUL_4442;
        auto MUL_4463 = ADD_4460 * 2.0;
        auto MUL_4524 = MUL_4463 * 0.02;
        auto MUL_4451 = SUB_1444 * ADD_1477;
        auto SUB_4486 = MUL_4446 - MUL_4451;
        auto MUL_4488 = SUB_4486 * 2.0;
        auto MUL_4537 = MUL_4488 * 0.02;
        auto SUB_4541 = MUL_4537 - MUL_4524;
        auto ADD_4544 = ADD_2231 + SUB_4541;
        auto SUB_4465 = MUL_4450 - MUL_4444;
        auto ADD_4489 = MUL_3892 + MUL_3888;
        auto MUL_4492 = ADD_4489 * 2.0;
        auto SUB_4495 = 1.0 - MUL_4492;
        auto MUL_4539 = SUB_4495 * 0.02;
        auto MUL_4467 = SUB_4465 * 2.0;
        auto MUL_4527 = MUL_4467 * 0.02;
        auto ADD_4542 = MUL_4527 + MUL_4539;
        auto MUL_2222 = SUB_1444 * ADD_2199;
        auto SUB_2223 = MUL_1940 - MUL_2222;
        auto SUB_2225 = SUB_2223 - MUL_1945;
        auto MUL_2227 = SUB_2225 * 2.0;
        auto ADD_2229 = MUL_2227 + 0.0693075;
        auto ADD_2232 = ADD_1531 + ADD_2229;
        auto ADD_4545 = ADD_2232 + ADD_4542;
        if (/*base_link vs. robotiq_85_right_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }   // (725, 776)
        if (/*forearm_link vs. robotiq_85_right_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }  // (776, 776)
        if (/*robotiq_85_right_inner_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            return false;
        }   // (776, 776)
        if (/*shoulder_link vs. robotiq_85_right_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }   // (776, 776)
        if (/*upper_arm_link vs. robotiq_85_right_inner_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4543, ADD_4544, ADD_4545, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4543, ADD_4544, ADD_4545, 0.02))
            {
                return false;
            }
        }  // (776, 776)
        auto MUL_4593 = ADD_4483 * 2.0;
        auto MUL_4617 = MUL_4593 * 0.013;
        auto MUL_2330 = SUB_1459 * 0.0458574;
        auto MUL_2320 = SUB_1444 * 0.0458574;
        auto MUL_2334 = ADD_1470 * MUL_2320;
        auto MUL_2332 = ADD_1477 * 0.034106;
        auto SUB_2333 = MUL_2330 - MUL_2332;
        auto MUL_2338 = ADD_1477 * SUB_2333;
        auto MUL_2327 = SUB_1444 * 0.034106;
        auto MUL_2335 = SUB_1444 * MUL_2327;
        auto SUB_2337 = MUL_2334 - MUL_2335;
        auto ADD_2339 = SUB_2337 + MUL_2338;
        auto MUL_2341 = ADD_2339 * 2.0;
        auto ADD_2343 = MUL_2341 + 0.034106;
        auto ADD_2366 = ADD_2230 + ADD_2343;
        auto ADD_4622 = ADD_2366 + MUL_4617;
        auto MUL_4596 = SUB_4486 * 2.0;
        auto MUL_4619 = MUL_4596 * 0.013;
        auto MUL_2347 = ADD_1477 * MUL_2320;
        auto MUL_2344 = ADD_1470 * SUB_2333;
        auto MUL_2345 = SUB_1459 * MUL_2327;
        auto SUB_2346 = MUL_2344 - MUL_2345;
        auto SUB_2349 = SUB_2346 - MUL_2347;
        auto MUL_2351 = SUB_2349 * 2.0;
        auto ADD_2367 = ADD_2231 + MUL_2351;
        auto ADD_4623 = ADD_2367 + MUL_4619;
        auto MUL_4600 = ADD_4489 * 2.0;
        auto SUB_4603 = 1.0 - MUL_4600;
        auto MUL_4621 = SUB_4603 * 0.013;
        auto MUL_2353 = ADD_1470 * MUL_2327;
        auto MUL_2355 = SUB_1459 * SUB_2333;
        auto ADD_2356 = MUL_2353 + MUL_2355;
        auto MUL_2358 = SUB_1444 * MUL_2320;
        auto ADD_2359 = ADD_2356 + MUL_2358;
        auto MUL_2362 = ADD_2359 * 2.0;
        auto SUB_2365 = 0.0458574 - MUL_2362;
        auto ADD_2368 = ADD_2232 + SUB_2365;
        auto ADD_4624 = ADD_2368 + MUL_4621;
        auto MUL_4638 = MUL_4593 * 0.025;
        auto ADD_4643 = ADD_2366 + MUL_4638;
        auto MUL_4640 = MUL_4596 * 0.025;
        auto ADD_4644 = ADD_2367 + MUL_4640;
        auto MUL_4642 = SUB_4603 * 0.025;
        auto ADD_4645 = ADD_2368 + MUL_4642;
        if (/*base_link vs. robotiq_85_right_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }   // (776, 820)
        if (/*forearm_link vs. robotiq_85_right_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }  // (820, 820)
        if (/*robotiq_85_right_finger_tip_link*/ sphere_environment_in_collision(
            environment, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_environment_in_collision(environment, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }   // (820, 820)
        if (/*shoulder_link vs. robotiq_85_right_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }   // (820, 820)
        if (/*upper_arm_link vs. robotiq_85_right_finger_tip_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4622, ADD_4623, ADD_4624, 0.033))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4643, ADD_4644, ADD_4645, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2366, ADD_2367, ADD_2368, 0.02))
            {
                return false;
            }
        }  // (820, 820)
        auto ADD_2476 = MUL_1633 + MUL_1634;
        auto ADD_2473 = MUL_1629 + MUL_1631;
        auto MUL_2477 = ADD_1470 * ADD_2473;
        auto ADD_2478 = ADD_2476 + MUL_2477;
        auto MUL_2480 = ADD_2478 * 2.0;
        auto SUB_2483 = MUL_2480 - 0.0306011;
        auto ADD_2504 = SUB_1529 + SUB_2483;
        auto MUL_2485 = ADD_1477 * ADD_2473;
        auto ADD_2488 = MUL_2485 + MUL_1646;
        auto SUB_2491 = MUL_1649 - ADD_2488;
        auto MUL_2493 = SUB_2491 * 2.0;
        auto ADD_2505 = ADD_1530 + MUL_2493;
        auto MUL_2496 = SUB_1444 * ADD_2473;
        auto SUB_2497 = MUL_1654 - MUL_2496;
        auto SUB_2499 = SUB_2497 - MUL_1659;
        auto MUL_2501 = SUB_2499 * 2.0;
        auto ADD_2503 = MUL_2501 + 0.062792;
        auto ADD_2506 = ADD_1531 + ADD_2503;
        if (/*base_link vs. robotiq_85_right_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }   // (820, 838)
        if (/*forearm_link vs. robotiq_85_right_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }  // (838, 838)
        if (/*robotiq_85_right_knuckle_link*/ sphere_environment_in_collision(
            environment, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            return false;
        }   // (838, 838)
        if (/*shoulder_link vs. robotiq_85_right_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }   // (838, 838)
        if (/*upper_arm_link vs. robotiq_85_right_knuckle_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_2504, ADD_2505, ADD_2506, 0.02))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2504, ADD_2505, ADD_2506, 0.02))
            {
                return false;
            }
        }  // (838, 838)
        auto MUL_4809 = ADD_4483 * 2.0;
        auto MUL_4833 = MUL_4809 * 0.02;
        auto MUL_2608 = ADD_1477 * 0.0317096;
        auto MUL_2601 = SUB_1444 * 0.0317096;
        auto MUL_2613 = SUB_1444 * MUL_2601;
        auto MUL_2605 = SUB_1459 * 0.0016014;
        auto ADD_2609 = MUL_2605 + MUL_2608;
        auto MUL_2617 = ADD_1477 * ADD_2609;
        auto MUL_2595 = SUB_1444 * 0.0016014;
        auto MUL_2611 = ADD_1470 * MUL_2595;
        auto ADD_2615 = MUL_2611 + MUL_2613;
        auto ADD_2619 = ADD_2615 + MUL_2617;
        auto MUL_2622 = ADD_2619 * 2.0;
        auto SUB_2625 = 0.0317096 - MUL_2622;
        auto ADD_2649 = ADD_2504 + SUB_2625;
        auto ADD_4838 = ADD_2649 + MUL_4833;
        auto MUL_4812 = SUB_4486 * 2.0;
        auto MUL_4835 = MUL_4812 * 0.02;
        auto MUL_2631 = ADD_1477 * MUL_2595;
        auto MUL_2626 = ADD_1470 * ADD_2609;
        auto MUL_2628 = SUB_1459 * MUL_2601;
        auto ADD_2629 = MUL_2626 + MUL_2628;
        auto SUB_2632 = MUL_2631 - ADD_2629;
        auto MUL_2634 = SUB_2632 * 2.0;
        auto ADD_2650 = ADD_2505 + MUL_2634;
        auto ADD_4839 = ADD_2650 + MUL_4835;
        auto MUL_4816 = ADD_4489 * 2.0;
        auto SUB_4819 = 1.0 - MUL_4816;
        auto MUL_4837 = SUB_4819 * 0.02;
        auto MUL_2636 = ADD_1470 * MUL_2601;
        auto MUL_2638 = SUB_1459 * ADD_2609;
        auto SUB_2640 = MUL_2638 - MUL_2636;
        auto MUL_2641 = SUB_1444 * MUL_2595;
        auto ADD_2643 = SUB_2640 + MUL_2641;
        auto MUL_2645 = ADD_2643 * 2.0;
        auto SUB_2648 = MUL_2645 - 0.0016014;
        auto ADD_2651 = ADD_2506 + SUB_2648;
        auto ADD_4840 = ADD_2651 + MUL_4837;
        auto MUL_4854 = MUL_4809 * 0.04;
        auto ADD_4859 = ADD_2649 + MUL_4854;
        auto MUL_4856 = MUL_4812 * 0.04;
        auto ADD_4860 = ADD_2650 + MUL_4856;
        auto MUL_4858 = SUB_4819 * 0.04;
        auto ADD_4861 = ADD_2651 + MUL_4858;
        if (/*base_link vs. robotiq_85_right_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 0.9144, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 0.9144, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
        }   // (838, 882)
        if (/*forearm_link vs. robotiq_85_right_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                ADD_3067, ADD_3068, ADD_3069, 0.265, ADD_4838, ADD_4839, ADD_4840, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_260, ADD_261, ADD_262, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3106, ADD_3107, ADD_3108, 0.06, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3127, ADD_3128, ADD_3129, 0.06, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3148, ADD_3149, ADD_3150, 0.06, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_3169, ADD_3170, ADD_3171, 0.06, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
        }  // (882, 882)
        if (/*robotiq_85_right_finger_link*/ sphere_environment_in_collision(
            environment, ADD_4838, ADD_4839, ADD_4840, 0.035))
        {
            if (sphere_environment_in_collision(environment, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
        }   // (882, 882)
        if (/*shoulder_link vs. robotiq_85_right_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                0.0, 0.0, 1.003559, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, 1.003559, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
        }   // (882, 882)
        if (/*upper_arm_link vs. robotiq_85_right_finger_link*/
            sphere_sphere_self_collision<decltype(q[0])>(
                SUB_2869, ADD_2870, ADD_2871, 0.29, ADD_4838, ADD_4839, ADD_4840, 0.035))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2890, ADD_2891, ADD_2892, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2869, ADD_2870, ADD_2871, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2932, ADD_2933, ADD_2934, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_2953, ADD_2954, ADD_2955, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4859, ADD_4860, ADD_4861, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_2649, ADD_2650, ADD_2651, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    NEGATE_101, SUB_112, 1.003559, 0.08, ADD_4838, ADD_4839, ADD_4840, 0.015))
            {
                return false;
            }
        }  // (882, 882)
        set_attachment_pose(
            environment, SUB_1529, ADD_1530, ADD_1531, SUB_1444, SUB_1459, ADD_1470, ADD_1477);
        if (/*attachment vs. base_link*/ attachment_sphere_collision<decltype(q[0])>(
            environment, 0.0, 0.0, 0.9144, 0.08))
        {
            return false;
        }  // (882, 889)
        if (/*attachment vs. forearm_link*/ attachment_sphere_collision<decltype(q[0])>(
            environment, ADD_3067, ADD_3068, ADD_3069, 0.265))
        {
            if (attachment_sphere_collision<decltype(q[0])>(environment, SUB_260, ADD_261, ADD_262, 0.08))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_3106, ADD_3107, ADD_3108, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_3127, ADD_3128, ADD_3129, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_3148, ADD_3149, ADD_3150, 0.06))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, ADD_3169, ADD_3170, ADD_3171, 0.06))
            {
                return false;
            }
        }  // (889, 889)
        if (/*attachment vs. shoulder_link*/ attachment_sphere_collision<decltype(q[0])>(
            environment, 0.0, 0.0, 1.003559, 0.08))
        {
            return false;
        }  // (889, 889)
        if (/*attachment vs. upper_arm_link*/ attachment_sphere_collision<decltype(q[0])>(
            environment, SUB_2869, ADD_2870, ADD_2871, 0.29))
        {
            if (attachment_sphere_collision<decltype(q[0])>(environment, SUB_2890, ADD_2891, ADD_2892, 0.08))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, SUB_2869, ADD_2870, ADD_2871, 0.08))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, SUB_2932, ADD_2933, ADD_2934, 0.08))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, SUB_2953, ADD_2954, ADD_2955, 0.08))
            {
                return false;
            }
            if (attachment_sphere_collision<decltype(q[0])>(environment, NEGATE_101, SUB_112, 1.003559, 0.08))
            {
                return false;
            }
        }  // (889, 889)
        if (attachment_environment_collision(environment))
        {
            return false;
        }  // (889, 889)
        return true;
    }

    inline auto eefk(const std::array<float, 6> &q) noexcept -> std::array<float, 7>
    {
        auto INPUT_5 = q[5];
        auto DIV_682 = INPUT_5 * 0.5;
        auto SIN_683 = std::sin(DIV_682);
        auto COS_689 = std::cos(DIV_682);
        auto INPUT_4 = q[4];
        auto DIV_550 = INPUT_4 * 0.5;
        auto SIN_551 = std::sin(DIV_550);
        auto COS_557 = std::cos(DIV_550);
        auto INPUT_3 = q[3];
        auto DIV_409 = INPUT_3 * 0.5;
        auto SIN_410 = std::sin(DIV_409);
        auto COS_416 = std::cos(DIV_409);
        auto INPUT_2 = q[2];
        auto DIV_264 = INPUT_2 * 0.5;
        auto SIN_265 = std::sin(DIV_264);
        auto COS_271 = std::cos(DIV_264);
        auto INPUT_1 = q[1];
        auto DIV_123 = INPUT_1 * 0.5;
        auto SIN_124 = std::sin(DIV_123);
        auto COS_130 = std::cos(DIV_123);
        auto INPUT_0 = q[0];
        auto DIV_7 = INPUT_0 * 0.5;
        auto SIN_8 = std::sin(DIV_7);
        auto COS_14 = std::cos(DIV_7);
        auto MUL_41 = COS_14 * 0.7073883;
        auto MUL_38 = COS_14 * 0.7068252;
        auto MUL_32 = SIN_8 * 0.7073883;
        auto ADD_39 = MUL_32 + MUL_38;
        auto MUL_47 = SIN_8 * 0.7068252;
        auto SUB_48 = MUL_41 - MUL_47;
        auto MUL_59 = SUB_48 * 0.7071068;
        auto MUL_140 = MUL_59 * COS_130;
        auto MUL_138 = MUL_59 * SIN_124;
        auto SUB_152 = MUL_140 - MUL_138;
        auto ADD_141 = MUL_138 + MUL_140;
        auto MUL_290 = SUB_152 * COS_271;
        auto MUL_279 = SUB_152 * SIN_265;
        auto MUL_281 = ADD_141 * COS_271;
        auto ADD_282 = MUL_279 + MUL_281;
        auto MUL_342 = ADD_282 * 0.7071068;
        auto MUL_366 = ADD_282 * 0.39225;
        auto MUL_292 = ADD_141 * SIN_265;
        auto SUB_293 = MUL_290 - MUL_292;
        auto MUL_378 = SUB_293 * MUL_366;
        auto MUL_338 = SUB_293 * 0.7071068;
        auto SUB_362 = MUL_338 - MUL_342;
        auto ADD_343 = MUL_338 + MUL_342;
        auto MUL_435 = SUB_362 * COS_416;
        auto MUL_424 = SUB_362 * SIN_410;
        auto MUL_426 = ADD_343 * COS_416;
        auto ADD_427 = MUL_424 + MUL_426;
        auto MUL_567 = ADD_427 * COS_557;
        auto MUL_561 = ADD_427 * SIN_551;
        auto MUL_437 = ADD_343 * SIN_410;
        auto SUB_438 = MUL_435 - MUL_437;
        auto MUL_575 = SUB_438 * COS_557;
        auto MUL_570 = SUB_438 * SIN_551;
        auto MUL_214 = ADD_141 * 0.425;
        auto MUL_73 = ADD_39 * 0.7071068;
        auto MUL_132 = MUL_73 * COS_130;
        auto MUL_135 = MUL_73 * SIN_124;
        auto SUB_148 = MUL_132 - MUL_135;
        auto ADD_136 = MUL_132 + MUL_135;
        auto MUL_288 = SUB_148 * COS_271;
        auto MUL_276 = SUB_148 * SIN_265;
        auto MUL_217 = SUB_148 * 0.1197;
        auto ADD_219 = MUL_214 + MUL_217;
        auto MUL_230 = SUB_152 * ADD_219;
        auto MUL_273 = ADD_136 * COS_271;
        auto ADD_277 = MUL_273 + MUL_276;
        auto MUL_329 = ADD_277 * 0.7071068;
        auto MUL_374 = ADD_277 * 0.39225;
        auto MUL_285 = ADD_136 * SIN_265;
        auto SUB_289 = MUL_288 - MUL_285;
        auto MUL_380 = SUB_289 * MUL_374;
        auto SUB_382 = MUL_378 - MUL_380;
        auto MUL_384 = SUB_382 * 2.0;
        auto MUL_334 = SUB_289 * 0.7071068;
        auto SUB_355 = MUL_334 - MUL_329;
        auto ADD_335 = MUL_329 + MUL_334;
        auto MUL_433 = SUB_355 * COS_416;
        auto MUL_421 = SUB_355 * SIN_410;
        auto MUL_418 = ADD_335 * COS_416;
        auto ADD_422 = MUL_418 + MUL_421;
        auto MUL_559 = ADD_422 * COS_557;
        auto SUB_562 = MUL_561 - MUL_559;
        auto MUL_691 = SUB_562 * COS_689;
        auto MUL_701 = SUB_562 * SIN_683;
        auto MUL_648 = SUB_562 * 0.09465;
        auto MUL_565 = ADD_422 * SIN_551;
        auto ADD_568 = MUL_565 + MUL_567;
        auto MUL_697 = ADD_568 * COS_689;
        auto MUL_707 = ADD_568 * SIN_683;
        auto MUL_640 = ADD_568 * 0.09465;
        auto MUL_505 = ADD_422 * 0.093;
        auto MUL_515 = ADD_427 * MUL_505;
        auto MUL_430 = ADD_335 * SIN_410;
        auto SUB_434 = MUL_433 - MUL_430;
        auto MUL_573 = SUB_434 * COS_557;
        auto ADD_574 = MUL_570 + MUL_573;
        auto MUL_703 = ADD_574 * COS_689;
        auto ADD_704 = MUL_701 + MUL_703;
        auto MUL_776 = ADD_704 * 0.7071068;
        auto MUL_781 = ADD_704 * 0.0823;
        auto MUL_693 = ADD_574 * SIN_683;
        auto SUB_694 = MUL_691 - MUL_693;
        auto MUL_753 = SUB_694 * 0.7071068;
        auto MUL_784 = SUB_694 * 0.0823;
        auto MUL_653 = ADD_574 * MUL_648;
        auto MUL_578 = SUB_434 * SIN_551;
        auto SUB_579 = MUL_575 - MUL_578;
        auto MUL_705 = SUB_579 * COS_689;
        auto SUB_708 = MUL_705 - MUL_707;
        auto MUL_791 = SUB_708 * MUL_781;
        auto MUL_770 = SUB_708 * 0.7071068;
        auto SUB_777 = MUL_770 - MUL_776;
        auto ADD_768 = MUL_770 + MUL_776;
        auto MUL_1064 = SUB_777 * 0.7073883;
        auto MUL_1054 = SUB_777 * 0.7068252;
        auto MUL_1061 = ADD_768 * 0.7073883;
        auto SUB_1062 = MUL_1061 - MUL_1054;
        auto MUL_1071 = ADD_768 * 0.7068252;
        auto ADD_1073 = MUL_1064 + MUL_1071;
        auto MUL_1086 = ADD_768 * 0.035;
        auto MUL_1091 = ADD_768 * MUL_1086;
        auto MUL_695 = SUB_579 * SIN_683;
        auto ADD_698 = MUL_695 + MUL_697;
        auto MUL_793 = ADD_698 * MUL_784;
        auto SUB_794 = MUL_793 - MUL_791;
        auto MUL_797 = SUB_794 * 2.0;
        auto MUL_756 = ADD_698 * 0.7071068;
        auto SUB_757 = MUL_756 - MUL_753;
        auto ADD_747 = MUL_753 + MUL_756;
        auto MUL_1048 = SUB_757 * 0.7073883;
        auto MUL_1036 = SUB_757 * 0.7068252;
        auto MUL_1081 = SUB_757 * 0.035;
        auto MUL_1089 = SUB_757 * MUL_1081;
        auto ADD_1093 = MUL_1089 + MUL_1091;
        auto MUL_1096 = ADD_1093 * 2.0;
        auto SUB_1099 = 0.035 - MUL_1096;
        auto MUL_1033 = ADD_747 * 0.7073883;
        auto SUB_1038 = MUL_1033 - MUL_1036;
        auto MUL_1346 = SUB_1038 * 0.0375;
        auto MUL_1351 = SUB_1062 * MUL_1346;
        auto MUL_1494 = SUB_1038 * 0.037;
        auto MUL_1501 = SUB_1062 * MUL_1494;
        auto MUL_1045 = ADD_747 * 0.7068252;
        auto ADD_1049 = MUL_1045 + MUL_1048;
        auto MUL_1338 = ADD_1049 * 0.0375;
        auto MUL_1349 = ADD_1073 * MUL_1338;
        auto ADD_1352 = MUL_1349 + MUL_1351;
        auto MUL_1354 = ADD_1352 * 2.0;
        auto MUL_1484 = ADD_1049 * 0.037;
        auto MUL_1498 = ADD_1073 * MUL_1484;
        auto ADD_1503 = MUL_1498 + MUL_1501;
        auto MUL_1506 = ADD_1503 * 2.0;
        auto MUL_651 = SUB_579 * MUL_640;
        auto ADD_654 = MUL_651 + MUL_653;
        auto MUL_656 = ADD_654 * 2.0;
        auto MUL_502 = SUB_434 * 0.093;
        auto MUL_513 = SUB_438 * MUL_502;
        auto ADD_517 = MUL_513 + MUL_515;
        auto MUL_521 = ADD_517 * 2.0;
        auto MUL_226 = ADD_136 * 0.425;
        auto MUL_233 = SUB_148 * MUL_226;
        auto MUL_222 = ADD_136 * 0.1197;
        auto MUL_231 = ADD_141 * MUL_222;
        auto ADD_232 = MUL_230 + MUL_231;
        auto SUB_235 = ADD_232 - MUL_233;
        auto MUL_237 = SUB_235 * 2.0;
        auto MUL_85 = ADD_39 * 0.13585;
        auto MUL_95 = SUB_48 * MUL_85;
        auto MUL_100 = MUL_95 * 2.0;
        auto SUB_260 = MUL_237 - MUL_100;
        auto ADD_405 = SUB_260 + MUL_384;
        auto SUB_546 = ADD_405 - MUL_521;
        auto ADD_678 = SUB_546 + MUL_656;
        auto ADD_820 = ADD_678 + MUL_797;
        auto ADD_1118 = ADD_820 + SUB_1099;
        auto ADD_1376 = ADD_1118 + MUL_1354;
        auto SUB_1529 = ADD_1376 - MUL_1506;
        auto MUL_1510 = ADD_1073 * MUL_1494;
        auto MUL_1357 = ADD_1073 * MUL_1346;
        auto MUL_1512 = SUB_1062 * MUL_1484;
        auto SUB_1514 = MUL_1510 - MUL_1512;
        auto MUL_1516 = SUB_1514 * 2.0;
        auto MUL_1360 = SUB_1062 * MUL_1338;
        auto SUB_1361 = MUL_1360 - MUL_1357;
        auto MUL_1363 = SUB_1361 * 2.0;
        auto MUL_1101 = SUB_777 * MUL_1086;
        auto MUL_1102 = ADD_747 * MUL_1081;
        auto ADD_1104 = MUL_1101 + MUL_1102;
        auto MUL_1107 = ADD_1104 * 2.0;
        auto MUL_803 = ADD_704 * MUL_781;
        auto MUL_801 = SUB_694 * MUL_784;
        auto ADD_805 = MUL_801 + MUL_803;
        auto MUL_808 = ADD_805 * 2.0;
        auto SUB_811 = 0.0823 - MUL_808;
        auto MUL_659 = SUB_579 * MUL_648;
        auto MUL_662 = ADD_574 * MUL_640;
        auto SUB_663 = MUL_662 - MUL_659;
        auto MUL_665 = SUB_663 * 2.0;
        auto MUL_528 = SUB_434 * MUL_502;
        auto MUL_526 = ADD_422 * MUL_505;
        auto ADD_530 = MUL_526 + MUL_528;
        auto MUL_533 = ADD_530 * 2.0;
        auto SUB_536 = 0.093 - MUL_533;
        auto MUL_387 = SUB_293 * MUL_374;
        auto MUL_389 = SUB_289 * MUL_366;
        auto ADD_390 = MUL_387 + MUL_389;
        auto MUL_392 = ADD_390 * 2.0;
        auto MUL_240 = SUB_152 * MUL_226;
        auto MUL_244 = SUB_148 * ADD_219;
        auto MUL_241 = ADD_136 * MUL_222;
        auto ADD_243 = MUL_240 + MUL_241;
        auto ADD_245 = ADD_243 + MUL_244;
        auto MUL_247 = ADD_245 * 2.0;
        auto SUB_250 = MUL_247 - 0.1197;
        auto MUL_106 = ADD_39 * MUL_85;
        auto MUL_109 = MUL_106 * 2.0;
        auto SUB_112 = 0.13585 - MUL_109;
        auto ADD_261 = SUB_112 + SUB_250;
        auto ADD_406 = ADD_261 + MUL_392;
        auto ADD_547 = ADD_406 + SUB_536;
        auto ADD_679 = ADD_547 + MUL_665;
        auto ADD_821 = ADD_679 + SUB_811;
        auto ADD_1119 = ADD_821 + MUL_1107;
        auto ADD_1377 = ADD_1119 + MUL_1363;
        auto ADD_1530 = ADD_1377 + MUL_1516;
        auto MUL_1519 = SUB_1038 * MUL_1494;
        auto MUL_1366 = SUB_1038 * MUL_1346;
        auto MUL_1521 = ADD_1049 * MUL_1484;
        auto ADD_1523 = MUL_1519 + MUL_1521;
        auto MUL_1525 = ADD_1523 * 2.0;
        auto SUB_1528 = MUL_1525 - 0.037;
        auto MUL_1368 = ADD_1049 * MUL_1338;
        auto ADD_1369 = MUL_1366 + MUL_1368;
        auto MUL_1372 = ADD_1369 * 2.0;
        auto SUB_1375 = 0.0375 - MUL_1372;
        auto MUL_1109 = SUB_777 * MUL_1081;
        auto MUL_1111 = ADD_747 * MUL_1086;
        auto SUB_1113 = MUL_1111 - MUL_1109;
        auto MUL_1116 = SUB_1113 * 2.0;
        auto MUL_812 = SUB_708 * MUL_784;
        auto MUL_814 = ADD_698 * MUL_781;
        auto ADD_816 = MUL_812 + MUL_814;
        auto MUL_818 = ADD_816 * 2.0;
        auto MUL_668 = SUB_562 * MUL_648;
        auto MUL_670 = ADD_568 * MUL_640;
        auto ADD_671 = MUL_668 + MUL_670;
        auto MUL_674 = ADD_671 * 2.0;
        auto SUB_677 = 0.09465 - MUL_674;
        auto MUL_537 = SUB_438 * MUL_505;
        auto MUL_540 = ADD_427 * MUL_502;
        auto SUB_542 = MUL_540 - MUL_537;
        auto MUL_544 = SUB_542 * 2.0;
        auto MUL_397 = ADD_282 * MUL_366;
        auto MUL_395 = ADD_277 * MUL_374;
        auto ADD_398 = MUL_395 + MUL_397;
        auto MUL_401 = ADD_398 * 2.0;
        auto SUB_404 = 0.39225 - MUL_401;
        auto MUL_251 = SUB_152 * MUL_222;
        auto MUL_254 = ADD_141 * ADD_219;
        auto MUL_252 = ADD_136 * MUL_226;
        auto SUB_253 = MUL_251 - MUL_252;
        auto SUB_255 = SUB_253 - MUL_254;
        auto MUL_257 = SUB_255 * 2.0;
        auto ADD_259 = MUL_257 + 0.425;
        auto ADD_262 = 1.003559 + ADD_259;
        auto ADD_407 = ADD_262 + SUB_404;
        auto ADD_548 = ADD_407 + MUL_544;
        auto ADD_680 = ADD_548 + SUB_677;
        auto ADD_822 = ADD_680 + MUL_818;
        auto ADD_1120 = ADD_822 + MUL_1116;
        auto ADD_1378 = ADD_1120 + SUB_1375;
        auto ADD_1531 = ADD_1378 + SUB_1528;
        auto MUL_1440 = ADD_1073 * 0.7068252;
        auto MUL_1443 = SUB_1038 * 0.7073883;
        auto SUB_1444 = MUL_1443 - MUL_1440;
        auto MUL_1457 = SUB_1062 * 0.7068252;
        auto MUL_1454 = ADD_1049 * 0.7073883;
        auto SUB_1459 = MUL_1454 - MUL_1457;
        auto MUL_1469 = SUB_1062 * 0.7073883;
        auto MUL_1466 = ADD_1049 * 0.7068252;
        auto ADD_1470 = MUL_1466 + MUL_1469;
        auto MUL_1472 = ADD_1073 * 0.7073883;
        auto MUL_1475 = SUB_1038 * 0.7068252;
        auto ADD_1477 = MUL_1472 + MUL_1475;
        return {SUB_1529, ADD_1530, ADD_1531, SUB_1444, SUB_1459, ADD_1470, ADD_1477};
    }
}  // namespace vamp::robots::ur5

// NOLINTEND(*-magic-numbers)
