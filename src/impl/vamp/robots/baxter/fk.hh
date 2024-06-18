#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots::baxter
{
    using Configuration = FloatVector<14>;
    template <std::size_t block_width>
    using ConfigurationBlock = FloatVector<block_width, 14>;

    alignas(Configuration::S::Alignment) constexpr std::array<float, 14> s_m_a{
        3.40335987756,
        3.194,
        6.10835987756,
        2.6679999999999997,
        6.118,
        3.66479632679,
        6.118,
        3.40335987756,
        3.194,
        6.10835987756,
        2.6679999999999997,
        6.118,
        3.66479632679,
        6.118};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 14> s_a_a{
        -1.70167993878,
        -2.147,
        -3.05417993878,
        -0.05,
        -3.059,
        -1.57079632679,
        -3.059,
        -1.70167993878,
        -2.147,
        -3.05417993878,
        -0.05,
        -3.059,
        -1.57079632679,
        -3.059};
    const Configuration s_m(s_m_a);
    const Configuration s_a(s_a_a);

    inline void scale_configuration(Configuration &q) noexcept
    {
        q = q * s_m + s_a;
    }

    template <std::size_t block_width>
    inline void scale_configuration_block(ConfigurationBlock<block_width> &q) noexcept
    {
        q[0] = -1.70167993878 + (q[0] * 3.40335987756);
        q[1] = -2.147 + (q[1] * 3.194);
        q[2] = -3.05417993878 + (q[2] * 6.10835987756);
        q[3] = -0.05 + (q[3] * 2.6679999999999997);
        q[4] = -3.059 + (q[4] * 6.118);
        q[5] = -1.57079632679 + (q[5] * 3.66479632679);
        q[6] = -3.059 + (q[6] * 6.118);
        q[7] = -1.70167993878 + (q[7] * 3.40335987756);
        q[8] = -2.147 + (q[8] * 3.194);
        q[9] = -3.05417993878 + (q[9] * 6.10835987756);
        q[10] = -0.05 + (q[10] * 2.6679999999999997);
        q[11] = -3.059 + (q[11] * 6.118);
        q[12] = -1.57079632679 + (q[12] * 3.66479632679);
        q[13] = -3.059 + (q[13] * 6.118);
    }

    alignas(Configuration::S::Alignment) constexpr std::array<float, 14> d_m_a{
        0.2938272871445316,
        0.31308703819661865,
        0.16371006621166082,
        0.37481259370314846,
        0.16345210853220005,
        0.2728664599148137,
        0.16345210853220005,
        0.2938272871445316,
        0.31308703819661865,
        0.16371006621166082,
        0.37481259370314846,
        0.16345210853220005,
        0.2728664599148137,
        0.16345210853220005};
    alignas(Configuration::S::Alignment) constexpr std::array<float, 14> d_s_a{
        -1.70167993878,
        -2.147,
        -3.05417993878,
        -0.05,
        -3.059,
        -1.57079632679,
        -3.059,
        -1.70167993878,
        -2.147,
        -3.05417993878,
        -0.05,
        -3.059,
        -1.57079632679,
        -3.059};
    const Configuration d_m(d_m_a);
    const Configuration d_s(d_s_a);

    inline void descale_configuration(Configuration &q) noexcept
    {
        q = (q - d_s) * d_m;
    }

    template <std::size_t block_width>
    inline void descale_configuration_block(ConfigurationBlock<block_width> &q) noexcept
    {
        q[0] = 0.2938272871445316 * (q[0] - -1.70167993878);
        q[1] = 0.31308703819661865 * (q[1] - -2.147);
        q[2] = 0.16371006621166082 * (q[2] - -3.05417993878);
        q[3] = 0.37481259370314846 * (q[3] - -0.05);
        q[4] = 0.16345210853220005 * (q[4] - -3.059);
        q[5] = 0.2728664599148137 * (q[5] - -1.57079632679);
        q[6] = 0.16345210853220005 * (q[6] - -3.059);
        q[7] = 0.2938272871445316 * (q[7] - -1.70167993878);
        q[8] = 0.31308703819661865 * (q[8] - -2.147);
        q[9] = 0.16371006621166082 * (q[9] - -3.05417993878);
        q[10] = 0.37481259370314846 * (q[10] - -0.05);
        q[11] = 0.16345210853220005 * (q[11] - -3.059);
        q[12] = 0.2728664599148137 * (q[12] - -1.57079632679);
        q[13] = 0.16345210853220005 * (q[13] - -3.059);
    }

    inline static auto space_measure() noexcept -> float
    {
        return 89641415145.821;
    }

    constexpr auto n_spheres = 75;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, 75> x;
        FloatVector<rake, 75> y;
        FloatVector<rake, 75> z;
        FloatVector<rake, 75> r;
    };

    template <std::size_t rake>
    inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
    {
        out.r[0] = 0.25;        // (0, 0)
        out.r[1] = 0.25;        // (0, 0)
        out.r[2] = 0.23;        // (0, 0)
        out.r[3] = 0.2;         // (0, 0)
        out.r[4] = 0.1;         // (0, 0)
        out.r[5] = 0.1;         // (0, 0)
        out.r[6] = 0.1;         // (0, 0)
        out.r[7] = 0.08;        // (0, 0)
        out.r[8] = 0.08;        // (0, 0)
        out.r[9] = 0.08;        // (0, 0)
        out.r[10] = 0.1;        // (0, 0)
        out.r[11] = 0.08;       // (0, 0)
        out.r[12] = 0.08;       // (0, 0)
        out.r[13] = 0.08;       // (0, 0)
        out.r[14] = 0.07;       // (0, 0)
        out.r[15] = 0.07;       // (0, 0)
        out.r[16] = 0.07;       // (0, 0)
        out.r[17] = 0.08;       // (0, 0)
        out.r[18] = 0.05;       // (0, 0)
        out.r[19] = 0.1;        // (0, 0)
        out.r[20] = 0.1;        // (0, 0)
        out.r[21] = 0.1;        // (0, 0)
        out.r[22] = 0.08;       // (0, 0)
        out.r[23] = 0.08;       // (0, 0)
        out.r[24] = 0.08;       // (0, 0)
        out.r[25] = 0.1;        // (0, 0)
        out.r[26] = 0.08;       // (0, 0)
        out.r[27] = 0.08;       // (0, 0)
        out.r[28] = 0.08;       // (0, 0)
        out.r[29] = 0.07;       // (0, 0)
        out.r[30] = 0.07;       // (0, 0)
        out.r[31] = 0.07;       // (0, 0)
        out.r[32] = 0.08;       // (0, 0)
        out.r[33] = 0.05;       // (0, 0)
        out.r[34] = 0.5;        // (0, 0)
        out.r[35] = 0.04;       // (0, 0)
        out.r[36] = 0.04;       // (0, 0)
        out.r[37] = 0.015;      // (0, 0)
        out.r[38] = 0.015;      // (0, 0)
        out.r[39] = 0.012;      // (0, 0)
        out.r[40] = 0.012;      // (0, 0)
        out.r[41] = 0.012;      // (0, 0)
        out.r[42] = 0.014;      // (0, 0)
        out.r[43] = 0.014;      // (0, 0)
        out.r[44] = 0.014;      // (0, 0)
        out.r[45] = 0.014;      // (0, 0)
        out.r[46] = 0.015;      // (0, 0)
        out.r[47] = 0.015;      // (0, 0)
        out.r[48] = 0.012;      // (0, 0)
        out.r[49] = 0.012;      // (0, 0)
        out.r[50] = 0.012;      // (0, 0)
        out.r[51] = 0.014;      // (0, 0)
        out.r[52] = 0.014;      // (0, 0)
        out.r[53] = 0.014;      // (0, 0)
        out.r[54] = 0.014;      // (0, 0)
        out.r[55] = 0.04;       // (0, 0)
        out.r[56] = 0.04;       // (0, 0)
        out.r[57] = 0.015;      // (0, 0)
        out.r[58] = 0.015;      // (0, 0)
        out.r[59] = 0.012;      // (0, 0)
        out.r[60] = 0.012;      // (0, 0)
        out.r[61] = 0.012;      // (0, 0)
        out.r[62] = 0.014;      // (0, 0)
        out.r[63] = 0.014;      // (0, 0)
        out.r[64] = 0.014;      // (0, 0)
        out.r[65] = 0.014;      // (0, 0)
        out.r[66] = 0.015;      // (0, 0)
        out.r[67] = 0.015;      // (0, 0)
        out.r[68] = 0.012;      // (0, 0)
        out.r[69] = 0.012;      // (0, 0)
        out.r[70] = 0.012;      // (0, 0)
        out.r[71] = 0.014;      // (0, 0)
        out.r[72] = 0.014;      // (0, 0)
        out.r[73] = 0.014;      // (0, 0)
        out.r[74] = 0.014;      // (0, 0)
        out.x[0] = -0.025;      // (0, 0)
        out.x[1] = -0.025;      // (0, 0)
        out.x[2] = -0.065;      // (0, 0)
        out.x[3] = 0.04;        // (0, 0)
        out.x[4] = 0.0640272;   // (0, 0)
        out.x[5] = 0.0640272;   // (0, 0)
        out.x[19] = 0.0640272;  // (0, 0)
        out.x[20] = 0.0640272;  // (0, 0)
        out.x[34] = 0.0;        // (0, 0)
        out.y[0] = -0.1;        // (0, 0)
        out.y[1] = 0.1;         // (0, 0)
        out.y[2] = 0.0;         // (0, 0)
        out.y[3] = 0.0;         // (0, 0)
        out.y[4] = -0.2590274;  // (0, 0)
        out.y[5] = -0.2590274;  // (0, 0)
        out.y[19] = 0.2590274;  // (0, 0)
        out.y[20] = 0.2590274;  // (0, 0)
        out.y[34] = 0.0;        // (0, 0)
        out.z[0] = 0.1;         // (0, 0)
        out.z[1] = 0.1;         // (0, 0)
        out.z[2] = 0.4;         // (0, 0)
        out.z[3] = 0.686;       // (0, 0)
        out.z[4] = 0.379626;    // (0, 0)
        out.z[5] = 0.229626;    // (0, 0)
        out.z[6] = 0.399976;    // (0, 0)
        out.z[19] = 0.379626;   // (0, 0)
        out.z[20] = 0.229626;   // (0, 0)
        out.z[21] = 0.399976;   // (0, 0)
        out.z[34] = -0.6;       // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_15 = INPUT_0 * 0.5;
        auto SIN_16 = DIV_15.sin();
        auto COS_22 = DIV_15.cos();
        auto MUL_49 = COS_22 * 0.3826843;
        auto MUL_42 = SIN_16 * 0.9238792;
        auto SUB_51 = MUL_42 - MUL_49;
        auto MUL_112 = SUB_51 * 0.069;
        auto MUL_116 = SUB_51 * MUL_112;
        auto MUL_119 = MUL_116 * 2.0;
        auto SUB_122 = 0.069 - MUL_119;
        auto ADD_137 = SUB_122 + 0.0640272;
        out.x[6] = ADD_137;  // (0, 12)
        auto MUL_53 = COS_22 * 0.9238792;
        auto MUL_60 = SIN_16 * 0.3826843;
        auto ADD_62 = MUL_53 + MUL_60;
        auto MUL_93 = ADD_62 * 0.7071068;
        auto MUL_91 = SUB_51 * 0.7071068;
        auto INPUT_2 = q[2];
        auto DIV_304 = INPUT_2 * 0.5;
        auto SIN_305 = DIV_304.sin();
        auto COS_311 = DIV_304.cos();
        auto INPUT_1 = q[1];
        auto DIV_144 = INPUT_1 * 0.5;
        auto SIN_145 = DIV_144.sin();
        auto COS_151 = DIV_144.cos();
        auto MUL_170 = MUL_91 * COS_151;
        auto MUL_155 = MUL_91 * SIN_145;
        auto MUL_153 = MUL_93 * COS_151;
        auto SUB_176 = MUL_153 - MUL_155;
        auto ADD_157 = MUL_153 + MUL_155;
        auto MUL_233 = SUB_176 * 0.5;
        auto MUL_235 = ADD_157 * 0.5;
        auto SUB_237 = MUL_233 - MUL_235;
        auto ADD_249 = MUL_233 + MUL_235;
        auto MUL_167 = MUL_93 * SIN_145;
        auto SUB_165 = MUL_167 - MUL_170;
        auto ADD_171 = MUL_167 + MUL_170;
        auto MUL_242 = ADD_171 * 0.5;
        auto MUL_268 = ADD_171 * 0.102;
        auto MUL_273 = ADD_171 * MUL_268;
        auto MUL_239 = SUB_165 * 0.5;
        auto SUB_252 = ADD_249 - MUL_239;
        auto SUB_255 = SUB_252 - MUL_242;
        auto SUB_240 = SUB_237 - MUL_239;
        auto ADD_243 = SUB_240 + MUL_242;
        auto ADD_228 = ADD_249 + MUL_239;
        auto ADD_231 = ADD_228 + MUL_242;
        auto ADD_216 = SUB_237 + MUL_239;
        auto SUB_219 = ADD_216 - MUL_242;
        auto MUL_328 = SUB_255 * COS_311;
        auto MUL_323 = SUB_255 * SIN_305;
        auto MUL_326 = ADD_243 * COS_311;
        auto ADD_327 = MUL_323 + MUL_326;
        auto MUL_4538 = ADD_327 * ADD_327;
        auto MUL_331 = ADD_243 * SIN_305;
        auto SUB_332 = MUL_328 - MUL_331;
        auto MUL_320 = ADD_231 * COS_311;
        auto MUL_314 = ADD_231 * SIN_305;
        auto MUL_313 = SUB_219 * COS_311;
        auto ADD_315 = MUL_313 + MUL_314;
        auto MUL_4544 = ADD_315 * ADD_327;
        auto MUL_318 = SUB_219 * SIN_305;
        auto SUB_321 = MUL_320 - MUL_318;
        auto MUL_4540 = SUB_332 * SUB_321;
        auto ADD_4572 = MUL_4544 + MUL_4540;
        auto MUL_4574 = ADD_4572 * 2.0;
        auto MUL_4604 = MUL_4574 * 0.22;
        auto MUL_4537 = SUB_321 * SUB_321;
        auto ADD_4546 = MUL_4537 + MUL_4538;
        auto MUL_4549 = ADD_4546 * 2.0;
        auto SUB_4552 = 1.0 - MUL_4549;
        auto MUL_4587 = SUB_4552 * 0.02;
        auto SUB_4609 = MUL_4604 - MUL_4587;
        auto MUL_263 = SUB_165 * 0.102;
        auto MUL_271 = SUB_165 * MUL_263;
        auto ADD_275 = MUL_271 + MUL_273;
        auto MUL_278 = ADD_275 * 2.0;
        auto SUB_281 = 0.102 - MUL_278;
        auto ADD_300 = ADD_137 + SUB_281;
        out.x[9] = ADD_300;  // (12, 79)
        auto ADD_4612 = ADD_300 + SUB_4609;
        out.x[7] = ADD_4612;  // (79, 80)
        auto MUL_4634 = MUL_4574 * 0.11;
        auto MUL_4617 = SUB_4552 * 0.01;
        auto SUB_4639 = MUL_4634 - MUL_4617;
        auto ADD_4642 = ADD_300 + SUB_4639;
        out.x[8] = ADD_4642;  // (80, 84)
        auto MUL_445 = ADD_327 * 0.069;
        auto MUL_440 = SUB_321 * 0.069;
        auto MUL_448 = SUB_321 * MUL_440;
        auto MUL_434 = SUB_321 * 0.26242;
        auto MUL_447 = SUB_332 * MUL_434;
        auto SUB_450 = MUL_447 - MUL_448;
        auto MUL_443 = ADD_315 * 0.26242;
        auto SUB_446 = MUL_443 - MUL_445;
        auto MUL_451 = ADD_327 * SUB_446;
        auto ADD_452 = SUB_450 + MUL_451;
        auto MUL_454 = ADD_452 * 2.0;
        auto ADD_456 = MUL_454 + 0.069;
        auto ADD_481 = ADD_300 + ADD_456;
        out.x[10] = ADD_481;  // (84, 97)
        auto MUL_400 = SUB_332 * 0.5;
        auto MUL_414 = ADD_327 * 0.5;
        auto MUL_410 = SUB_321 * 0.5;
        auto MUL_404 = ADD_315 * 0.5;
        auto SUB_370 = MUL_404 - MUL_400;
        auto SUB_375 = SUB_370 - MUL_410;
        auto ADD_406 = MUL_400 + MUL_404;
        auto SUB_412 = MUL_410 - ADD_406;
        auto ADD_427 = ADD_406 + MUL_410;
        auto ADD_432 = ADD_427 + MUL_414;
        auto ADD_415 = SUB_412 + MUL_414;
        auto ADD_392 = SUB_370 + MUL_410;
        auto SUB_397 = ADD_392 - MUL_414;
        auto ADD_380 = SUB_375 + MUL_414;
        auto INPUT_3 = q[3];
        auto DIV_485 = INPUT_3 * 0.5;
        auto SIN_486 = DIV_485.sin();
        auto COS_492 = DIV_485.cos();
        auto MUL_504 = ADD_432 * SIN_486;
        auto MUL_507 = ADD_415 * COS_492;
        auto ADD_508 = MUL_504 + MUL_507;
        auto MUL_601 = ADD_508 * 0.10359;
        auto MUL_606 = ADD_508 * MUL_601;
        auto MUL_501 = SUB_397 * COS_492;
        auto MUL_499 = ADD_380 * SIN_486;
        auto SUB_502 = MUL_501 - MUL_499;
        auto MUL_596 = SUB_502 * 0.10359;
        auto MUL_604 = SUB_502 * MUL_596;
        auto ADD_608 = MUL_604 + MUL_606;
        auto MUL_611 = ADD_608 * 2.0;
        auto SUB_614 = 0.10359 - MUL_611;
        auto ADD_633 = ADD_481 + SUB_614;
        out.x[11] = ADD_633;  // (97, 129)
        auto MUL_554 = ADD_508 * 0.5;
        auto MUL_551 = SUB_502 * 0.5;
        auto MUL_509 = ADD_432 * COS_492;
        auto MUL_512 = ADD_415 * SIN_486;
        auto SUB_513 = MUL_509 - MUL_512;
        auto MUL_546 = SUB_513 * 0.5;
        auto MUL_495 = SUB_397 * SIN_486;
        auto MUL_494 = ADD_380 * COS_492;
        auto ADD_496 = MUL_494 + MUL_495;
        auto MUL_548 = ADD_496 * 0.5;
        auto SUB_560 = MUL_546 - MUL_548;
        auto SUB_585 = SUB_560 - MUL_551;
        auto SUB_588 = SUB_585 - MUL_554;
        auto ADD_563 = SUB_560 + MUL_551;
        auto ADD_566 = ADD_563 + MUL_554;
        auto ADD_549 = MUL_546 + MUL_548;
        auto SUB_574 = ADD_549 - MUL_551;
        auto ADD_577 = SUB_574 + MUL_554;
        auto ADD_552 = ADD_549 + MUL_551;
        auto SUB_555 = ADD_552 - MUL_554;
        auto INPUT_4 = q[4];
        auto DIV_637 = INPUT_4 * 0.5;
        auto SIN_638 = DIV_637.sin();
        auto COS_644 = DIV_637.cos();
        auto MUL_661 = SUB_588 * COS_644;
        auto MUL_656 = SUB_588 * SIN_638;
        auto MUL_659 = ADD_577 * COS_644;
        auto ADD_660 = MUL_656 + MUL_659;
        auto MUL_664 = ADD_577 * SIN_638;
        auto SUB_665 = MUL_661 - MUL_664;
        auto MUL_653 = ADD_566 * COS_644;
        auto MUL_647 = ADD_566 * SIN_638;
        auto MUL_646 = SUB_555 * COS_644;
        auto ADD_648 = MUL_646 + MUL_647;
        auto MUL_4736 = ADD_648 * ADD_660;
        auto MUL_651 = SUB_555 * SIN_638;
        auto SUB_654 = MUL_653 - MUL_651;
        auto MUL_4732 = SUB_665 * SUB_654;
        auto ADD_4764 = MUL_4736 + MUL_4732;
        auto MUL_4766 = ADD_4764 * 2.0;
        auto MUL_4808 = MUL_4766 * 0.22;
        auto ADD_4813 = ADD_633 + MUL_4808;
        out.x[12] = ADD_4813;  // (129, 171)
        auto MUL_4829 = MUL_4766 * 0.11;
        auto ADD_4834 = ADD_633 + MUL_4829;
        out.x[13] = ADD_4834;  // (171, 173)
        auto MUL_699 = SUB_665 * 0.5;
        auto MUL_711 = ADD_660 * 0.5;
        auto MUL_778 = ADD_660 * 0.01;
        auto MUL_706 = SUB_654 * 0.5;
        auto MUL_773 = SUB_654 * 0.01;
        auto MUL_781 = SUB_654 * MUL_773;
        auto MUL_702 = ADD_648 * 0.5;
        auto SUB_703 = MUL_702 - MUL_699;
        auto SUB_708 = SUB_703 - MUL_706;
        auto ADD_725 = SUB_703 + MUL_706;
        auto SUB_730 = ADD_725 - MUL_711;
        auto ADD_739 = MUL_699 + MUL_702;
        auto SUB_745 = MUL_706 - ADD_739;
        auto ADD_760 = ADD_739 + MUL_706;
        auto ADD_765 = ADD_760 + MUL_711;
        auto ADD_748 = SUB_745 + MUL_711;
        auto ADD_713 = SUB_708 + MUL_711;
        auto MUL_767 = SUB_654 * 0.2707;
        auto MUL_780 = SUB_665 * MUL_767;
        auto SUB_783 = MUL_780 - MUL_781;
        auto MUL_776 = ADD_648 * 0.2707;
        auto SUB_779 = MUL_776 - MUL_778;
        auto MUL_784 = ADD_660 * SUB_779;
        auto ADD_785 = SUB_783 + MUL_784;
        auto MUL_787 = ADD_785 * 2.0;
        auto ADD_789 = MUL_787 + 0.01;
        auto ADD_814 = ADD_633 + ADD_789;
        auto INPUT_5 = q[5];
        auto DIV_818 = INPUT_5 * 0.5;
        auto SIN_819 = DIV_818.sin();
        auto COS_825 = DIV_818.cos();
        auto MUL_842 = ADD_765 * COS_825;
        auto MUL_837 = ADD_765 * SIN_819;
        auto MUL_840 = ADD_748 * COS_825;
        auto ADD_841 = MUL_837 + MUL_840;
        auto MUL_845 = ADD_748 * SIN_819;
        auto SUB_846 = MUL_842 - MUL_845;
        auto MUL_834 = SUB_730 * COS_825;
        auto MUL_828 = SUB_730 * SIN_819;
        auto MUL_827 = ADD_713 * COS_825;
        auto ADD_829 = MUL_827 + MUL_828;
        auto MUL_4844 = ADD_829 * ADD_841;
        auto MUL_832 = ADD_713 * SIN_819;
        auto SUB_835 = MUL_834 - MUL_832;
        auto MUL_4840 = SUB_846 * SUB_835;
        auto ADD_4872 = MUL_4844 + MUL_4840;
        auto MUL_4874 = ADD_4872 * 2.0;
        auto MUL_4898 = MUL_4874 * 0.03;
        auto ADD_4903 = ADD_814 + MUL_4898;
        out.x[14] = ADD_4903;  // (173, 222)
        auto SUB_4930 = ADD_814 - MUL_4898;
        out.x[15] = SUB_4930;  // (222, 223)
        auto MUL_879 = SUB_846 * 0.5;
        auto MUL_887 = ADD_841 * 0.5;
        auto MUL_884 = SUB_835 * 0.5;
        auto MUL_881 = ADD_829 * 0.5;
        auto SUB_893 = MUL_879 - MUL_881;
        auto SUB_918 = SUB_893 - MUL_884;
        auto SUB_921 = SUB_918 - MUL_887;
        auto ADD_896 = SUB_893 + MUL_884;
        auto ADD_899 = ADD_896 + MUL_887;
        auto ADD_882 = MUL_879 + MUL_881;
        auto SUB_907 = ADD_882 - MUL_884;
        auto ADD_910 = SUB_907 + MUL_887;
        auto ADD_885 = ADD_882 + MUL_884;
        auto SUB_888 = ADD_885 - MUL_887;
        auto MUL_934 = ADD_841 * 0.115975;
        auto MUL_939 = ADD_841 * MUL_934;
        auto MUL_929 = SUB_835 * 0.115975;
        auto MUL_937 = SUB_835 * MUL_929;
        auto ADD_941 = MUL_937 + MUL_939;
        auto MUL_944 = ADD_941 * 2.0;
        auto SUB_947 = 0.115975 - MUL_944;
        auto ADD_966 = ADD_814 + SUB_947;
        auto INPUT_6 = q[6];
        auto DIV_970 = INPUT_6 * 0.5;
        auto SIN_971 = DIV_970.sin();
        auto COS_977 = DIV_970.cos();
        auto MUL_994 = SUB_921 * COS_977;
        auto MUL_989 = SUB_921 * SIN_971;
        auto MUL_992 = ADD_910 * COS_977;
        auto ADD_993 = MUL_989 + MUL_992;
        auto MUL_997 = ADD_910 * SIN_971;
        auto SUB_998 = MUL_994 - MUL_997;
        auto MUL_986 = ADD_899 * COS_977;
        auto MUL_980 = ADD_899 * SIN_971;
        auto MUL_979 = SUB_888 * COS_977;
        auto ADD_981 = MUL_979 + MUL_980;
        auto MUL_4940 = ADD_981 * ADD_993;
        auto MUL_984 = SUB_888 * SIN_971;
        auto SUB_987 = MUL_986 - MUL_984;
        auto MUL_4936 = SUB_998 * SUB_987;
        auto ADD_4968 = MUL_4940 + MUL_4936;
        auto MUL_4970 = ADD_4968 * 2.0;
        auto MUL_4994 = MUL_4970 * 0.02;
        auto ADD_4999 = ADD_966 + MUL_4994;
        out.x[16] = ADD_4999;  // (223, 267)
        auto MUL_5016 = MUL_4970 * 0.04;
        auto SUB_5026 = ADD_966 - MUL_5016;
        out.x[17] = SUB_5026;  // (267, 269)
        auto MUL_5066 = ADD_4968 * 2.0;
        auto MUL_5091 = MUL_5066 * 0.02;
        auto MUL_5030 = ADD_993 * ADD_993;
        auto MUL_5029 = SUB_987 * SUB_987;
        auto ADD_5038 = MUL_5029 + MUL_5030;
        auto MUL_5041 = ADD_5038 * 2.0;
        auto SUB_5044 = 1.0 - MUL_5041;
        auto MUL_5078 = SUB_5044 * 0.01;
        auto SUB_5101 = MUL_5078 - MUL_5091;
        auto MUL_1059 = SUB_987 * 0.11355;
        auto MUL_1070 = SUB_998 * MUL_1059;
        auto MUL_1067 = ADD_981 * 0.11355;
        auto MUL_1072 = ADD_993 * MUL_1067;
        auto ADD_1073 = MUL_1070 + MUL_1072;
        auto MUL_1075 = ADD_1073 * 2.0;
        auto ADD_1097 = ADD_966 + MUL_1075;
        auto ADD_5104 = ADD_1097 + SUB_5101;
        out.x[18] = ADD_5104;  // (269, 286)
        auto INPUT_7 = q[7];
        auto DIV_1160 = INPUT_7 * 0.5;
        auto SIN_1161 = DIV_1160.sin();
        auto COS_1167 = DIV_1160.cos();
        auto MUL_1185 = SIN_1161 * 0.9238792;
        auto MUL_1191 = COS_1167 * 0.3826843;
        auto ADD_1192 = MUL_1185 + MUL_1191;
        auto MUL_1251 = ADD_1192 * 0.069;
        auto MUL_1255 = ADD_1192 * MUL_1251;
        auto MUL_1258 = MUL_1255 * 2.0;
        auto SUB_1261 = 0.069 - MUL_1258;
        auto ADD_1276 = SUB_1261 + 0.0640272;
        out.x[21] = ADD_1276;  // (286, 298)
        auto MUL_1230 = ADD_1192 * 0.7071068;
        auto MUL_1200 = SIN_1161 * 0.3826843;
        auto MUL_1194 = COS_1167 * 0.9238792;
        auto SUB_1201 = MUL_1194 - MUL_1200;
        auto MUL_1232 = SUB_1201 * 0.7071068;
        auto INPUT_9 = q[9];
        auto DIV_1442 = INPUT_9 * 0.5;
        auto SIN_1443 = DIV_1442.sin();
        auto COS_1449 = DIV_1442.cos();
        auto INPUT_8 = q[8];
        auto DIV_1282 = INPUT_8 * 0.5;
        auto SIN_1283 = DIV_1282.sin();
        auto COS_1289 = DIV_1282.cos();
        auto MUL_1308 = MUL_1230 * COS_1289;
        auto MUL_1293 = MUL_1230 * SIN_1283;
        auto MUL_1291 = MUL_1232 * COS_1289;
        auto SUB_1314 = MUL_1291 - MUL_1293;
        auto ADD_1295 = MUL_1291 + MUL_1293;
        auto MUL_1371 = SUB_1314 * 0.5;
        auto MUL_1373 = ADD_1295 * 0.5;
        auto SUB_1375 = MUL_1371 - MUL_1373;
        auto ADD_1387 = MUL_1371 + MUL_1373;
        auto MUL_1305 = MUL_1232 * SIN_1283;
        auto SUB_1303 = MUL_1305 - MUL_1308;
        auto ADD_1309 = MUL_1305 + MUL_1308;
        auto MUL_1380 = ADD_1309 * 0.5;
        auto MUL_1406 = ADD_1309 * 0.102;
        auto MUL_1411 = ADD_1309 * MUL_1406;
        auto MUL_1377 = SUB_1303 * 0.5;
        auto SUB_1390 = ADD_1387 - MUL_1377;
        auto SUB_1393 = SUB_1390 - MUL_1380;
        auto SUB_1378 = SUB_1375 - MUL_1377;
        auto ADD_1381 = SUB_1378 + MUL_1380;
        auto ADD_1366 = ADD_1387 + MUL_1377;
        auto ADD_1369 = ADD_1366 + MUL_1380;
        auto ADD_1354 = SUB_1375 + MUL_1377;
        auto SUB_1357 = ADD_1354 - MUL_1380;
        auto MUL_1466 = SUB_1393 * COS_1449;
        auto MUL_1461 = SUB_1393 * SIN_1443;
        auto MUL_1464 = ADD_1381 * COS_1449;
        auto ADD_1465 = MUL_1461 + MUL_1464;
        auto MUL_5262 = ADD_1465 * ADD_1465;
        auto MUL_1469 = ADD_1381 * SIN_1443;
        auto SUB_1470 = MUL_1466 - MUL_1469;
        auto MUL_1458 = ADD_1369 * COS_1449;
        auto MUL_1452 = ADD_1369 * SIN_1443;
        auto MUL_1451 = SUB_1357 * COS_1449;
        auto ADD_1453 = MUL_1451 + MUL_1452;
        auto MUL_5268 = ADD_1453 * ADD_1465;
        auto MUL_1456 = SUB_1357 * SIN_1443;
        auto SUB_1459 = MUL_1458 - MUL_1456;
        auto MUL_5264 = SUB_1470 * SUB_1459;
        auto ADD_5296 = MUL_5268 + MUL_5264;
        auto MUL_5298 = ADD_5296 * 2.0;
        auto MUL_5328 = MUL_5298 * 0.22;
        auto MUL_5261 = SUB_1459 * SUB_1459;
        auto ADD_5270 = MUL_5261 + MUL_5262;
        auto MUL_5273 = ADD_5270 * 2.0;
        auto SUB_5276 = 1.0 - MUL_5273;
        auto MUL_5311 = SUB_5276 * 0.02;
        auto SUB_5333 = MUL_5328 - MUL_5311;
        auto MUL_1401 = SUB_1303 * 0.102;
        auto MUL_1409 = SUB_1303 * MUL_1401;
        auto ADD_1413 = MUL_1409 + MUL_1411;
        auto MUL_1416 = ADD_1413 * 2.0;
        auto SUB_1419 = 0.102 - MUL_1416;
        auto ADD_1438 = ADD_1276 + SUB_1419;
        out.x[24] = ADD_1438;  // (298, 365)
        auto ADD_5336 = ADD_1438 + SUB_5333;
        out.x[22] = ADD_5336;  // (365, 366)
        auto MUL_5358 = MUL_5298 * 0.11;
        auto MUL_5341 = SUB_5276 * 0.01;
        auto SUB_5363 = MUL_5358 - MUL_5341;
        auto ADD_5366 = ADD_1438 + SUB_5363;
        out.x[23] = ADD_5366;  // (366, 370)
        auto MUL_1583 = ADD_1465 * 0.069;
        auto MUL_1572 = SUB_1459 * 0.26242;
        auto MUL_1585 = SUB_1470 * MUL_1572;
        auto MUL_1578 = SUB_1459 * 0.069;
        auto MUL_1586 = SUB_1459 * MUL_1578;
        auto SUB_1588 = MUL_1585 - MUL_1586;
        auto MUL_1581 = ADD_1453 * 0.26242;
        auto SUB_1584 = MUL_1581 - MUL_1583;
        auto MUL_1589 = ADD_1465 * SUB_1584;
        auto ADD_1590 = SUB_1588 + MUL_1589;
        auto MUL_1592 = ADD_1590 * 2.0;
        auto ADD_1594 = MUL_1592 + 0.069;
        auto ADD_1619 = ADD_1438 + ADD_1594;
        out.x[25] = ADD_1619;  // (370, 383)
        auto MUL_1538 = SUB_1470 * 0.5;
        auto MUL_1552 = ADD_1465 * 0.5;
        auto MUL_1548 = SUB_1459 * 0.5;
        auto MUL_1542 = ADD_1453 * 0.5;
        auto SUB_1508 = MUL_1542 - MUL_1538;
        auto SUB_1513 = SUB_1508 - MUL_1548;
        auto ADD_1544 = MUL_1538 + MUL_1542;
        auto SUB_1550 = MUL_1548 - ADD_1544;
        auto ADD_1565 = ADD_1544 + MUL_1548;
        auto ADD_1570 = ADD_1565 + MUL_1552;
        auto ADD_1553 = SUB_1550 + MUL_1552;
        auto ADD_1530 = SUB_1508 + MUL_1548;
        auto SUB_1535 = ADD_1530 - MUL_1552;
        auto ADD_1518 = SUB_1513 + MUL_1552;
        auto INPUT_10 = q[10];
        auto DIV_1623 = INPUT_10 * 0.5;
        auto SIN_1624 = DIV_1623.sin();
        auto COS_1630 = DIV_1623.cos();
        auto MUL_1642 = ADD_1570 * SIN_1624;
        auto MUL_1645 = ADD_1553 * COS_1630;
        auto ADD_1646 = MUL_1642 + MUL_1645;
        auto MUL_1739 = ADD_1646 * 0.10359;
        auto MUL_1744 = ADD_1646 * MUL_1739;
        auto MUL_1639 = SUB_1535 * COS_1630;
        auto MUL_1637 = ADD_1518 * SIN_1624;
        auto SUB_1640 = MUL_1639 - MUL_1637;
        auto MUL_1734 = SUB_1640 * 0.10359;
        auto MUL_1742 = SUB_1640 * MUL_1734;
        auto ADD_1746 = MUL_1742 + MUL_1744;
        auto MUL_1749 = ADD_1746 * 2.0;
        auto SUB_1752 = 0.10359 - MUL_1749;
        auto ADD_1771 = ADD_1619 + SUB_1752;
        out.x[26] = ADD_1771;  // (383, 415)
        auto MUL_1692 = ADD_1646 * 0.5;
        auto MUL_1689 = SUB_1640 * 0.5;
        auto MUL_1647 = ADD_1570 * COS_1630;
        auto MUL_1650 = ADD_1553 * SIN_1624;
        auto SUB_1651 = MUL_1647 - MUL_1650;
        auto MUL_1684 = SUB_1651 * 0.5;
        auto MUL_1633 = SUB_1535 * SIN_1624;
        auto MUL_1632 = ADD_1518 * COS_1630;
        auto ADD_1634 = MUL_1632 + MUL_1633;
        auto MUL_1686 = ADD_1634 * 0.5;
        auto SUB_1698 = MUL_1684 - MUL_1686;
        auto SUB_1723 = SUB_1698 - MUL_1689;
        auto SUB_1726 = SUB_1723 - MUL_1692;
        auto ADD_1701 = SUB_1698 + MUL_1689;
        auto ADD_1704 = ADD_1701 + MUL_1692;
        auto ADD_1687 = MUL_1684 + MUL_1686;
        auto SUB_1712 = ADD_1687 - MUL_1689;
        auto ADD_1715 = SUB_1712 + MUL_1692;
        auto ADD_1690 = ADD_1687 + MUL_1689;
        auto SUB_1693 = ADD_1690 - MUL_1692;
        auto INPUT_11 = q[11];
        auto DIV_1775 = INPUT_11 * 0.5;
        auto SIN_1776 = DIV_1775.sin();
        auto COS_1782 = DIV_1775.cos();
        auto MUL_1799 = SUB_1726 * COS_1782;
        auto MUL_1794 = SUB_1726 * SIN_1776;
        auto MUL_1797 = ADD_1715 * COS_1782;
        auto ADD_1798 = MUL_1794 + MUL_1797;
        auto MUL_1802 = ADD_1715 * SIN_1776;
        auto SUB_1803 = MUL_1799 - MUL_1802;
        auto MUL_1791 = ADD_1704 * COS_1782;
        auto MUL_1785 = ADD_1704 * SIN_1776;
        auto MUL_1784 = SUB_1693 * COS_1782;
        auto ADD_1786 = MUL_1784 + MUL_1785;
        auto MUL_5460 = ADD_1786 * ADD_1798;
        auto MUL_1789 = SUB_1693 * SIN_1776;
        auto SUB_1792 = MUL_1791 - MUL_1789;
        auto MUL_5456 = SUB_1803 * SUB_1792;
        auto ADD_5488 = MUL_5460 + MUL_5456;
        auto MUL_5490 = ADD_5488 * 2.0;
        auto MUL_5532 = MUL_5490 * 0.22;
        auto ADD_5537 = ADD_1771 + MUL_5532;
        out.x[27] = ADD_5537;  // (415, 457)
        auto MUL_5553 = MUL_5490 * 0.11;
        auto ADD_5558 = ADD_1771 + MUL_5553;
        out.x[28] = ADD_5558;  // (457, 459)
        auto MUL_1837 = SUB_1803 * 0.5;
        auto MUL_1849 = ADD_1798 * 0.5;
        auto MUL_1916 = ADD_1798 * 0.01;
        auto MUL_1844 = SUB_1792 * 0.5;
        auto MUL_1905 = SUB_1792 * 0.2707;
        auto MUL_1918 = SUB_1803 * MUL_1905;
        auto MUL_1911 = SUB_1792 * 0.01;
        auto MUL_1919 = SUB_1792 * MUL_1911;
        auto SUB_1921 = MUL_1918 - MUL_1919;
        auto MUL_1840 = ADD_1786 * 0.5;
        auto SUB_1841 = MUL_1840 - MUL_1837;
        auto SUB_1846 = SUB_1841 - MUL_1844;
        auto ADD_1863 = SUB_1841 + MUL_1844;
        auto SUB_1868 = ADD_1863 - MUL_1849;
        auto ADD_1851 = SUB_1846 + MUL_1849;
        auto ADD_1877 = MUL_1837 + MUL_1840;
        auto SUB_1883 = MUL_1844 - ADD_1877;
        auto ADD_1898 = ADD_1877 + MUL_1844;
        auto ADD_1903 = ADD_1898 + MUL_1849;
        auto ADD_1886 = SUB_1883 + MUL_1849;
        auto MUL_1914 = ADD_1786 * 0.2707;
        auto SUB_1917 = MUL_1914 - MUL_1916;
        auto MUL_1922 = ADD_1798 * SUB_1917;
        auto ADD_1923 = SUB_1921 + MUL_1922;
        auto MUL_1925 = ADD_1923 * 2.0;
        auto ADD_1927 = MUL_1925 + 0.01;
        auto ADD_1952 = ADD_1771 + ADD_1927;
        auto INPUT_12 = q[12];
        auto DIV_1956 = INPUT_12 * 0.5;
        auto SIN_1957 = DIV_1956.sin();
        auto COS_1963 = DIV_1956.cos();
        auto MUL_1980 = ADD_1903 * COS_1963;
        auto MUL_1975 = ADD_1903 * SIN_1957;
        auto MUL_1978 = ADD_1886 * COS_1963;
        auto ADD_1979 = MUL_1975 + MUL_1978;
        auto MUL_1983 = ADD_1886 * SIN_1957;
        auto SUB_1984 = MUL_1980 - MUL_1983;
        auto MUL_1972 = SUB_1868 * COS_1963;
        auto MUL_1966 = SUB_1868 * SIN_1957;
        auto MUL_1965 = ADD_1851 * COS_1963;
        auto ADD_1967 = MUL_1965 + MUL_1966;
        auto MUL_5568 = ADD_1967 * ADD_1979;
        auto MUL_1970 = ADD_1851 * SIN_1957;
        auto SUB_1973 = MUL_1972 - MUL_1970;
        auto MUL_5564 = SUB_1984 * SUB_1973;
        auto ADD_5596 = MUL_5568 + MUL_5564;
        auto MUL_5598 = ADD_5596 * 2.0;
        auto MUL_5622 = MUL_5598 * 0.03;
        auto ADD_5627 = ADD_1952 + MUL_5622;
        out.x[29] = ADD_5627;  // (459, 508)
        auto SUB_5654 = ADD_1952 - MUL_5622;
        out.x[30] = SUB_5654;  // (508, 509)
        auto MUL_2017 = SUB_1984 * 0.5;
        auto MUL_2025 = ADD_1979 * 0.5;
        auto MUL_2072 = ADD_1979 * 0.115975;
        auto MUL_2077 = ADD_1979 * MUL_2072;
        auto MUL_2022 = SUB_1973 * 0.5;
        auto MUL_2067 = SUB_1973 * 0.115975;
        auto MUL_2075 = SUB_1973 * MUL_2067;
        auto ADD_2079 = MUL_2075 + MUL_2077;
        auto MUL_2082 = ADD_2079 * 2.0;
        auto SUB_2085 = 0.115975 - MUL_2082;
        auto ADD_2104 = ADD_1952 + SUB_2085;
        auto MUL_2019 = ADD_1967 * 0.5;
        auto SUB_2031 = MUL_2017 - MUL_2019;
        auto SUB_2056 = SUB_2031 - MUL_2022;
        auto SUB_2059 = SUB_2056 - MUL_2025;
        auto ADD_2034 = SUB_2031 + MUL_2022;
        auto ADD_2037 = ADD_2034 + MUL_2025;
        auto ADD_2020 = MUL_2017 + MUL_2019;
        auto SUB_2045 = ADD_2020 - MUL_2022;
        auto ADD_2048 = SUB_2045 + MUL_2025;
        auto ADD_2023 = ADD_2020 + MUL_2022;
        auto SUB_2026 = ADD_2023 - MUL_2025;
        auto INPUT_13 = q[13];
        auto DIV_2108 = INPUT_13 * 0.5;
        auto SIN_2109 = DIV_2108.sin();
        auto COS_2115 = DIV_2108.cos();
        auto MUL_2132 = SUB_2059 * COS_2115;
        auto MUL_2127 = SUB_2059 * SIN_2109;
        auto MUL_2130 = ADD_2048 * COS_2115;
        auto ADD_2131 = MUL_2127 + MUL_2130;
        auto MUL_2135 = ADD_2048 * SIN_2109;
        auto SUB_2136 = MUL_2132 - MUL_2135;
        auto MUL_2124 = ADD_2037 * COS_2115;
        auto MUL_2118 = ADD_2037 * SIN_2109;
        auto MUL_2117 = SUB_2026 * COS_2115;
        auto ADD_2119 = MUL_2117 + MUL_2118;
        auto MUL_5664 = ADD_2119 * ADD_2131;
        auto MUL_2122 = SUB_2026 * SIN_2109;
        auto SUB_2125 = MUL_2124 - MUL_2122;
        auto MUL_5660 = SUB_2136 * SUB_2125;
        auto ADD_5692 = MUL_5664 + MUL_5660;
        auto MUL_5694 = ADD_5692 * 2.0;
        auto MUL_5718 = MUL_5694 * 0.02;
        auto ADD_5723 = ADD_2104 + MUL_5718;
        out.x[31] = ADD_5723;  // (509, 553)
        auto MUL_5740 = MUL_5694 * 0.04;
        auto SUB_5750 = ADD_2104 - MUL_5740;
        out.x[32] = SUB_5750;  // (553, 555)
        auto MUL_5790 = ADD_5692 * 2.0;
        auto MUL_5815 = MUL_5790 * 0.02;
        auto MUL_5754 = ADD_2131 * ADD_2131;
        auto MUL_5753 = SUB_2125 * SUB_2125;
        auto ADD_5762 = MUL_5753 + MUL_5754;
        auto MUL_5765 = ADD_5762 * 2.0;
        auto SUB_5768 = 1.0 - MUL_5765;
        auto MUL_5802 = SUB_5768 * 0.01;
        auto SUB_5825 = MUL_5802 - MUL_5815;
        auto MUL_2197 = SUB_2125 * 0.11355;
        auto MUL_2208 = SUB_2136 * MUL_2197;
        auto MUL_2205 = ADD_2119 * 0.11355;
        auto MUL_2210 = ADD_2131 * MUL_2205;
        auto ADD_2211 = MUL_2208 + MUL_2210;
        auto MUL_2213 = ADD_2211 * 2.0;
        auto ADD_2235 = ADD_2104 + MUL_2213;
        auto ADD_5828 = ADD_2235 + SUB_5825;
        out.x[33] = ADD_5828;  // (555, 572)
        auto MUL_5833 = SUB_2136 * ADD_2131;
        auto MUL_5837 = ADD_2119 * SUB_2125;
        auto SUB_5853 = MUL_5837 - MUL_5833;
        auto MUL_5855 = SUB_5853 * 2.0;
        auto MUL_5886 = MUL_5855 * 0.02;
        auto MUL_2326 = SUB_2125 * 0.025;
        auto MUL_2337 = SUB_2136 * MUL_2326;
        auto MUL_2334 = ADD_2119 * 0.025;
        auto MUL_2339 = ADD_2131 * MUL_2334;
        auto ADD_2340 = MUL_2337 + MUL_2339;
        auto MUL_2342 = ADD_2340 * 2.0;
        auto ADD_2364 = ADD_2235 + MUL_2342;
        auto ADD_5897 = ADD_2364 + MUL_5886;
        out.x[35] = ADD_5897;  // (572, 585)
        auto SUB_5924 = ADD_2364 - MUL_5886;
        out.x[36] = SUB_5924;  // (585, 586)
        auto MUL_5964 = ADD_5692 * 2.0;
        auto MUL_5951 = SUB_5853 * 2.0;
        auto MUL_5939 = ADD_5762 * 2.0;
        auto SUB_5942 = 1.0 - MUL_5939;
        auto MUL_2455 = SUB_2125 * 0.02;
        auto MUL_2464 = ADD_2119 * 0.02;
        auto MUL_2470 = ADD_2131 * MUL_2464;
        auto MUL_2457 = ADD_2131 * 0.069333;
        auto SUB_2458 = MUL_2455 - MUL_2457;
        auto MUL_2467 = SUB_2136 * SUB_2458;
        auto MUL_2460 = ADD_2119 * 0.069333;
        auto MUL_2468 = SUB_2125 * MUL_2460;
        auto ADD_2469 = MUL_2467 + MUL_2468;
        auto ADD_2471 = ADD_2469 + MUL_2470;
        auto MUL_2473 = ADD_2471 * 2.0;
        auto ADD_2496 = ADD_2364 + MUL_2473;
        auto MUL_5988 = MUL_5951 * 0.012;
        auto MUL_5994 = MUL_5964 * 0.008;
        auto MUL_5977 = SUB_5942 * 0.005;
        auto SUB_5999 = MUL_5988 - MUL_5977;
        auto ADD_6002 = SUB_5999 + MUL_5994;
        auto ADD_6005 = ADD_2496 + ADD_6002;
        out.x[37] = ADD_6005;  // (586, 608)
        auto ADD_6038 = MUL_5977 + MUL_5988;
        auto SUB_6044 = MUL_5994 - ADD_6038;
        auto ADD_6047 = ADD_2496 + SUB_6044;
        out.x[38] = ADD_6047;  // (608, 611)
        auto MUL_6087 = ADD_5692 * 2.0;
        auto MUL_6111 = MUL_6087 * 0.03;
        auto MUL_6074 = SUB_5853 * 2.0;
        auto MUL_6105 = MUL_6074 * 0.01725;
        auto ADD_6116 = MUL_6105 + MUL_6111;
        auto ADD_6119 = ADD_2496 + ADD_6116;
        out.x[39] = ADD_6119;  // (611, 617)
        auto MUL_6135 = MUL_6087 * 0.05;
        auto ADD_6140 = MUL_6105 + MUL_6135;
        auto ADD_6143 = ADD_2496 + ADD_6140;
        out.x[40] = ADD_6143;  // (617, 620)
        auto MUL_6159 = MUL_6087 * 0.07;
        auto ADD_6164 = MUL_6105 + MUL_6159;
        auto ADD_6167 = ADD_2496 + ADD_6164;
        out.x[41] = ADD_6167;  // (620, 623)
        auto MUL_6194 = SUB_5853 * 2.0;
        auto MUL_6182 = ADD_5762 * 2.0;
        auto SUB_6185 = 1.0 - MUL_6182;
        auto MUL_6219 = SUB_6185 * 0.01;
        auto MUL_2707 = ADD_2131 * 0.01725;
        auto MUL_2710 = ADD_2119 * 0.01725;
        auto MUL_2718 = SUB_2125 * MUL_2710;
        auto MUL_6207 = ADD_5692 * 2.0;
        auto MUL_6238 = MUL_6207 * 0.005;
        auto MUL_2705 = SUB_2125 * 0.1127;
        auto SUB_2708 = MUL_2705 - MUL_2707;
        auto MUL_2717 = SUB_2136 * SUB_2708;
        auto ADD_2719 = MUL_2717 + MUL_2718;
        auto MUL_2714 = ADD_2119 * 0.1127;
        auto MUL_2720 = ADD_2131 * MUL_2714;
        auto ADD_2721 = ADD_2719 + MUL_2720;
        auto MUL_2723 = ADD_2721 * 2.0;
        auto ADD_2746 = ADD_2496 + MUL_2723;
        auto MUL_6226 = MUL_6194 * 0.0045;
        auto SUB_6248 = MUL_6219 - MUL_6226;
        auto SUB_6251 = SUB_6248 - MUL_6238;
        auto ADD_6254 = ADD_2746 + SUB_6251;
        out.x[42] = ADD_6254;  // (623, 645)
        auto ADD_6293 = MUL_6219 + MUL_6226;
        auto ADD_6299 = ADD_6293 + MUL_6238;
        auto SUB_6305 = ADD_2746 - ADD_6299;
        out.x[43] = SUB_6305;  // (645, 648)
        auto MUL_6334 = MUL_6207 * 0.025;
        auto ADD_6350 = ADD_6293 + MUL_6334;
        auto SUB_6356 = ADD_2746 - ADD_6350;
        out.x[44] = SUB_6356;  // (648, 651)
        auto SUB_6392 = SUB_6248 - MUL_6334;
        auto ADD_6395 = ADD_2746 + SUB_6392;
        out.x[45] = ADD_6395;  // (651, 653)
        auto ADD_2842 = MUL_2455 + MUL_2457;
        auto MUL_2853 = SUB_2136 * ADD_2842;
        auto SUB_2856 = MUL_2853 - MUL_2468;
        auto ADD_2858 = SUB_2856 + MUL_2470;
        auto MUL_2860 = ADD_2858 * 2.0;
        auto ADD_2888 = ADD_2364 + MUL_2860;
        auto MUL_6435 = ADD_5692 * 2.0;
        auto MUL_6459 = MUL_6435 * 0.008;
        auto MUL_6422 = SUB_5853 * 2.0;
        auto MUL_6453 = MUL_6422 * 0.01;
        auto MUL_6410 = ADD_5762 * 2.0;
        auto SUB_6413 = 1.0 - MUL_6410;
        auto MUL_6447 = SUB_6413 * 0.005;
        auto ADD_6464 = MUL_6447 + MUL_6453;
        auto ADD_6467 = ADD_6464 + MUL_6459;
        auto ADD_6470 = ADD_2888 + ADD_6467;
        out.x[46] = ADD_6470;  // (653, 669)
        auto SUB_6497 = MUL_6447 - MUL_6453;
        auto ADD_6500 = SUB_6497 + MUL_6459;
        auto ADD_6503 = ADD_2888 + ADD_6500;
        out.x[47] = ADD_6503;  // (669, 672)
        auto MUL_6543 = ADD_5692 * 2.0;
        auto MUL_6573 = MUL_6543 * 0.03;
        auto MUL_6530 = SUB_5853 * 2.0;
        auto MUL_6562 = MUL_6530 * 0.01725;
        auto SUB_6578 = MUL_6573 - MUL_6562;
        auto ADD_6581 = ADD_2888 + SUB_6578;
        out.x[48] = ADD_6581;  // (672, 678)
        auto MUL_6603 = MUL_6543 * 0.05;
        auto SUB_6608 = MUL_6603 - MUL_6562;
        auto ADD_6611 = ADD_2888 + SUB_6608;
        out.x[49] = ADD_6611;  // (678, 681)
        auto MUL_6633 = MUL_6543 * 0.07;
        auto SUB_6638 = MUL_6633 - MUL_6562;
        auto ADD_6641 = ADD_2888 + SUB_6638;
        out.x[50] = ADD_6641;  // (681, 684)
        auto ADD_3102 = MUL_2705 + MUL_2707;
        auto MUL_3113 = SUB_2136 * ADD_3102;
        auto SUB_3116 = MUL_3113 - MUL_2718;
        auto ADD_3118 = SUB_3116 + MUL_2720;
        auto MUL_3120 = ADD_3118 * 2.0;
        auto ADD_3148 = ADD_2888 + MUL_3120;
        auto MUL_6681 = ADD_5692 * 2.0;
        auto MUL_6706 = MUL_6681 * 0.005;
        auto MUL_6668 = SUB_5853 * 2.0;
        auto MUL_6699 = MUL_6668 * 0.0045;
        auto MUL_6656 = ADD_5762 * 2.0;
        auto SUB_6659 = 1.0 - MUL_6656;
        auto MUL_6693 = SUB_6659 * 0.01;
        auto ADD_6716 = MUL_6693 + MUL_6699;
        auto SUB_6719 = ADD_6716 - MUL_6706;
        auto ADD_6722 = ADD_3148 + SUB_6719;
        out.x[51] = ADD_6722;  // (684, 700)
        auto SUB_6755 = MUL_6699 - MUL_6693;
        auto SUB_6758 = SUB_6755 - MUL_6706;
        auto ADD_6761 = ADD_3148 + SUB_6758;
        out.x[52] = ADD_6761;  // (700, 703)
        auto MUL_6784 = MUL_6681 * 0.025;
        auto SUB_6797 = SUB_6755 - MUL_6784;
        auto ADD_6800 = ADD_3148 + SUB_6797;
        out.x[53] = ADD_6800;  // (703, 706)
        auto SUB_6830 = ADD_6716 - MUL_6784;
        auto ADD_6833 = ADD_3148 + SUB_6830;
        out.x[54] = ADD_6833;  // (706, 708)
        auto MUL_6886 = SUB_998 * ADD_993;
        auto MUL_3368 = SUB_987 * 0.025;
        auto MUL_3379 = SUB_998 * MUL_3368;
        auto MUL_6890 = ADD_981 * SUB_987;
        auto SUB_6906 = MUL_6890 - MUL_6886;
        auto MUL_6908 = SUB_6906 * 2.0;
        auto MUL_6939 = MUL_6908 * 0.02;
        auto MUL_3376 = ADD_981 * 0.025;
        auto MUL_3381 = ADD_993 * MUL_3376;
        auto ADD_3382 = MUL_3379 + MUL_3381;
        auto MUL_3384 = ADD_3382 * 2.0;
        auto ADD_3406 = ADD_1097 + MUL_3384;
        auto ADD_6950 = ADD_3406 + MUL_6939;
        out.x[55] = ADD_6950;  // (708, 721)
        auto SUB_6977 = ADD_3406 - MUL_6939;
        out.x[56] = SUB_6977;  // (721, 722)
        auto MUL_7017 = ADD_4968 * 2.0;
        auto MUL_7047 = MUL_7017 * 0.008;
        auto MUL_7004 = SUB_6906 * 2.0;
        auto MUL_7041 = MUL_7004 * 0.012;
        auto MUL_6992 = ADD_5038 * 2.0;
        auto SUB_6995 = 1.0 - MUL_6992;
        auto MUL_7030 = SUB_6995 * 0.005;
        auto SUB_7052 = MUL_7041 - MUL_7030;
        auto ADD_7055 = SUB_7052 + MUL_7047;
        auto MUL_3499 = ADD_993 * 0.069333;
        auto MUL_3497 = SUB_987 * 0.02;
        auto SUB_3500 = MUL_3497 - MUL_3499;
        auto MUL_3509 = SUB_998 * SUB_3500;
        auto MUL_3502 = ADD_981 * 0.069333;
        auto MUL_3510 = SUB_987 * MUL_3502;
        auto ADD_3511 = MUL_3509 + MUL_3510;
        auto MUL_3506 = ADD_981 * 0.02;
        auto MUL_3512 = ADD_993 * MUL_3506;
        auto ADD_3513 = ADD_3511 + MUL_3512;
        auto MUL_3515 = ADD_3513 * 2.0;
        auto ADD_3538 = ADD_3406 + MUL_3515;
        auto ADD_7058 = ADD_3538 + ADD_7055;
        out.x[57] = ADD_7058;  // (722, 744)
        auto ADD_7091 = MUL_7030 + MUL_7041;
        auto SUB_7097 = MUL_7047 - ADD_7091;
        auto ADD_7100 = ADD_3538 + SUB_7097;
        out.x[58] = ADD_7100;  // (744, 747)
        auto MUL_7140 = ADD_4968 * 2.0;
        auto MUL_7164 = MUL_7140 * 0.03;
        auto MUL_7127 = SUB_6906 * 2.0;
        auto MUL_7158 = MUL_7127 * 0.01725;
        auto ADD_7169 = MUL_7158 + MUL_7164;
        auto ADD_7172 = ADD_3538 + ADD_7169;
        out.x[59] = ADD_7172;  // (747, 753)
        auto MUL_7188 = MUL_7140 * 0.05;
        auto ADD_7193 = MUL_7158 + MUL_7188;
        auto ADD_7196 = ADD_3538 + ADD_7193;
        out.x[60] = ADD_7196;  // (753, 756)
        auto MUL_7212 = MUL_7140 * 0.07;
        auto ADD_7217 = MUL_7158 + MUL_7212;
        auto ADD_7220 = ADD_3538 + ADD_7217;
        out.x[61] = ADD_7220;  // (756, 759)
        auto MUL_3749 = ADD_993 * 0.01725;
        auto MUL_3747 = SUB_987 * 0.1127;
        auto SUB_3750 = MUL_3747 - MUL_3749;
        auto MUL_3759 = SUB_998 * SUB_3750;
        auto MUL_3756 = ADD_981 * 0.1127;
        auto MUL_3762 = ADD_993 * MUL_3756;
        auto MUL_3752 = ADD_981 * 0.01725;
        auto MUL_3760 = SUB_987 * MUL_3752;
        auto ADD_3761 = MUL_3759 + MUL_3760;
        auto ADD_3763 = ADD_3761 + MUL_3762;
        auto MUL_3765 = ADD_3763 * 2.0;
        auto ADD_3788 = ADD_3538 + MUL_3765;
        auto MUL_7260 = ADD_4968 * 2.0;
        auto MUL_7291 = MUL_7260 * 0.005;
        auto MUL_7247 = SUB_6906 * 2.0;
        auto MUL_7279 = MUL_7247 * 0.0045;
        auto MUL_7235 = ADD_5038 * 2.0;
        auto SUB_7238 = 1.0 - MUL_7235;
        auto MUL_7272 = SUB_7238 * 0.01;
        auto SUB_7301 = MUL_7272 - MUL_7279;
        auto SUB_7304 = SUB_7301 - MUL_7291;
        auto ADD_7307 = ADD_3788 + SUB_7304;
        out.x[62] = ADD_7307;  // (759, 781)
        auto ADD_7346 = MUL_7272 + MUL_7279;
        auto ADD_7352 = ADD_7346 + MUL_7291;
        auto SUB_7358 = ADD_3788 - ADD_7352;
        out.x[63] = SUB_7358;  // (781, 784)
        auto MUL_7387 = MUL_7260 * 0.025;
        auto ADD_7403 = ADD_7346 + MUL_7387;
        auto SUB_7409 = ADD_3788 - ADD_7403;
        out.x[64] = SUB_7409;  // (784, 787)
        auto SUB_7445 = SUB_7301 - MUL_7387;
        auto ADD_7448 = ADD_3788 + SUB_7445;
        out.x[65] = ADD_7448;  // (787, 789)
        auto ADD_3884 = MUL_3497 + MUL_3499;
        auto MUL_3895 = SUB_998 * ADD_3884;
        auto SUB_3898 = MUL_3895 - MUL_3510;
        auto ADD_3900 = SUB_3898 + MUL_3512;
        auto MUL_3902 = ADD_3900 * 2.0;
        auto ADD_3930 = ADD_3406 + MUL_3902;
        auto MUL_7488 = ADD_4968 * 2.0;
        auto MUL_7512 = MUL_7488 * 0.008;
        auto MUL_7475 = SUB_6906 * 2.0;
        auto MUL_7506 = MUL_7475 * 0.01;
        auto MUL_7463 = ADD_5038 * 2.0;
        auto SUB_7466 = 1.0 - MUL_7463;
        auto MUL_7500 = SUB_7466 * 0.005;
        auto ADD_7517 = MUL_7500 + MUL_7506;
        auto ADD_7520 = ADD_7517 + MUL_7512;
        auto ADD_7523 = ADD_3930 + ADD_7520;
        out.x[66] = ADD_7523;  // (789, 805)
        auto SUB_7550 = MUL_7500 - MUL_7506;
        auto ADD_7553 = SUB_7550 + MUL_7512;
        auto ADD_7556 = ADD_3930 + ADD_7553;
        out.x[67] = ADD_7556;  // (805, 808)
        auto MUL_7596 = ADD_4968 * 2.0;
        auto MUL_7626 = MUL_7596 * 0.03;
        auto MUL_7583 = SUB_6906 * 2.0;
        auto MUL_7615 = MUL_7583 * 0.01725;
        auto SUB_7631 = MUL_7626 - MUL_7615;
        auto ADD_7634 = ADD_3930 + SUB_7631;
        out.x[68] = ADD_7634;  // (808, 814)
        auto MUL_7656 = MUL_7596 * 0.05;
        auto SUB_7661 = MUL_7656 - MUL_7615;
        auto ADD_7664 = ADD_3930 + SUB_7661;
        out.x[69] = ADD_7664;  // (814, 817)
        auto MUL_7686 = MUL_7596 * 0.07;
        auto SUB_7691 = MUL_7686 - MUL_7615;
        auto ADD_7694 = ADD_3930 + SUB_7691;
        out.x[70] = ADD_7694;  // (817, 820)
        auto ADD_4144 = MUL_3747 + MUL_3749;
        auto MUL_4155 = SUB_998 * ADD_4144;
        auto SUB_4158 = MUL_4155 - MUL_3760;
        auto ADD_4160 = SUB_4158 + MUL_3762;
        auto MUL_4162 = ADD_4160 * 2.0;
        auto ADD_4190 = ADD_3930 + MUL_4162;
        auto MUL_7734 = ADD_4968 * 2.0;
        auto MUL_7759 = MUL_7734 * 0.005;
        auto MUL_7721 = SUB_6906 * 2.0;
        auto MUL_7752 = MUL_7721 * 0.0045;
        auto MUL_7709 = ADD_5038 * 2.0;
        auto SUB_7712 = 1.0 - MUL_7709;
        auto MUL_7746 = SUB_7712 * 0.01;
        auto ADD_7769 = MUL_7746 + MUL_7752;
        auto SUB_7772 = ADD_7769 - MUL_7759;
        auto ADD_7775 = ADD_4190 + SUB_7772;
        out.x[71] = ADD_7775;  // (820, 836)
        auto SUB_7808 = MUL_7752 - MUL_7746;
        auto SUB_7811 = SUB_7808 - MUL_7759;
        auto ADD_7814 = ADD_4190 + SUB_7811;
        out.x[72] = ADD_7814;  // (836, 839)
        auto MUL_7837 = MUL_7734 * 0.025;
        auto SUB_7850 = SUB_7808 - MUL_7837;
        auto ADD_7853 = ADD_4190 + SUB_7850;
        out.x[73] = ADD_7853;  // (839, 842)
        auto SUB_7883 = ADD_7769 - MUL_7837;
        auto ADD_7886 = ADD_4190 + SUB_7883;
        out.x[74] = ADD_7886;  // (842, 844)
        auto MUL_124 = ADD_62 * MUL_112;
        auto MUL_128 = MUL_124 * 2.0;
        auto SUB_140 = MUL_128 - 0.2590274;
        out.y[6] = SUB_140;  // (844, 847)
        auto MUL_4539 = SUB_332 * ADD_327;
        auto MUL_4542 = SUB_332 * ADD_315;
        auto MUL_4545 = SUB_321 * ADD_327;
        auto SUB_4575 = MUL_4545 - MUL_4542;
        auto MUL_4577 = SUB_4575 * 2.0;
        auto MUL_4606 = MUL_4577 * 0.22;
        auto MUL_4543 = ADD_315 * SUB_321;
        auto ADD_4553 = MUL_4543 + MUL_4539;
        auto MUL_4555 = ADD_4553 * 2.0;
        auto MUL_4591 = MUL_4555 * 0.02;
        auto SUB_4610 = MUL_4606 - MUL_4591;
        auto MUL_283 = SUB_176 * MUL_268;
        auto MUL_284 = ADD_157 * MUL_263;
        auto SUB_285 = MUL_283 - MUL_284;
        auto MUL_288 = SUB_285 * 2.0;
        auto ADD_301 = SUB_140 + MUL_288;
        out.y[9] = ADD_301;  // (847, 863)
        auto ADD_4613 = ADD_301 + SUB_4610;
        out.y[7] = ADD_4613;  // (863, 864)
        auto MUL_4636 = MUL_4577 * 0.11;
        auto MUL_4621 = MUL_4555 * 0.01;
        auto SUB_4640 = MUL_4636 - MUL_4621;
        auto ADD_4643 = ADD_301 + SUB_4640;
        out.y[8] = ADD_4643;  // (864, 868)
        auto MUL_458 = SUB_332 * SUB_446;
        auto MUL_463 = ADD_327 * MUL_434;
        auto MUL_460 = ADD_315 * MUL_440;
        auto SUB_462 = MUL_460 - MUL_458;
        auto ADD_464 = SUB_462 + MUL_463;
        auto MUL_466 = ADD_464 * 2.0;
        auto ADD_482 = ADD_301 + MUL_466;
        out.y[10] = ADD_482;  // (868, 875)
        auto MUL_616 = SUB_513 * MUL_601;
        auto MUL_617 = ADD_496 * MUL_596;
        auto ADD_619 = MUL_616 + MUL_617;
        auto MUL_622 = ADD_619 * 2.0;
        auto ADD_634 = ADD_482 + MUL_622;
        out.y[11] = ADD_634;  // (875, 880)
        auto MUL_4734 = SUB_665 * ADD_648;
        auto MUL_4737 = SUB_654 * ADD_660;
        auto SUB_4767 = MUL_4737 - MUL_4734;
        auto MUL_4769 = SUB_4767 * 2.0;
        auto MUL_4810 = MUL_4769 * 0.22;
        auto ADD_4814 = ADD_634 + MUL_4810;
        out.y[12] = ADD_4814;  // (880, 886)
        auto MUL_4831 = MUL_4769 * 0.11;
        auto ADD_4835 = ADD_634 + MUL_4831;
        out.y[13] = ADD_4835;  // (886, 888)
        auto MUL_4842 = SUB_846 * ADD_829;
        auto MUL_4845 = SUB_835 * ADD_841;
        auto SUB_4875 = MUL_4845 - MUL_4842;
        auto MUL_4877 = SUB_4875 * 2.0;
        auto MUL_4900 = MUL_4877 * 0.03;
        auto MUL_791 = SUB_665 * SUB_779;
        auto MUL_796 = ADD_660 * MUL_767;
        auto MUL_793 = ADD_648 * MUL_773;
        auto SUB_795 = MUL_793 - MUL_791;
        auto ADD_797 = SUB_795 + MUL_796;
        auto MUL_799 = ADD_797 * 2.0;
        auto ADD_815 = ADD_634 + MUL_799;
        auto ADD_4904 = ADD_815 + MUL_4900;
        out.y[14] = ADD_4904;  // (888, 901)
        auto SUB_4931 = ADD_815 - MUL_4900;
        out.y[15] = SUB_4931;  // (901, 902)
        auto MUL_4938 = SUB_998 * ADD_981;
        auto MUL_4941 = SUB_987 * ADD_993;
        auto SUB_4971 = MUL_4941 - MUL_4938;
        auto MUL_4973 = SUB_4971 * 2.0;
        auto MUL_4996 = MUL_4973 * 0.02;
        auto MUL_949 = SUB_846 * MUL_934;
        auto MUL_950 = ADD_829 * MUL_929;
        auto ADD_952 = MUL_949 + MUL_950;
        auto MUL_955 = ADD_952 * 2.0;
        auto ADD_967 = ADD_815 + MUL_955;
        auto ADD_5000 = ADD_967 + MUL_4996;
        out.y[16] = ADD_5000;  // (902, 913)
        auto MUL_5020 = MUL_4973 * 0.04;
        auto SUB_5027 = ADD_967 - MUL_5020;
        out.y[17] = SUB_5027;  // (913, 915)
        auto ADD_5045 = MUL_6890 + MUL_6886;
        auto MUL_5069 = SUB_4971 * 2.0;
        auto MUL_5095 = MUL_5069 * 0.02;
        auto MUL_5047 = ADD_5045 * 2.0;
        auto MUL_5080 = MUL_5047 * 0.01;
        auto SUB_5102 = MUL_5080 - MUL_5095;
        auto MUL_1078 = SUB_998 * MUL_1067;
        auto MUL_1081 = ADD_993 * MUL_1059;
        auto SUB_1082 = MUL_1081 - MUL_1078;
        auto MUL_1084 = SUB_1082 * 2.0;
        auto ADD_1098 = ADD_967 + MUL_1084;
        auto ADD_5105 = ADD_1098 + SUB_5102;
        out.y[18] = ADD_5105;  // (915, 927)
        auto MUL_1263 = SUB_1201 * MUL_1251;
        auto MUL_1267 = MUL_1263 * 2.0;
        auto ADD_1278 = MUL_1267 + 0.2590274;
        out.y[21] = ADD_1278;  // (927, 930)
        auto MUL_5263 = SUB_1470 * ADD_1465;
        auto MUL_5266 = SUB_1470 * ADD_1453;
        auto MUL_5269 = SUB_1459 * ADD_1465;
        auto SUB_5299 = MUL_5269 - MUL_5266;
        auto MUL_5301 = SUB_5299 * 2.0;
        auto MUL_5330 = MUL_5301 * 0.22;
        auto MUL_5267 = ADD_1453 * SUB_1459;
        auto ADD_5277 = MUL_5267 + MUL_5263;
        auto MUL_5279 = ADD_5277 * 2.0;
        auto MUL_5315 = MUL_5279 * 0.02;
        auto SUB_5334 = MUL_5330 - MUL_5315;
        auto MUL_1421 = SUB_1314 * MUL_1406;
        auto MUL_1422 = ADD_1295 * MUL_1401;
        auto SUB_1423 = MUL_1421 - MUL_1422;
        auto MUL_1426 = SUB_1423 * 2.0;
        auto ADD_1439 = ADD_1278 + MUL_1426;
        out.y[24] = ADD_1439;  // (930, 946)
        auto ADD_5337 = ADD_1439 + SUB_5334;
        out.y[22] = ADD_5337;  // (946, 947)
        auto MUL_5360 = MUL_5301 * 0.11;
        auto MUL_5345 = MUL_5279 * 0.01;
        auto SUB_5364 = MUL_5360 - MUL_5345;
        auto ADD_5367 = ADD_1439 + SUB_5364;
        out.y[23] = ADD_5367;  // (947, 951)
        auto MUL_1596 = SUB_1470 * SUB_1584;
        auto MUL_1601 = ADD_1465 * MUL_1572;
        auto MUL_1598 = ADD_1453 * MUL_1578;
        auto SUB_1600 = MUL_1598 - MUL_1596;
        auto ADD_1602 = SUB_1600 + MUL_1601;
        auto MUL_1604 = ADD_1602 * 2.0;
        auto ADD_1620 = ADD_1439 + MUL_1604;
        out.y[25] = ADD_1620;  // (951, 958)
        auto MUL_1754 = SUB_1651 * MUL_1739;
        auto MUL_1755 = ADD_1634 * MUL_1734;
        auto ADD_1757 = MUL_1754 + MUL_1755;
        auto MUL_1760 = ADD_1757 * 2.0;
        auto ADD_1772 = ADD_1620 + MUL_1760;
        out.y[26] = ADD_1772;  // (958, 963)
        auto MUL_5458 = SUB_1803 * ADD_1786;
        auto MUL_5461 = SUB_1792 * ADD_1798;
        auto SUB_5491 = MUL_5461 - MUL_5458;
        auto MUL_5493 = SUB_5491 * 2.0;
        auto MUL_5534 = MUL_5493 * 0.22;
        auto ADD_5538 = ADD_1772 + MUL_5534;
        out.y[27] = ADD_5538;  // (963, 969)
        auto MUL_5555 = MUL_5493 * 0.11;
        auto ADD_5559 = ADD_1772 + MUL_5555;
        out.y[28] = ADD_5559;  // (969, 971)
        auto MUL_5566 = SUB_1984 * ADD_1967;
        auto MUL_5569 = SUB_1973 * ADD_1979;
        auto SUB_5599 = MUL_5569 - MUL_5566;
        auto MUL_5601 = SUB_5599 * 2.0;
        auto MUL_5624 = MUL_5601 * 0.03;
        auto MUL_1929 = SUB_1803 * SUB_1917;
        auto MUL_1934 = ADD_1798 * MUL_1905;
        auto MUL_1931 = ADD_1786 * MUL_1911;
        auto SUB_1933 = MUL_1931 - MUL_1929;
        auto ADD_1935 = SUB_1933 + MUL_1934;
        auto MUL_1937 = ADD_1935 * 2.0;
        auto ADD_1953 = ADD_1772 + MUL_1937;
        auto ADD_5628 = ADD_1953 + MUL_5624;
        out.y[29] = ADD_5628;  // (971, 984)
        auto SUB_5655 = ADD_1953 - MUL_5624;
        out.y[30] = SUB_5655;  // (984, 985)
        auto MUL_5662 = SUB_2136 * ADD_2119;
        auto MUL_5665 = SUB_2125 * ADD_2131;
        auto SUB_5695 = MUL_5665 - MUL_5662;
        auto MUL_5697 = SUB_5695 * 2.0;
        auto MUL_5720 = MUL_5697 * 0.02;
        auto MUL_2087 = SUB_1984 * MUL_2072;
        auto MUL_2088 = ADD_1967 * MUL_2067;
        auto ADD_2090 = MUL_2087 + MUL_2088;
        auto MUL_2093 = ADD_2090 * 2.0;
        auto ADD_2105 = ADD_1953 + MUL_2093;
        auto ADD_5724 = ADD_2105 + MUL_5720;
        out.y[31] = ADD_5724;  // (985, 996)
        auto MUL_5744 = MUL_5697 * 0.04;
        auto SUB_5751 = ADD_2105 - MUL_5744;
        out.y[32] = SUB_5751;  // (996, 998)
        auto ADD_5769 = MUL_5837 + MUL_5833;
        auto MUL_5793 = SUB_5695 * 2.0;
        auto MUL_5819 = MUL_5793 * 0.02;
        auto MUL_5771 = ADD_5769 * 2.0;
        auto MUL_5804 = MUL_5771 * 0.01;
        auto SUB_5826 = MUL_5804 - MUL_5819;
        auto MUL_2216 = SUB_2136 * MUL_2205;
        auto MUL_2219 = ADD_2131 * MUL_2197;
        auto SUB_2220 = MUL_2219 - MUL_2216;
        auto MUL_2222 = SUB_2220 * 2.0;
        auto ADD_2236 = ADD_2105 + MUL_2222;
        auto ADD_5829 = ADD_2236 + SUB_5826;
        out.y[33] = ADD_5829;  // (998, 1010)
        auto MUL_2345 = SUB_2136 * MUL_2334;
        auto MUL_2348 = ADD_2131 * MUL_2326;
        auto SUB_2349 = MUL_2348 - MUL_2345;
        auto MUL_2351 = SUB_2349 * 2.0;
        auto ADD_2365 = ADD_2236 + MUL_2351;
        auto MUL_5835 = ADD_2119 * ADD_2119;
        auto ADD_5856 = MUL_5754 + MUL_5835;
        auto MUL_5859 = ADD_5856 * 2.0;
        auto SUB_5862 = 1.0 - MUL_5859;
        auto MUL_5888 = SUB_5862 * 0.02;
        auto ADD_5898 = ADD_2365 + MUL_5888;
        out.y[35] = ADD_5898;  // (1010, 1021)
        auto SUB_5925 = ADD_2365 - MUL_5888;
        out.y[36] = SUB_5925;  // (1021, 1022)
        auto MUL_5967 = SUB_5695 * 2.0;
        auto MUL_5996 = MUL_5967 * 0.008;
        auto MUL_5955 = ADD_5856 * 2.0;
        auto SUB_5958 = 1.0 - MUL_5955;
        auto MUL_5990 = SUB_5958 * 0.012;
        auto MUL_5945 = ADD_5769 * 2.0;
        auto MUL_5981 = MUL_5945 * 0.005;
        auto SUB_6000 = MUL_5990 - MUL_5981;
        auto ADD_6003 = SUB_6000 + MUL_5996;
        auto MUL_2476 = SUB_2136 * MUL_2464;
        auto MUL_2481 = ADD_2131 * SUB_2458;
        auto MUL_2478 = ADD_2119 * MUL_2460;
        auto ADD_2479 = MUL_2476 + MUL_2478;
        auto SUB_2482 = MUL_2481 - ADD_2479;
        auto MUL_2484 = SUB_2482 * 2.0;
        auto ADD_2486 = MUL_2484 + 0.069333;
        auto ADD_2497 = ADD_2365 + ADD_2486;
        auto ADD_6006 = ADD_2497 + ADD_6003;
        out.y[37] = ADD_6006;  // (1022, 1040)
        auto ADD_6040 = MUL_5981 + MUL_5990;
        auto SUB_6045 = MUL_5996 - ADD_6040;
        auto ADD_6048 = ADD_2497 + SUB_6045;
        out.y[38] = ADD_6048;  // (1040, 1043)
        auto MUL_6090 = SUB_5695 * 2.0;
        auto MUL_6113 = MUL_6090 * 0.03;
        auto MUL_6078 = ADD_5856 * 2.0;
        auto SUB_6081 = 1.0 - MUL_6078;
        auto MUL_6107 = SUB_6081 * 0.01725;
        auto ADD_6117 = MUL_6107 + MUL_6113;
        auto ADD_6120 = ADD_2497 + ADD_6117;
        out.y[39] = ADD_6120;  // (1043, 1050)
        auto MUL_6137 = MUL_6090 * 0.05;
        auto ADD_6141 = MUL_6107 + MUL_6137;
        auto ADD_6144 = ADD_2497 + ADD_6141;
        out.y[40] = ADD_6144;  // (1050, 1053)
        auto MUL_6161 = MUL_6090 * 0.07;
        auto ADD_6165 = MUL_6107 + MUL_6161;
        auto ADD_6168 = ADD_2497 + ADD_6165;
        out.y[41] = ADD_6168;  // (1053, 1056)
        auto MUL_6198 = ADD_5856 * 2.0;
        auto SUB_6201 = 1.0 - MUL_6198;
        auto MUL_6188 = ADD_5769 * 2.0;
        auto MUL_6221 = MUL_6188 * 0.01;
        auto MUL_2726 = SUB_2136 * MUL_2714;
        auto MUL_2731 = ADD_2131 * SUB_2708;
        auto MUL_2728 = ADD_2119 * MUL_2710;
        auto ADD_2729 = MUL_2726 + MUL_2728;
        auto SUB_2732 = MUL_2731 - ADD_2729;
        auto MUL_2734 = SUB_2732 * 2.0;
        auto ADD_2736 = MUL_2734 + 0.01725;
        auto ADD_2747 = ADD_2497 + ADD_2736;
        auto MUL_6210 = SUB_5695 * 2.0;
        auto MUL_6242 = MUL_6210 * 0.005;
        auto MUL_6230 = SUB_6201 * 0.0045;
        auto SUB_6249 = MUL_6221 - MUL_6230;
        auto SUB_6252 = SUB_6249 - MUL_6242;
        auto ADD_6255 = ADD_2747 + SUB_6252;
        out.y[42] = ADD_6255;  // (1056, 1074)
        auto ADD_6295 = MUL_6221 + MUL_6230;
        auto ADD_6301 = ADD_6295 + MUL_6242;
        auto SUB_6306 = ADD_2747 - ADD_6301;
        out.y[43] = SUB_6306;  // (1074, 1077)
        auto MUL_6338 = MUL_6210 * 0.025;
        auto ADD_6352 = ADD_6295 + MUL_6338;
        auto SUB_6357 = ADD_2747 - ADD_6352;
        out.y[44] = SUB_6357;  // (1077, 1080)
        auto SUB_6393 = SUB_6249 - MUL_6338;
        auto ADD_6396 = ADD_2747 + SUB_6393;
        out.y[45] = ADD_6396;  // (1080, 1082)
        auto SUB_2867 = MUL_2478 - MUL_2476;
        auto MUL_2868 = ADD_2131 * ADD_2842;
        auto ADD_2869 = SUB_2867 + MUL_2868;
        auto MUL_2871 = ADD_2869 * 2.0;
        auto SUB_2874 = MUL_2871 - 0.069333;
        auto ADD_2889 = ADD_2365 + SUB_2874;
        auto MUL_6438 = SUB_5695 * 2.0;
        auto MUL_6461 = MUL_6438 * 0.008;
        auto MUL_6426 = ADD_5856 * 2.0;
        auto SUB_6429 = 1.0 - MUL_6426;
        auto MUL_6455 = SUB_6429 * 0.01;
        auto MUL_6416 = ADD_5769 * 2.0;
        auto MUL_6449 = MUL_6416 * 0.005;
        auto ADD_6465 = MUL_6449 + MUL_6455;
        auto ADD_6468 = ADD_6465 + MUL_6461;
        auto ADD_6471 = ADD_2889 + ADD_6468;
        out.y[46] = ADD_6471;  // (1082, 1098)
        auto SUB_6498 = MUL_6449 - MUL_6455;
        auto ADD_6501 = SUB_6498 + MUL_6461;
        auto ADD_6504 = ADD_2889 + ADD_6501;
        out.y[47] = ADD_6504;  // (1098, 1101)
        auto MUL_6546 = SUB_5695 * 2.0;
        auto MUL_6575 = MUL_6546 * 0.03;
        auto MUL_6534 = ADD_5856 * 2.0;
        auto SUB_6537 = 1.0 - MUL_6534;
        auto MUL_6566 = SUB_6537 * 0.01725;
        auto SUB_6579 = MUL_6575 - MUL_6566;
        auto ADD_6582 = ADD_2889 + SUB_6579;
        out.y[48] = ADD_6582;  // (1101, 1108)
        auto MUL_6605 = MUL_6546 * 0.05;
        auto SUB_6609 = MUL_6605 - MUL_6566;
        auto ADD_6612 = ADD_2889 + SUB_6609;
        out.y[49] = ADD_6612;  // (1108, 1111)
        auto MUL_6635 = MUL_6546 * 0.07;
        auto SUB_6639 = MUL_6635 - MUL_6566;
        auto ADD_6642 = ADD_2889 + SUB_6639;
        out.y[50] = ADD_6642;  // (1111, 1114)
        auto SUB_3127 = MUL_2728 - MUL_2726;
        auto MUL_3128 = ADD_2131 * ADD_3102;
        auto ADD_3129 = SUB_3127 + MUL_3128;
        auto MUL_3131 = ADD_3129 * 2.0;
        auto SUB_3134 = MUL_3131 - 0.01725;
        auto ADD_3149 = ADD_2889 + SUB_3134;
        auto MUL_6684 = SUB_5695 * 2.0;
        auto MUL_6710 = MUL_6684 * 0.005;
        auto MUL_6672 = ADD_5856 * 2.0;
        auto SUB_6675 = 1.0 - MUL_6672;
        auto MUL_6701 = SUB_6675 * 0.0045;
        auto MUL_6662 = ADD_5769 * 2.0;
        auto MUL_6695 = MUL_6662 * 0.01;
        auto ADD_6717 = MUL_6695 + MUL_6701;
        auto SUB_6720 = ADD_6717 - MUL_6710;
        auto ADD_6723 = ADD_3149 + SUB_6720;
        out.y[51] = ADD_6723;  // (1114, 1130)
        auto SUB_6756 = MUL_6701 - MUL_6695;
        auto SUB_6759 = SUB_6756 - MUL_6710;
        auto ADD_6762 = ADD_3149 + SUB_6759;
        out.y[52] = ADD_6762;  // (1130, 1133)
        auto MUL_6788 = MUL_6684 * 0.025;
        auto SUB_6798 = SUB_6756 - MUL_6788;
        auto ADD_6801 = ADD_3149 + SUB_6798;
        out.y[53] = ADD_6801;  // (1133, 1136)
        auto SUB_6831 = ADD_6717 - MUL_6788;
        auto ADD_6834 = ADD_3149 + SUB_6831;
        out.y[54] = ADD_6834;  // (1136, 1138)
        auto MUL_3387 = SUB_998 * MUL_3376;
        auto MUL_3390 = ADD_993 * MUL_3368;
        auto SUB_3391 = MUL_3390 - MUL_3387;
        auto MUL_3393 = SUB_3391 * 2.0;
        auto ADD_3407 = ADD_1098 + MUL_3393;
        auto MUL_6888 = ADD_981 * ADD_981;
        auto ADD_6909 = MUL_5030 + MUL_6888;
        auto MUL_6912 = ADD_6909 * 2.0;
        auto SUB_6915 = 1.0 - MUL_6912;
        auto MUL_6941 = SUB_6915 * 0.02;
        auto ADD_6951 = ADD_3407 + MUL_6941;
        out.y[55] = ADD_6951;  // (1138, 1149)
        auto SUB_6978 = ADD_3407 - MUL_6941;
        out.y[56] = SUB_6978;  // (1149, 1150)
        auto MUL_7020 = SUB_4971 * 2.0;
        auto MUL_7049 = MUL_7020 * 0.008;
        auto MUL_7008 = ADD_6909 * 2.0;
        auto SUB_7011 = 1.0 - MUL_7008;
        auto MUL_7043 = SUB_7011 * 0.012;
        auto MUL_6998 = ADD_5045 * 2.0;
        auto MUL_7034 = MUL_6998 * 0.005;
        auto SUB_7053 = MUL_7043 - MUL_7034;
        auto ADD_7056 = SUB_7053 + MUL_7049;
        auto MUL_3518 = SUB_998 * MUL_3506;
        auto MUL_3523 = ADD_993 * SUB_3500;
        auto MUL_3520 = ADD_981 * MUL_3502;
        auto ADD_3521 = MUL_3518 + MUL_3520;
        auto SUB_3524 = MUL_3523 - ADD_3521;
        auto MUL_3526 = SUB_3524 * 2.0;
        auto ADD_3528 = MUL_3526 + 0.069333;
        auto ADD_3539 = ADD_3407 + ADD_3528;
        auto ADD_7059 = ADD_3539 + ADD_7056;
        out.y[57] = ADD_7059;  // (1150, 1168)
        auto ADD_7093 = MUL_7034 + MUL_7043;
        auto SUB_7098 = MUL_7049 - ADD_7093;
        auto ADD_7101 = ADD_3539 + SUB_7098;
        out.y[58] = ADD_7101;  // (1168, 1171)
        auto MUL_7143 = SUB_4971 * 2.0;
        auto MUL_7166 = MUL_7143 * 0.03;
        auto MUL_7131 = ADD_6909 * 2.0;
        auto SUB_7134 = 1.0 - MUL_7131;
        auto MUL_7160 = SUB_7134 * 0.01725;
        auto ADD_7170 = MUL_7160 + MUL_7166;
        auto ADD_7173 = ADD_3539 + ADD_7170;
        out.y[59] = ADD_7173;  // (1171, 1178)
        auto MUL_7190 = MUL_7143 * 0.05;
        auto ADD_7194 = MUL_7160 + MUL_7190;
        auto ADD_7197 = ADD_3539 + ADD_7194;
        out.y[60] = ADD_7197;  // (1178, 1181)
        auto MUL_7214 = MUL_7143 * 0.07;
        auto ADD_7218 = MUL_7160 + MUL_7214;
        auto ADD_7221 = ADD_3539 + ADD_7218;
        out.y[61] = ADD_7221;  // (1181, 1184)
        auto MUL_3768 = SUB_998 * MUL_3756;
        auto MUL_3773 = ADD_993 * SUB_3750;
        auto MUL_3770 = ADD_981 * MUL_3752;
        auto ADD_3771 = MUL_3768 + MUL_3770;
        auto SUB_3774 = MUL_3773 - ADD_3771;
        auto MUL_3776 = SUB_3774 * 2.0;
        auto ADD_3778 = MUL_3776 + 0.01725;
        auto ADD_3789 = ADD_3539 + ADD_3778;
        auto MUL_7263 = SUB_4971 * 2.0;
        auto MUL_7295 = MUL_7263 * 0.005;
        auto MUL_7251 = ADD_6909 * 2.0;
        auto SUB_7254 = 1.0 - MUL_7251;
        auto MUL_7283 = SUB_7254 * 0.0045;
        auto MUL_7241 = ADD_5045 * 2.0;
        auto MUL_7274 = MUL_7241 * 0.01;
        auto SUB_7302 = MUL_7274 - MUL_7283;
        auto SUB_7305 = SUB_7302 - MUL_7295;
        auto ADD_7308 = ADD_3789 + SUB_7305;
        out.y[62] = ADD_7308;  // (1184, 1202)
        auto ADD_7348 = MUL_7274 + MUL_7283;
        auto ADD_7354 = ADD_7348 + MUL_7295;
        auto SUB_7359 = ADD_3789 - ADD_7354;
        out.y[63] = SUB_7359;  // (1202, 1205)
        auto MUL_7391 = MUL_7263 * 0.025;
        auto ADD_7405 = ADD_7348 + MUL_7391;
        auto SUB_7410 = ADD_3789 - ADD_7405;
        out.y[64] = SUB_7410;  // (1205, 1208)
        auto SUB_7446 = SUB_7302 - MUL_7391;
        auto ADD_7449 = ADD_3789 + SUB_7446;
        out.y[65] = ADD_7449;  // (1208, 1210)
        auto SUB_3909 = MUL_3520 - MUL_3518;
        auto MUL_3910 = ADD_993 * ADD_3884;
        auto ADD_3911 = SUB_3909 + MUL_3910;
        auto MUL_3913 = ADD_3911 * 2.0;
        auto SUB_3916 = MUL_3913 - 0.069333;
        auto ADD_3931 = ADD_3407 + SUB_3916;
        auto MUL_7491 = SUB_4971 * 2.0;
        auto MUL_7514 = MUL_7491 * 0.008;
        auto MUL_7479 = ADD_6909 * 2.0;
        auto SUB_7482 = 1.0 - MUL_7479;
        auto MUL_7508 = SUB_7482 * 0.01;
        auto MUL_7469 = ADD_5045 * 2.0;
        auto MUL_7502 = MUL_7469 * 0.005;
        auto ADD_7518 = MUL_7502 + MUL_7508;
        auto ADD_7521 = ADD_7518 + MUL_7514;
        auto ADD_7524 = ADD_3931 + ADD_7521;
        out.y[66] = ADD_7524;  // (1210, 1226)
        auto SUB_7551 = MUL_7502 - MUL_7508;
        auto ADD_7554 = SUB_7551 + MUL_7514;
        auto ADD_7557 = ADD_3931 + ADD_7554;
        out.y[67] = ADD_7557;  // (1226, 1229)
        auto MUL_7599 = SUB_4971 * 2.0;
        auto MUL_7628 = MUL_7599 * 0.03;
        auto MUL_7587 = ADD_6909 * 2.0;
        auto SUB_7590 = 1.0 - MUL_7587;
        auto MUL_7619 = SUB_7590 * 0.01725;
        auto SUB_7632 = MUL_7628 - MUL_7619;
        auto ADD_7635 = ADD_3931 + SUB_7632;
        out.y[68] = ADD_7635;  // (1229, 1236)
        auto MUL_7658 = MUL_7599 * 0.05;
        auto SUB_7662 = MUL_7658 - MUL_7619;
        auto ADD_7665 = ADD_3931 + SUB_7662;
        out.y[69] = ADD_7665;  // (1236, 1239)
        auto MUL_7688 = MUL_7599 * 0.07;
        auto SUB_7692 = MUL_7688 - MUL_7619;
        auto ADD_7695 = ADD_3931 + SUB_7692;
        out.y[70] = ADD_7695;  // (1239, 1242)
        auto SUB_4169 = MUL_3770 - MUL_3768;
        auto MUL_4170 = ADD_993 * ADD_4144;
        auto ADD_4171 = SUB_4169 + MUL_4170;
        auto MUL_4173 = ADD_4171 * 2.0;
        auto SUB_4176 = MUL_4173 - 0.01725;
        auto ADD_4191 = ADD_3931 + SUB_4176;
        auto MUL_7737 = SUB_4971 * 2.0;
        auto MUL_7763 = MUL_7737 * 0.005;
        auto MUL_7725 = ADD_6909 * 2.0;
        auto SUB_7728 = 1.0 - MUL_7725;
        auto MUL_7754 = SUB_7728 * 0.0045;
        auto MUL_7715 = ADD_5045 * 2.0;
        auto MUL_7748 = MUL_7715 * 0.01;
        auto ADD_7770 = MUL_7748 + MUL_7754;
        auto SUB_7773 = ADD_7770 - MUL_7763;
        auto ADD_7776 = ADD_4191 + SUB_7773;
        out.y[71] = ADD_7776;  // (1242, 1258)
        auto SUB_7809 = MUL_7754 - MUL_7748;
        auto SUB_7812 = SUB_7809 - MUL_7763;
        auto ADD_7815 = ADD_4191 + SUB_7812;
        out.y[72] = ADD_7815;  // (1258, 1261)
        auto MUL_7841 = MUL_7737 * 0.025;
        auto SUB_7851 = SUB_7809 - MUL_7841;
        auto ADD_7854 = ADD_4191 + SUB_7851;
        out.y[73] = ADD_7854;  // (1261, 1264)
        auto SUB_7884 = ADD_7770 - MUL_7841;
        auto ADD_7887 = ADD_4191 + SUB_7884;
        out.y[74] = ADD_7887;  // (1264, 1266)
        auto SUB_4556 = MUL_4544 - MUL_4540;
        auto MUL_4558 = SUB_4556 * 2.0;
        auto MUL_4595 = MUL_4558 * 0.02;
        auto MUL_4541 = ADD_315 * ADD_315;
        auto ADD_4578 = MUL_4537 + MUL_4541;
        auto MUL_4581 = ADD_4578 * 2.0;
        auto SUB_4584 = 1.0 - MUL_4581;
        auto MUL_4608 = SUB_4584 * 0.22;
        auto SUB_4611 = MUL_4608 - MUL_4595;
        auto MUL_290 = SUB_176 * MUL_263;
        auto MUL_292 = ADD_157 * MUL_268;
        auto ADD_293 = MUL_290 + MUL_292;
        auto MUL_297 = ADD_293 * 2.0;
        auto SUB_302 = 0.399976 - MUL_297;
        out.z[9] = SUB_302;  // (1266, 1280)
        auto ADD_4614 = SUB_302 + SUB_4611;
        out.z[7] = ADD_4614;  // (1280, 1281)
        auto MUL_4638 = SUB_4584 * 0.11;
        auto MUL_4625 = MUL_4558 * 0.01;
        auto SUB_4641 = MUL_4638 - MUL_4625;
        auto ADD_4644 = SUB_302 + SUB_4641;
        out.z[8] = ADD_4644;  // (1281, 1285)
        auto MUL_468 = SUB_332 * MUL_440;
        auto MUL_473 = SUB_321 * MUL_434;
        auto MUL_470 = ADD_315 * SUB_446;
        auto ADD_471 = MUL_468 + MUL_470;
        auto ADD_474 = ADD_471 + MUL_473;
        auto MUL_477 = ADD_474 * 2.0;
        auto SUB_480 = 0.26242 - MUL_477;
        auto ADD_483 = SUB_302 + SUB_480;
        out.z[10] = ADD_483;  // (1285, 1293)
        auto MUL_624 = SUB_513 * MUL_596;
        auto MUL_626 = ADD_496 * MUL_601;
        auto SUB_628 = MUL_626 - MUL_624;
        auto MUL_631 = SUB_628 * 2.0;
        auto ADD_635 = ADD_483 + MUL_631;
        out.z[11] = ADD_635;  // (1293, 1298)
        auto MUL_4729 = SUB_654 * SUB_654;
        auto MUL_4733 = ADD_648 * ADD_648;
        auto ADD_4770 = MUL_4729 + MUL_4733;
        auto MUL_4773 = ADD_4770 * 2.0;
        auto SUB_4776 = 1.0 - MUL_4773;
        auto MUL_4812 = SUB_4776 * 0.22;
        auto ADD_4815 = ADD_635 + MUL_4812;
        out.z[12] = ADD_4815;  // (1298, 1305)
        auto MUL_4833 = SUB_4776 * 0.11;
        auto ADD_4836 = ADD_635 + MUL_4833;
        out.z[13] = ADD_4836;  // (1305, 1307)
        auto MUL_4837 = SUB_835 * SUB_835;
        auto MUL_4841 = ADD_829 * ADD_829;
        auto ADD_4878 = MUL_4837 + MUL_4841;
        auto MUL_4881 = ADD_4878 * 2.0;
        auto SUB_4884 = 1.0 - MUL_4881;
        auto MUL_4902 = SUB_4884 * 0.03;
        auto MUL_801 = SUB_665 * MUL_773;
        auto MUL_806 = SUB_654 * MUL_767;
        auto MUL_803 = ADD_648 * SUB_779;
        auto ADD_804 = MUL_801 + MUL_803;
        auto ADD_807 = ADD_804 + MUL_806;
        auto MUL_810 = ADD_807 * 2.0;
        auto SUB_813 = 0.2707 - MUL_810;
        auto ADD_816 = ADD_635 + SUB_813;
        auto ADD_4905 = ADD_816 + MUL_4902;
        out.z[14] = ADD_4905;  // (1307, 1322)
        auto SUB_4932 = ADD_816 - MUL_4902;
        out.z[15] = SUB_4932;  // (1322, 1323)
        auto ADD_4974 = MUL_5029 + MUL_6888;
        auto MUL_4977 = ADD_4974 * 2.0;
        auto SUB_4980 = 1.0 - MUL_4977;
        auto MUL_4998 = SUB_4980 * 0.02;
        auto MUL_957 = SUB_846 * MUL_929;
        auto MUL_959 = ADD_829 * MUL_934;
        auto SUB_961 = MUL_959 - MUL_957;
        auto MUL_964 = SUB_961 * 2.0;
        auto ADD_968 = ADD_816 + MUL_964;
        auto ADD_5001 = ADD_968 + MUL_4998;
        out.z[16] = ADD_5001;  // (1323, 1333)
        auto MUL_5024 = SUB_4980 * 0.04;
        auto SUB_5028 = ADD_968 - MUL_5024;
        out.z[17] = SUB_5028;  // (1333, 1335)
        auto SUB_5048 = MUL_4940 - MUL_4936;
        auto MUL_5073 = ADD_4974 * 2.0;
        auto SUB_5076 = 1.0 - MUL_5073;
        auto MUL_5099 = SUB_5076 * 0.02;
        auto MUL_5050 = SUB_5048 * 2.0;
        auto MUL_5082 = MUL_5050 * 0.01;
        auto SUB_5103 = MUL_5082 - MUL_5099;
        auto MUL_1089 = SUB_987 * MUL_1059;
        auto MUL_1087 = ADD_981 * MUL_1067;
        auto ADD_1090 = MUL_1087 + MUL_1089;
        auto MUL_1093 = ADD_1090 * 2.0;
        auto SUB_1096 = 0.11355 - MUL_1093;
        auto ADD_1099 = ADD_968 + SUB_1096;
        auto ADD_5106 = ADD_1099 + SUB_5103;
        out.z[18] = ADD_5106;  // (1335, 1349)
        auto SUB_5280 = MUL_5268 - MUL_5264;
        auto MUL_5282 = SUB_5280 * 2.0;
        auto MUL_5319 = MUL_5282 * 0.02;
        auto MUL_5265 = ADD_1453 * ADD_1453;
        auto ADD_5302 = MUL_5261 + MUL_5265;
        auto MUL_5305 = ADD_5302 * 2.0;
        auto SUB_5308 = 1.0 - MUL_5305;
        auto MUL_5332 = SUB_5308 * 0.22;
        auto SUB_5335 = MUL_5332 - MUL_5319;
        auto MUL_1428 = SUB_1314 * MUL_1401;
        auto MUL_1430 = ADD_1295 * MUL_1406;
        auto ADD_1431 = MUL_1428 + MUL_1430;
        auto MUL_1435 = ADD_1431 * 2.0;
        auto SUB_1440 = 0.399976 - MUL_1435;
        out.z[24] = SUB_1440;  // (1349, 1363)
        auto ADD_5338 = SUB_1440 + SUB_5335;
        out.z[22] = ADD_5338;  // (1363, 1364)
        auto MUL_5362 = SUB_5308 * 0.11;
        auto MUL_5349 = MUL_5282 * 0.01;
        auto SUB_5365 = MUL_5362 - MUL_5349;
        auto ADD_5368 = SUB_1440 + SUB_5365;
        out.z[23] = ADD_5368;  // (1364, 1368)
        auto MUL_1606 = SUB_1470 * MUL_1578;
        auto MUL_1611 = SUB_1459 * MUL_1572;
        auto MUL_1608 = ADD_1453 * SUB_1584;
        auto ADD_1609 = MUL_1606 + MUL_1608;
        auto ADD_1612 = ADD_1609 + MUL_1611;
        auto MUL_1615 = ADD_1612 * 2.0;
        auto SUB_1618 = 0.26242 - MUL_1615;
        auto ADD_1621 = SUB_1440 + SUB_1618;
        out.z[25] = ADD_1621;  // (1368, 1376)
        auto MUL_1762 = SUB_1651 * MUL_1734;
        auto MUL_1764 = ADD_1634 * MUL_1739;
        auto SUB_1766 = MUL_1764 - MUL_1762;
        auto MUL_1769 = SUB_1766 * 2.0;
        auto ADD_1773 = ADD_1621 + MUL_1769;
        out.z[26] = ADD_1773;  // (1376, 1381)
        auto MUL_5453 = SUB_1792 * SUB_1792;
        auto MUL_5457 = ADD_1786 * ADD_1786;
        auto ADD_5494 = MUL_5453 + MUL_5457;
        auto MUL_5497 = ADD_5494 * 2.0;
        auto SUB_5500 = 1.0 - MUL_5497;
        auto MUL_5536 = SUB_5500 * 0.22;
        auto ADD_5539 = ADD_1773 + MUL_5536;
        out.z[27] = ADD_5539;  // (1381, 1388)
        auto MUL_5557 = SUB_5500 * 0.11;
        auto ADD_5560 = ADD_1773 + MUL_5557;
        out.z[28] = ADD_5560;  // (1388, 1390)
        auto MUL_5561 = SUB_1973 * SUB_1973;
        auto MUL_5565 = ADD_1967 * ADD_1967;
        auto ADD_5602 = MUL_5561 + MUL_5565;
        auto MUL_5605 = ADD_5602 * 2.0;
        auto SUB_5608 = 1.0 - MUL_5605;
        auto MUL_5626 = SUB_5608 * 0.03;
        auto MUL_1939 = SUB_1803 * MUL_1911;
        auto MUL_1944 = SUB_1792 * MUL_1905;
        auto MUL_1941 = ADD_1786 * SUB_1917;
        auto ADD_1942 = MUL_1939 + MUL_1941;
        auto ADD_1945 = ADD_1942 + MUL_1944;
        auto MUL_1948 = ADD_1945 * 2.0;
        auto SUB_1951 = 0.2707 - MUL_1948;
        auto ADD_1954 = ADD_1773 + SUB_1951;
        auto ADD_5629 = ADD_1954 + MUL_5626;
        out.z[29] = ADD_5629;  // (1390, 1405)
        auto SUB_5656 = ADD_1954 - MUL_5626;
        out.z[30] = SUB_5656;  // (1405, 1406)
        auto ADD_5698 = MUL_5753 + MUL_5835;
        auto MUL_5701 = ADD_5698 * 2.0;
        auto SUB_5704 = 1.0 - MUL_5701;
        auto MUL_5722 = SUB_5704 * 0.02;
        auto MUL_2095 = SUB_1984 * MUL_2067;
        auto MUL_2097 = ADD_1967 * MUL_2072;
        auto SUB_2099 = MUL_2097 - MUL_2095;
        auto MUL_2102 = SUB_2099 * 2.0;
        auto ADD_2106 = ADD_1954 + MUL_2102;
        auto ADD_5725 = ADD_2106 + MUL_5722;
        out.z[31] = ADD_5725;  // (1406, 1416)
        auto MUL_5748 = SUB_5704 * 0.04;
        auto SUB_5752 = ADD_2106 - MUL_5748;
        out.z[32] = SUB_5752;  // (1416, 1418)
        auto SUB_5772 = MUL_5664 - MUL_5660;
        auto MUL_5797 = ADD_5698 * 2.0;
        auto SUB_5800 = 1.0 - MUL_5797;
        auto MUL_5823 = SUB_5800 * 0.02;
        auto MUL_5774 = SUB_5772 * 2.0;
        auto MUL_5806 = MUL_5774 * 0.01;
        auto SUB_5827 = MUL_5806 - MUL_5823;
        auto MUL_2227 = SUB_2125 * MUL_2197;
        auto MUL_2225 = ADD_2119 * MUL_2205;
        auto ADD_2228 = MUL_2225 + MUL_2227;
        auto MUL_2231 = ADD_2228 * 2.0;
        auto SUB_2234 = 0.11355 - MUL_2231;
        auto ADD_2237 = ADD_2106 + SUB_2234;
        auto ADD_5830 = ADD_2237 + SUB_5827;
        out.z[33] = ADD_5830;  // (1418, 1432)
        auto ADD_5863 = MUL_5665 + MUL_5662;
        auto MUL_5865 = ADD_5863 * 2.0;
        auto MUL_5890 = MUL_5865 * 0.02;
        auto MUL_2356 = SUB_2125 * MUL_2326;
        auto MUL_2354 = ADD_2119 * MUL_2334;
        auto ADD_2357 = MUL_2354 + MUL_2356;
        auto MUL_2360 = ADD_2357 * 2.0;
        auto SUB_2363 = 0.025 - MUL_2360;
        auto ADD_2366 = ADD_2237 + SUB_2363;
        auto ADD_5899 = ADD_2366 + MUL_5890;
        out.z[35] = ADD_5899;  // (1432, 1442)
        auto SUB_5926 = ADD_2366 - MUL_5890;
        out.z[36] = SUB_5926;  // (1442, 1443)
        auto MUL_5971 = ADD_5698 * 2.0;
        auto SUB_5974 = 1.0 - MUL_5971;
        auto MUL_5998 = SUB_5974 * 0.008;
        auto MUL_5961 = ADD_5863 * 2.0;
        auto MUL_5992 = MUL_5961 * 0.012;
        auto MUL_5948 = SUB_5772 * 2.0;
        auto MUL_5985 = MUL_5948 * 0.005;
        auto SUB_6001 = MUL_5992 - MUL_5985;
        auto ADD_6004 = SUB_6001 + MUL_5998;
        auto MUL_2487 = SUB_2136 * MUL_2460;
        auto MUL_2490 = SUB_2125 * SUB_2458;
        auto MUL_2488 = ADD_2119 * MUL_2464;
        auto SUB_2489 = MUL_2487 - MUL_2488;
        auto SUB_2491 = SUB_2489 - MUL_2490;
        auto MUL_2493 = SUB_2491 * 2.0;
        auto ADD_2495 = MUL_2493 + 0.02;
        auto ADD_2498 = ADD_2366 + ADD_2495;
        auto ADD_6007 = ADD_2498 + ADD_6004;
        out.z[37] = ADD_6007;  // (1443, 1461)
        auto ADD_6042 = MUL_5985 + MUL_5992;
        auto SUB_6046 = MUL_5998 - ADD_6042;
        auto ADD_6049 = ADD_2498 + SUB_6046;
        out.z[38] = ADD_6049;  // (1461, 1464)
        auto MUL_6094 = ADD_5698 * 2.0;
        auto SUB_6097 = 1.0 - MUL_6094;
        auto MUL_6115 = SUB_6097 * 0.03;
        auto MUL_6084 = ADD_5863 * 2.0;
        auto MUL_6109 = MUL_6084 * 0.01725;
        auto ADD_6118 = MUL_6109 + MUL_6115;
        auto ADD_6121 = ADD_2498 + ADD_6118;
        out.z[39] = ADD_6121;  // (1464, 1471)
        auto MUL_6139 = SUB_6097 * 0.05;
        auto ADD_6142 = MUL_6109 + MUL_6139;
        auto ADD_6145 = ADD_2498 + ADD_6142;
        out.z[40] = ADD_6145;  // (1471, 1474)
        auto MUL_6163 = SUB_6097 * 0.07;
        auto ADD_6166 = MUL_6109 + MUL_6163;
        auto ADD_6169 = ADD_2498 + ADD_6166;
        out.z[41] = ADD_6169;  // (1474, 1477)
        auto MUL_6191 = SUB_5772 * 2.0;
        auto MUL_6223 = MUL_6191 * 0.01;
        auto MUL_2737 = SUB_2136 * MUL_2710;
        auto MUL_2740 = SUB_2125 * SUB_2708;
        auto MUL_2738 = ADD_2119 * MUL_2714;
        auto SUB_2739 = MUL_2737 - MUL_2738;
        auto SUB_2741 = SUB_2739 - MUL_2740;
        auto MUL_2743 = SUB_2741 * 2.0;
        auto ADD_2745 = MUL_2743 + 0.1127;
        auto ADD_2748 = ADD_2498 + ADD_2745;
        auto MUL_6214 = ADD_5698 * 2.0;
        auto SUB_6217 = 1.0 - MUL_6214;
        auto MUL_6246 = SUB_6217 * 0.005;
        auto MUL_6204 = ADD_5863 * 2.0;
        auto MUL_6234 = MUL_6204 * 0.0045;
        auto SUB_6250 = MUL_6223 - MUL_6234;
        auto SUB_6253 = SUB_6250 - MUL_6246;
        auto ADD_6256 = ADD_2748 + SUB_6253;
        out.z[42] = ADD_6256;  // (1477, 1495)
        auto ADD_6297 = MUL_6223 + MUL_6234;
        auto ADD_6303 = ADD_6297 + MUL_6246;
        auto SUB_6307 = ADD_2748 - ADD_6303;
        out.z[43] = SUB_6307;  // (1495, 1498)
        auto MUL_6342 = SUB_6217 * 0.025;
        auto ADD_6354 = ADD_6297 + MUL_6342;
        auto SUB_6358 = ADD_2748 - ADD_6354;
        out.z[44] = SUB_6358;  // (1498, 1501)
        auto SUB_6394 = SUB_6250 - MUL_6342;
        auto ADD_6397 = ADD_2748 + SUB_6394;
        out.z[45] = ADD_6397;  // (1501, 1503)
        auto ADD_2878 = MUL_2487 + MUL_2488;
        auto MUL_2880 = SUB_2125 * ADD_2842;
        auto ADD_2881 = ADD_2878 + MUL_2880;
        auto MUL_2884 = ADD_2881 * 2.0;
        auto SUB_2887 = 0.02 - MUL_2884;
        auto ADD_2890 = ADD_2366 + SUB_2887;
        auto MUL_6442 = ADD_5698 * 2.0;
        auto SUB_6445 = 1.0 - MUL_6442;
        auto MUL_6463 = SUB_6445 * 0.008;
        auto MUL_6432 = ADD_5863 * 2.0;
        auto MUL_6457 = MUL_6432 * 0.01;
        auto MUL_6419 = SUB_5772 * 2.0;
        auto MUL_6451 = MUL_6419 * 0.005;
        auto ADD_6466 = MUL_6451 + MUL_6457;
        auto ADD_6469 = ADD_6466 + MUL_6463;
        auto ADD_6472 = ADD_2890 + ADD_6469;
        out.z[46] = ADD_6472;  // (1503, 1519)
        auto SUB_6499 = MUL_6451 - MUL_6457;
        auto ADD_6502 = SUB_6499 + MUL_6463;
        auto ADD_6505 = ADD_2890 + ADD_6502;
        out.z[47] = ADD_6505;  // (1519, 1522)
        auto MUL_6550 = ADD_5698 * 2.0;
        auto SUB_6553 = 1.0 - MUL_6550;
        auto MUL_6577 = SUB_6553 * 0.03;
        auto MUL_6540 = ADD_5863 * 2.0;
        auto MUL_6570 = MUL_6540 * 0.01725;
        auto SUB_6580 = MUL_6577 - MUL_6570;
        auto ADD_6583 = ADD_2890 + SUB_6580;
        out.z[48] = ADD_6583;  // (1522, 1529)
        auto MUL_6607 = SUB_6553 * 0.05;
        auto SUB_6610 = MUL_6607 - MUL_6570;
        auto ADD_6613 = ADD_2890 + SUB_6610;
        out.z[49] = ADD_6613;  // (1529, 1532)
        auto MUL_6637 = SUB_6553 * 0.07;
        auto SUB_6640 = MUL_6637 - MUL_6570;
        auto ADD_6643 = ADD_2890 + SUB_6640;
        out.z[50] = ADD_6643;  // (1532, 1535)
        auto ADD_3138 = MUL_2737 + MUL_2738;
        auto MUL_3140 = SUB_2125 * ADD_3102;
        auto ADD_3141 = ADD_3138 + MUL_3140;
        auto MUL_3144 = ADD_3141 * 2.0;
        auto SUB_3147 = 0.1127 - MUL_3144;
        auto ADD_3150 = ADD_2890 + SUB_3147;
        auto MUL_6688 = ADD_5698 * 2.0;
        auto SUB_6691 = 1.0 - MUL_6688;
        auto MUL_6714 = SUB_6691 * 0.005;
        auto MUL_6678 = ADD_5863 * 2.0;
        auto MUL_6703 = MUL_6678 * 0.0045;
        auto MUL_6665 = SUB_5772 * 2.0;
        auto MUL_6697 = MUL_6665 * 0.01;
        auto ADD_6718 = MUL_6697 + MUL_6703;
        auto SUB_6721 = ADD_6718 - MUL_6714;
        auto ADD_6724 = ADD_3150 + SUB_6721;
        out.z[51] = ADD_6724;  // (1535, 1551)
        auto SUB_6757 = MUL_6703 - MUL_6697;
        auto SUB_6760 = SUB_6757 - MUL_6714;
        auto ADD_6763 = ADD_3150 + SUB_6760;
        out.z[52] = ADD_6763;  // (1551, 1554)
        auto MUL_6792 = SUB_6691 * 0.025;
        auto SUB_6799 = SUB_6757 - MUL_6792;
        auto ADD_6802 = ADD_3150 + SUB_6799;
        out.z[53] = ADD_6802;  // (1554, 1557)
        auto SUB_6832 = ADD_6718 - MUL_6792;
        auto ADD_6835 = ADD_3150 + SUB_6832;
        out.z[54] = ADD_6835;  // (1557, 1559)
        auto ADD_6916 = MUL_4941 + MUL_4938;
        auto MUL_6918 = ADD_6916 * 2.0;
        auto MUL_6943 = MUL_6918 * 0.02;
        auto MUL_3398 = SUB_987 * MUL_3368;
        auto MUL_3396 = ADD_981 * MUL_3376;
        auto ADD_3399 = MUL_3396 + MUL_3398;
        auto MUL_3402 = ADD_3399 * 2.0;
        auto SUB_3405 = 0.025 - MUL_3402;
        auto ADD_3408 = ADD_1099 + SUB_3405;
        auto ADD_6952 = ADD_3408 + MUL_6943;
        out.z[55] = ADD_6952;  // (1559, 1569)
        auto SUB_6979 = ADD_3408 - MUL_6943;
        out.z[56] = SUB_6979;  // (1569, 1570)
        auto MUL_7024 = ADD_4974 * 2.0;
        auto SUB_7027 = 1.0 - MUL_7024;
        auto MUL_7051 = SUB_7027 * 0.008;
        auto MUL_7014 = ADD_6916 * 2.0;
        auto MUL_7045 = MUL_7014 * 0.012;
        auto MUL_7001 = SUB_5048 * 2.0;
        auto MUL_7038 = MUL_7001 * 0.005;
        auto SUB_7054 = MUL_7045 - MUL_7038;
        auto ADD_7057 = SUB_7054 + MUL_7051;
        auto MUL_3529 = SUB_998 * MUL_3502;
        auto MUL_3532 = SUB_987 * SUB_3500;
        auto MUL_3530 = ADD_981 * MUL_3506;
        auto SUB_3531 = MUL_3529 - MUL_3530;
        auto SUB_3533 = SUB_3531 - MUL_3532;
        auto MUL_3535 = SUB_3533 * 2.0;
        auto ADD_3537 = MUL_3535 + 0.02;
        auto ADD_3540 = ADD_3408 + ADD_3537;
        auto ADD_7060 = ADD_3540 + ADD_7057;
        out.z[57] = ADD_7060;  // (1570, 1588)
        auto ADD_7095 = MUL_7038 + MUL_7045;
        auto SUB_7099 = MUL_7051 - ADD_7095;
        auto ADD_7102 = ADD_3540 + SUB_7099;
        out.z[58] = ADD_7102;  // (1588, 1591)
        auto MUL_7147 = ADD_4974 * 2.0;
        auto SUB_7150 = 1.0 - MUL_7147;
        auto MUL_7168 = SUB_7150 * 0.03;
        auto MUL_7137 = ADD_6916 * 2.0;
        auto MUL_7162 = MUL_7137 * 0.01725;
        auto ADD_7171 = MUL_7162 + MUL_7168;
        auto ADD_7174 = ADD_3540 + ADD_7171;
        out.z[59] = ADD_7174;  // (1591, 1598)
        auto MUL_7192 = SUB_7150 * 0.05;
        auto ADD_7195 = MUL_7162 + MUL_7192;
        auto ADD_7198 = ADD_3540 + ADD_7195;
        out.z[60] = ADD_7198;  // (1598, 1601)
        auto MUL_7216 = SUB_7150 * 0.07;
        auto ADD_7219 = MUL_7162 + MUL_7216;
        auto ADD_7222 = ADD_3540 + ADD_7219;
        out.z[61] = ADD_7222;  // (1601, 1604)
        auto MUL_3779 = SUB_998 * MUL_3752;
        auto MUL_3782 = SUB_987 * SUB_3750;
        auto MUL_3780 = ADD_981 * MUL_3756;
        auto SUB_3781 = MUL_3779 - MUL_3780;
        auto SUB_3783 = SUB_3781 - MUL_3782;
        auto MUL_3785 = SUB_3783 * 2.0;
        auto ADD_3787 = MUL_3785 + 0.1127;
        auto ADD_3790 = ADD_3540 + ADD_3787;
        auto MUL_7267 = ADD_4974 * 2.0;
        auto SUB_7270 = 1.0 - MUL_7267;
        auto MUL_7299 = SUB_7270 * 0.005;
        auto MUL_7257 = ADD_6916 * 2.0;
        auto MUL_7287 = MUL_7257 * 0.0045;
        auto MUL_7244 = SUB_5048 * 2.0;
        auto MUL_7276 = MUL_7244 * 0.01;
        auto SUB_7303 = MUL_7276 - MUL_7287;
        auto SUB_7306 = SUB_7303 - MUL_7299;
        auto ADD_7309 = ADD_3790 + SUB_7306;
        out.z[62] = ADD_7309;  // (1604, 1622)
        auto ADD_7350 = MUL_7276 + MUL_7287;
        auto ADD_7356 = ADD_7350 + MUL_7299;
        auto SUB_7360 = ADD_3790 - ADD_7356;
        out.z[63] = SUB_7360;  // (1622, 1625)
        auto MUL_7395 = SUB_7270 * 0.025;
        auto ADD_7407 = ADD_7350 + MUL_7395;
        auto SUB_7411 = ADD_3790 - ADD_7407;
        out.z[64] = SUB_7411;  // (1625, 1628)
        auto SUB_7447 = SUB_7303 - MUL_7395;
        auto ADD_7450 = ADD_3790 + SUB_7447;
        out.z[65] = ADD_7450;  // (1628, 1630)
        auto ADD_3920 = MUL_3529 + MUL_3530;
        auto MUL_3922 = SUB_987 * ADD_3884;
        auto ADD_3923 = ADD_3920 + MUL_3922;
        auto MUL_3926 = ADD_3923 * 2.0;
        auto SUB_3929 = 0.02 - MUL_3926;
        auto ADD_3932 = ADD_3408 + SUB_3929;
        auto MUL_7495 = ADD_4974 * 2.0;
        auto SUB_7498 = 1.0 - MUL_7495;
        auto MUL_7516 = SUB_7498 * 0.008;
        auto MUL_7485 = ADD_6916 * 2.0;
        auto MUL_7510 = MUL_7485 * 0.01;
        auto MUL_7472 = SUB_5048 * 2.0;
        auto MUL_7504 = MUL_7472 * 0.005;
        auto ADD_7519 = MUL_7504 + MUL_7510;
        auto ADD_7522 = ADD_7519 + MUL_7516;
        auto ADD_7525 = ADD_3932 + ADD_7522;
        out.z[66] = ADD_7525;  // (1630, 1646)
        auto SUB_7552 = MUL_7504 - MUL_7510;
        auto ADD_7555 = SUB_7552 + MUL_7516;
        auto ADD_7558 = ADD_3932 + ADD_7555;
        out.z[67] = ADD_7558;  // (1646, 1649)
        auto MUL_7603 = ADD_4974 * 2.0;
        auto SUB_7606 = 1.0 - MUL_7603;
        auto MUL_7630 = SUB_7606 * 0.03;
        auto MUL_7593 = ADD_6916 * 2.0;
        auto MUL_7623 = MUL_7593 * 0.01725;
        auto SUB_7633 = MUL_7630 - MUL_7623;
        auto ADD_7636 = ADD_3932 + SUB_7633;
        out.z[68] = ADD_7636;  // (1649, 1656)
        auto MUL_7660 = SUB_7606 * 0.05;
        auto SUB_7663 = MUL_7660 - MUL_7623;
        auto ADD_7666 = ADD_3932 + SUB_7663;
        out.z[69] = ADD_7666;  // (1656, 1659)
        auto MUL_7690 = SUB_7606 * 0.07;
        auto SUB_7693 = MUL_7690 - MUL_7623;
        auto ADD_7696 = ADD_3932 + SUB_7693;
        out.z[70] = ADD_7696;  // (1659, 1662)
        auto ADD_4180 = MUL_3779 + MUL_3780;
        auto MUL_4182 = SUB_987 * ADD_4144;
        auto ADD_4183 = ADD_4180 + MUL_4182;
        auto MUL_4186 = ADD_4183 * 2.0;
        auto SUB_4189 = 0.1127 - MUL_4186;
        auto ADD_4192 = ADD_3932 + SUB_4189;
        auto MUL_7741 = ADD_4974 * 2.0;
        auto SUB_7744 = 1.0 - MUL_7741;
        auto MUL_7767 = SUB_7744 * 0.005;
        auto MUL_7731 = ADD_6916 * 2.0;
        auto MUL_7756 = MUL_7731 * 0.0045;
        auto MUL_7718 = SUB_5048 * 2.0;
        auto MUL_7750 = MUL_7718 * 0.01;
        auto ADD_7771 = MUL_7750 + MUL_7756;
        auto SUB_7774 = ADD_7771 - MUL_7767;
        auto ADD_7777 = ADD_4192 + SUB_7774;
        out.z[71] = ADD_7777;  // (1662, 1678)
        auto SUB_7810 = MUL_7756 - MUL_7750;
        auto SUB_7813 = SUB_7810 - MUL_7767;
        auto ADD_7816 = ADD_4192 + SUB_7813;
        out.z[72] = ADD_7816;  // (1678, 1681)
        auto MUL_7845 = SUB_7744 * 0.025;
        auto SUB_7852 = SUB_7810 - MUL_7845;
        auto ADD_7855 = ADD_4192 + SUB_7852;
        out.z[73] = ADD_7855;  // (1681, 1684)
        auto SUB_7885 = ADD_7771 - MUL_7845;
        auto ADD_7888 = ADD_4192 + SUB_7885;
        out.z[74] = ADD_7888;  // (1684, 1686)
    }

    template <std::size_t rake>
    inline bool interleaved_sphere_fk(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept
    {
        if (/*left_upper_shoulder vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (0, 0)
        if (/*left_upper_shoulder*/ sphere_environment_in_collision(
            environment, 0.0640272, 0.2590274, 0.304626, 0.175))
        {
            if (sphere_environment_in_collision(environment, 0.0640272, 0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0640272, 0.2590274, 0.229626, 0.1))
            {
                return false;
            }
        }  // (0, 0)
        if (/*right_upper_shoulder*/ sphere_environment_in_collision(
            environment, 0.0640272, -0.2590274, 0.304626, 0.175))
        {
            if (sphere_environment_in_collision(environment, 0.0640272, -0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, 0.0640272, -0.2590274, 0.229626, 0.1))
            {
                return false;
            }
        }  // (0, 0)
        if (/*torso*/ sphere_environment_in_collision(environment, -0.044, 0.0, 0.222, 0.409))
        {
            if (sphere_environment_in_collision(environment, -0.025, -0.1, 0.1, 0.25))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.025, 0.1, 0.1, 0.25))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, -0.065, 0.0, 0.4, 0.23))
            {
                return false;
            }
        }  // (0, 0)
        if (sphere_environment_in_collision(environment, 0.0, 0.0, -0.6, 0.5))
        {
            return false;
        }  // (0, 0)
        if (sphere_environment_in_collision(environment, 0.04, 0.0, 0.686, 0.2))
        {
            return false;
        }  // (0, 0)
        auto INPUT_0 = q[0];
        auto DIV_15 = INPUT_0 * 0.5;
        auto SIN_16 = DIV_15.sin();
        auto COS_22 = DIV_15.cos();
        auto MUL_49 = COS_22 * 0.3826843;
        auto MUL_42 = SIN_16 * 0.9238792;
        auto SUB_51 = MUL_42 - MUL_49;
        auto MUL_112 = SUB_51 * 0.069;
        auto MUL_116 = SUB_51 * MUL_112;
        auto MUL_119 = MUL_116 * 2.0;
        auto SUB_122 = 0.069 - MUL_119;
        auto ADD_137 = SUB_122 + 0.0640272;
        auto MUL_53 = COS_22 * 0.9238792;
        auto MUL_60 = SIN_16 * 0.3826843;
        auto ADD_62 = MUL_53 + MUL_60;
        auto MUL_124 = ADD_62 * MUL_112;
        auto MUL_128 = MUL_124 * 2.0;
        auto SUB_140 = MUL_128 - 0.2590274;
        if (sphere_environment_in_collision(environment, ADD_137, SUB_140, 0.399976, 0.1))
        {
            return false;
        }  // (0, 18)
        auto MUL_93 = ADD_62 * 0.7071068;
        auto MUL_91 = SUB_51 * 0.7071068;
        auto INPUT_2 = q[2];
        auto DIV_304 = INPUT_2 * 0.5;
        auto SIN_305 = DIV_304.sin();
        auto COS_311 = DIV_304.cos();
        auto INPUT_1 = q[1];
        auto DIV_144 = INPUT_1 * 0.5;
        auto SIN_145 = DIV_144.sin();
        auto COS_151 = DIV_144.cos();
        auto MUL_170 = MUL_91 * COS_151;
        auto MUL_155 = MUL_91 * SIN_145;
        auto MUL_153 = MUL_93 * COS_151;
        auto SUB_176 = MUL_153 - MUL_155;
        auto ADD_157 = MUL_153 + MUL_155;
        auto MUL_233 = SUB_176 * 0.5;
        auto MUL_235 = ADD_157 * 0.5;
        auto SUB_237 = MUL_233 - MUL_235;
        auto ADD_249 = MUL_233 + MUL_235;
        auto MUL_167 = MUL_93 * SIN_145;
        auto SUB_165 = MUL_167 - MUL_170;
        auto ADD_171 = MUL_167 + MUL_170;
        auto MUL_242 = ADD_171 * 0.5;
        auto MUL_268 = ADD_171 * 0.102;
        auto MUL_273 = ADD_171 * MUL_268;
        auto MUL_239 = SUB_165 * 0.5;
        auto SUB_252 = ADD_249 - MUL_239;
        auto SUB_255 = SUB_252 - MUL_242;
        auto SUB_240 = SUB_237 - MUL_239;
        auto ADD_243 = SUB_240 + MUL_242;
        auto ADD_228 = ADD_249 + MUL_239;
        auto ADD_231 = ADD_228 + MUL_242;
        auto ADD_216 = SUB_237 + MUL_239;
        auto SUB_219 = ADD_216 - MUL_242;
        auto MUL_328 = SUB_255 * COS_311;
        auto MUL_323 = SUB_255 * SIN_305;
        auto MUL_326 = ADD_243 * COS_311;
        auto ADD_327 = MUL_323 + MUL_326;
        auto MUL_4616 = ADD_327 * ADD_327;
        auto MUL_331 = ADD_243 * SIN_305;
        auto SUB_332 = MUL_328 - MUL_331;
        auto MUL_320 = ADD_231 * COS_311;
        auto MUL_314 = ADD_231 * SIN_305;
        auto MUL_313 = SUB_219 * COS_311;
        auto ADD_315 = MUL_313 + MUL_314;
        auto MUL_4622 = ADD_315 * ADD_327;
        auto MUL_318 = SUB_219 * SIN_305;
        auto SUB_321 = MUL_320 - MUL_318;
        auto MUL_4618 = SUB_332 * SUB_321;
        auto ADD_4650 = MUL_4622 + MUL_4618;
        auto MUL_4652 = ADD_4650 * 2.0;
        auto MUL_4682 = MUL_4652 * 0.11;
        auto MUL_4615 = SUB_321 * SUB_321;
        auto ADD_4624 = MUL_4615 + MUL_4616;
        auto MUL_4627 = ADD_4624 * 2.0;
        auto SUB_4630 = 1.0 - MUL_4627;
        auto MUL_4665 = SUB_4630 * 0.01;
        auto SUB_4687 = MUL_4682 - MUL_4665;
        auto MUL_263 = SUB_165 * 0.102;
        auto MUL_271 = SUB_165 * MUL_263;
        auto ADD_275 = MUL_271 + MUL_273;
        auto MUL_278 = ADD_275 * 2.0;
        auto SUB_281 = 0.102 - MUL_278;
        auto ADD_300 = ADD_137 + SUB_281;
        auto ADD_4690 = ADD_300 + SUB_4687;
        auto MUL_4617 = SUB_332 * ADD_327;
        auto MUL_4620 = SUB_332 * ADD_315;
        auto MUL_4623 = SUB_321 * ADD_327;
        auto SUB_4653 = MUL_4623 - MUL_4620;
        auto MUL_4655 = SUB_4653 * 2.0;
        auto MUL_4684 = MUL_4655 * 0.11;
        auto MUL_4621 = ADD_315 * SUB_321;
        auto ADD_4631 = MUL_4621 + MUL_4617;
        auto MUL_4633 = ADD_4631 * 2.0;
        auto MUL_4669 = MUL_4633 * 0.01;
        auto SUB_4688 = MUL_4684 - MUL_4669;
        auto MUL_283 = SUB_176 * MUL_268;
        auto MUL_284 = ADD_157 * MUL_263;
        auto SUB_285 = MUL_283 - MUL_284;
        auto MUL_288 = SUB_285 * 2.0;
        auto ADD_301 = SUB_140 + MUL_288;
        auto ADD_4691 = ADD_301 + SUB_4688;
        auto SUB_4634 = MUL_4622 - MUL_4618;
        auto MUL_4636 = SUB_4634 * 2.0;
        auto MUL_4673 = MUL_4636 * 0.01;
        auto MUL_4619 = ADD_315 * ADD_315;
        auto ADD_4656 = MUL_4615 + MUL_4619;
        auto MUL_4659 = ADD_4656 * 2.0;
        auto SUB_4662 = 1.0 - MUL_4659;
        auto MUL_4686 = SUB_4662 * 0.11;
        auto SUB_4689 = MUL_4686 - MUL_4673;
        auto MUL_290 = SUB_176 * MUL_263;
        auto MUL_292 = ADD_157 * MUL_268;
        auto ADD_293 = MUL_290 + MUL_292;
        auto MUL_297 = ADD_293 * 2.0;
        auto SUB_302 = 0.399976 - MUL_297;
        auto ADD_4692 = SUB_302 + SUB_4689;
        auto MUL_4712 = MUL_4652 * 0.22;
        auto MUL_4695 = SUB_4630 * 0.02;
        auto SUB_4717 = MUL_4712 - MUL_4695;
        auto ADD_4720 = ADD_300 + SUB_4717;
        auto MUL_4714 = MUL_4655 * 0.22;
        auto MUL_4699 = MUL_4633 * 0.02;
        auto SUB_4718 = MUL_4714 - MUL_4699;
        auto ADD_4721 = ADD_301 + SUB_4718;
        auto MUL_4716 = SUB_4662 * 0.22;
        auto MUL_4703 = MUL_4636 * 0.02;
        auto SUB_4719 = MUL_4716 - MUL_4703;
        auto ADD_4722 = SUB_302 + SUB_4719;
        if (/*right_upper_elbow*/ sphere_environment_in_collision(
            environment, ADD_4690, ADD_4691, ADD_4692, 0.19))
        {
            if (sphere_environment_in_collision(environment, ADD_4720, ADD_4721, ADD_4722, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4690, ADD_4691, ADD_4692, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_300, ADD_301, SUB_302, 0.08))
            {
                return false;
            }
        }  // (18, 127)
        auto MUL_445 = ADD_327 * 0.069;
        auto MUL_440 = SUB_321 * 0.069;
        auto MUL_448 = SUB_321 * MUL_440;
        auto MUL_434 = SUB_321 * 0.26242;
        auto MUL_447 = SUB_332 * MUL_434;
        auto SUB_450 = MUL_447 - MUL_448;
        auto MUL_443 = ADD_315 * 0.26242;
        auto SUB_446 = MUL_443 - MUL_445;
        auto MUL_451 = ADD_327 * SUB_446;
        auto ADD_452 = SUB_450 + MUL_451;
        auto MUL_454 = ADD_452 * 2.0;
        auto ADD_456 = MUL_454 + 0.069;
        auto ADD_481 = ADD_300 + ADD_456;
        auto MUL_458 = SUB_332 * SUB_446;
        auto MUL_463 = ADD_327 * MUL_434;
        auto MUL_460 = ADD_315 * MUL_440;
        auto SUB_462 = MUL_460 - MUL_458;
        auto ADD_464 = SUB_462 + MUL_463;
        auto MUL_466 = ADD_464 * 2.0;
        auto ADD_482 = ADD_301 + MUL_466;
        auto MUL_468 = SUB_332 * MUL_440;
        auto MUL_473 = SUB_321 * MUL_434;
        auto MUL_470 = ADD_315 * SUB_446;
        auto ADD_471 = MUL_468 + MUL_470;
        auto ADD_474 = ADD_471 + MUL_473;
        auto MUL_477 = ADD_474 * 2.0;
        auto SUB_480 = 0.26242 - MUL_477;
        auto ADD_483 = SUB_302 + SUB_480;
        auto MUL_400 = SUB_332 * 0.5;
        auto MUL_414 = ADD_327 * 0.5;
        auto MUL_410 = SUB_321 * 0.5;
        if (/*head vs. right_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_481, ADD_482, ADD_483, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_481, ADD_482, ADD_483, 0.1))
            {
                return false;
            }
        }  // (127, 158)
        if (/*torso vs. right_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_481, ADD_482, ADD_483, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_481, ADD_482, ADD_483, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_481, ADD_482, ADD_483, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_481, ADD_482, ADD_483, 0.1))
            {
                return false;
            }
        }  // (158, 158)
        if (sphere_environment_in_collision(environment, ADD_481, ADD_482, ADD_483, 0.1))
        {
            return false;
        }  // (158, 158)
        auto MUL_404 = ADD_315 * 0.5;
        auto SUB_370 = MUL_404 - MUL_400;
        auto SUB_375 = SUB_370 - MUL_410;
        auto ADD_406 = MUL_400 + MUL_404;
        auto SUB_412 = MUL_410 - ADD_406;
        auto ADD_427 = ADD_406 + MUL_410;
        auto ADD_432 = ADD_427 + MUL_414;
        auto ADD_415 = SUB_412 + MUL_414;
        auto ADD_392 = SUB_370 + MUL_410;
        auto SUB_397 = ADD_392 - MUL_414;
        auto ADD_380 = SUB_375 + MUL_414;
        auto INPUT_4 = q[4];
        auto DIV_637 = INPUT_4 * 0.5;
        auto SIN_638 = DIV_637.sin();
        auto COS_644 = DIV_637.cos();
        auto INPUT_3 = q[3];
        auto DIV_485 = INPUT_3 * 0.5;
        auto SIN_486 = DIV_485.sin();
        auto COS_492 = DIV_485.cos();
        auto MUL_509 = ADD_432 * COS_492;
        auto MUL_504 = ADD_432 * SIN_486;
        auto MUL_507 = ADD_415 * COS_492;
        auto ADD_508 = MUL_504 + MUL_507;
        auto MUL_554 = ADD_508 * 0.5;
        auto MUL_601 = ADD_508 * 0.10359;
        auto MUL_606 = ADD_508 * MUL_601;
        auto MUL_512 = ADD_415 * SIN_486;
        auto SUB_513 = MUL_509 - MUL_512;
        auto MUL_546 = SUB_513 * 0.5;
        auto MUL_501 = SUB_397 * COS_492;
        auto MUL_495 = SUB_397 * SIN_486;
        auto MUL_494 = ADD_380 * COS_492;
        auto ADD_496 = MUL_494 + MUL_495;
        auto MUL_548 = ADD_496 * 0.5;
        auto SUB_560 = MUL_546 - MUL_548;
        auto ADD_549 = MUL_546 + MUL_548;
        auto MUL_499 = ADD_380 * SIN_486;
        auto SUB_502 = MUL_501 - MUL_499;
        auto MUL_551 = SUB_502 * 0.5;
        auto SUB_585 = SUB_560 - MUL_551;
        auto SUB_588 = SUB_585 - MUL_554;
        auto SUB_574 = ADD_549 - MUL_551;
        auto ADD_577 = SUB_574 + MUL_554;
        auto ADD_563 = SUB_560 + MUL_551;
        auto ADD_566 = ADD_563 + MUL_554;
        auto ADD_552 = ADD_549 + MUL_551;
        auto SUB_555 = ADD_552 - MUL_554;
        auto MUL_661 = SUB_588 * COS_644;
        auto MUL_656 = SUB_588 * SIN_638;
        auto MUL_659 = ADD_577 * COS_644;
        auto ADD_660 = MUL_656 + MUL_659;
        auto MUL_664 = ADD_577 * SIN_638;
        auto SUB_665 = MUL_661 - MUL_664;
        auto MUL_653 = ADD_566 * COS_644;
        auto MUL_647 = ADD_566 * SIN_638;
        auto MUL_646 = SUB_555 * COS_644;
        auto ADD_648 = MUL_646 + MUL_647;
        auto MUL_4872 = ADD_648 * ADD_660;
        auto MUL_651 = SUB_555 * SIN_638;
        auto SUB_654 = MUL_653 - MUL_651;
        auto MUL_4868 = SUB_665 * SUB_654;
        auto ADD_4900 = MUL_4872 + MUL_4868;
        auto MUL_4902 = ADD_4900 * 2.0;
        auto MUL_4926 = MUL_4902 * 0.11;
        auto MUL_596 = SUB_502 * 0.10359;
        auto MUL_604 = SUB_502 * MUL_596;
        auto ADD_608 = MUL_604 + MUL_606;
        auto MUL_611 = ADD_608 * 2.0;
        auto SUB_614 = 0.10359 - MUL_611;
        auto ADD_633 = ADD_481 + SUB_614;
        auto ADD_4931 = ADD_633 + MUL_4926;
        auto MUL_4870 = SUB_665 * ADD_648;
        auto MUL_4873 = SUB_654 * ADD_660;
        auto SUB_4903 = MUL_4873 - MUL_4870;
        auto MUL_4905 = SUB_4903 * 2.0;
        auto MUL_4928 = MUL_4905 * 0.11;
        auto MUL_616 = SUB_513 * MUL_601;
        auto MUL_617 = ADD_496 * MUL_596;
        auto ADD_619 = MUL_616 + MUL_617;
        auto MUL_622 = ADD_619 * 2.0;
        auto ADD_634 = ADD_482 + MUL_622;
        auto ADD_4932 = ADD_634 + MUL_4928;
        auto MUL_4865 = SUB_654 * SUB_654;
        auto MUL_4869 = ADD_648 * ADD_648;
        auto ADD_4906 = MUL_4865 + MUL_4869;
        auto MUL_4909 = ADD_4906 * 2.0;
        auto SUB_4912 = 1.0 - MUL_4909;
        auto MUL_4930 = SUB_4912 * 0.11;
        auto MUL_624 = SUB_513 * MUL_596;
        auto MUL_626 = ADD_496 * MUL_601;
        auto SUB_628 = MUL_626 - MUL_624;
        auto MUL_631 = SUB_628 * 2.0;
        auto ADD_635 = ADD_483 + MUL_631;
        auto ADD_4933 = ADD_635 + MUL_4930;
        auto MUL_4965 = MUL_4902 * 0.22;
        auto ADD_4970 = ADD_633 + MUL_4965;
        auto MUL_4967 = MUL_4905 * 0.22;
        auto ADD_4971 = ADD_634 + MUL_4967;
        auto MUL_4969 = SUB_4912 * 0.22;
        auto ADD_4972 = ADD_635 + MUL_4969;
        auto MUL_778 = ADD_660 * 0.01;
        auto MUL_773 = SUB_654 * 0.01;
        auto MUL_781 = SUB_654 * MUL_773;
        if (/*head vs. right_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_4931, ADD_4932, ADD_4933, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
        }  // (158, 261)
        if (/*right_upper_forearm vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (261, 261)
        if (/*right_upper_forearm*/ sphere_environment_in_collision(
            environment, ADD_4931, ADD_4932, ADD_4933, 0.19))
        {
            if (sphere_environment_in_collision(environment, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
        }  // (261, 261)
        if (/*right_upper_shoulder vs. right_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_4931, ADD_4932, ADD_4933, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
        }  // (261, 261)
        if (/*torso vs. right_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_4931, ADD_4932, ADD_4933, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_633, ADD_634, ADD_635, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_4970, ADD_4971, ADD_4972, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_4931, ADD_4932, ADD_4933, 0.08))
            {
                return false;
            }
        }  // (261, 261)
        auto MUL_767 = SUB_654 * 0.2707;
        auto MUL_780 = SUB_665 * MUL_767;
        auto SUB_783 = MUL_780 - MUL_781;
        auto MUL_776 = ADD_648 * 0.2707;
        auto SUB_779 = MUL_776 - MUL_778;
        auto MUL_784 = ADD_660 * SUB_779;
        auto ADD_785 = SUB_783 + MUL_784;
        auto MUL_787 = ADD_785 * 2.0;
        auto ADD_789 = MUL_787 + 0.01;
        auto ADD_814 = ADD_633 + ADD_789;
        auto MUL_791 = SUB_665 * SUB_779;
        auto MUL_796 = ADD_660 * MUL_767;
        auto MUL_793 = ADD_648 * MUL_773;
        auto SUB_795 = MUL_793 - MUL_791;
        auto ADD_797 = SUB_795 + MUL_796;
        auto MUL_799 = ADD_797 * 2.0;
        auto ADD_815 = ADD_634 + MUL_799;
        auto MUL_801 = SUB_665 * MUL_773;
        auto MUL_806 = SUB_654 * MUL_767;
        auto MUL_803 = ADD_648 * SUB_779;
        auto ADD_804 = MUL_801 + MUL_803;
        auto ADD_807 = ADD_804 + MUL_806;
        auto MUL_810 = ADD_807 * 2.0;
        auto SUB_813 = 0.2707 - MUL_810;
        auto ADD_816 = ADD_635 + SUB_813;
        auto MUL_699 = SUB_665 * 0.5;
        auto MUL_711 = ADD_660 * 0.5;
        auto MUL_706 = SUB_654 * 0.5;
        auto MUL_702 = ADD_648 * 0.5;
        auto SUB_703 = MUL_702 - MUL_699;
        auto SUB_708 = SUB_703 - MUL_706;
        auto ADD_725 = SUB_703 + MUL_706;
        auto SUB_730 = ADD_725 - MUL_711;
        auto ADD_739 = MUL_699 + MUL_702;
        auto SUB_745 = MUL_706 - ADD_739;
        auto ADD_760 = ADD_739 + MUL_706;
        auto ADD_765 = ADD_760 + MUL_711;
        auto ADD_748 = SUB_745 + MUL_711;
        auto ADD_713 = SUB_708 + MUL_711;
        auto INPUT_5 = q[5];
        auto DIV_818 = INPUT_5 * 0.5;
        auto SIN_819 = DIV_818.sin();
        auto COS_825 = DIV_818.cos();
        auto MUL_842 = ADD_765 * COS_825;
        auto MUL_837 = ADD_765 * SIN_819;
        auto MUL_840 = ADD_748 * COS_825;
        auto ADD_841 = MUL_837 + MUL_840;
        auto MUL_845 = ADD_748 * SIN_819;
        auto SUB_846 = MUL_842 - MUL_845;
        auto MUL_834 = SUB_730 * COS_825;
        auto MUL_828 = SUB_730 * SIN_819;
        auto MUL_827 = ADD_713 * COS_825;
        auto ADD_829 = MUL_827 + MUL_828;
        auto MUL_5009 = ADD_829 * ADD_841;
        auto MUL_832 = ADD_713 * SIN_819;
        auto SUB_835 = MUL_834 - MUL_832;
        auto MUL_5005 = SUB_846 * SUB_835;
        auto ADD_5037 = MUL_5009 + MUL_5005;
        auto MUL_5039 = ADD_5037 * 2.0;
        auto MUL_5081 = MUL_5039 * 0.03;
        auto ADD_5086 = ADD_814 + MUL_5081;
        auto MUL_5007 = SUB_846 * ADD_829;
        auto MUL_5010 = SUB_835 * ADD_841;
        auto SUB_5040 = MUL_5010 - MUL_5007;
        auto MUL_5042 = SUB_5040 * 2.0;
        auto MUL_5083 = MUL_5042 * 0.03;
        auto ADD_5087 = ADD_815 + MUL_5083;
        auto MUL_5002 = SUB_835 * SUB_835;
        auto MUL_5006 = ADD_829 * ADD_829;
        auto ADD_5043 = MUL_5002 + MUL_5006;
        auto MUL_5046 = ADD_5043 * 2.0;
        auto SUB_5049 = 1.0 - MUL_5046;
        auto MUL_5085 = SUB_5049 * 0.03;
        auto ADD_5088 = ADD_816 + MUL_5085;
        auto SUB_5113 = ADD_814 - MUL_5081;
        auto SUB_5114 = ADD_815 - MUL_5083;
        auto SUB_5115 = ADD_816 - MUL_5085;
        auto MUL_879 = SUB_846 * 0.5;
        auto MUL_887 = ADD_841 * 0.5;
        auto MUL_884 = SUB_835 * 0.5;
        if (/*head vs. right_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_814, ADD_815, ADD_816, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
        }  // (261, 341)
        if (/*right_lower_forearm vs. left_upper_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, 0.0640272, 0.2590274, 0.304626, 0.175))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, 0.0640272, 0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, 0.0640272, 0.2590274, 0.229626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, 0.0640272, 0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, 0.0640272, 0.2590274, 0.229626, 0.1))
            {
                return false;
            }
        }  // (341, 341)
        if (/*right_lower_forearm vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (341, 341)
        if (/*right_lower_forearm*/ sphere_environment_in_collision(
            environment, ADD_814, ADD_815, ADD_816, 0.1))
        {
            if (sphere_environment_in_collision(environment, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
        }  // (341, 341)
        if (/*right_upper_shoulder vs. right_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_814, ADD_815, ADD_816, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
        }  // (341, 341)
        if (/*torso vs. right_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_814, ADD_815, ADD_816, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_5086, ADD_5087, ADD_5088, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_5113, SUB_5114, SUB_5115, 0.07))
            {
                return false;
            }
        }  // (341, 341)
        auto MUL_881 = ADD_829 * 0.5;
        auto SUB_893 = MUL_879 - MUL_881;
        auto SUB_918 = SUB_893 - MUL_884;
        auto SUB_921 = SUB_918 - MUL_887;
        auto ADD_896 = SUB_893 + MUL_884;
        auto ADD_899 = ADD_896 + MUL_887;
        auto ADD_882 = MUL_879 + MUL_881;
        auto SUB_907 = ADD_882 - MUL_884;
        auto ADD_910 = SUB_907 + MUL_887;
        auto ADD_885 = ADD_882 + MUL_884;
        auto SUB_888 = ADD_885 - MUL_887;
        auto MUL_934 = ADD_841 * 0.115975;
        auto MUL_939 = ADD_841 * MUL_934;
        auto MUL_929 = SUB_835 * 0.115975;
        auto MUL_937 = SUB_835 * MUL_929;
        auto ADD_941 = MUL_937 + MUL_939;
        auto MUL_944 = ADD_941 * 2.0;
        auto SUB_947 = 0.115975 - MUL_944;
        auto ADD_966 = ADD_814 + SUB_947;
        auto INPUT_6 = q[6];
        auto DIV_970 = INPUT_6 * 0.5;
        auto SIN_971 = DIV_970.sin();
        auto COS_977 = DIV_970.cos();
        auto MUL_994 = SUB_921 * COS_977;
        auto MUL_989 = SUB_921 * SIN_971;
        auto MUL_992 = ADD_910 * COS_977;
        auto ADD_993 = MUL_989 + MUL_992;
        auto MUL_997 = ADD_910 * SIN_971;
        auto SUB_998 = MUL_994 - MUL_997;
        auto MUL_986 = ADD_899 * COS_977;
        auto MUL_980 = ADD_899 * SIN_971;
        auto MUL_979 = SUB_888 * COS_977;
        auto ADD_981 = MUL_979 + MUL_980;
        auto MUL_5129 = ADD_981 * ADD_993;
        auto MUL_984 = SUB_888 * SIN_971;
        auto SUB_987 = MUL_986 - MUL_984;
        auto MUL_5125 = SUB_998 * SUB_987;
        auto ADD_5157 = MUL_5129 + MUL_5125;
        auto MUL_5159 = ADD_5157 * 2.0;
        auto MUL_5184 = MUL_5159 * 0.015;
        auto SUB_5194 = ADD_966 - MUL_5184;
        auto MUL_5127 = SUB_998 * ADD_981;
        auto MUL_5130 = SUB_987 * ADD_993;
        auto SUB_5160 = MUL_5130 - MUL_5127;
        auto MUL_5162 = SUB_5160 * 2.0;
        auto MUL_5188 = MUL_5162 * 0.015;
        auto MUL_949 = SUB_846 * MUL_934;
        auto MUL_950 = ADD_829 * MUL_929;
        auto ADD_952 = MUL_949 + MUL_950;
        auto MUL_955 = ADD_952 * 2.0;
        auto ADD_967 = ADD_815 + MUL_955;
        auto SUB_5195 = ADD_967 - MUL_5188;
        auto MUL_5122 = SUB_987 * SUB_987;
        auto MUL_5126 = ADD_981 * ADD_981;
        auto ADD_5163 = MUL_5122 + MUL_5126;
        auto MUL_5166 = ADD_5163 * 2.0;
        auto SUB_5169 = 1.0 - MUL_5166;
        auto MUL_5192 = SUB_5169 * 0.015;
        auto MUL_957 = SUB_846 * MUL_929;
        auto MUL_959 = ADD_829 * MUL_934;
        auto SUB_961 = MUL_959 - MUL_957;
        auto MUL_964 = SUB_961 * 2.0;
        auto ADD_968 = ADD_816 + MUL_964;
        auto SUB_5196 = ADD_968 - MUL_5192;
        auto MUL_5210 = MUL_5159 * 0.02;
        auto ADD_5215 = ADD_966 + MUL_5210;
        auto MUL_5212 = MUL_5162 * 0.02;
        auto ADD_5216 = ADD_967 + MUL_5212;
        auto MUL_5214 = SUB_5169 * 0.02;
        auto ADD_5217 = ADD_968 + MUL_5214;
        auto MUL_5232 = MUL_5159 * 0.04;
        auto SUB_5242 = ADD_966 - MUL_5232;
        auto MUL_5236 = MUL_5162 * 0.04;
        auto SUB_5243 = ADD_967 - MUL_5236;
        auto MUL_5240 = SUB_5169 * 0.04;
        auto SUB_5244 = ADD_968 - MUL_5240;
        auto MUL_5288 = ADD_5157 * 2.0;
        auto MUL_5343 = MUL_5288 * 0.02;
        auto MUL_5252 = ADD_993 * ADD_993;
        if (/*head vs. right_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, SUB_5194, SUB_5195, SUB_5196, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
        }  // (341, 420)
        if (/*right_lower_shoulder vs. right_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, SUB_5194, SUB_5195, SUB_5196, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
        }  // (420, 420)
        if (/*right_upper_elbow vs. right_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, SUB_5194, SUB_5195, SUB_5196, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
        }  // (420, 420)
        if (/*right_upper_shoulder vs. right_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, SUB_5194, SUB_5195, SUB_5196, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
        }  // (420, 420)
        if (/*right_wrist vs. left_upper_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, 0.0640272, 0.2590274, 0.304626, 0.175))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, 0.0640272, 0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, 0.0640272, 0.2590274, 0.229626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, 0.0640272, 0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, 0.0640272, 0.2590274, 0.229626, 0.1))
            {
                return false;
            }
        }  // (420, 420)
        if (/*right_wrist vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (420, 420)
        if (/*right_wrist*/ sphere_environment_in_collision(environment, SUB_5194, SUB_5195, SUB_5196, 0.105))
        {
            if (sphere_environment_in_collision(environment, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
        }  // (420, 420)
        if (/*torso vs. right_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, SUB_5194, SUB_5195, SUB_5196, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_5215, ADD_5216, ADD_5217, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_5242, SUB_5243, SUB_5244, 0.08))
            {
                return false;
            }
        }  // (420, 420)
        auto ADD_5260 = MUL_5122 + MUL_5252;
        auto MUL_5263 = ADD_5260 * 2.0;
        auto SUB_5266 = 1.0 - MUL_5263;
        auto MUL_5330 = SUB_5266 * 0.01;
        auto SUB_5353 = MUL_5330 - MUL_5343;
        auto MUL_1059 = SUB_987 * 0.11355;
        auto MUL_1070 = SUB_998 * MUL_1059;
        auto MUL_1067 = ADD_981 * 0.11355;
        auto MUL_1072 = ADD_993 * MUL_1067;
        auto ADD_1073 = MUL_1070 + MUL_1072;
        auto MUL_1075 = ADD_1073 * 2.0;
        auto ADD_1097 = ADD_966 + MUL_1075;
        auto ADD_5356 = ADD_1097 + SUB_5353;
        auto MUL_5291 = SUB_5160 * 2.0;
        auto MUL_5347 = MUL_5291 * 0.02;
        auto MUL_1078 = SUB_998 * MUL_1067;
        auto MUL_5253 = SUB_998 * ADD_993;
        auto MUL_1081 = ADD_993 * MUL_1059;
        auto SUB_1082 = MUL_1081 - MUL_1078;
        auto MUL_1084 = SUB_1082 * 2.0;
        auto ADD_1098 = ADD_967 + MUL_1084;
        auto MUL_5257 = ADD_981 * SUB_987;
        auto ADD_5267 = MUL_5257 + MUL_5253;
        auto MUL_5269 = ADD_5267 * 2.0;
        auto MUL_5332 = MUL_5269 * 0.01;
        auto SUB_5354 = MUL_5332 - MUL_5347;
        auto ADD_5357 = ADD_1098 + SUB_5354;
        auto SUB_5270 = MUL_5129 - MUL_5125;
        auto MUL_5295 = ADD_5163 * 2.0;
        auto SUB_5298 = 1.0 - MUL_5295;
        auto MUL_5351 = SUB_5298 * 0.02;
        auto MUL_5272 = SUB_5270 * 2.0;
        auto MUL_5334 = MUL_5272 * 0.01;
        auto SUB_5355 = MUL_5334 - MUL_5351;
        auto MUL_1089 = SUB_987 * MUL_1059;
        auto MUL_1087 = ADD_981 * MUL_1067;
        auto ADD_1090 = MUL_1087 + MUL_1089;
        auto MUL_1093 = ADD_1090 * 2.0;
        auto SUB_1096 = 0.11355 - MUL_1093;
        auto ADD_1099 = ADD_968 + SUB_1096;
        auto ADD_5358 = ADD_1099 + SUB_5355;
        auto INPUT_7 = q[7];
        auto DIV_1160 = INPUT_7 * 0.5;
        auto SIN_1161 = DIV_1160.sin();
        if (/*head vs. right_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_5356, ADD_5357, ADD_5358, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
        }  // (420, 464)
        if (/*right_hand vs. left_upper_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, 0.0640272, 0.2590274, 0.304626, 0.175))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, 0.0640272, 0.2590274, 0.379626, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, 0.0640272, 0.2590274, 0.229626, 0.1))
            {
                return false;
            }
        }  // (464, 464)
        if (/*right_hand vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (464, 464)
        if (/*right_lower_shoulder vs. right_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_5356, ADD_5357, ADD_5358, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
        }  // (464, 464)
        if (/*right_upper_elbow vs. right_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_5356, ADD_5357, ADD_5358, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
        }  // (464, 464)
        if (/*right_upper_shoulder vs. right_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_5356, ADD_5357, ADD_5358, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
        }  // (464, 464)
        if (/*torso vs. right_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_5356, ADD_5357, ADD_5358, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_5356, ADD_5357, ADD_5358, 0.05))
            {
                return false;
            }
        }  // (464, 464)
        if (sphere_environment_in_collision(environment, ADD_5356, ADD_5357, ADD_5358, 0.05))
        {
            return false;
        }  // (464, 464)
        auto COS_1167 = DIV_1160.cos();
        auto MUL_1185 = SIN_1161 * 0.9238792;
        auto MUL_1191 = COS_1167 * 0.3826843;
        auto ADD_1192 = MUL_1185 + MUL_1191;
        auto MUL_1251 = ADD_1192 * 0.069;
        auto MUL_1255 = ADD_1192 * MUL_1251;
        auto MUL_1258 = MUL_1255 * 2.0;
        auto SUB_1261 = 0.069 - MUL_1258;
        auto ADD_1276 = SUB_1261 + 0.0640272;
        auto MUL_1200 = SIN_1161 * 0.3826843;
        auto MUL_1194 = COS_1167 * 0.9238792;
        auto SUB_1201 = MUL_1194 - MUL_1200;
        auto MUL_1263 = SUB_1201 * MUL_1251;
        auto MUL_1267 = MUL_1263 * 2.0;
        auto ADD_1278 = MUL_1267 + 0.2590274;
        auto MUL_1232 = SUB_1201 * 0.7071068;
        auto MUL_1230 = ADD_1192 * 0.7071068;
        auto INPUT_9 = q[9];
        if (/*right_hand vs. left_lower_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1276, ADD_1278, 0.399976, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
        }  // (464, 482)
        if (/*right_lower_elbow vs. left_lower_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_1276, ADD_1278, 0.399976, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
        }  // (482, 482)
        if (/*right_lower_forearm vs. left_lower_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_1276, ADD_1278, 0.399976, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
        }  // (482, 482)
        if (/*right_upper_forearm vs. left_lower_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_1276, ADD_1278, 0.399976, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
        }  // (482, 482)
        if (/*right_wrist vs. left_lower_shoulder*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_1276, ADD_1278, 0.399976, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_1276, ADD_1278, 0.399976, 0.1))
            {
                return false;
            }
        }  // (482, 482)
        if (sphere_environment_in_collision(environment, ADD_1276, ADD_1278, 0.399976, 0.1))
        {
            return false;
        }  // (482, 482)
        auto DIV_1442 = INPUT_9 * 0.5;
        auto SIN_1443 = DIV_1442.sin();
        auto COS_1449 = DIV_1442.cos();
        auto INPUT_8 = q[8];
        auto DIV_1282 = INPUT_8 * 0.5;
        auto SIN_1283 = DIV_1282.sin();
        auto COS_1289 = DIV_1282.cos();
        auto MUL_1308 = MUL_1230 * COS_1289;
        auto MUL_1293 = MUL_1230 * SIN_1283;
        auto MUL_1291 = MUL_1232 * COS_1289;
        auto SUB_1314 = MUL_1291 - MUL_1293;
        auto ADD_1295 = MUL_1291 + MUL_1293;
        auto MUL_1371 = SUB_1314 * 0.5;
        auto MUL_1373 = ADD_1295 * 0.5;
        auto SUB_1375 = MUL_1371 - MUL_1373;
        auto ADD_1387 = MUL_1371 + MUL_1373;
        auto MUL_1305 = MUL_1232 * SIN_1283;
        auto SUB_1303 = MUL_1305 - MUL_1308;
        auto ADD_1309 = MUL_1305 + MUL_1308;
        auto MUL_1380 = ADD_1309 * 0.5;
        auto MUL_1406 = ADD_1309 * 0.102;
        auto MUL_1411 = ADD_1309 * MUL_1406;
        auto MUL_1377 = SUB_1303 * 0.5;
        auto SUB_1390 = ADD_1387 - MUL_1377;
        auto SUB_1393 = SUB_1390 - MUL_1380;
        auto SUB_1378 = SUB_1375 - MUL_1377;
        auto ADD_1381 = SUB_1378 + MUL_1380;
        auto ADD_1366 = ADD_1387 + MUL_1377;
        auto ADD_1369 = ADD_1366 + MUL_1380;
        auto ADD_1354 = SUB_1375 + MUL_1377;
        auto SUB_1357 = ADD_1354 - MUL_1380;
        auto MUL_1466 = SUB_1393 * COS_1449;
        auto MUL_1461 = SUB_1393 * SIN_1443;
        auto MUL_1464 = ADD_1381 * COS_1449;
        auto ADD_1465 = MUL_1461 + MUL_1464;
        auto MUL_5563 = ADD_1465 * ADD_1465;
        auto MUL_1469 = ADD_1381 * SIN_1443;
        auto SUB_1470 = MUL_1466 - MUL_1469;
        auto MUL_1458 = ADD_1369 * COS_1449;
        auto MUL_1452 = ADD_1369 * SIN_1443;
        auto MUL_1451 = SUB_1357 * COS_1449;
        auto ADD_1453 = MUL_1451 + MUL_1452;
        auto MUL_5569 = ADD_1453 * ADD_1465;
        auto MUL_1456 = SUB_1357 * SIN_1443;
        auto SUB_1459 = MUL_1458 - MUL_1456;
        auto MUL_5565 = SUB_1470 * SUB_1459;
        auto ADD_5597 = MUL_5569 + MUL_5565;
        auto MUL_5599 = ADD_5597 * 2.0;
        auto MUL_5629 = MUL_5599 * 0.11;
        auto MUL_5562 = SUB_1459 * SUB_1459;
        auto ADD_5571 = MUL_5562 + MUL_5563;
        auto MUL_5574 = ADD_5571 * 2.0;
        auto SUB_5577 = 1.0 - MUL_5574;
        auto MUL_5612 = SUB_5577 * 0.01;
        auto SUB_5634 = MUL_5629 - MUL_5612;
        auto MUL_1401 = SUB_1303 * 0.102;
        auto MUL_1409 = SUB_1303 * MUL_1401;
        auto ADD_1413 = MUL_1409 + MUL_1411;
        auto MUL_1416 = ADD_1413 * 2.0;
        auto SUB_1419 = 0.102 - MUL_1416;
        auto ADD_1438 = ADD_1276 + SUB_1419;
        auto ADD_5637 = ADD_1438 + SUB_5634;
        auto MUL_5564 = SUB_1470 * ADD_1465;
        auto MUL_5567 = SUB_1470 * ADD_1453;
        auto MUL_5570 = SUB_1459 * ADD_1465;
        auto SUB_5600 = MUL_5570 - MUL_5567;
        auto MUL_5602 = SUB_5600 * 2.0;
        auto MUL_5631 = MUL_5602 * 0.11;
        auto MUL_5568 = ADD_1453 * SUB_1459;
        auto ADD_5578 = MUL_5568 + MUL_5564;
        auto MUL_5580 = ADD_5578 * 2.0;
        auto MUL_5616 = MUL_5580 * 0.01;
        auto SUB_5635 = MUL_5631 - MUL_5616;
        auto MUL_1421 = SUB_1314 * MUL_1406;
        auto MUL_1422 = ADD_1295 * MUL_1401;
        auto SUB_1423 = MUL_1421 - MUL_1422;
        auto MUL_1426 = SUB_1423 * 2.0;
        auto ADD_1439 = ADD_1278 + MUL_1426;
        auto ADD_5638 = ADD_1439 + SUB_5635;
        auto SUB_5581 = MUL_5569 - MUL_5565;
        auto MUL_5583 = SUB_5581 * 2.0;
        auto MUL_5620 = MUL_5583 * 0.01;
        auto MUL_5566 = ADD_1453 * ADD_1453;
        auto ADD_5603 = MUL_5562 + MUL_5566;
        auto MUL_5606 = ADD_5603 * 2.0;
        auto SUB_5609 = 1.0 - MUL_5606;
        auto MUL_5633 = SUB_5609 * 0.11;
        auto SUB_5636 = MUL_5633 - MUL_5620;
        auto MUL_1428 = SUB_1314 * MUL_1401;
        auto MUL_1430 = ADD_1295 * MUL_1406;
        auto ADD_1431 = MUL_1428 + MUL_1430;
        auto MUL_1435 = ADD_1431 * 2.0;
        auto SUB_1440 = 0.399976 - MUL_1435;
        auto ADD_5639 = SUB_1440 + SUB_5636;
        auto MUL_5659 = MUL_5599 * 0.22;
        auto MUL_5642 = SUB_5577 * 0.02;
        auto SUB_5664 = MUL_5659 - MUL_5642;
        auto ADD_5667 = ADD_1438 + SUB_5664;
        auto MUL_5661 = MUL_5602 * 0.22;
        auto MUL_5646 = MUL_5580 * 0.02;
        auto SUB_5665 = MUL_5661 - MUL_5646;
        auto ADD_5668 = ADD_1439 + SUB_5665;
        auto MUL_5663 = SUB_5609 * 0.22;
        auto MUL_5650 = MUL_5583 * 0.02;
        auto SUB_5666 = MUL_5663 - MUL_5650;
        auto ADD_5669 = SUB_1440 + SUB_5666;
        auto MUL_1583 = ADD_1465 * 0.069;
        auto MUL_1572 = SUB_1459 * 0.26242;
        auto MUL_1585 = SUB_1470 * MUL_1572;
        if (/*left_upper_elbow*/ sphere_environment_in_collision(
            environment, ADD_5637, ADD_5638, ADD_5639, 0.19))
        {
            if (sphere_environment_in_collision(environment, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
        }  // (482, 591)
        if (/*right_hand vs. left_upper_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_5637, ADD_5638, ADD_5639, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
        }  // (591, 591)
        if (/*right_lower_elbow vs. left_upper_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_5637, ADD_5638, ADD_5639, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
        }  // (591, 591)
        if (/*right_lower_forearm vs. left_upper_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_5637, ADD_5638, ADD_5639, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
        }  // (591, 591)
        if (/*right_upper_forearm vs. left_upper_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_5637, ADD_5638, ADD_5639, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
        }  // (591, 591)
        if (/*right_wrist vs. left_upper_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_5637, ADD_5638, ADD_5639, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_5667, ADD_5668, ADD_5669, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_5637, ADD_5638, ADD_5639, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_1438, ADD_1439, SUB_1440, 0.08))
            {
                return false;
            }
        }  // (591, 591)
        auto MUL_1578 = SUB_1459 * 0.069;
        auto MUL_1586 = SUB_1459 * MUL_1578;
        auto SUB_1588 = MUL_1585 - MUL_1586;
        auto MUL_1581 = ADD_1453 * 0.26242;
        auto SUB_1584 = MUL_1581 - MUL_1583;
        auto MUL_1589 = ADD_1465 * SUB_1584;
        auto ADD_1590 = SUB_1588 + MUL_1589;
        auto MUL_1592 = ADD_1590 * 2.0;
        auto ADD_1594 = MUL_1592 + 0.069;
        auto ADD_1619 = ADD_1438 + ADD_1594;
        auto MUL_1596 = SUB_1470 * SUB_1584;
        auto MUL_1601 = ADD_1465 * MUL_1572;
        auto MUL_1598 = ADD_1453 * MUL_1578;
        auto SUB_1600 = MUL_1598 - MUL_1596;
        auto ADD_1602 = SUB_1600 + MUL_1601;
        auto MUL_1604 = ADD_1602 * 2.0;
        auto ADD_1620 = ADD_1439 + MUL_1604;
        auto MUL_1606 = SUB_1470 * MUL_1578;
        auto MUL_1611 = SUB_1459 * MUL_1572;
        auto MUL_1608 = ADD_1453 * SUB_1584;
        auto ADD_1609 = MUL_1606 + MUL_1608;
        auto ADD_1612 = ADD_1609 + MUL_1611;
        auto MUL_1615 = ADD_1612 * 2.0;
        auto SUB_1618 = 0.26242 - MUL_1615;
        auto ADD_1621 = SUB_1440 + SUB_1618;
        auto MUL_1538 = SUB_1470 * 0.5;
        auto MUL_1552 = ADD_1465 * 0.5;
        auto MUL_1548 = SUB_1459 * 0.5;
        auto MUL_1542 = ADD_1453 * 0.5;
        auto SUB_1508 = MUL_1542 - MUL_1538;
        auto SUB_1513 = SUB_1508 - MUL_1548;
        if (/*head vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (591, 622)
        if (/*right_hand vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*right_lower_elbow vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*right_lower_forearm vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*right_lower_shoulder vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*right_upper_elbow vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*right_upper_forearm vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*right_wrist vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (/*torso vs. left_lower_elbow*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_1619, ADD_1620, ADD_1621, 0.1))
            {
                return false;
            }
        }  // (622, 622)
        if (sphere_environment_in_collision(environment, ADD_1619, ADD_1620, ADD_1621, 0.1))
        {
            return false;
        }  // (622, 622)
        auto ADD_1544 = MUL_1538 + MUL_1542;
        auto SUB_1550 = MUL_1548 - ADD_1544;
        auto ADD_1565 = ADD_1544 + MUL_1548;
        auto ADD_1570 = ADD_1565 + MUL_1552;
        auto ADD_1553 = SUB_1550 + MUL_1552;
        auto ADD_1530 = SUB_1508 + MUL_1548;
        auto SUB_1535 = ADD_1530 - MUL_1552;
        auto ADD_1518 = SUB_1513 + MUL_1552;
        auto INPUT_11 = q[11];
        auto DIV_1775 = INPUT_11 * 0.5;
        auto SIN_1776 = DIV_1775.sin();
        auto COS_1782 = DIV_1775.cos();
        auto INPUT_10 = q[10];
        auto DIV_1623 = INPUT_10 * 0.5;
        auto SIN_1624 = DIV_1623.sin();
        auto COS_1630 = DIV_1623.cos();
        auto MUL_1647 = ADD_1570 * COS_1630;
        auto MUL_1642 = ADD_1570 * SIN_1624;
        auto MUL_1645 = ADD_1553 * COS_1630;
        auto ADD_1646 = MUL_1642 + MUL_1645;
        auto MUL_1692 = ADD_1646 * 0.5;
        auto MUL_1739 = ADD_1646 * 0.10359;
        auto MUL_1744 = ADD_1646 * MUL_1739;
        auto MUL_1650 = ADD_1553 * SIN_1624;
        auto SUB_1651 = MUL_1647 - MUL_1650;
        auto MUL_1684 = SUB_1651 * 0.5;
        auto MUL_1639 = SUB_1535 * COS_1630;
        auto MUL_1633 = SUB_1535 * SIN_1624;
        auto MUL_1632 = ADD_1518 * COS_1630;
        auto ADD_1634 = MUL_1632 + MUL_1633;
        auto MUL_1686 = ADD_1634 * 0.5;
        auto SUB_1698 = MUL_1684 - MUL_1686;
        auto ADD_1687 = MUL_1684 + MUL_1686;
        auto MUL_1637 = ADD_1518 * SIN_1624;
        auto SUB_1640 = MUL_1639 - MUL_1637;
        auto MUL_1689 = SUB_1640 * 0.5;
        auto SUB_1723 = SUB_1698 - MUL_1689;
        auto SUB_1726 = SUB_1723 - MUL_1692;
        auto SUB_1712 = ADD_1687 - MUL_1689;
        auto ADD_1715 = SUB_1712 + MUL_1692;
        auto ADD_1701 = SUB_1698 + MUL_1689;
        auto ADD_1704 = ADD_1701 + MUL_1692;
        auto ADD_1690 = ADD_1687 + MUL_1689;
        auto SUB_1693 = ADD_1690 - MUL_1692;
        auto MUL_1799 = SUB_1726 * COS_1782;
        auto MUL_1794 = SUB_1726 * SIN_1776;
        auto MUL_1797 = ADD_1715 * COS_1782;
        auto ADD_1798 = MUL_1794 + MUL_1797;
        auto MUL_1802 = ADD_1715 * SIN_1776;
        auto SUB_1803 = MUL_1799 - MUL_1802;
        auto MUL_1791 = ADD_1704 * COS_1782;
        auto MUL_1785 = ADD_1704 * SIN_1776;
        auto MUL_1784 = SUB_1693 * COS_1782;
        auto ADD_1786 = MUL_1784 + MUL_1785;
        auto MUL_5819 = ADD_1786 * ADD_1798;
        auto MUL_1789 = SUB_1693 * SIN_1776;
        auto SUB_1792 = MUL_1791 - MUL_1789;
        auto MUL_5815 = SUB_1803 * SUB_1792;
        auto ADD_5847 = MUL_5819 + MUL_5815;
        auto MUL_5849 = ADD_5847 * 2.0;
        auto MUL_5873 = MUL_5849 * 0.11;
        auto MUL_1734 = SUB_1640 * 0.10359;
        auto MUL_1742 = SUB_1640 * MUL_1734;
        auto ADD_1746 = MUL_1742 + MUL_1744;
        auto MUL_1749 = ADD_1746 * 2.0;
        auto SUB_1752 = 0.10359 - MUL_1749;
        auto ADD_1771 = ADD_1619 + SUB_1752;
        auto ADD_5878 = ADD_1771 + MUL_5873;
        auto MUL_5817 = SUB_1803 * ADD_1786;
        auto MUL_5820 = SUB_1792 * ADD_1798;
        auto SUB_5850 = MUL_5820 - MUL_5817;
        auto MUL_5852 = SUB_5850 * 2.0;
        auto MUL_5875 = MUL_5852 * 0.11;
        auto MUL_1754 = SUB_1651 * MUL_1739;
        auto MUL_1755 = ADD_1634 * MUL_1734;
        auto ADD_1757 = MUL_1754 + MUL_1755;
        auto MUL_1760 = ADD_1757 * 2.0;
        auto ADD_1772 = ADD_1620 + MUL_1760;
        auto ADD_5879 = ADD_1772 + MUL_5875;
        auto MUL_5812 = SUB_1792 * SUB_1792;
        auto MUL_5816 = ADD_1786 * ADD_1786;
        auto ADD_5853 = MUL_5812 + MUL_5816;
        auto MUL_5856 = ADD_5853 * 2.0;
        auto SUB_5859 = 1.0 - MUL_5856;
        auto MUL_5877 = SUB_5859 * 0.11;
        auto MUL_1762 = SUB_1651 * MUL_1734;
        auto MUL_1764 = ADD_1634 * MUL_1739;
        auto SUB_1766 = MUL_1764 - MUL_1762;
        auto MUL_1769 = SUB_1766 * 2.0;
        auto ADD_1773 = ADD_1621 + MUL_1769;
        auto ADD_5880 = ADD_1773 + MUL_5877;
        auto MUL_5912 = MUL_5849 * 0.22;
        auto ADD_5917 = ADD_1771 + MUL_5912;
        auto MUL_5914 = MUL_5852 * 0.22;
        auto ADD_5918 = ADD_1772 + MUL_5914;
        auto MUL_5916 = SUB_5859 * 0.22;
        auto ADD_5919 = ADD_1773 + MUL_5916;
        auto MUL_1916 = ADD_1798 * 0.01;
        auto MUL_1905 = SUB_1792 * 0.2707;
        auto MUL_1918 = SUB_1803 * MUL_1905;
        auto MUL_1911 = SUB_1792 * 0.01;
        auto MUL_1919 = SUB_1792 * MUL_1911;
        auto SUB_1921 = MUL_1918 - MUL_1919;
        if (/*head vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (622, 725)
        if (/*left_upper_forearm vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (725, 725)
        if (/*left_upper_forearm*/ sphere_environment_in_collision(
            environment, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_environment_in_collision(environment, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*left_upper_shoulder vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_hand vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_lower_elbow vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_lower_forearm vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_lower_shoulder vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_upper_elbow vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_upper_forearm vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*right_wrist vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        if (/*torso vs. left_upper_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_5878, ADD_5879, ADD_5880, 0.19))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_1771, ADD_1772, ADD_1773, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_5917, ADD_5918, ADD_5919, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_5878, ADD_5879, ADD_5880, 0.08))
            {
                return false;
            }
        }  // (725, 725)
        auto MUL_1914 = ADD_1786 * 0.2707;
        auto SUB_1917 = MUL_1914 - MUL_1916;
        auto MUL_1922 = ADD_1798 * SUB_1917;
        auto ADD_1923 = SUB_1921 + MUL_1922;
        auto MUL_1925 = ADD_1923 * 2.0;
        auto ADD_1927 = MUL_1925 + 0.01;
        auto ADD_1952 = ADD_1771 + ADD_1927;
        auto MUL_1929 = SUB_1803 * SUB_1917;
        auto MUL_1934 = ADD_1798 * MUL_1905;
        auto MUL_1931 = ADD_1786 * MUL_1911;
        auto SUB_1933 = MUL_1931 - MUL_1929;
        auto ADD_1935 = SUB_1933 + MUL_1934;
        auto MUL_1937 = ADD_1935 * 2.0;
        auto ADD_1953 = ADD_1772 + MUL_1937;
        auto MUL_1939 = SUB_1803 * MUL_1911;
        auto MUL_1944 = SUB_1792 * MUL_1905;
        auto MUL_1941 = ADD_1786 * SUB_1917;
        auto ADD_1942 = MUL_1939 + MUL_1941;
        auto ADD_1945 = ADD_1942 + MUL_1944;
        auto MUL_1948 = ADD_1945 * 2.0;
        auto SUB_1951 = 0.2707 - MUL_1948;
        auto ADD_1954 = ADD_1773 + SUB_1951;
        auto MUL_1837 = SUB_1803 * 0.5;
        auto MUL_1849 = ADD_1798 * 0.5;
        auto MUL_1844 = SUB_1792 * 0.5;
        auto MUL_1840 = ADD_1786 * 0.5;
        auto SUB_1841 = MUL_1840 - MUL_1837;
        auto SUB_1846 = SUB_1841 - MUL_1844;
        auto ADD_1863 = SUB_1841 + MUL_1844;
        auto SUB_1868 = ADD_1863 - MUL_1849;
        auto ADD_1877 = MUL_1837 + MUL_1840;
        auto SUB_1883 = MUL_1844 - ADD_1877;
        auto ADD_1898 = ADD_1877 + MUL_1844;
        auto ADD_1903 = ADD_1898 + MUL_1849;
        auto ADD_1886 = SUB_1883 + MUL_1849;
        auto ADD_1851 = SUB_1846 + MUL_1849;
        auto INPUT_12 = q[12];
        auto DIV_1956 = INPUT_12 * 0.5;
        auto SIN_1957 = DIV_1956.sin();
        auto COS_1963 = DIV_1956.cos();
        auto MUL_1980 = ADD_1903 * COS_1963;
        auto MUL_1975 = ADD_1903 * SIN_1957;
        auto MUL_1978 = ADD_1886 * COS_1963;
        auto ADD_1979 = MUL_1975 + MUL_1978;
        auto MUL_1983 = ADD_1886 * SIN_1957;
        auto SUB_1984 = MUL_1980 - MUL_1983;
        auto MUL_1972 = SUB_1868 * COS_1963;
        auto MUL_1966 = SUB_1868 * SIN_1957;
        auto MUL_1965 = ADD_1851 * COS_1963;
        auto ADD_1967 = MUL_1965 + MUL_1966;
        auto MUL_5956 = ADD_1967 * ADD_1979;
        auto MUL_1970 = ADD_1851 * SIN_1957;
        auto SUB_1973 = MUL_1972 - MUL_1970;
        auto MUL_5952 = SUB_1984 * SUB_1973;
        auto ADD_5984 = MUL_5956 + MUL_5952;
        auto MUL_5986 = ADD_5984 * 2.0;
        auto MUL_6028 = MUL_5986 * 0.03;
        auto ADD_6033 = ADD_1952 + MUL_6028;
        auto MUL_5954 = SUB_1984 * ADD_1967;
        auto MUL_5957 = SUB_1973 * ADD_1979;
        auto SUB_5987 = MUL_5957 - MUL_5954;
        auto MUL_5989 = SUB_5987 * 2.0;
        auto MUL_6030 = MUL_5989 * 0.03;
        auto ADD_6034 = ADD_1953 + MUL_6030;
        auto MUL_5949 = SUB_1973 * SUB_1973;
        auto MUL_5953 = ADD_1967 * ADD_1967;
        auto ADD_5990 = MUL_5949 + MUL_5953;
        auto MUL_5993 = ADD_5990 * 2.0;
        auto SUB_5996 = 1.0 - MUL_5993;
        auto MUL_6032 = SUB_5996 * 0.03;
        auto ADD_6035 = ADD_1954 + MUL_6032;
        auto SUB_6060 = ADD_1952 - MUL_6028;
        auto SUB_6061 = ADD_1953 - MUL_6030;
        auto SUB_6062 = ADD_1954 - MUL_6032;
        auto MUL_2017 = SUB_1984 * 0.5;
        auto MUL_2025 = ADD_1979 * 0.5;
        auto MUL_2072 = ADD_1979 * 0.115975;
        auto MUL_2077 = ADD_1979 * MUL_2072;
        auto MUL_2022 = SUB_1973 * 0.5;
        auto MUL_2067 = SUB_1973 * 0.115975;
        if (/*head vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (725, 805)
        if (/*left_lower_forearm vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (805, 805)
        if (/*left_lower_forearm*/ sphere_environment_in_collision(
            environment, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_environment_in_collision(environment, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*left_upper_shoulder vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_hand vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_lower_elbow vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_lower_forearm vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_lower_shoulder vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_upper_elbow vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_upper_forearm vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_upper_shoulder vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*right_wrist vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        if (/*torso vs. left_lower_forearm*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_1952, ADD_1953, ADD_1954, 0.1))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6033, ADD_6034, ADD_6035, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_6060, SUB_6061, SUB_6062, 0.07))
            {
                return false;
            }
        }  // (805, 805)
        auto MUL_2075 = SUB_1973 * MUL_2067;
        auto ADD_2079 = MUL_2075 + MUL_2077;
        auto MUL_2082 = ADD_2079 * 2.0;
        auto SUB_2085 = 0.115975 - MUL_2082;
        auto ADD_2104 = ADD_1952 + SUB_2085;
        auto MUL_2019 = ADD_1967 * 0.5;
        auto SUB_2031 = MUL_2017 - MUL_2019;
        auto SUB_2056 = SUB_2031 - MUL_2022;
        auto SUB_2059 = SUB_2056 - MUL_2025;
        auto ADD_2034 = SUB_2031 + MUL_2022;
        auto ADD_2037 = ADD_2034 + MUL_2025;
        auto ADD_2020 = MUL_2017 + MUL_2019;
        auto SUB_2045 = ADD_2020 - MUL_2022;
        auto ADD_2048 = SUB_2045 + MUL_2025;
        auto ADD_2023 = ADD_2020 + MUL_2022;
        auto SUB_2026 = ADD_2023 - MUL_2025;
        auto INPUT_13 = q[13];
        auto DIV_2108 = INPUT_13 * 0.5;
        auto SIN_2109 = DIV_2108.sin();
        auto COS_2115 = DIV_2108.cos();
        auto MUL_2132 = SUB_2059 * COS_2115;
        auto MUL_2127 = SUB_2059 * SIN_2109;
        auto MUL_2130 = ADD_2048 * COS_2115;
        auto ADD_2131 = MUL_2127 + MUL_2130;
        auto MUL_2135 = ADD_2048 * SIN_2109;
        auto SUB_2136 = MUL_2132 - MUL_2135;
        auto MUL_2124 = ADD_2037 * COS_2115;
        auto MUL_2118 = ADD_2037 * SIN_2109;
        auto MUL_2117 = SUB_2026 * COS_2115;
        auto ADD_2119 = MUL_2117 + MUL_2118;
        auto MUL_6076 = ADD_2119 * ADD_2131;
        auto MUL_2122 = SUB_2026 * SIN_2109;
        auto SUB_2125 = MUL_2124 - MUL_2122;
        auto MUL_6072 = SUB_2136 * SUB_2125;
        auto ADD_6104 = MUL_6076 + MUL_6072;
        auto MUL_6106 = ADD_6104 * 2.0;
        auto MUL_6131 = MUL_6106 * 0.015;
        auto SUB_6141 = ADD_2104 - MUL_6131;
        auto MUL_6074 = SUB_2136 * ADD_2119;
        auto MUL_6077 = SUB_2125 * ADD_2131;
        auto SUB_6107 = MUL_6077 - MUL_6074;
        auto MUL_6109 = SUB_6107 * 2.0;
        auto MUL_6135 = MUL_6109 * 0.015;
        auto MUL_2087 = SUB_1984 * MUL_2072;
        auto MUL_2088 = ADD_1967 * MUL_2067;
        auto ADD_2090 = MUL_2087 + MUL_2088;
        auto MUL_2093 = ADD_2090 * 2.0;
        auto ADD_2105 = ADD_1953 + MUL_2093;
        auto SUB_6142 = ADD_2105 - MUL_6135;
        auto MUL_6069 = SUB_2125 * SUB_2125;
        auto MUL_6073 = ADD_2119 * ADD_2119;
        auto ADD_6110 = MUL_6069 + MUL_6073;
        auto MUL_6113 = ADD_6110 * 2.0;
        auto SUB_6116 = 1.0 - MUL_6113;
        auto MUL_6139 = SUB_6116 * 0.015;
        auto MUL_2095 = SUB_1984 * MUL_2067;
        auto MUL_2097 = ADD_1967 * MUL_2072;
        auto SUB_2099 = MUL_2097 - MUL_2095;
        auto MUL_2102 = SUB_2099 * 2.0;
        auto ADD_2106 = ADD_1954 + MUL_2102;
        auto SUB_6143 = ADD_2106 - MUL_6139;
        auto MUL_6157 = MUL_6106 * 0.02;
        auto ADD_6162 = ADD_2104 + MUL_6157;
        auto MUL_6159 = MUL_6109 * 0.02;
        auto ADD_6163 = ADD_2105 + MUL_6159;
        auto MUL_6161 = SUB_6116 * 0.02;
        auto ADD_6164 = ADD_2106 + MUL_6161;
        auto MUL_6179 = MUL_6106 * 0.04;
        auto SUB_6189 = ADD_2104 - MUL_6179;
        auto MUL_6183 = MUL_6109 * 0.04;
        auto SUB_6190 = ADD_2105 - MUL_6183;
        auto MUL_6187 = SUB_6116 * 0.04;
        auto SUB_6191 = ADD_2106 - MUL_6187;
        auto MUL_6235 = ADD_6104 * 2.0;
        auto MUL_6290 = MUL_6235 * 0.02;
        auto MUL_6199 = ADD_2131 * ADD_2131;
        auto ADD_6207 = MUL_6069 + MUL_6199;
        auto MUL_6210 = ADD_6207 * 2.0;
        auto SUB_6213 = 1.0 - MUL_6210;
        if (/*head vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (805, 884)
        if (/*left_lower_shoulder vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*left_upper_elbow vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*left_upper_shoulder vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*left_wrist vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (884, 884)
        if (/*left_wrist*/ sphere_environment_in_collision(environment, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_environment_in_collision(environment, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_hand vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_lower_elbow vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_lower_forearm vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_lower_shoulder vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_upper_elbow vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_upper_forearm vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_upper_shoulder vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*right_wrist vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        if (/*torso vs. left_wrist*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, SUB_6141, SUB_6142, SUB_6143, 0.105))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6162, ADD_6163, ADD_6164, 0.07))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_6189, SUB_6190, SUB_6191, 0.08))
            {
                return false;
            }
        }  // (884, 884)
        auto MUL_6277 = SUB_6213 * 0.01;
        auto SUB_6300 = MUL_6277 - MUL_6290;
        auto MUL_2197 = SUB_2125 * 0.11355;
        auto MUL_2208 = SUB_2136 * MUL_2197;
        auto MUL_2205 = ADD_2119 * 0.11355;
        auto MUL_2210 = ADD_2131 * MUL_2205;
        auto ADD_2211 = MUL_2208 + MUL_2210;
        auto MUL_2213 = ADD_2211 * 2.0;
        auto ADD_2235 = ADD_2104 + MUL_2213;
        auto ADD_6303 = ADD_2235 + SUB_6300;
        auto MUL_6238 = SUB_6107 * 2.0;
        auto MUL_6294 = MUL_6238 * 0.02;
        auto MUL_2216 = SUB_2136 * MUL_2205;
        auto MUL_6200 = SUB_2136 * ADD_2131;
        auto MUL_2219 = ADD_2131 * MUL_2197;
        auto SUB_2220 = MUL_2219 - MUL_2216;
        auto MUL_2222 = SUB_2220 * 2.0;
        auto ADD_2236 = ADD_2105 + MUL_2222;
        auto MUL_6204 = ADD_2119 * SUB_2125;
        auto ADD_6214 = MUL_6204 + MUL_6200;
        auto MUL_6216 = ADD_6214 * 2.0;
        auto MUL_6279 = MUL_6216 * 0.01;
        auto SUB_6301 = MUL_6279 - MUL_6294;
        auto ADD_6304 = ADD_2236 + SUB_6301;
        auto SUB_6217 = MUL_6076 - MUL_6072;
        auto MUL_6242 = ADD_6110 * 2.0;
        auto SUB_6245 = 1.0 - MUL_6242;
        auto MUL_6298 = SUB_6245 * 0.02;
        auto MUL_6219 = SUB_6217 * 2.0;
        auto MUL_6281 = MUL_6219 * 0.01;
        auto SUB_6302 = MUL_6281 - MUL_6298;
        auto MUL_2227 = SUB_2125 * MUL_2197;
        auto MUL_2225 = ADD_2119 * MUL_2205;
        auto ADD_2228 = MUL_2225 + MUL_2227;
        auto MUL_2231 = ADD_2228 * 2.0;
        auto SUB_2234 = 0.11355 - MUL_2231;
        auto ADD_2237 = ADD_2106 + SUB_2234;
        auto ADD_6305 = ADD_2237 + SUB_6302;
        auto MUL_2326 = SUB_2125 * 0.025;
        auto MUL_2337 = SUB_2136 * MUL_2326;
        auto MUL_2334 = ADD_2119 * 0.025;
        auto MUL_2339 = ADD_2131 * MUL_2334;
        auto ADD_2340 = MUL_2337 + MUL_2339;
        auto MUL_2342 = ADD_2340 * 2.0;
        if (/*head vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (884, 928)
        if (/*left_hand vs. pedestal*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, 0.0, 0.0, -0.6, 0.5))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, 0.0, 0.0, -0.6, 0.5))
            {
                return false;
            }
        }  // (928, 928)
        if (/*left_lower_shoulder vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*left_upper_elbow vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*left_upper_shoulder vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_hand vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_lower_elbow vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_lower_forearm vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_lower_shoulder vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_upper_elbow vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_upper_forearm vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_upper_shoulder vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*right_wrist vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (/*torso vs. left_hand*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6303, ADD_6304, ADD_6305, 0.05))
            {
                return false;
            }
        }  // (928, 928)
        if (sphere_environment_in_collision(environment, ADD_6303, ADD_6304, ADD_6305, 0.05))
        {
            return false;
        }  // (928, 928)
        auto ADD_2364 = ADD_2235 + MUL_2342;
        auto MUL_2345 = SUB_2136 * MUL_2334;
        auto MUL_2348 = ADD_2131 * MUL_2326;
        auto SUB_2349 = MUL_2348 - MUL_2345;
        auto MUL_2351 = SUB_2349 * 2.0;
        auto ADD_2365 = ADD_2236 + MUL_2351;
        auto MUL_2356 = SUB_2125 * MUL_2326;
        auto MUL_2354 = ADD_2119 * MUL_2334;
        auto ADD_2357 = MUL_2354 + MUL_2356;
        auto MUL_2360 = ADD_2357 * 2.0;
        auto SUB_2363 = 0.025 - MUL_2360;
        auto ADD_2366 = ADD_2237 + SUB_2363;
        auto SUB_6336 = MUL_6204 - MUL_6200;
        auto MUL_6338 = SUB_6336 * 2.0;
        auto MUL_6387 = MUL_6338 * 0.02;
        auto ADD_6398 = ADD_2364 + MUL_6387;
        auto ADD_6339 = MUL_6199 + MUL_6073;
        auto MUL_6342 = ADD_6339 * 2.0;
        auto SUB_6345 = 1.0 - MUL_6342;
        auto MUL_6389 = SUB_6345 * 0.02;
        auto ADD_6399 = ADD_2365 + MUL_6389;
        auto ADD_6346 = MUL_6077 + MUL_6074;
        auto MUL_6348 = ADD_6346 * 2.0;
        auto MUL_6391 = MUL_6348 * 0.02;
        auto ADD_6400 = ADD_2366 + MUL_6391;
        auto SUB_6425 = ADD_2364 - MUL_6387;
        auto SUB_6426 = ADD_2365 - MUL_6389;
        auto SUB_6427 = ADD_2366 - MUL_6391;
        auto MUL_6471 = ADD_6104 * 2.0;
        auto MUL_6446 = ADD_6207 * 2.0;
        auto SUB_6449 = 1.0 - MUL_6446;
        auto MUL_2455 = SUB_2125 * 0.02;
        auto MUL_2464 = ADD_2119 * 0.02;
        auto MUL_2470 = ADD_2131 * MUL_2464;
        if (/*head vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (928, 962)
        if (/*left_gripper_base*/ sphere_environment_in_collision(
            environment, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_environment_in_collision(environment, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*left_lower_shoulder vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*left_upper_elbow vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*left_upper_shoulder vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*pedestal vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_hand vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_lower_elbow vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_lower_forearm vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_lower_shoulder vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_upper_elbow vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_upper_forearm vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_upper_shoulder vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*right_wrist vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        if (/*torso vs. left_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_2364, ADD_2365, ADD_2366, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6398, ADD_6399, ADD_6400, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_6425, SUB_6426, SUB_6427, 0.04))
            {
                return false;
            }
        }  // (962, 962)
        auto MUL_2457 = ADD_2131 * 0.069333;
        auto SUB_2458 = MUL_2455 - MUL_2457;
        auto MUL_2467 = SUB_2136 * SUB_2458;
        auto MUL_2460 = ADD_2119 * 0.069333;
        auto MUL_2468 = SUB_2125 * MUL_2460;
        auto ADD_2469 = MUL_2467 + MUL_2468;
        auto ADD_2471 = ADD_2469 + MUL_2470;
        auto MUL_2473 = ADD_2471 * 2.0;
        auto ADD_2496 = ADD_2364 + MUL_2473;
        auto MUL_6501 = MUL_6471 * 0.008;
        auto MUL_6484 = SUB_6449 * 0.005;
        auto SUB_6506 = MUL_6501 - MUL_6484;
        auto ADD_6509 = ADD_2496 + SUB_6506;
        auto MUL_6474 = SUB_6107 * 2.0;
        auto MUL_6503 = MUL_6474 * 0.008;
        auto MUL_6452 = ADD_6214 * 2.0;
        auto MUL_6488 = MUL_6452 * 0.005;
        auto SUB_6507 = MUL_6503 - MUL_6488;
        auto MUL_2476 = SUB_2136 * MUL_2464;
        auto MUL_2481 = ADD_2131 * SUB_2458;
        auto MUL_2478 = ADD_2119 * MUL_2460;
        auto ADD_2479 = MUL_2476 + MUL_2478;
        auto SUB_2482 = MUL_2481 - ADD_2479;
        auto MUL_2484 = SUB_2482 * 2.0;
        auto ADD_2486 = MUL_2484 + 0.069333;
        auto ADD_2497 = ADD_2365 + ADD_2486;
        auto ADD_6510 = ADD_2497 + SUB_6507;
        auto MUL_6478 = ADD_6110 * 2.0;
        auto SUB_6481 = 1.0 - MUL_6478;
        auto MUL_6505 = SUB_6481 * 0.008;
        auto MUL_6455 = SUB_6217 * 2.0;
        auto MUL_6492 = MUL_6455 * 0.005;
        auto SUB_6508 = MUL_6505 - MUL_6492;
        auto MUL_2487 = SUB_2136 * MUL_2460;
        auto MUL_2490 = SUB_2125 * SUB_2458;
        auto MUL_2488 = ADD_2119 * MUL_2464;
        auto SUB_2489 = MUL_2487 - MUL_2488;
        auto SUB_2491 = SUB_2489 - MUL_2490;
        auto MUL_2493 = SUB_2491 * 2.0;
        auto ADD_2495 = MUL_2493 + 0.02;
        auto ADD_2498 = ADD_2366 + ADD_2495;
        auto ADD_6511 = ADD_2498 + SUB_6508;
        auto MUL_6458 = SUB_6336 * 2.0;
        auto MUL_6525 = MUL_6458 * 0.012;
        auto SUB_6536 = MUL_6525 - MUL_6484;
        auto ADD_6539 = SUB_6536 + MUL_6501;
        auto ADD_6542 = ADD_2496 + ADD_6539;
        auto MUL_6462 = ADD_6339 * 2.0;
        auto SUB_6465 = 1.0 - MUL_6462;
        auto MUL_6527 = SUB_6465 * 0.012;
        auto SUB_6537 = MUL_6527 - MUL_6488;
        auto ADD_6540 = SUB_6537 + MUL_6503;
        auto ADD_6543 = ADD_2497 + ADD_6540;
        auto MUL_6468 = ADD_6346 * 2.0;
        auto MUL_6529 = MUL_6468 * 0.012;
        auto SUB_6538 = MUL_6529 - MUL_6492;
        auto ADD_6541 = SUB_6538 + MUL_6505;
        auto ADD_6544 = ADD_2498 + ADD_6541;
        auto ADD_6575 = MUL_6484 + MUL_6525;
        auto SUB_6581 = MUL_6501 - ADD_6575;
        auto ADD_6584 = ADD_2496 + SUB_6581;
        auto ADD_6577 = MUL_6488 + MUL_6527;
        auto SUB_6582 = MUL_6503 - ADD_6577;
        auto ADD_6585 = ADD_2497 + SUB_6582;
        auto ADD_6579 = MUL_6492 + MUL_6529;
        auto SUB_6583 = MUL_6505 - ADD_6579;
        auto ADD_6586 = ADD_2498 + SUB_6583;
        auto MUL_6630 = ADD_6104 * 2.0;
        auto MUL_6617 = SUB_6336 * 2.0;
        auto MUL_6654 = MUL_6630 * 0.05;
        auto MUL_6648 = MUL_6617 * 0.017;
        auto ADD_6659 = MUL_6648 + MUL_6654;
        auto ADD_6662 = ADD_2496 + ADD_6659;
        if (/*l_gripper_l_finger*/ sphere_environment_in_collision(
            environment, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_environment_in_collision(environment, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (962, 1035)
        if (/*left_lower_shoulder vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*left_upper_elbow vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*left_upper_shoulder vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*pedestal vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_hand vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_lower_elbow vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_lower_forearm vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_lower_shoulder vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_upper_elbow vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_upper_forearm vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_upper_shoulder vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*right_wrist vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        if (/*torso vs. l_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_6509, ADD_6510, ADD_6511, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6542, ADD_6543, ADD_6544, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6584, ADD_6585, ADD_6586, 0.015))
            {
                return false;
            }
        }  // (1035, 1035)
        auto MUL_6633 = SUB_6107 * 2.0;
        auto MUL_6656 = MUL_6633 * 0.05;
        auto MUL_6621 = ADD_6339 * 2.0;
        auto SUB_6624 = 1.0 - MUL_6621;
        auto MUL_6650 = SUB_6624 * 0.017;
        auto ADD_6660 = MUL_6650 + MUL_6656;
        auto ADD_6663 = ADD_2497 + ADD_6660;
        auto MUL_6637 = ADD_6110 * 2.0;
        auto SUB_6640 = 1.0 - MUL_6637;
        auto MUL_6658 = SUB_6640 * 0.05;
        auto MUL_6627 = ADD_6346 * 2.0;
        auto MUL_6652 = MUL_6627 * 0.017;
        auto ADD_6661 = MUL_6652 + MUL_6658;
        auto ADD_6664 = ADD_2498 + ADD_6661;
        auto MUL_6678 = MUL_6630 * 0.03;
        auto MUL_6672 = MUL_6617 * 0.01725;
        auto ADD_6683 = MUL_6672 + MUL_6678;
        auto ADD_6686 = ADD_2496 + ADD_6683;
        auto MUL_6680 = MUL_6633 * 0.03;
        auto MUL_6674 = SUB_6624 * 0.01725;
        auto ADD_6684 = MUL_6674 + MUL_6680;
        auto ADD_6687 = ADD_2497 + ADD_6684;
        auto MUL_6682 = SUB_6640 * 0.03;
        auto MUL_6676 = MUL_6627 * 0.01725;
        auto ADD_6685 = MUL_6676 + MUL_6682;
        auto ADD_6688 = ADD_2498 + ADD_6685;
        auto ADD_6707 = MUL_6672 + MUL_6654;
        auto ADD_6710 = ADD_2496 + ADD_6707;
        auto ADD_6708 = MUL_6674 + MUL_6656;
        auto ADD_6711 = ADD_2497 + ADD_6708;
        auto ADD_6709 = MUL_6676 + MUL_6658;
        auto ADD_6712 = ADD_2498 + ADD_6709;
        auto MUL_6726 = MUL_6630 * 0.07;
        auto ADD_6731 = MUL_6672 + MUL_6726;
        auto ADD_6734 = ADD_2496 + ADD_6731;
        auto MUL_6728 = MUL_6633 * 0.07;
        auto ADD_6732 = MUL_6674 + MUL_6728;
        auto ADD_6735 = ADD_2497 + ADD_6732;
        auto MUL_6730 = SUB_6640 * 0.07;
        auto ADD_6733 = MUL_6676 + MUL_6730;
        auto ADD_6736 = ADD_2498 + ADD_6733;
        auto MUL_6782 = ADD_6104 * 2.0;
        auto MUL_6813 = MUL_6782 * 0.015;
        auto MUL_6769 = SUB_6336 * 2.0;
        auto MUL_2707 = ADD_2131 * 0.01725;
        auto MUL_2710 = ADD_2119 * 0.01725;
        auto MUL_2718 = SUB_2125 * MUL_2710;
        if (/*l_gripper_l_finger_2*/ sphere_environment_in_collision(
            environment, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_environment_in_collision(environment, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1035, 1082)
        if (/*left_lower_shoulder vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*left_upper_elbow vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*left_upper_shoulder vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*pedestal vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_hand vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_lower_elbow vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_lower_forearm vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_lower_shoulder vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_upper_elbow vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_upper_forearm vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_upper_shoulder vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*right_wrist vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        if (/*torso vs. l_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_6662, ADD_6663, ADD_6664, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6686, ADD_6687, ADD_6688, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6710, ADD_6711, ADD_6712, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6734, ADD_6735, ADD_6736, 0.012))
            {
                return false;
            }
        }  // (1082, 1082)
        auto MUL_2705 = SUB_2125 * 0.1127;
        auto SUB_2708 = MUL_2705 - MUL_2707;
        auto MUL_2717 = SUB_2136 * SUB_2708;
        auto ADD_2719 = MUL_2717 + MUL_2718;
        auto MUL_2714 = ADD_2119 * 0.1127;
        auto MUL_2720 = ADD_2131 * MUL_2714;
        auto ADD_2721 = ADD_2719 + MUL_2720;
        auto MUL_2723 = ADD_2721 * 2.0;
        auto ADD_2746 = ADD_2496 + MUL_2723;
        auto MUL_6801 = MUL_6769 * 0.004;
        auto ADD_6823 = MUL_6801 + MUL_6813;
        auto SUB_6829 = ADD_2746 - ADD_6823;
        auto MUL_6785 = SUB_6107 * 2.0;
        auto MUL_6817 = MUL_6785 * 0.015;
        auto MUL_6773 = ADD_6339 * 2.0;
        auto SUB_6776 = 1.0 - MUL_6773;
        auto MUL_6805 = SUB_6776 * 0.004;
        auto ADD_6825 = MUL_6805 + MUL_6817;
        auto MUL_2726 = SUB_2136 * MUL_2714;
        auto MUL_2731 = ADD_2131 * SUB_2708;
        auto MUL_2728 = ADD_2119 * MUL_2710;
        auto ADD_2729 = MUL_2726 + MUL_2728;
        auto SUB_2732 = MUL_2731 - ADD_2729;
        auto MUL_2734 = SUB_2732 * 2.0;
        auto ADD_2736 = MUL_2734 + 0.01725;
        auto ADD_2747 = ADD_2497 + ADD_2736;
        auto SUB_6830 = ADD_2747 - ADD_6825;
        auto MUL_6789 = ADD_6110 * 2.0;
        auto SUB_6792 = 1.0 - MUL_6789;
        auto MUL_6821 = SUB_6792 * 0.015;
        auto MUL_6779 = ADD_6346 * 2.0;
        auto MUL_6809 = MUL_6779 * 0.004;
        auto ADD_6827 = MUL_6809 + MUL_6821;
        auto MUL_2737 = SUB_2136 * MUL_2710;
        auto MUL_2740 = SUB_2125 * SUB_2708;
        auto MUL_2738 = ADD_2119 * MUL_2714;
        auto SUB_2739 = MUL_2737 - MUL_2738;
        auto SUB_2741 = SUB_2739 - MUL_2740;
        auto MUL_2743 = SUB_2741 * 2.0;
        auto ADD_2745 = MUL_2743 + 0.1127;
        auto ADD_2748 = ADD_2498 + ADD_2745;
        auto SUB_6831 = ADD_2748 - ADD_6827;
        auto MUL_6852 = MUL_6782 * 0.005;
        auto MUL_6757 = ADD_6207 * 2.0;
        auto SUB_6760 = 1.0 - MUL_6757;
        auto MUL_6833 = SUB_6760 * 0.01;
        auto MUL_6840 = MUL_6769 * 0.0045;
        auto SUB_6862 = MUL_6833 - MUL_6840;
        auto SUB_6865 = SUB_6862 - MUL_6852;
        auto ADD_6868 = ADD_2746 + SUB_6865;
        auto MUL_6856 = MUL_6785 * 0.005;
        auto MUL_6844 = SUB_6776 * 0.0045;
        auto MUL_6763 = ADD_6214 * 2.0;
        auto MUL_6835 = MUL_6763 * 0.01;
        auto SUB_6863 = MUL_6835 - MUL_6844;
        auto SUB_6866 = SUB_6863 - MUL_6856;
        auto ADD_6869 = ADD_2747 + SUB_6866;
        auto MUL_6860 = SUB_6792 * 0.005;
        auto MUL_6848 = MUL_6779 * 0.0045;
        auto MUL_6766 = SUB_6217 * 2.0;
        auto MUL_6837 = MUL_6766 * 0.01;
        auto SUB_6864 = MUL_6837 - MUL_6848;
        auto SUB_6867 = SUB_6864 - MUL_6860;
        auto ADD_6870 = ADD_2748 + SUB_6867;
        auto ADD_6907 = MUL_6833 + MUL_6840;
        auto ADD_6913 = ADD_6907 + MUL_6852;
        auto SUB_6919 = ADD_2746 - ADD_6913;
        auto ADD_6909 = MUL_6835 + MUL_6844;
        auto ADD_6915 = ADD_6909 + MUL_6856;
        auto SUB_6920 = ADD_2747 - ADD_6915;
        auto ADD_6911 = MUL_6837 + MUL_6848;
        auto ADD_6917 = ADD_6911 + MUL_6860;
        auto SUB_6921 = ADD_2748 - ADD_6917;
        auto MUL_6948 = MUL_6782 * 0.025;
        auto ADD_6964 = ADD_6907 + MUL_6948;
        auto SUB_6970 = ADD_2746 - ADD_6964;
        auto MUL_6952 = MUL_6785 * 0.025;
        auto ADD_6966 = ADD_6909 + MUL_6952;
        auto SUB_6971 = ADD_2747 - ADD_6966;
        auto MUL_6956 = SUB_6792 * 0.025;
        auto ADD_6968 = ADD_6911 + MUL_6956;
        auto SUB_6972 = ADD_2748 - ADD_6968;
        auto SUB_7006 = SUB_6862 - MUL_6948;
        auto ADD_7009 = ADD_2746 + SUB_7006;
        auto SUB_7007 = SUB_6863 - MUL_6952;
        auto ADD_7010 = ADD_2747 + SUB_7007;
        auto SUB_7008 = SUB_6864 - MUL_6956;
        auto ADD_7011 = ADD_2748 + SUB_7008;
        auto ADD_2842 = MUL_2455 + MUL_2457;
        auto MUL_7059 = ADD_6104 * 2.0;
        auto MUL_7083 = MUL_7059 * 0.008;
        auto MUL_7034 = ADD_6207 * 2.0;
        auto SUB_7037 = 1.0 - MUL_7034;
        auto MUL_7071 = SUB_7037 * 0.005;
        if (/*head vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1082, 1176)
        if (/*l_gripper_l_finger_tip*/ sphere_environment_in_collision(
            environment, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_environment_in_collision(environment, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*left_lower_elbow vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*left_lower_shoulder vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*left_upper_elbow vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*left_upper_forearm vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*left_upper_shoulder vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*pedestal vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_hand vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_lower_elbow vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_lower_forearm vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_lower_shoulder vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_upper_elbow vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_upper_forearm vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_upper_shoulder vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*right_wrist vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        if (/*torso vs. l_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, SUB_6829, SUB_6830, SUB_6831, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_6868, ADD_6869, ADD_6870, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_6919, SUB_6920, SUB_6921, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_6970, SUB_6971, SUB_6972, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7009, ADD_7010, ADD_7011, 0.014))
            {
                return false;
            }
        }  // (1176, 1176)
        auto ADD_7088 = MUL_7071 + MUL_7083;
        auto MUL_2853 = SUB_2136 * ADD_2842;
        auto SUB_2856 = MUL_2853 - MUL_2468;
        auto ADD_2858 = SUB_2856 + MUL_2470;
        auto MUL_2860 = ADD_2858 * 2.0;
        auto ADD_2888 = ADD_2364 + MUL_2860;
        auto ADD_7091 = ADD_2888 + ADD_7088;
        auto SUB_2867 = MUL_2478 - MUL_2476;
        auto MUL_7062 = SUB_6107 * 2.0;
        auto MUL_7085 = MUL_7062 * 0.008;
        auto MUL_7040 = ADD_6214 * 2.0;
        auto MUL_7073 = MUL_7040 * 0.005;
        auto ADD_7089 = MUL_7073 + MUL_7085;
        auto MUL_2868 = ADD_2131 * ADD_2842;
        auto ADD_2869 = SUB_2867 + MUL_2868;
        auto MUL_2871 = ADD_2869 * 2.0;
        auto SUB_2874 = MUL_2871 - 0.069333;
        auto ADD_2889 = ADD_2365 + SUB_2874;
        auto ADD_7092 = ADD_2889 + ADD_7089;
        auto ADD_2878 = MUL_2487 + MUL_2488;
        auto MUL_7066 = ADD_6110 * 2.0;
        auto SUB_7069 = 1.0 - MUL_7066;
        auto MUL_7087 = SUB_7069 * 0.008;
        auto MUL_7043 = SUB_6217 * 2.0;
        auto MUL_7075 = MUL_7043 * 0.005;
        auto ADD_7090 = MUL_7075 + MUL_7087;
        auto MUL_2880 = SUB_2125 * ADD_2842;
        auto ADD_2881 = ADD_2878 + MUL_2880;
        auto MUL_2884 = ADD_2881 * 2.0;
        auto SUB_2887 = 0.02 - MUL_2884;
        auto ADD_2890 = ADD_2366 + SUB_2887;
        auto ADD_7093 = ADD_2890 + ADD_7090;
        auto MUL_7046 = SUB_6336 * 2.0;
        auto MUL_7101 = MUL_7046 * 0.01;
        auto ADD_7112 = MUL_7071 + MUL_7101;
        auto ADD_7115 = ADD_7112 + MUL_7083;
        auto ADD_7118 = ADD_2888 + ADD_7115;
        auto MUL_7050 = ADD_6339 * 2.0;
        auto SUB_7053 = 1.0 - MUL_7050;
        auto MUL_7103 = SUB_7053 * 0.01;
        auto ADD_7113 = MUL_7073 + MUL_7103;
        auto ADD_7116 = ADD_7113 + MUL_7085;
        auto ADD_7119 = ADD_2889 + ADD_7116;
        auto MUL_7056 = ADD_6346 * 2.0;
        auto MUL_7105 = MUL_7056 * 0.01;
        auto ADD_7114 = MUL_7075 + MUL_7105;
        auto ADD_7117 = ADD_7114 + MUL_7087;
        auto ADD_7120 = ADD_2890 + ADD_7117;
        auto SUB_7145 = MUL_7071 - MUL_7101;
        auto ADD_7148 = SUB_7145 + MUL_7083;
        auto ADD_7151 = ADD_2888 + ADD_7148;
        auto SUB_7146 = MUL_7073 - MUL_7103;
        auto ADD_7149 = SUB_7146 + MUL_7085;
        auto ADD_7152 = ADD_2889 + ADD_7149;
        auto SUB_7147 = MUL_7075 - MUL_7105;
        auto ADD_7150 = SUB_7147 + MUL_7087;
        auto ADD_7153 = ADD_2890 + ADD_7150;
        auto MUL_7197 = ADD_6104 * 2.0;
        auto MUL_7227 = MUL_7197 * 0.05;
        auto MUL_7184 = SUB_6336 * 2.0;
        auto MUL_7216 = MUL_7184 * 0.017;
        auto SUB_7232 = MUL_7227 - MUL_7216;
        auto ADD_7235 = ADD_2888 + SUB_7232;
        if (/*head vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1176, 1239)
        if (/*l_gripper_l_finger_tip vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*l_gripper_r_finger*/ sphere_environment_in_collision(
            environment, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_environment_in_collision(environment, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*left_lower_shoulder vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*left_upper_elbow vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*left_upper_shoulder vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*pedestal vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_hand vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_lower_elbow vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_lower_forearm vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_lower_shoulder vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_upper_elbow vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_upper_forearm vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_upper_shoulder vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*right_wrist vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        if (/*torso vs. l_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_7091, ADD_7092, ADD_7093, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7118, ADD_7119, ADD_7120, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7151, ADD_7152, ADD_7153, 0.015))
            {
                return false;
            }
        }  // (1239, 1239)
        auto MUL_7200 = SUB_6107 * 2.0;
        auto MUL_7229 = MUL_7200 * 0.05;
        auto MUL_7188 = ADD_6339 * 2.0;
        auto SUB_7191 = 1.0 - MUL_7188;
        auto MUL_7220 = SUB_7191 * 0.017;
        auto SUB_7233 = MUL_7229 - MUL_7220;
        auto ADD_7236 = ADD_2889 + SUB_7233;
        auto MUL_7204 = ADD_6110 * 2.0;
        auto SUB_7207 = 1.0 - MUL_7204;
        auto MUL_7231 = SUB_7207 * 0.05;
        auto MUL_7194 = ADD_6346 * 2.0;
        auto MUL_7224 = MUL_7194 * 0.017;
        auto SUB_7234 = MUL_7231 - MUL_7224;
        auto ADD_7237 = ADD_2890 + SUB_7234;
        auto MUL_7257 = MUL_7197 * 0.03;
        auto MUL_7246 = MUL_7184 * 0.01725;
        auto SUB_7262 = MUL_7257 - MUL_7246;
        auto ADD_7265 = ADD_2888 + SUB_7262;
        auto MUL_7259 = MUL_7200 * 0.03;
        auto MUL_7250 = SUB_7191 * 0.01725;
        auto SUB_7263 = MUL_7259 - MUL_7250;
        auto ADD_7266 = ADD_2889 + SUB_7263;
        auto MUL_7261 = SUB_7207 * 0.03;
        auto MUL_7254 = MUL_7194 * 0.01725;
        auto SUB_7264 = MUL_7261 - MUL_7254;
        auto ADD_7267 = ADD_2890 + SUB_7264;
        auto SUB_7292 = MUL_7227 - MUL_7246;
        auto ADD_7295 = ADD_2888 + SUB_7292;
        auto SUB_7293 = MUL_7229 - MUL_7250;
        auto ADD_7296 = ADD_2889 + SUB_7293;
        auto SUB_7294 = MUL_7231 - MUL_7254;
        auto ADD_7297 = ADD_2890 + SUB_7294;
        auto MUL_7317 = MUL_7197 * 0.07;
        auto SUB_7322 = MUL_7317 - MUL_7246;
        auto ADD_7325 = ADD_2888 + SUB_7322;
        auto MUL_7319 = MUL_7200 * 0.07;
        auto SUB_7323 = MUL_7319 - MUL_7250;
        auto ADD_7326 = ADD_2889 + SUB_7323;
        auto MUL_7321 = SUB_7207 * 0.07;
        auto SUB_7324 = MUL_7321 - MUL_7254;
        auto ADD_7327 = ADD_2890 + SUB_7324;
        auto ADD_3102 = MUL_2705 + MUL_2707;
        auto MUL_7373 = ADD_6104 * 2.0;
        auto MUL_7398 = MUL_7373 * 0.015;
        auto MUL_7360 = SUB_6336 * 2.0;
        auto MUL_7391 = MUL_7360 * 0.004;
        auto SUB_7408 = MUL_7391 - MUL_7398;
        if (/*head vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1239, 1286)
        if (/*l_gripper_l_finger_tip vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*l_gripper_r_finger_2*/ sphere_environment_in_collision(
            environment, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_environment_in_collision(environment, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*left_lower_shoulder vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*left_upper_elbow vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*left_upper_shoulder vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*pedestal vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_hand vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_lower_elbow vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_lower_forearm vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_lower_shoulder vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_upper_elbow vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_upper_forearm vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_upper_shoulder vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*right_wrist vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        if (/*torso vs. l_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_7235, ADD_7236, ADD_7237, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7265, ADD_7266, ADD_7267, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7295, ADD_7296, ADD_7297, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7325, ADD_7326, ADD_7327, 0.012))
            {
                return false;
            }
        }  // (1286, 1286)
        auto MUL_3113 = SUB_2136 * ADD_3102;
        auto SUB_3116 = MUL_3113 - MUL_2718;
        auto ADD_3118 = SUB_3116 + MUL_2720;
        auto MUL_3120 = ADD_3118 * 2.0;
        auto ADD_3148 = ADD_2888 + MUL_3120;
        auto ADD_7411 = ADD_3148 + SUB_7408;
        auto SUB_3127 = MUL_2728 - MUL_2726;
        auto MUL_7376 = SUB_6107 * 2.0;
        auto MUL_7402 = MUL_7376 * 0.015;
        auto MUL_7364 = ADD_6339 * 2.0;
        auto SUB_7367 = 1.0 - MUL_7364;
        auto MUL_7393 = SUB_7367 * 0.004;
        auto SUB_7409 = MUL_7393 - MUL_7402;
        auto MUL_3128 = ADD_2131 * ADD_3102;
        auto ADD_3129 = SUB_3127 + MUL_3128;
        auto MUL_3131 = ADD_3129 * 2.0;
        auto SUB_3134 = MUL_3131 - 0.01725;
        auto ADD_3149 = ADD_2889 + SUB_3134;
        auto ADD_7412 = ADD_3149 + SUB_7409;
        auto ADD_3138 = MUL_2737 + MUL_2738;
        auto MUL_7380 = ADD_6110 * 2.0;
        auto SUB_7383 = 1.0 - MUL_7380;
        auto MUL_7406 = SUB_7383 * 0.015;
        auto MUL_7370 = ADD_6346 * 2.0;
        auto MUL_7395 = MUL_7370 * 0.004;
        auto SUB_7410 = MUL_7395 - MUL_7406;
        auto MUL_3140 = SUB_2125 * ADD_3102;
        auto ADD_3141 = ADD_3138 + MUL_3140;
        auto MUL_3144 = ADD_3141 * 2.0;
        auto SUB_3147 = 0.1127 - MUL_3144;
        auto ADD_3150 = ADD_2890 + SUB_3147;
        auto ADD_7413 = ADD_3150 + SUB_7410;
        auto MUL_7428 = MUL_7373 * 0.005;
        auto MUL_7421 = MUL_7360 * 0.0045;
        auto MUL_7348 = ADD_6207 * 2.0;
        auto SUB_7351 = 1.0 - MUL_7348;
        auto MUL_7415 = SUB_7351 * 0.01;
        auto ADD_7438 = MUL_7415 + MUL_7421;
        auto SUB_7441 = ADD_7438 - MUL_7428;
        auto ADD_7444 = ADD_3148 + SUB_7441;
        auto MUL_7432 = MUL_7376 * 0.005;
        auto MUL_7423 = SUB_7367 * 0.0045;
        auto MUL_7354 = ADD_6214 * 2.0;
        auto MUL_7417 = MUL_7354 * 0.01;
        auto ADD_7439 = MUL_7417 + MUL_7423;
        auto SUB_7442 = ADD_7439 - MUL_7432;
        auto ADD_7445 = ADD_3149 + SUB_7442;
        auto MUL_7436 = SUB_7383 * 0.005;
        auto MUL_7425 = MUL_7370 * 0.0045;
        auto MUL_7357 = SUB_6217 * 2.0;
        auto MUL_7419 = MUL_7357 * 0.01;
        auto ADD_7440 = MUL_7419 + MUL_7425;
        auto SUB_7443 = ADD_7440 - MUL_7436;
        auto ADD_7446 = ADD_3150 + SUB_7443;
        auto SUB_7477 = MUL_7421 - MUL_7415;
        auto SUB_7480 = SUB_7477 - MUL_7428;
        auto ADD_7483 = ADD_3148 + SUB_7480;
        auto SUB_7478 = MUL_7423 - MUL_7417;
        auto SUB_7481 = SUB_7478 - MUL_7432;
        auto ADD_7484 = ADD_3149 + SUB_7481;
        auto SUB_7479 = MUL_7425 - MUL_7419;
        auto SUB_7482 = SUB_7479 - MUL_7436;
        auto ADD_7485 = ADD_3150 + SUB_7482;
        auto MUL_7506 = MUL_7373 * 0.025;
        auto SUB_7519 = SUB_7477 - MUL_7506;
        auto ADD_7522 = ADD_3148 + SUB_7519;
        auto MUL_7510 = MUL_7376 * 0.025;
        auto SUB_7520 = SUB_7478 - MUL_7510;
        auto ADD_7523 = ADD_3149 + SUB_7520;
        auto MUL_7514 = SUB_7383 * 0.025;
        auto SUB_7521 = SUB_7479 - MUL_7514;
        auto ADD_7524 = ADD_3150 + SUB_7521;
        auto SUB_7552 = ADD_7438 - MUL_7506;
        auto ADD_7555 = ADD_3148 + SUB_7552;
        auto SUB_7553 = ADD_7439 - MUL_7510;
        auto ADD_7556 = ADD_3149 + SUB_7553;
        auto SUB_7554 = ADD_7440 - MUL_7514;
        auto ADD_7557 = ADD_3150 + SUB_7554;
        auto MUL_3368 = SUB_987 * 0.025;
        auto MUL_3379 = SUB_998 * MUL_3368;
        auto MUL_3376 = ADD_981 * 0.025;
        auto MUL_3381 = ADD_993 * MUL_3376;
        auto ADD_3382 = MUL_3379 + MUL_3381;
        auto MUL_3384 = ADD_3382 * 2.0;
        if (/*head vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1286, 1370)
        if (/*l_gripper_l_finger vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*l_gripper_l_finger_2 vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*l_gripper_l_finger_tip vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*l_gripper_r_finger_tip*/ sphere_environment_in_collision(
            environment, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_environment_in_collision(environment, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*left_lower_elbow vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*left_lower_shoulder vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*left_upper_elbow vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*left_upper_forearm vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*left_upper_shoulder vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*pedestal vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_hand vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5356, ADD_5357, ADD_5358, 0.05, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_lower_elbow vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_lower_forearm vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_814, ADD_815, ADD_816, 0.1, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5086, ADD_5087, ADD_5088, 0.07, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5113, SUB_5114, SUB_5115, 0.07, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_lower_shoulder vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_upper_elbow vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_upper_forearm vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_upper_shoulder vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*right_wrist vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_5194, SUB_5195, SUB_5196, 0.105, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5215, ADD_5216, ADD_5217, 0.07, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_5242, SUB_5243, SUB_5244, 0.08, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        if (/*torso vs. l_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_7411, ADD_7412, ADD_7413, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7444, ADD_7445, ADD_7446, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7483, ADD_7484, ADD_7485, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7522, ADD_7523, ADD_7524, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7555, ADD_7556, ADD_7557, 0.014))
            {
                return false;
            }
        }  // (1370, 1370)
        auto ADD_3406 = ADD_1097 + MUL_3384;
        auto MUL_3387 = SUB_998 * MUL_3376;
        auto MUL_3390 = ADD_993 * MUL_3368;
        auto SUB_3391 = MUL_3390 - MUL_3387;
        auto MUL_3393 = SUB_3391 * 2.0;
        auto ADD_3407 = ADD_1098 + MUL_3393;
        auto MUL_3398 = SUB_987 * MUL_3368;
        auto MUL_3396 = ADD_981 * MUL_3376;
        auto ADD_3399 = MUL_3396 + MUL_3398;
        auto MUL_3402 = ADD_3399 * 2.0;
        auto SUB_3405 = 0.025 - MUL_3402;
        auto ADD_3408 = ADD_1099 + SUB_3405;
        auto SUB_7638 = MUL_5257 - MUL_5253;
        auto MUL_7640 = SUB_7638 * 2.0;
        auto MUL_7689 = MUL_7640 * 0.02;
        auto ADD_7700 = ADD_3406 + MUL_7689;
        auto ADD_7641 = MUL_5252 + MUL_5126;
        auto MUL_7644 = ADD_7641 * 2.0;
        auto SUB_7647 = 1.0 - MUL_7644;
        auto MUL_7691 = SUB_7647 * 0.02;
        auto ADD_7701 = ADD_3407 + MUL_7691;
        auto ADD_7648 = MUL_5130 + MUL_5127;
        auto MUL_7650 = ADD_7648 * 2.0;
        auto MUL_7693 = MUL_7650 * 0.02;
        auto ADD_7702 = ADD_3408 + MUL_7693;
        auto SUB_7727 = ADD_3406 - MUL_7689;
        auto SUB_7728 = ADD_3407 - MUL_7691;
        auto SUB_7729 = ADD_3408 - MUL_7693;
        auto MUL_3499 = ADD_993 * 0.069333;
        auto MUL_3497 = SUB_987 * 0.02;
        auto SUB_3500 = MUL_3497 - MUL_3499;
        auto MUL_3509 = SUB_998 * SUB_3500;
        auto MUL_3502 = ADD_981 * 0.069333;
        auto MUL_3510 = SUB_987 * MUL_3502;
        if (/*head vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1370, 1404)
        if (/*l_gripper_l_finger vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*l_gripper_l_finger_2 vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*l_gripper_l_finger_tip vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*l_gripper_r_finger vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*l_gripper_r_finger_2 vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*l_gripper_r_finger_tip vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7411, ADD_7412, ADD_7413, 0.028, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_gripper_base vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_hand vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_lower_elbow vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_lower_forearm vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_lower_shoulder vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_upper_elbow vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_upper_forearm vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_upper_shoulder vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*left_wrist vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*pedestal vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*right_gripper_base*/ sphere_environment_in_collision(
            environment, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_environment_in_collision(environment, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*right_lower_shoulder vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*right_upper_elbow vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*right_upper_shoulder vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        if (/*torso vs. right_gripper_base*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_3406, ADD_3407, ADD_3408, 0.06))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7700, ADD_7701, ADD_7702, 0.04))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_7727, SUB_7728, SUB_7729, 0.04))
            {
                return false;
            }
        }  // (1404, 1404)
        auto ADD_3511 = MUL_3509 + MUL_3510;
        auto MUL_3506 = ADD_981 * 0.02;
        auto MUL_3512 = ADD_993 * MUL_3506;
        auto ADD_3513 = ADD_3511 + MUL_3512;
        auto MUL_3515 = ADD_3513 * 2.0;
        auto ADD_3538 = ADD_3406 + MUL_3515;
        auto MUL_7773 = ADD_5157 * 2.0;
        auto MUL_7803 = MUL_7773 * 0.008;
        auto MUL_7748 = ADD_5260 * 2.0;
        auto SUB_7751 = 1.0 - MUL_7748;
        auto MUL_7786 = SUB_7751 * 0.005;
        auto SUB_7808 = MUL_7803 - MUL_7786;
        auto ADD_7811 = ADD_3538 + SUB_7808;
        auto MUL_3518 = SUB_998 * MUL_3506;
        auto MUL_3523 = ADD_993 * SUB_3500;
        auto MUL_3520 = ADD_981 * MUL_3502;
        auto ADD_3521 = MUL_3518 + MUL_3520;
        auto SUB_3524 = MUL_3523 - ADD_3521;
        auto MUL_3526 = SUB_3524 * 2.0;
        auto ADD_3528 = MUL_3526 + 0.069333;
        auto ADD_3539 = ADD_3407 + ADD_3528;
        auto MUL_7776 = SUB_5160 * 2.0;
        auto MUL_7805 = MUL_7776 * 0.008;
        auto MUL_7754 = ADD_5267 * 2.0;
        auto MUL_7790 = MUL_7754 * 0.005;
        auto SUB_7809 = MUL_7805 - MUL_7790;
        auto ADD_7812 = ADD_3539 + SUB_7809;
        auto MUL_3529 = SUB_998 * MUL_3502;
        auto MUL_3532 = SUB_987 * SUB_3500;
        auto MUL_3530 = ADD_981 * MUL_3506;
        auto SUB_3531 = MUL_3529 - MUL_3530;
        auto SUB_3533 = SUB_3531 - MUL_3532;
        auto MUL_3535 = SUB_3533 * 2.0;
        auto ADD_3537 = MUL_3535 + 0.02;
        auto ADD_3540 = ADD_3408 + ADD_3537;
        auto MUL_7780 = ADD_5163 * 2.0;
        auto SUB_7783 = 1.0 - MUL_7780;
        auto MUL_7807 = SUB_7783 * 0.008;
        auto MUL_7757 = SUB_5270 * 2.0;
        auto MUL_7794 = MUL_7757 * 0.005;
        auto SUB_7810 = MUL_7807 - MUL_7794;
        auto ADD_7813 = ADD_3540 + SUB_7810;
        auto MUL_7760 = SUB_7638 * 2.0;
        auto MUL_7827 = MUL_7760 * 0.012;
        auto SUB_7838 = MUL_7827 - MUL_7786;
        auto ADD_7841 = SUB_7838 + MUL_7803;
        auto ADD_7844 = ADD_3538 + ADD_7841;
        auto MUL_7764 = ADD_7641 * 2.0;
        auto SUB_7767 = 1.0 - MUL_7764;
        auto MUL_7829 = SUB_7767 * 0.012;
        auto SUB_7839 = MUL_7829 - MUL_7790;
        auto ADD_7842 = SUB_7839 + MUL_7805;
        auto ADD_7845 = ADD_3539 + ADD_7842;
        auto MUL_7770 = ADD_7648 * 2.0;
        auto MUL_7831 = MUL_7770 * 0.012;
        auto SUB_7840 = MUL_7831 - MUL_7794;
        auto ADD_7843 = SUB_7840 + MUL_7807;
        auto ADD_7846 = ADD_3540 + ADD_7843;
        auto ADD_7877 = MUL_7786 + MUL_7827;
        auto SUB_7883 = MUL_7803 - ADD_7877;
        auto ADD_7886 = ADD_3538 + SUB_7883;
        auto ADD_7879 = MUL_7790 + MUL_7829;
        auto SUB_7884 = MUL_7805 - ADD_7879;
        auto ADD_7887 = ADD_3539 + SUB_7884;
        auto ADD_7881 = MUL_7794 + MUL_7831;
        auto SUB_7885 = MUL_7807 - ADD_7881;
        auto ADD_7888 = ADD_3540 + SUB_7885;
        auto MUL_7932 = ADD_5157 * 2.0;
        auto MUL_7956 = MUL_7932 * 0.05;
        auto MUL_7919 = SUB_7638 * 2.0;
        auto MUL_7950 = MUL_7919 * 0.017;
        auto ADD_7961 = MUL_7950 + MUL_7956;
        auto ADD_7964 = ADD_3538 + ADD_7961;
        if (/*l_gripper_l_finger vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1404, 1477)
        if (/*l_gripper_l_finger_2 vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*l_gripper_l_finger_tip vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*l_gripper_r_finger vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*l_gripper_r_finger_2 vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*l_gripper_r_finger_tip vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7411, ADD_7412, ADD_7413, 0.028, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_gripper_base vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_hand vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_lower_elbow vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_lower_forearm vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_lower_shoulder vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_upper_elbow vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_upper_forearm vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_upper_shoulder vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*left_wrist vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*pedestal vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*r_gripper_l_finger*/ sphere_environment_in_collision(
            environment, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_environment_in_collision(environment, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*right_lower_shoulder vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*right_upper_elbow vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*right_upper_shoulder vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        if (/*torso vs. r_gripper_l_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_7811, ADD_7812, ADD_7813, 0.027))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7844, ADD_7845, ADD_7846, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7886, ADD_7887, ADD_7888, 0.015))
            {
                return false;
            }
        }  // (1477, 1477)
        auto MUL_7935 = SUB_5160 * 2.0;
        auto MUL_7958 = MUL_7935 * 0.05;
        auto MUL_7923 = ADD_7641 * 2.0;
        auto SUB_7926 = 1.0 - MUL_7923;
        auto MUL_7952 = SUB_7926 * 0.017;
        auto ADD_7962 = MUL_7952 + MUL_7958;
        auto ADD_7965 = ADD_3539 + ADD_7962;
        auto MUL_7939 = ADD_5163 * 2.0;
        auto SUB_7942 = 1.0 - MUL_7939;
        auto MUL_7960 = SUB_7942 * 0.05;
        auto MUL_7929 = ADD_7648 * 2.0;
        auto MUL_7954 = MUL_7929 * 0.017;
        auto ADD_7963 = MUL_7954 + MUL_7960;
        auto ADD_7966 = ADD_3540 + ADD_7963;
        auto MUL_7980 = MUL_7932 * 0.03;
        auto MUL_7974 = MUL_7919 * 0.01725;
        auto ADD_7985 = MUL_7974 + MUL_7980;
        auto ADD_7988 = ADD_3538 + ADD_7985;
        auto MUL_7982 = MUL_7935 * 0.03;
        auto MUL_7976 = SUB_7926 * 0.01725;
        auto ADD_7986 = MUL_7976 + MUL_7982;
        auto ADD_7989 = ADD_3539 + ADD_7986;
        auto MUL_7984 = SUB_7942 * 0.03;
        auto MUL_7978 = MUL_7929 * 0.01725;
        auto ADD_7987 = MUL_7978 + MUL_7984;
        auto ADD_7990 = ADD_3540 + ADD_7987;
        auto ADD_8009 = MUL_7974 + MUL_7956;
        auto ADD_8012 = ADD_3538 + ADD_8009;
        auto ADD_8010 = MUL_7976 + MUL_7958;
        auto ADD_8013 = ADD_3539 + ADD_8010;
        auto ADD_8011 = MUL_7978 + MUL_7960;
        auto ADD_8014 = ADD_3540 + ADD_8011;
        auto MUL_8028 = MUL_7932 * 0.07;
        auto ADD_8033 = MUL_7974 + MUL_8028;
        auto ADD_8036 = ADD_3538 + ADD_8033;
        auto MUL_8030 = MUL_7935 * 0.07;
        auto ADD_8034 = MUL_7976 + MUL_8030;
        auto ADD_8037 = ADD_3539 + ADD_8034;
        auto MUL_8032 = SUB_7942 * 0.07;
        auto ADD_8035 = MUL_7978 + MUL_8032;
        auto ADD_8038 = ADD_3540 + ADD_8035;
        auto MUL_3749 = ADD_993 * 0.01725;
        auto MUL_3747 = SUB_987 * 0.1127;
        auto SUB_3750 = MUL_3747 - MUL_3749;
        auto MUL_3759 = SUB_998 * SUB_3750;
        auto MUL_3756 = ADD_981 * 0.1127;
        auto MUL_3762 = ADD_993 * MUL_3756;
        if (/*l_gripper_l_finger vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1477, 1524)
        if (/*l_gripper_l_finger_2 vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*l_gripper_l_finger_tip vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*l_gripper_r_finger vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*l_gripper_r_finger_2 vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*l_gripper_r_finger_tip vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7411, ADD_7412, ADD_7413, 0.028, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_gripper_base vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_hand vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_lower_elbow vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_lower_forearm vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_lower_shoulder vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_upper_elbow vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_upper_forearm vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_upper_shoulder vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*left_wrist vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*pedestal vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*r_gripper_l_finger_2*/ sphere_environment_in_collision(
            environment, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_environment_in_collision(environment, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*right_lower_shoulder vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*right_upper_elbow vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*right_upper_shoulder vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        if (/*torso vs. r_gripper_l_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_7964, ADD_7965, ADD_7966, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_7988, ADD_7989, ADD_7990, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8012, ADD_8013, ADD_8014, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8036, ADD_8037, ADD_8038, 0.012))
            {
                return false;
            }
        }  // (1524, 1524)
        auto MUL_3752 = ADD_981 * 0.01725;
        auto MUL_3760 = SUB_987 * MUL_3752;
        auto ADD_3761 = MUL_3759 + MUL_3760;
        auto ADD_3763 = ADD_3761 + MUL_3762;
        auto MUL_3765 = ADD_3763 * 2.0;
        auto ADD_3788 = ADD_3538 + MUL_3765;
        auto MUL_8084 = ADD_5157 * 2.0;
        auto MUL_8115 = MUL_8084 * 0.015;
        auto MUL_8071 = SUB_7638 * 2.0;
        auto MUL_8103 = MUL_8071 * 0.004;
        auto ADD_8125 = MUL_8103 + MUL_8115;
        auto SUB_8131 = ADD_3788 - ADD_8125;
        auto MUL_3768 = SUB_998 * MUL_3756;
        auto MUL_3773 = ADD_993 * SUB_3750;
        auto MUL_3770 = ADD_981 * MUL_3752;
        auto ADD_3771 = MUL_3768 + MUL_3770;
        auto SUB_3774 = MUL_3773 - ADD_3771;
        auto MUL_3776 = SUB_3774 * 2.0;
        auto ADD_3778 = MUL_3776 + 0.01725;
        auto ADD_3789 = ADD_3539 + ADD_3778;
        auto MUL_8087 = SUB_5160 * 2.0;
        auto MUL_8119 = MUL_8087 * 0.015;
        auto MUL_8075 = ADD_7641 * 2.0;
        auto SUB_8078 = 1.0 - MUL_8075;
        auto MUL_8107 = SUB_8078 * 0.004;
        auto ADD_8127 = MUL_8107 + MUL_8119;
        auto SUB_8132 = ADD_3789 - ADD_8127;
        auto MUL_3779 = SUB_998 * MUL_3752;
        auto MUL_3782 = SUB_987 * SUB_3750;
        auto MUL_3780 = ADD_981 * MUL_3756;
        auto SUB_3781 = MUL_3779 - MUL_3780;
        auto SUB_3783 = SUB_3781 - MUL_3782;
        auto MUL_3785 = SUB_3783 * 2.0;
        auto ADD_3787 = MUL_3785 + 0.1127;
        auto ADD_3790 = ADD_3540 + ADD_3787;
        auto MUL_8091 = ADD_5163 * 2.0;
        auto SUB_8094 = 1.0 - MUL_8091;
        auto MUL_8123 = SUB_8094 * 0.015;
        auto MUL_8081 = ADD_7648 * 2.0;
        auto MUL_8111 = MUL_8081 * 0.004;
        auto ADD_8129 = MUL_8111 + MUL_8123;
        auto SUB_8133 = ADD_3790 - ADD_8129;
        auto MUL_8154 = MUL_8084 * 0.005;
        auto MUL_8142 = MUL_8071 * 0.0045;
        auto MUL_8059 = ADD_5260 * 2.0;
        auto SUB_8062 = 1.0 - MUL_8059;
        auto MUL_8135 = SUB_8062 * 0.01;
        auto SUB_8164 = MUL_8135 - MUL_8142;
        auto SUB_8167 = SUB_8164 - MUL_8154;
        auto ADD_8170 = ADD_3788 + SUB_8167;
        auto MUL_8158 = MUL_8087 * 0.005;
        auto MUL_8146 = SUB_8078 * 0.0045;
        auto MUL_8065 = ADD_5267 * 2.0;
        auto MUL_8137 = MUL_8065 * 0.01;
        auto SUB_8165 = MUL_8137 - MUL_8146;
        auto SUB_8168 = SUB_8165 - MUL_8158;
        auto ADD_8171 = ADD_3789 + SUB_8168;
        auto MUL_8162 = SUB_8094 * 0.005;
        auto MUL_8150 = MUL_8081 * 0.0045;
        auto MUL_8068 = SUB_5270 * 2.0;
        auto MUL_8139 = MUL_8068 * 0.01;
        auto SUB_8166 = MUL_8139 - MUL_8150;
        auto SUB_8169 = SUB_8166 - MUL_8162;
        auto ADD_8172 = ADD_3790 + SUB_8169;
        auto ADD_8209 = MUL_8135 + MUL_8142;
        auto ADD_8215 = ADD_8209 + MUL_8154;
        auto SUB_8221 = ADD_3788 - ADD_8215;
        auto ADD_8211 = MUL_8137 + MUL_8146;
        auto ADD_8217 = ADD_8211 + MUL_8158;
        auto SUB_8222 = ADD_3789 - ADD_8217;
        auto ADD_8213 = MUL_8139 + MUL_8150;
        auto ADD_8219 = ADD_8213 + MUL_8162;
        auto SUB_8223 = ADD_3790 - ADD_8219;
        auto MUL_8250 = MUL_8084 * 0.025;
        auto ADD_8266 = ADD_8209 + MUL_8250;
        auto SUB_8272 = ADD_3788 - ADD_8266;
        auto MUL_8254 = MUL_8087 * 0.025;
        auto ADD_8268 = ADD_8211 + MUL_8254;
        auto SUB_8273 = ADD_3789 - ADD_8268;
        auto MUL_8258 = SUB_8094 * 0.025;
        auto ADD_8270 = ADD_8213 + MUL_8258;
        auto SUB_8274 = ADD_3790 - ADD_8270;
        auto SUB_8308 = SUB_8164 - MUL_8250;
        auto ADD_8311 = ADD_3788 + SUB_8308;
        auto SUB_8309 = SUB_8165 - MUL_8254;
        auto ADD_8312 = ADD_3789 + SUB_8309;
        auto SUB_8310 = SUB_8166 - MUL_8258;
        auto ADD_8313 = ADD_3790 + SUB_8310;
        auto ADD_3884 = MUL_3497 + MUL_3499;
        auto MUL_3895 = SUB_998 * ADD_3884;
        auto SUB_3898 = MUL_3895 - MUL_3510;
        auto ADD_3900 = SUB_3898 + MUL_3512;
        auto MUL_3902 = ADD_3900 * 2.0;
        auto ADD_3930 = ADD_3406 + MUL_3902;
        if (/*head vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1524, 1618)
        if (/*l_gripper_l_finger vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*l_gripper_l_finger_2 vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*l_gripper_l_finger_tip vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_6829, SUB_6830, SUB_6831, 0.028, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*l_gripper_r_finger vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*l_gripper_r_finger_2 vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*l_gripper_r_finger_tip vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<
            decltype(q[0])>(ADD_7411, ADD_7412, ADD_7413, 0.028, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_gripper_base vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_hand vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_lower_elbow vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_lower_forearm vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_lower_shoulder vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_upper_elbow vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_upper_forearm vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_upper_shoulder vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*left_wrist vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*pedestal vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*r_gripper_l_finger_tip*/ sphere_environment_in_collision(
            environment, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_environment_in_collision(environment, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*right_lower_elbow vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*right_lower_shoulder vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*right_upper_elbow vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*right_upper_forearm vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*right_upper_shoulder vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        if (/*torso vs. r_gripper_l_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, SUB_8131, SUB_8132, SUB_8133, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8170, ADD_8171, ADD_8172, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_8221, SUB_8222, SUB_8223, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, SUB_8272, SUB_8273, SUB_8274, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8311, ADD_8312, ADD_8313, 0.014))
            {
                return false;
            }
        }  // (1618, 1618)
        auto MUL_8361 = ADD_5157 * 2.0;
        auto MUL_8385 = MUL_8361 * 0.008;
        auto MUL_8336 = ADD_5260 * 2.0;
        auto SUB_8339 = 1.0 - MUL_8336;
        auto MUL_8373 = SUB_8339 * 0.005;
        auto ADD_8390 = MUL_8373 + MUL_8385;
        auto ADD_8393 = ADD_3930 + ADD_8390;
        auto SUB_3909 = MUL_3520 - MUL_3518;
        auto MUL_3910 = ADD_993 * ADD_3884;
        auto ADD_3911 = SUB_3909 + MUL_3910;
        auto MUL_3913 = ADD_3911 * 2.0;
        auto SUB_3916 = MUL_3913 - 0.069333;
        auto ADD_3931 = ADD_3407 + SUB_3916;
        auto MUL_8364 = SUB_5160 * 2.0;
        auto MUL_8387 = MUL_8364 * 0.008;
        auto MUL_8342 = ADD_5267 * 2.0;
        auto MUL_8375 = MUL_8342 * 0.005;
        auto ADD_8391 = MUL_8375 + MUL_8387;
        auto ADD_8394 = ADD_3931 + ADD_8391;
        auto ADD_3920 = MUL_3529 + MUL_3530;
        auto MUL_3922 = SUB_987 * ADD_3884;
        auto ADD_3923 = ADD_3920 + MUL_3922;
        auto MUL_3926 = ADD_3923 * 2.0;
        auto SUB_3929 = 0.02 - MUL_3926;
        auto ADD_3932 = ADD_3408 + SUB_3929;
        auto MUL_8368 = ADD_5163 * 2.0;
        auto SUB_8371 = 1.0 - MUL_8368;
        auto MUL_8389 = SUB_8371 * 0.008;
        auto MUL_8345 = SUB_5270 * 2.0;
        auto MUL_8377 = MUL_8345 * 0.005;
        auto ADD_8392 = MUL_8377 + MUL_8389;
        auto ADD_8395 = ADD_3932 + ADD_8392;
        auto MUL_8348 = SUB_7638 * 2.0;
        auto MUL_8403 = MUL_8348 * 0.01;
        auto ADD_8414 = MUL_8373 + MUL_8403;
        auto ADD_8417 = ADD_8414 + MUL_8385;
        auto ADD_8420 = ADD_3930 + ADD_8417;
        auto MUL_8352 = ADD_7641 * 2.0;
        auto SUB_8355 = 1.0 - MUL_8352;
        auto MUL_8405 = SUB_8355 * 0.01;
        auto ADD_8415 = MUL_8375 + MUL_8405;
        auto ADD_8418 = ADD_8415 + MUL_8387;
        auto ADD_8421 = ADD_3931 + ADD_8418;
        auto MUL_8358 = ADD_7648 * 2.0;
        auto MUL_8407 = MUL_8358 * 0.01;
        auto ADD_8416 = MUL_8377 + MUL_8407;
        auto ADD_8419 = ADD_8416 + MUL_8389;
        auto ADD_8422 = ADD_3932 + ADD_8419;
        auto SUB_8447 = MUL_8373 - MUL_8403;
        auto ADD_8450 = SUB_8447 + MUL_8385;
        auto ADD_8453 = ADD_3930 + ADD_8450;
        auto SUB_8448 = MUL_8375 - MUL_8405;
        auto ADD_8451 = SUB_8448 + MUL_8387;
        auto ADD_8454 = ADD_3931 + ADD_8451;
        auto SUB_8449 = MUL_8377 - MUL_8407;
        auto ADD_8452 = SUB_8449 + MUL_8389;
        auto ADD_8455 = ADD_3932 + ADD_8452;
        auto MUL_8499 = ADD_5157 * 2.0;
        auto MUL_8529 = MUL_8499 * 0.05;
        auto MUL_8486 = SUB_7638 * 2.0;
        auto MUL_8518 = MUL_8486 * 0.017;
        auto SUB_8534 = MUL_8529 - MUL_8518;
        auto ADD_8537 = ADD_3930 + SUB_8534;
        if (/*head vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1618, 1681)
        if (/*l_gripper_l_finger vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*l_gripper_l_finger_2 vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*l_gripper_l_finger_tip vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*l_gripper_r_finger vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*l_gripper_r_finger_2 vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*l_gripper_r_finger_tip vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7411, ADD_7412, ADD_7413, 0.028, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_gripper_base vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_hand vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_lower_elbow vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_lower_forearm vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_lower_shoulder vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_upper_elbow vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_upper_forearm vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_upper_shoulder vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*left_wrist vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*pedestal vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*r_gripper_l_finger_tip vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_8131, SUB_8132, SUB_8133, 0.028, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*r_gripper_r_finger*/ sphere_environment_in_collision(
            environment, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_environment_in_collision(environment, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*right_lower_shoulder vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*right_upper_elbow vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*right_upper_shoulder vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        if (/*torso vs. r_gripper_r_finger*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_8393, ADD_8394, ADD_8395, 0.025))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8420, ADD_8421, ADD_8422, 0.015))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8453, ADD_8454, ADD_8455, 0.015))
            {
                return false;
            }
        }  // (1681, 1681)
        auto MUL_8502 = SUB_5160 * 2.0;
        auto MUL_8531 = MUL_8502 * 0.05;
        auto MUL_8490 = ADD_7641 * 2.0;
        auto SUB_8493 = 1.0 - MUL_8490;
        auto MUL_8522 = SUB_8493 * 0.017;
        auto SUB_8535 = MUL_8531 - MUL_8522;
        auto ADD_8538 = ADD_3931 + SUB_8535;
        auto MUL_8506 = ADD_5163 * 2.0;
        auto SUB_8509 = 1.0 - MUL_8506;
        auto MUL_8533 = SUB_8509 * 0.05;
        auto MUL_8496 = ADD_7648 * 2.0;
        auto MUL_8526 = MUL_8496 * 0.017;
        auto SUB_8536 = MUL_8533 - MUL_8526;
        auto ADD_8539 = ADD_3932 + SUB_8536;
        auto MUL_8559 = MUL_8499 * 0.03;
        auto MUL_8548 = MUL_8486 * 0.01725;
        auto SUB_8564 = MUL_8559 - MUL_8548;
        auto ADD_8567 = ADD_3930 + SUB_8564;
        auto MUL_8561 = MUL_8502 * 0.03;
        auto MUL_8552 = SUB_8493 * 0.01725;
        auto SUB_8565 = MUL_8561 - MUL_8552;
        auto ADD_8568 = ADD_3931 + SUB_8565;
        auto MUL_8563 = SUB_8509 * 0.03;
        auto MUL_8556 = MUL_8496 * 0.01725;
        auto SUB_8566 = MUL_8563 - MUL_8556;
        auto ADD_8569 = ADD_3932 + SUB_8566;
        auto SUB_8594 = MUL_8529 - MUL_8548;
        auto ADD_8597 = ADD_3930 + SUB_8594;
        auto SUB_8595 = MUL_8531 - MUL_8552;
        auto ADD_8598 = ADD_3931 + SUB_8595;
        auto SUB_8596 = MUL_8533 - MUL_8556;
        auto ADD_8599 = ADD_3932 + SUB_8596;
        auto MUL_8619 = MUL_8499 * 0.07;
        auto SUB_8624 = MUL_8619 - MUL_8548;
        auto ADD_8627 = ADD_3930 + SUB_8624;
        auto MUL_8621 = MUL_8502 * 0.07;
        auto SUB_8625 = MUL_8621 - MUL_8552;
        auto ADD_8628 = ADD_3931 + SUB_8625;
        auto MUL_8623 = SUB_8509 * 0.07;
        auto SUB_8626 = MUL_8623 - MUL_8556;
        auto ADD_8629 = ADD_3932 + SUB_8626;
        auto ADD_4144 = MUL_3747 + MUL_3749;
        auto MUL_4155 = SUB_998 * ADD_4144;
        auto SUB_4158 = MUL_4155 - MUL_3760;
        auto ADD_4160 = SUB_4158 + MUL_3762;
        auto MUL_4162 = ADD_4160 * 2.0;
        auto ADD_4190 = ADD_3930 + MUL_4162;
        if (/*head vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1681, 1728)
        if (/*l_gripper_l_finger vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*l_gripper_l_finger_2 vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*l_gripper_l_finger_tip vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*l_gripper_r_finger vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*l_gripper_r_finger_2 vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*l_gripper_r_finger_tip vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7411, ADD_7412, ADD_7413, 0.028, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_gripper_base vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_hand vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_lower_elbow vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_lower_forearm vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_lower_shoulder vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_upper_elbow vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_upper_forearm vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_upper_shoulder vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*left_wrist vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*pedestal vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*r_gripper_l_finger_tip vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_8131, SUB_8132, SUB_8133, 0.028, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*r_gripper_r_finger_2*/ sphere_environment_in_collision(
            environment, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_environment_in_collision(environment, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*right_lower_shoulder vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*right_upper_elbow vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*right_upper_shoulder vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        if (/*torso vs. r_gripper_r_finger_2*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_8537, ADD_8538, ADD_8539, 0.032))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8567, ADD_8568, ADD_8569, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8597, ADD_8598, ADD_8599, 0.012))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8627, ADD_8628, ADD_8629, 0.012))
            {
                return false;
            }
        }  // (1728, 1728)
        auto MUL_8675 = ADD_5157 * 2.0;
        auto MUL_8700 = MUL_8675 * 0.015;
        auto MUL_8662 = SUB_7638 * 2.0;
        auto MUL_8693 = MUL_8662 * 0.004;
        auto SUB_8710 = MUL_8693 - MUL_8700;
        auto ADD_8713 = ADD_4190 + SUB_8710;
        auto SUB_4169 = MUL_3770 - MUL_3768;
        auto MUL_4170 = ADD_993 * ADD_4144;
        auto ADD_4171 = SUB_4169 + MUL_4170;
        auto MUL_4173 = ADD_4171 * 2.0;
        auto SUB_4176 = MUL_4173 - 0.01725;
        auto ADD_4191 = ADD_3931 + SUB_4176;
        auto MUL_8678 = SUB_5160 * 2.0;
        auto MUL_8704 = MUL_8678 * 0.015;
        auto MUL_8666 = ADD_7641 * 2.0;
        auto SUB_8669 = 1.0 - MUL_8666;
        auto MUL_8695 = SUB_8669 * 0.004;
        auto SUB_8711 = MUL_8695 - MUL_8704;
        auto ADD_8714 = ADD_4191 + SUB_8711;
        auto ADD_4180 = MUL_3779 + MUL_3780;
        auto MUL_4182 = SUB_987 * ADD_4144;
        auto ADD_4183 = ADD_4180 + MUL_4182;
        auto MUL_4186 = ADD_4183 * 2.0;
        auto SUB_4189 = 0.1127 - MUL_4186;
        auto ADD_4192 = ADD_3932 + SUB_4189;
        auto MUL_8682 = ADD_5163 * 2.0;
        auto SUB_8685 = 1.0 - MUL_8682;
        auto MUL_8708 = SUB_8685 * 0.015;
        auto MUL_8672 = ADD_7648 * 2.0;
        auto MUL_8697 = MUL_8672 * 0.004;
        auto SUB_8712 = MUL_8697 - MUL_8708;
        auto ADD_8715 = ADD_4192 + SUB_8712;
        auto MUL_8730 = MUL_8675 * 0.005;
        auto MUL_8723 = MUL_8662 * 0.0045;
        auto MUL_8650 = ADD_5260 * 2.0;
        auto SUB_8653 = 1.0 - MUL_8650;
        auto MUL_8717 = SUB_8653 * 0.01;
        auto ADD_8740 = MUL_8717 + MUL_8723;
        auto SUB_8743 = ADD_8740 - MUL_8730;
        auto ADD_8746 = ADD_4190 + SUB_8743;
        auto MUL_8734 = MUL_8678 * 0.005;
        auto MUL_8725 = SUB_8669 * 0.0045;
        auto MUL_8656 = ADD_5267 * 2.0;
        auto MUL_8719 = MUL_8656 * 0.01;
        auto ADD_8741 = MUL_8719 + MUL_8725;
        auto SUB_8744 = ADD_8741 - MUL_8734;
        auto ADD_8747 = ADD_4191 + SUB_8744;
        auto MUL_8738 = SUB_8685 * 0.005;
        auto MUL_8727 = MUL_8672 * 0.0045;
        auto MUL_8659 = SUB_5270 * 2.0;
        auto MUL_8721 = MUL_8659 * 0.01;
        auto ADD_8742 = MUL_8721 + MUL_8727;
        auto SUB_8745 = ADD_8742 - MUL_8738;
        auto ADD_8748 = ADD_4192 + SUB_8745;
        auto SUB_8779 = MUL_8723 - MUL_8717;
        auto SUB_8782 = SUB_8779 - MUL_8730;
        auto ADD_8785 = ADD_4190 + SUB_8782;
        auto SUB_8780 = MUL_8725 - MUL_8719;
        auto SUB_8783 = SUB_8780 - MUL_8734;
        auto ADD_8786 = ADD_4191 + SUB_8783;
        auto SUB_8781 = MUL_8727 - MUL_8721;
        auto SUB_8784 = SUB_8781 - MUL_8738;
        auto ADD_8787 = ADD_4192 + SUB_8784;
        auto MUL_8808 = MUL_8675 * 0.025;
        auto SUB_8821 = SUB_8779 - MUL_8808;
        auto ADD_8824 = ADD_4190 + SUB_8821;
        auto MUL_8812 = MUL_8678 * 0.025;
        auto SUB_8822 = SUB_8780 - MUL_8812;
        auto ADD_8825 = ADD_4191 + SUB_8822;
        auto MUL_8816 = SUB_8685 * 0.025;
        auto SUB_8823 = SUB_8781 - MUL_8816;
        auto ADD_8826 = ADD_4192 + SUB_8823;
        auto SUB_8854 = ADD_8740 - MUL_8808;
        auto ADD_8857 = ADD_4190 + SUB_8854;
        auto SUB_8855 = ADD_8741 - MUL_8812;
        auto ADD_8858 = ADD_4191 + SUB_8855;
        auto SUB_8856 = ADD_8742 - MUL_8816;
        auto ADD_8859 = ADD_4192 + SUB_8856;
        if (/*head vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.04, 0.0, 0.686, 0.2, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.04, 0.0, 0.686, 0.2, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1728, 1812)
        if (/*l_gripper_l_finger vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6509, ADD_6510, ADD_6511, 0.027, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6542, ADD_6543, ADD_6544, 0.015, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6584, ADD_6585, ADD_6586, 0.015, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*l_gripper_l_finger_2 vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6662, ADD_6663, ADD_6664, 0.032, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6686, ADD_6687, ADD_6688, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6710, ADD_6711, ADD_6712, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6734, ADD_6735, ADD_6736, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*l_gripper_l_finger_tip vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_6829, SUB_6830, SUB_6831, 0.028, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6868, ADD_6869, ADD_6870, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6919, SUB_6920, SUB_6921, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6970, SUB_6971, SUB_6972, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7009, ADD_7010, ADD_7011, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*l_gripper_r_finger vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7091, ADD_7092, ADD_7093, 0.025, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7118, ADD_7119, ADD_7120, 0.015, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7151, ADD_7152, ADD_7153, 0.015, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*l_gripper_r_finger_2 vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7235, ADD_7236, ADD_7237, 0.032, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7265, ADD_7266, ADD_7267, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7295, ADD_7296, ADD_7297, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7325, ADD_7326, ADD_7327, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*l_gripper_r_finger_tip vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<
            decltype(q[0])>(ADD_7411, ADD_7412, ADD_7413, 0.028, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7444, ADD_7445, ADD_7446, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7483, ADD_7484, ADD_7485, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7522, ADD_7523, ADD_7524, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7555, ADD_7556, ADD_7557, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_gripper_base vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_2364, ADD_2365, ADD_2366, 0.06, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6398, ADD_6399, ADD_6400, 0.04, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6425, SUB_6426, SUB_6427, 0.04, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_hand vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6303, ADD_6304, ADD_6305, 0.05, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_lower_elbow vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1619, ADD_1620, ADD_1621, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_lower_forearm vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1952, ADD_1953, ADD_1954, 0.1, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6033, ADD_6034, ADD_6035, 0.07, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6060, SUB_6061, SUB_6062, 0.07, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_lower_shoulder vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1276, ADD_1278, 0.399976, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_upper_elbow vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5637, ADD_5638, ADD_5639, 0.19, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5667, ADD_5668, ADD_5669, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5637, ADD_5638, ADD_5639, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1438, ADD_1439, SUB_1440, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_upper_forearm vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_5878, ADD_5879, ADD_5880, 0.19, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_1771, ADD_1772, ADD_1773, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5917, ADD_5918, ADD_5919, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_5878, ADD_5879, ADD_5880, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_upper_shoulder vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, 0.2590274, 0.304626, 0.175, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.379626, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, 0.2590274, 0.229626, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*left_wrist vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            SUB_6141, SUB_6142, SUB_6143, 0.105, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_6162, ADD_6163, ADD_6164, 0.07, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_6189, SUB_6190, SUB_6191, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*pedestal vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0, 0.0, -0.6, 0.5, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0, 0.0, -0.6, 0.5, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*r_gripper_l_finger vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7811, ADD_7812, ADD_7813, 0.027, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7844, ADD_7845, ADD_7846, 0.015, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7844, ADD_7845, ADD_7846, 0.015, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7844, ADD_7845, ADD_7846, 0.015, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7844, ADD_7845, ADD_7846, 0.015, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7886, ADD_7887, ADD_7888, 0.015, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7886, ADD_7887, ADD_7888, 0.015, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7886, ADD_7887, ADD_7888, 0.015, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7886, ADD_7887, ADD_7888, 0.015, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*r_gripper_l_finger_2 vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_7964, ADD_7965, ADD_7966, 0.032, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7988, ADD_7989, ADD_7990, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7988, ADD_7989, ADD_7990, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7988, ADD_7989, ADD_7990, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_7988, ADD_7989, ADD_7990, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8012, ADD_8013, ADD_8014, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8012, ADD_8013, ADD_8014, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8012, ADD_8013, ADD_8014, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8012, ADD_8013, ADD_8014, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8036, ADD_8037, ADD_8038, 0.012, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8036, ADD_8037, ADD_8038, 0.012, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8036, ADD_8037, ADD_8038, 0.012, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8036, ADD_8037, ADD_8038, 0.012, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*r_gripper_l_finger_tip vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<
            decltype(q[0])>(SUB_8131, SUB_8132, SUB_8133, 0.028, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8170, ADD_8171, ADD_8172, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8221, SUB_8222, SUB_8223, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    SUB_8272, SUB_8273, SUB_8274, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_8311, ADD_8312, ADD_8313, 0.014, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*r_gripper_r_finger_tip*/ sphere_environment_in_collision(
            environment, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_environment_in_collision(environment, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_environment_in_collision(environment, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*right_lower_elbow vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_481, ADD_482, ADD_483, 0.1, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_481, ADD_482, ADD_483, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*right_lower_shoulder vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_137, SUB_140, 0.399976, 0.1, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_137, SUB_140, 0.399976, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*right_upper_elbow vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4690, ADD_4691, ADD_4692, 0.19, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4720, ADD_4721, ADD_4722, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4690, ADD_4691, ADD_4692, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_300, ADD_301, SUB_302, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*right_upper_forearm vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            ADD_4931, ADD_4932, ADD_4933, 0.19, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_633, ADD_634, ADD_635, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4970, ADD_4971, ADD_4972, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    ADD_4931, ADD_4932, ADD_4933, 0.08, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*right_upper_shoulder vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            0.0640272, -0.2590274, 0.304626, 0.175, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.379626, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    0.0640272, -0.2590274, 0.229626, 0.1, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)
        if (/*torso vs. r_gripper_r_finger_tip*/ sphere_sphere_self_collision<decltype(q[0])>(
            -0.044, 0.0, 0.222, 0.409, ADD_8713, ADD_8714, ADD_8715, 0.028))
        {
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, -0.1, 0.1, 0.25, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.025, 0.1, 0.1, 0.25, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8746, ADD_8747, ADD_8748, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8785, ADD_8786, ADD_8787, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8824, ADD_8825, ADD_8826, 0.014))
            {
                return false;
            }
            if (sphere_sphere_self_collision<decltype(q[0])>(
                    -0.065, 0.0, 0.4, 0.23, ADD_8857, ADD_8858, ADD_8859, 0.014))
            {
                return false;
            }
        }  // (1812, 1812)

        return true;
    }

    inline auto eefk(const std::array<float, 14> &q) noexcept -> std::array<float, 7>
    {
    }
}  // namespace vamp::robots::baxter

// NOLINTEND(*-magic-numbers)
