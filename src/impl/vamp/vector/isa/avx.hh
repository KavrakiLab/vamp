#pragma once

#if not defined(__x86_64__)
#error "Tried to compile x86 intrinsics on non-x86 platform!"
#endif

#include <vamp/vector/interface.hh>

#include <immintrin.h>
#include <limits>

namespace vamp
{
    template <>
    struct SIMDVector<__m256i>
    {
        using VectorT = __m256i;
        using ScalarT = int;
        static constexpr std::size_t VectorWidth = 8;
        static constexpr std::size_t Alignment = 32;

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            // Awful, but so is extracting an int in AVX2
            switch (idx)
            {
                case 0:
                    return _mm256_extract_epi32(v, 0);
                case 1:
                    return _mm256_extract_epi32(v, 1);
                case 2:
                    return _mm256_extract_epi32(v, 2);
                case 3:
                    return _mm256_extract_epi32(v, 3);
                case 4:
                    return _mm256_extract_epi32(v, 4);
                case 5:
                    return _mm256_extract_epi32(v, 5);
                case 6:
                    return _mm256_extract_epi32(v, 6);
                case 7:
                    return _mm256_extract_epi32(v, 7);
            };
        }

        template <unsigned int = 0>
        inline static constexpr auto constant(ScalarT v) noexcept -> VectorT
        {
            return _mm256_set1_epi32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_sub_epi32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_add_epi32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            // NOTE: Kinda slow. Note footgun with overflow to 64 bits, as well as footgun with
            // "normal" intmul
            return _mm256_mullo_epi32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return _mm256_xor_si256(l, _mm256_cmpeq_epi32(l, l));
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_and_si256(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_or_si256(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmpeq_epi32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmpgt_epi32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            return _mm256_testz_si256(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto load(const ScalarT *const i) noexcept -> VectorT
        {
            return _mm256_load_si256((const __m256i *const)i);
        }

        template <unsigned int = 0>
        inline static constexpr auto load_unaligned(const ScalarT *const i) noexcept -> VectorT
        {
            return _mm256_loadu_si256((const __m256i *const)i);
        }

        template <unsigned int = 0>
        inline static constexpr auto store(ScalarT *f, VectorT v) noexcept -> void
        {
            _mm256_store_si256(reinterpret_cast<VectorT *>(f), v);
        }

        template <unsigned int = 0>
        inline static constexpr auto store_unaligned(ScalarT *f, VectorT v) noexcept -> void
        {
            _mm256_storeu_si256(reinterpret_cast<VectorT *>(f), v);
        }

        template <unsigned int = 0>
        inline static constexpr auto mask(VectorT v) noexcept -> unsigned int
        {
            // HACK: This will create more FP port contention. We could use _mm256_movemask_epi8, but this has
            // poor latency and would require a change to the logic for all_true
            return _mm256_movemask_ps(_mm256_castsi256_ps(v));
        }

        // TODO: Figure out how to support vector shifting; currently causes a template deduction error
        // inline static constexpr auto shift_left(VectorT v, VectorT i) noexcept -> VectorT
        // {
        //     return _mm256_sllv_epi32(v, i);
        // }

        template <unsigned int = 0>
        inline static constexpr auto shift_left(VectorT v, unsigned int i) noexcept -> VectorT
        {
            return _mm256_slli_epi32(v, i);
        }

        // inline static constexpr auto shift_right(VectorT v, VectorT i) noexcept -> VectorT
        // {
        //     return _mm256_srlv_epi32(v, i);
        // }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, unsigned int i) noexcept -> VectorT
        {
            return _mm256_srli_epi32(v, i);
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return _mm256_setzero_si256();
        }

        template <typename = void>
        inline static constexpr auto gather(__m256i idxs, const ScalarT *base) noexcept -> VectorT
        {
            return _mm256_i32gather_epi32(base, idxs, sizeof(ScalarT));
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(__m256i idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            return _mm256_mask_i32gather_epi32(alternative, base, idxs, mask, sizeof(ScalarT));
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, __m256>)
            {
                return _mm256_cvtepi32_ps(v);
            }
            else if constexpr (std::is_same_v<OtherVectorT, VectorT>)
            {
                return v;
            }
            else
            {
                static_assert("Invalid cast-to type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            {
                if constexpr (std::is_same_v<OtherVectorT, __m256>)
                {
                    return _mm256_cvtps_epi32(v);
                }
                else
                {
                    static_assert("Invalid cast-from type!");
                }
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, __m256>)
            {
                return _mm256_castsi256_ps(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }
    };

    template <>
    struct SIMDVector<__m256>
    {
        using VectorT = __m256;
        using ScalarT = float;
        static constexpr std::size_t VectorWidth = 8;
        static constexpr std::size_t Alignment = 32;

        template <unsigned int = 0>
        inline static constexpr auto constant(ScalarT v) noexcept -> VectorT
        {
            return _mm256_set1_ps(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto constant_int(unsigned int v) noexcept -> VectorT
        {
            return _mm256_castsi256_ps(_mm256_set1_epi32(v));
        }

        template <unsigned int = 0>
        inline static constexpr auto load(const ScalarT *const f) noexcept -> VectorT
        {
            return _mm256_load_ps(f);
        }

        template <unsigned int = 0>
        inline static constexpr auto load_unaligned(const ScalarT *const f) noexcept -> VectorT
        {
            return _mm256_loadu_ps(f);
        }

        template <unsigned int = 0>
        inline static constexpr auto store(ScalarT *f, VectorT v) noexcept -> void
        {
            _mm256_store_ps(f, v);
        }

        template <unsigned int = 0>
        inline static constexpr auto store_unaligned(ScalarT *f, VectorT v) noexcept -> void
        {
            _mm256_storeu_ps(f, v);
        }

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            return v[idx];
        }

        template <unsigned int = 0>
        inline static constexpr auto broadcast(VectorT v, int idx) noexcept -> VectorT
        {
            return _mm256_permutevar8x32_ps(v, _mm256_set1_epi32(idx));
        }

        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return _mm256_xor_ps(
                l, _mm256_castsi256_ps(_mm256_cmpeq_epi32(_mm256_castps_si256(l), _mm256_castps_si256(l))));
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static constexpr auto neg(VectorT l) noexcept -> VectorT
        {
            return _mm256_xor_ps(l, _mm256_set1_ps(-0.0));
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_add_ps(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_sub_ps(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_mul_ps(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmp_ps(l, r, _CMP_LE_OQ);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmp_ps(l, r, _CMP_LT_OQ);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmp_ps(l, r, _CMP_GE_OQ);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmp_ps(l, r, _CMP_GT_OQ);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmp_ps(l, r, _CMP_EQ_OQ);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_not_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_cmp_ps(l, r, _CMP_NEQ_OQ);
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static auto floor(VectorT v) noexcept -> VectorT
        {
            return _mm256_floor_ps(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto div(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_div_ps(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto rcp(VectorT l) noexcept -> VectorT
        {
            return _mm256_rcp_ps(l);
        }

        template <unsigned int = 0>
        inline static constexpr auto mask(VectorT v) noexcept -> unsigned int
        {
            return _mm256_movemask_ps(v);
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return _mm256_setzero_ps();
        }

        template <unsigned int = 0>
        inline static constexpr auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            return _mm256_testz_ps(l, r);
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static constexpr auto abs(VectorT v) noexcept -> VectorT
        {
            const auto abs_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x7fffffff));
            return _mm256_and_ps(v, abs_mask);
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_and_ps(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return _mm256_or_ps(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto sqrt(VectorT v) noexcept -> VectorT
        {
            return _mm256_mul_ps(v, _mm256_rsqrt_ps(v));
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_left(VectorT v, unsigned int i) noexcept -> VectorT
        {
            return _mm256_castsi256_ps(_mm256_slli_epi32(_mm256_castps_si256(v), i));
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, unsigned int i) noexcept -> VectorT
        {
            return _mm256_castsi256_ps(_mm256_srli_epi32(_mm256_castps_si256(v), i));
        }

        template <unsigned int = 0>
        inline static constexpr auto clamp(VectorT v, VectorT lower, VectorT upper) noexcept -> VectorT
        {
            return _mm256_min_ps(_mm256_max_ps(v, lower), upper);
        }

        template <unsigned int = 0>
        inline static constexpr auto max(VectorT v, VectorT other) noexcept -> VectorT
        {
            return _mm256_max_ps(v, other);
        }

        template <unsigned int = 0>
        inline static constexpr auto min(VectorT v, VectorT other) noexcept -> VectorT
        {
            return _mm256_min_ps(v, other);
        }

        template <unsigned int = 0>
        inline static constexpr auto hsum(VectorT v) noexcept -> ScalarT
        {
            auto vhigh = _mm256_extractf128_ps(v, 1);
            auto vlow = _mm256_castps256_ps128(v);
            auto sum_1 = _mm_add_ps(vhigh, vlow);
            auto shuf_1 = _mm_castpd_ps(_mm_permute_pd(_mm_castps_pd(sum_1), 0b01));
            auto sum_2 = _mm_add_ps(sum_1, shuf_1);
            auto shuf_2 = _mm_movehdup_ps(sum_2);
            auto sum_3 = _mm_add_ps(sum_2, shuf_2);
            return _mm_cvtss_f32(sum_3);
        }

        // converted from http://gruntthepeon.free.fr/ssemath/
        template <unsigned int = 0>
        inline static constexpr auto sin(VectorT x) noexcept -> VectorT
        {
            using IntVector = SIMDVector<__m256i>;

            const auto ps_sign_mask = constant_int(0x80000000);
            const auto ps_cephes_FOPI = constant(1.27323954473516f);  // 4/Pi
            const auto pi32_1 = IntVector::constant(1);
            const auto pi32_inv1 = IntVector::constant(~1);
            const auto pi32_4 = IntVector::constant(4);
            const auto pi32_2 = IntVector::constant(2);
            const auto ps_minus_cephes_DP1 = constant(-0.78515625f);
            const auto ps_minus_cephes_DP2 = constant(-2.4187564849853515625e-4f);
            const auto ps_minus_cephes_DP3 = constant(-3.77489497744594108e-8f);
            const auto ps_coscof_p0 = constant(2.443315711809948E-005f);
            const auto ps_coscof_p1 = constant(-1.388731625493765E-003f);
            const auto ps_coscof_p2 = constant(4.166664568298827E-002f);
            const auto ps_0p5 = constant(0.5f);
            const auto ps_1 = constant(1.0f);
            const auto ps_sincof_p0 = constant(-1.9515295891E-4f);
            const auto ps_sincof_p1 = constant(8.3321608736E-3f);
            const auto ps_sincof_p2 = constant(-1.6666654611E-1f);

            auto sign_bit = x;

            x = abs(x);
            sign_bit = and_(sign_bit, ps_sign_mask);
            auto y = mul(x, ps_cephes_FOPI);

            auto emm2 = IntVector::from(y);

            // j=(j+1) & (~1) (see the cephes sources)
            emm2 = IntVector::add(emm2, pi32_1);
            emm2 = IntVector::and_(emm2, pi32_inv1);
            y = from<IntVector::VectorT>(emm2);

            // Get the swap sign flag
            auto emm0 = IntVector::and_(emm2, pi32_4);
            emm0 = IntVector::shift_left(emm0, 29);

            // Get the polynom selection mask
            // There is one polynom for 0 <= x <= Pi/4
            // and another one for Pi/4 < x <= Pi/2
            // Both branches will be computed.
            emm2 = IntVector::and_(emm2, pi32_2);
            emm2 = IntVector::cmp_equal(emm2, IntVector::zero_vector());

            auto swap_sign_bit = IntVector::template as<VectorT>(emm0);
            auto poly_mask = IntVector::template as<VectorT>(emm2);
            sign_bit = _mm256_xor_ps(sign_bit, swap_sign_bit);

            // The magic pass: "Extended precision modular arithmetic"
            // x = ((x - y * DP1) - y * DP2) - y * DP3;
            auto xmm1 = mul(y, ps_minus_cephes_DP1);
            auto xmm2 = mul(y, ps_minus_cephes_DP2);
            auto xmm3 = mul(y, ps_minus_cephes_DP3);
            x = add(x, xmm1);
            x = add(x, xmm2);
            x = add(x, xmm3);

            // Evaluate the first polynom (0 <= x <= Pi/4)
            y = ps_coscof_p0;
            auto z = mul(x, x);
            y = mul(y, z);
            y = add(y, ps_coscof_p1);
            y = mul(y, z);
            y = add(y, ps_coscof_p2);
            y = mul(y, z);
            y = mul(y, z);
            auto tmp = mul(z, ps_0p5);
            y = sub(y, tmp);
            y = add(y, ps_1);

            // Evaluate the second polynom (Pi/4 <= x <= 0)
            auto y2 = ps_sincof_p0;
            y2 = mul(y2, z);
            y2 = add(y2, ps_sincof_p1);
            y2 = mul(y2, z);
            y2 = add(y2, ps_sincof_p2);
            y2 = mul(y2, z);
            y2 = mul(y2, x);
            y2 = add(y2, x);

            // Select the correct result from the two polynoms
            xmm3 = poly_mask;
            y2 = and_(xmm3, y2);
            y = _mm256_andnot_ps(xmm3, y);
            y = add(y, y2);

            // Update the sign
            y = _mm256_xor_ps(y, sign_bit);

            return y;
        }

        template <unsigned int = 0>
        inline static constexpr auto asin(VectorT x) noexcept -> VectorT
        {
            // Sign mask for float vectors (negative zero)
            const auto ps_sign_mask = constant(-0.0f);

            // Cephes coefficients
            const auto ps_425 = constant(0.5f);
            const auto ps_1 = constant(1.0f);
            const auto ps_pi2 = constant(1.5707963267948966f);  // pi/2

            const auto ps_A0 = constant(4.2163199048E-2f);
            const auto ps_A1 = constant(2.4181311049E-2f);
            const auto ps_A2 = constant(4.5470025998E-2f);
            const auto ps_A3 = constant(7.4953002686E-2f);
            const auto ps_A4 = constant(1.6666752422E-1f);

            auto sign_bit = and_(x, ps_sign_mask);
            auto a = abs(x);

            // Mask for |x| > 0.5
            auto gt_half_mask = cmp_greater_than(a, ps_425);

            // Branch 1 (|x| > 0.5): zz = 0.5 * (1 - a), x_branch = sqrt(zz)
            auto zz_high = mul(ps_425, sub(ps_1, a));
            auto x_high = _mm256_sqrt_ps(zz_high);

            // Branch 2 (|x| <= 0.5): zz = a*a, x_branch = a
            auto zz_low = mul(a, a);
            auto x_low = a;

            auto zz = _mm256_blendv_ps(zz_low, zz_high, gt_half_mask);
            auto x_branch = _mm256_blendv_ps(x_low, x_high, gt_half_mask);

            // Polynomial evaluation
            auto z = ps_A0;
            z = mul(z, zz);
            z = add(z, ps_A1);
            z = mul(z, zz);
            z = add(z, ps_A2);
            z = mul(z, zz);
            z = add(z, ps_A3);
            z = mul(z, zz);
            z = add(z, ps_A4);
            z = mul(z, zz);
            z = mul(z, x_branch);
            z = add(z, x_branch);

            // For |x| > 0.5: result = pi/2 - 2*z
            auto z_large = sub(ps_pi2, add(z, z));
            auto res = _mm256_blendv_ps(z, z_large, gt_half_mask);

            // Restore sign
            res = _mm256_xor_ps(res, sign_bit);
            return res;
        }

        // TODO (shru) -- acos has some 3rd decimal errors, fix it pls
        template <unsigned int = 0>
        inline static constexpr auto acos(VectorT x) noexcept -> VectorT
        {
            const auto ps_cephes_morebits = constant(6.123233995736765886130E-17);

            const auto ps_05 = constant(0.5);                                // 0.5
            const auto ps_cephes_pi4 = constant(7.85398163397448309616E-1);  // pi/4
            const auto ps_2 = constant(2.0f);                                // 0.5

            auto z = mul(ps_05, x);
            z = sub(ps_05, z);
            z = _mm256_sqrt_ps(z);
            z = asin(z);
            z = mul(ps_2, z);

            auto z2 = asin(x);
            z2 = sub(ps_cephes_pi4, z2);
            z2 = add(z2, ps_cephes_morebits);
            z2 = add(z2, ps_cephes_pi4);

            auto gt_05_mask = cmp_greater_than(x, ps_05);
            z = and_(gt_05_mask, z);
            z2 = _mm256_andnot_ps(gt_05_mask, z2);
            z = add(z, z2);

            return z;
        }

        template <unsigned int = 0>
        inline static auto atan(VectorT x) noexcept -> VectorT
        {
            const auto ps_cephes_P0 = constant(-8.750608600031904122785E-1f);
            const auto ps_cephes_P1 = constant(-1.615753718733365076637E1f);
            const auto ps_cephes_P2 = constant(-7.500855792314704667340E1f);
            const auto ps_cephes_P3 = constant(-1.228866684490136173410E2f);
            const auto ps_cephes_P4 = constant(-6.485021904942025371773E1f);

            const auto ps_cephes_Q0 = constant(2.485846490142306297962E1f);
            const auto ps_cephes_Q1 = constant(1.650270098316988542046E2f);
            const auto ps_cephes_Q2 = constant(4.328810604912902668951E2f);
            const auto ps_cephes_Q3 = constant(4.853903996359136964868E2f);
            const auto ps_cephes_Q4 = constant(1.945506571482613964425E2f);

            const auto ps_0 = constant(0.0f);
            const auto ps_1 = constant(1.0f);

            // tan(pi/8)
            const auto ps_tan_pi8 = constant(0.41421356237309504880f);

            // tan(3*pi/8)
            const auto ps_tan_3pi8 = constant(2.41421356237309504880f);

            const auto ps_pi4 = constant(0.785398163397448309616f);
            const auto ps_pi2 = constant(1.57079632679489661923f);

            //--------------------------------------------------------
            // sign extraction
            //--------------------------------------------------------

            const __m256 sign_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000));

            auto sign_bit = _mm256_and_ps(x, sign_mask);
            auto a = abs(x);

            //--------------------------------------------------------
            // range reduction
            //
            // 0 <= a <= tan(pi/8)
            //--------------------------------------------------------

            auto gt_3pi8 = cmp_greater_than(a, ps_tan_3pi8);
            auto gt_pi8 = cmp_greater_than(a, ps_tan_pi8);

            auto mid_mask = _mm256_andnot_ps(gt_3pi8, gt_pi8);

            auto reduced_big = div(constant(-1.0f), a);
            auto reduced_mid = div(sub(a, ps_1), add(a, ps_1));

            a = blend(a, reduced_big, gt_3pi8);
            a = blend(a, reduced_mid, mid_mask);

            auto y = ps_0;
            y = blend(y, ps_pi2, gt_3pi8);
            y = blend(y, ps_pi4, mid_mask);

            //--------------------------------------------------------
            // rational approximation
            //--------------------------------------------------------

            auto zz = mul(a, a);

            auto p = ps_cephes_P0;
            p = add(mul(p, zz), ps_cephes_P1);
            p = add(mul(p, zz), ps_cephes_P2);
            p = add(mul(p, zz), ps_cephes_P3);
            p = add(mul(p, zz), ps_cephes_P4);
            p = mul(mul(p, zz), a);

            auto q = add(zz, ps_cephes_Q0);
            q = add(mul(q, zz), ps_cephes_Q1);
            q = add(mul(q, zz), ps_cephes_Q2);
            q = add(mul(q, zz), ps_cephes_Q3);
            q = add(mul(q, zz), ps_cephes_Q4);

            auto z = div(p, q);
            z = add(z, a);
            z = add(z, y);

            //--------------------------------------------------------
            // restore sign
            //--------------------------------------------------------

            z = _mm256_xor_ps(z, sign_bit);

            return z;
        }

        template <unsigned int = 0>
        inline static auto atan2(VectorT y, VectorT x) noexcept -> VectorT
        {
            const auto ps_zero = constant(0.0f);
            const auto ps_pi = constant(3.14159265358979323846f);
            const auto ps_pi_2 = constant(1.57079632679489661923f);

            auto z = atan(div(y, x));

            auto x_lt_0 = cmp_less_than(x, ps_zero);
            auto y_lt_0 = cmp_less_than(y, ps_zero);
            auto y_ge_0 = cmp_greater_equal(y, ps_zero);

            //--------------------------------------------------------
            // x < 0, y >= 0  -> +pi
            //--------------------------------------------------------

            auto add_pi_mask = _mm256_and_ps(x_lt_0, y_ge_0);
            z = blend(z, add(z, ps_pi), add_pi_mask);

            //--------------------------------------------------------
            // x < 0, y < 0   -> -pi
            //--------------------------------------------------------

            auto sub_pi_mask = _mm256_and_ps(x_lt_0, y_lt_0);
            z = blend(z, sub(z, ps_pi), sub_pi_mask);

            //--------------------------------------------------------
            // x == 0
            //--------------------------------------------------------

            auto x_eq_0 = cmp_equal(x, ps_zero);

            auto pos_y = cmp_greater_than(y, ps_zero);
            auto neg_y = cmp_less_than(y, ps_zero);

            z = blend(z, ps_pi_2, _mm256_and_ps(x_eq_0, pos_y));
            z = blend(z, sub(ps_zero, ps_pi_2), _mm256_and_ps(x_eq_0, neg_y));

            //--------------------------------------------------------
            // x == 0 && y == 0
            // matches most implementations
            //--------------------------------------------------------

            auto y_eq_0 = cmp_equal(y, ps_zero);
            auto both_zero = _mm256_and_ps(x_eq_0, y_eq_0);

            z = blend(z, ps_zero, both_zero);

            return z;
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static constexpr auto log(VectorT x) noexcept -> VectorT
        {
            using IntVector = SIMDVector<__m256i>;

            const auto half = constant(0.5F);
            const auto one = constant(1.0F);
            auto invalid_mask = cmp_less_equal(x, zero_vector());

            // cut off denormalized values
            x = max(x, constant_int(0x00800000));

            auto emm0 = IntVector::shift_right(as<IntVector::VectorT>(x), 23);

            x = and_(x, constant_int(~0x7f800000));
            x = or_(x, half);

            // keep only the fractional part
            emm0 = IntVector::sub(emm0, IntVector::constant(0x7f));
            auto e = from<IntVector::VectorT>(emm0);

            e = add(e, one);

            // compute approx
            auto mask = cmp_less_than(x, constant(0.707106781186547524f));
            auto tmp = and_(x, mask);
            x = sub(x, one);
            e = sub(e, and_(one, mask));
            x = add(x, tmp);

            auto z = mul(x, x);

            auto y = constant(7.0376836292E-2f);
            y = mul(y, x);
            y = add(y, constant(-1.1514610310E-1f));
            y = mul(y, x);
            y = add(y, constant(1.1676998740E-1f));
            y = mul(y, x);
            y = add(y, constant(-1.2420140846E-1f));
            y = mul(y, x);
            y = add(y, constant(+1.4249322787E-1f));
            y = mul(y, x);
            y = add(y, constant(-1.6668057665E-1f));
            y = mul(y, x);
            y = add(y, constant(+2.0000714765E-1f));
            y = mul(y, x);
            y = add(y, constant(-2.4999993993E-1f));
            y = mul(y, x);
            y = add(y, constant(+3.3333331174E-1f));
            y = mul(y, mul(x, z));
            tmp = mul(e, constant(-2.12194440e-4f));
            y = add(y, tmp);
            tmp = mul(z, half);
            y = sub(y, tmp);
            tmp = mul(e, constant(0.693359375f));
            x = add(x, add(y, tmp));

            x = or_(x, invalid_mask);  // negative arg will be NAN
            return x;
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return _mm256_blendv_ps(a, b, blend_mask);
        }

        template <unsigned int blend_mask>
        inline static constexpr auto blend_constant(VectorT a, VectorT b) noexcept -> VectorT
        {
            return _mm256_blend_ps(a, b, blend_mask);
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, __m256i>)
            {
                return _mm256_cvtps_epi32(v);
            }
            else if constexpr (std::is_same_v<OtherVectorT, VectorT>)
            {
                return v;
            }
            else
            {
                static_assert("Invalid cast-to type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, __m256i>)
            {
                return _mm256_cvtepi32_ps(v);
            }
            else
            {
                static_assert("Invalid cast-from type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, __m256i>)
            {
                return _mm256_castps_si256(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static auto map_to_range(OtherVectorT v) -> VectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, __m256i>)
            {
                const auto v_1 = _mm256_and_si256(v, _mm256_set1_epi32(1));
                const auto v1_f = _mm256_cvtepi32_ps(v_1);
                const auto v_scaled = _mm256_add_ps(_mm256_cvtepi32_ps(v), v1_f);
                return _mm256_mul_ps(
                    v_scaled,
                    _mm256_set1_ps(1.F / static_cast<float>(std::numeric_limits<unsigned int>::max())));
            }
            else
            {
                static_assert("Invalid range-map type!");
            }
        }

        template <typename = void>
        inline static constexpr auto gather(__m256i idxs, const ScalarT *base) noexcept -> VectorT
        {
            return _mm256_i32gather_ps(base, idxs, sizeof(ScalarT));
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(__m256i idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            return _mm256_mask_i32gather_ps(alternative, base, idxs, mask, sizeof(ScalarT));
        }
    };
}  // namespace vamp
