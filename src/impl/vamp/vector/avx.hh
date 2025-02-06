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
}  // namespace vamp
