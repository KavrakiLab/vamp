#pragma once

#include <initializer_list>
#if not defined(__ARM_NEON)
#error "Tried to compile NEON intrinsics on non-ARM platform!"
#endif

#include <cstdint>

#include <vamp/vector/interface.hh>

#include <arm_neon.h>
#include <limits>

namespace vamp
{
    template <>
    struct SIMDVector<float32x4_t>
    {
        using VectorT = float32x4_t;
        using ScalarT = float32_t;
        static constexpr std::size_t VectorWidth = 4;
        static constexpr std::size_t Alignment = 16;

        template <unsigned int = 0>
        inline static auto constant(ScalarT v) noexcept -> VectorT
        {
            return vdupq_n_f32(v);
        }

        template <unsigned int = 0>
        inline static auto load(const ScalarT *const f) noexcept -> VectorT
        {
            return vld1q_f32(f);
        }

        template <unsigned int = 0>
        inline static auto store(ScalarT *f, VectorT v) noexcept -> void
        {
            return vst1q_f32(f, v);
        }

        template <unsigned int = 0>
        inline static auto store_unaligned(ScalarT *f, VectorT v) noexcept -> void
        {
            return vst1q_f32(f, v);
        }

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            return v[idx];
        }

        // C++ is so dumb. We have to do this (unless someone has a cleverer idea) because (1) vdupq_laneq_f32
        // is a macro and the preprocessor hates commas and (2) you can't use the usual parenthesis trick f or
        // commas with parameter packs, apparently
        template <std::size_t idx>
        inline static constexpr auto broadcast_dispatch(VectorT v) noexcept -> VectorT
        {
            return vdupq_laneq_f32(v, idx);
        }

        template <std::size_t... I>
        inline static constexpr auto
        broadcast_lookup(VectorT v, std::size_t lane, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(lane == I ? (ret = broadcast_dispatch<std::integral_constant<int, I>{}>(v)), 0 : 0)...});
            return ret;
        }

        template <unsigned int = 0>
        inline static constexpr auto broadcast(VectorT v, std::size_t lane) noexcept -> VectorT
        {
            return broadcast_lookup(v, lane, std::make_index_sequence<VectorWidth>());
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vmvnq_u32(vreinterpretq_u32_f32(l)));  // maybe a reverse is needed
        }

        template <unsigned int = 0>
        inline static constexpr auto neg(VectorT l) noexcept -> VectorT
        {
            return vreinterpretq_f32_s32(veorq_s32(
                vreinterpretq_s32_f32(l),
                vreinterpretq_s32_f32(vdupq_n_f32(-0.0))));  // maybe a reverse is needed
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vaddq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vsubq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vmulq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vcleq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vcgeq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vceqq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_not_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vmvnq_u32(vceqq_f32(l, r)));
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static auto floor(VectorT v) noexcept -> VectorT
        {
            return vrndmq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto div(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vdivq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto rcp(VectorT l) noexcept -> VectorT
        {
            auto s = vrecpeq_f32(l);
            auto p = vrecpsq_f32(l, s);
            return vmulq_f32(s, p);
        }

        template <unsigned int = 0>
        inline static auto mask(VectorT v) noexcept -> unsigned int
        {
            auto MSB = vsliq_n_u32(vdupq_n_u32(0), vreinterpretq_u32_f32(v), 16);
            auto sumtwo = vreinterpret_u32_u16(
                vpadd_u16(vreinterpret_u16_u32(vget_low_u32(MSB)), vreinterpret_u16_u32(vget_high_u32(MSB))));
            auto attempt = vreinterpret_u16_u32(sumtwo);
            auto attempt2 = vreinterpret_u8_u16(attempt);
            auto reorg = vshrn_n_u16(vreinterpretq_u16_u8(vcombine_u8(attempt2, attempt2)), 8);
            return vget_lane_u32(vreinterpret_u32_u8(reorg), 0);
            // IT MAY NEED A REVERSE vrev32_u8
            // vget_lane_u32(vreinterpret_u32_u8(vrev32_u8(reorg)), 0);
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return vmovq_n_f32(0.0f);
        }

        template <unsigned int = 0>
        inline static auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            auto andlr = vandq_u32(vreinterpretq_u32_f32(l), vreinterpretq_u32_f32(r));
            auto horizor = vorr_u32(vget_low_u32(andlr), vget_high_u32(andlr));
            uint32x2_t mask = {0x80000000, 0x80000000};
            auto test = vand_u32(horizor, mask);
            return (vget_lane_u32(test, 0) || vget_lane_u32(test, 1)) == 0;
        }

        template <unsigned int = 0>
        inline static constexpr auto abs(VectorT v) noexcept -> VectorT
        {
            return vabsq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(l), vreinterpretq_u32_f32(r)));
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(l), vreinterpretq_u32_f32(r)));
        }

        template <std::size_t... I>
        inline static constexpr auto
        lshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I ? (ret = lshift_dispatch<std::integral_constant<int, I>{}>(v)), 0 : 0)...});
            return ret;
        }

        template <unsigned int i>
        inline static constexpr auto lshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vshlq_n_u32(vreinterpretq_u32_f32(v), i));
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_left(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return lshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <std::size_t... I>
        inline static constexpr auto
        rshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I + 1 ? (ret = rshift_dispatch<std::integral_constant<int, I + 1>{}>(v)),
                  0 :
                                   0)...});
            return ret;
        }

        template <unsigned int i>
        inline static constexpr auto rshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vshrq_n_u32(vreinterpretq_u32_f32(v), i));
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return rshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <unsigned int = 0>
        inline static constexpr auto sqrt(VectorT v) noexcept -> VectorT
        {
            return vsqrtq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto clamp(VectorT v, VectorT lower, VectorT upper) noexcept -> VectorT
        {
            return vminq_f32(vmaxq_f32(v, lower), upper);
        }

        template <unsigned int = 0>
        inline static constexpr auto max(VectorT v, VectorT other) noexcept -> VectorT
        {
            return vmaxq_f32(v, other);
        }

        template <unsigned int = 0>
        inline static constexpr auto hsum(VectorT v) noexcept -> float
        {
            return vaddvq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return vbslq_f32(vreinterpretq_u32_f32(blend_mask), b, a);
        }

        // HACK: We only ever use this for trim() and pack_and_pad, and ARM makes it hard to go from
        // a scalar mask to an appropriate vector mask for vbslq. So, we special-case for the values
        // we could possibly get
        template <unsigned int blend_mask>
        inline static constexpr auto blend_constant(VectorT a, VectorT b) noexcept -> VectorT
        {
            if constexpr (blend_mask == 8)
            {
                return vbslq_f32(vcombine_u32(vcreate_u32(0l), vcreate_u32(0xffffffff00000000)), b, a);
            }
            else if constexpr (blend_mask == 12)
            {
                return vbslq_f32(vcombine_u32(vcreate_u32(0l), vcreate_u32(0xffffffffffffffff)), b, a);
            }
            else if constexpr (blend_mask == 14)
            {
                return vbslq_f32(
                    vcombine_u32(vcreate_u32(0xffffffff00000000), vcreate_u32(0xffffffffffffffff)), b, a);
            }
            else
            {
                static_assert(always_false<blend_mask>, "blend_mask not in allowed value set!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            return vcvtq_s32_f32(v);
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            return vcvtq_f32_s32(v);
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            return vreinterpretq_s32_f32(v);
        }

        template <typename OtherVectorT>
        inline auto map_to_range(OtherVectorT v) -> VectorT
        {
            const auto v_1 = vandq_s32(v, vdupq_n_s32(1));
            const auto v1_f = vcvtq_f32_s32(v_1);
            const auto v_scaled = vaddq_f32(vcvtq_f32_s32(v), v1_f);
            return vmulq_f32(
                v_scaled, vdupq_n_f32(1.f / static_cast<float>(std::numeric_limits<unsigned int>::max())));
        }

        template <typename = void>
        inline static auto gather(int32x4_t idxs, const ScalarT *base) noexcept -> VectorT
        {
            // Pretty sure there isn't a better way to do a 32-bit lookup table...
            float32x4_t result = vdupq_n_f32(0);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 0)], result, 0);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 1)], result, 1);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 2)], result, 2);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 3)], result, 3);
            return result;
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(int32x4_t idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            auto overlay = gather(idxs, base);
            return blend(overlay, alternative, mask);
        }
    };

    template <>
    struct SIMDVector<int32x4_t>
    {
        using VectorT = int32x4_t;
        using ScalarT = int32_t;
        static constexpr std::size_t VectorWidth = 4;
        static constexpr std::size_t Alignment = 16;

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            return ((int *)(&v))[idx];
        }

        template <unsigned int = 0>
        inline static constexpr auto constant(ScalarT v) noexcept -> VectorT
        {
            return vdupq_n_s32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vsubq_s32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vaddq_s32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vmulq_s32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vmvnq_u32(vreinterpretq_u32_s32(l)));  // maybe a reverse is needed
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vceqq_s32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vcgeq_s32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_s32(l), vreinterpretq_u32_s32(r)));
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vorrq_u32(vreinterpretq_u32_s32(l), vreinterpretq_u32_s32(r)));
        }

        template <std::size_t... I>
        inline static constexpr auto
        lshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I ? (ret = lshift_dispatch<std::integral_constant<int, I>{}>(v)), 0 : 0)...});
            return ret;
        }

        template <ScalarT i>
        inline static constexpr auto lshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vshlq_n_s32(v, i);
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_left(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return lshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <std::size_t... I>
        inline static constexpr auto
        rshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I + 1 ? (ret = rshift_dispatch<std::integral_constant<int, I + 1>{}>(v)),
                  0 :
                                   0)...});
            return ret;
        }

        template <ScalarT i>
        inline static constexpr auto rshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vshrq_n_s32(v, i);
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return rshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return vmovq_n_s32(0);
        }

        template <unsigned int = 0>
        inline static auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            auto andlr = vandq_u32(vreinterpretq_u32_s32(l), vreinterpretq_u32_s32(r));
            auto horizor = vorr_u32(vget_low_u32(andlr), vget_high_u32(andlr));
            uint32x2_t mask = {0x80000000, 0x80000000};
            auto test = vand_u32(horizor, mask);
            return (vget_lane_u32(test, 0) || vget_lane_u32(test, 1)) == 0;
        }

        template <unsigned int = 0>
        inline static auto load(const ScalarT *const i) noexcept -> VectorT
        {
            return vld1q_s32((const int32_t *const)i);
        }

        template <unsigned int = 0>
        inline static auto store(ScalarT *i, VectorT v) noexcept -> void
        {
            return vst1q_s32(i, v);
        }

        template <unsigned int = 0>
        inline static auto store_unaligned(ScalarT *i, VectorT v) noexcept -> void
        {
            return vst1q_s32(i, v);
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return vbslq_s32(vreinterpretq_u32_s32(blend_mask), b, a);
        }

        template <unsigned int = 0>
        inline static auto mask(VectorT v) noexcept -> unsigned int
        {
            auto MSB = vsliq_n_u32(vdupq_n_u32(0), vreinterpretq_u32_s32(v), 16);
            auto sumtwo = vreinterpret_u32_u16(
                vpadd_u16(vreinterpret_u16_u32(vget_low_u32(MSB)), vreinterpret_u16_u32(vget_high_u32(MSB))));
            auto attempt = vreinterpret_u16_u32(sumtwo);
            auto attempt2 = vreinterpret_u8_u16(attempt);
            auto reorg = vshrn_n_u16(vreinterpretq_u16_u8(vcombine_u8(attempt2, attempt2)), 8);
            return vget_lane_u32(vreinterpret_u32_u8(reorg), 0);
            // IT MAY NEED A REVERSE vrev32_u8
            // vget_lane_u32(vreinterpret_u32_u8(vrev32_u8(reorg)), 0);
        }

        template <typename = void>
        inline static constexpr auto gather(int32x4_t idxs, const ScalarT *base) noexcept -> VectorT
        {
            // Pretty sure there isn't a better way to do a 32-bit lookup table...
            int32x4_t result = vdupq_n_s32(0);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 0)], result, 0);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 1)], result, 1);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 2)], result, 2);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 3)], result, 3);
            return result;
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(int32x4_t idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            auto overlay = gather(idxs, base);
            return blend(overlay, alternative, mask);
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, float32x4_t>)
            {
                return vcvtq_f32_s32(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, float32x4_t>)
            {
                return vcvtq_s32_f32(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, float32x4_t>)
            {
                return vreinterpretq_f32_s32(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }
    };
}  // namespace vamp
