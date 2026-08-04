#pragma once

#include <initializer_list>
#if not defined(__EMSCRIPTEN__)
#error "Tried to compile WASM SIMD intrinsics on non-Emscripten platform!"
#endif

#include <cstdint>

#include <vamp/vector/interface.hh>

#include <wasm_simd128.h>
#include <limits>

namespace vamp
{
    // WASM SIMD uses v128_t for all vector types
    // We create separate type aliases to distinguish float vs int vectors
    struct WasmIntVec
    {
        v128_t v;

        inline WasmIntVec operator+(WasmIntVec other) const noexcept
        {
            return {wasm_i32x4_add(v, other.v)};
        }

        inline WasmIntVec operator-(WasmIntVec other) const noexcept
        {
            return {wasm_i32x4_sub(v, other.v)};
        }

        inline WasmIntVec operator*(WasmIntVec other) const noexcept
        {
            return {wasm_i32x4_mul(v, other.v)};
        }
    };

    struct WasmFloatVec
    {
        v128_t v;

        inline WasmFloatVec operator+(WasmFloatVec other) const noexcept
        {
            return {wasm_f32x4_add(v, other.v)};
        }

        inline WasmFloatVec operator-(WasmFloatVec other) const noexcept
        {
            return {wasm_f32x4_sub(v, other.v)};
        }

        inline WasmFloatVec operator*(WasmFloatVec other) const noexcept
        {
            return {wasm_f32x4_mul(v, other.v)};
        }

        inline WasmFloatVec operator/(WasmFloatVec other) const noexcept
        {
            return {wasm_f32x4_div(v, other.v)};
        }
    };

    template <>
    struct SIMDVector<WasmIntVec>
    {
        using VectorT = WasmIntVec;
        using ScalarT = int32_t;
        static constexpr std::size_t VectorWidth = 4;
        static constexpr std::size_t Alignment = 16;

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            alignas(16) int32_t arr[4];
            wasm_v128_store(arr, v.v);
            return arr[idx];
        }

        template <unsigned int = 0>
        inline static constexpr auto constant(ScalarT val) noexcept -> VectorT
        {
            return {wasm_i32x4_splat(val)};
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_i32x4_sub(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_i32x4_add(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_i32x4_mul(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return {wasm_v128_not(l.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_i32x4_eq(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_not_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_v128_not(wasm_i32x4_eq(l.v, r.v))};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_i32x4_gt(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_v128_and(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_v128_or(l.v, r.v)};
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
            return {wasm_i32x4_shl(v.v, i)};
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
            return {wasm_i32x4_shr(v.v, i)};
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return rshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return {wasm_i32x4_splat(0)};
        }

        template <unsigned int = 0>
        inline static auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            // Match NEON semantics: check only sign bits (0x80000000)
            // Returns true if no sign bits are set (all values non-negative)
            auto andlr = wasm_v128_and(l.v, r.v);
            auto sign_mask = wasm_i32x4_splat(static_cast<int32_t>(0x80000000));
            auto sign_bits = wasm_v128_and(andlr, sign_mask);
            return !wasm_v128_any_true(sign_bits);
        }

        template <unsigned int = 0>
        inline static auto load(const ScalarT *const i) noexcept -> VectorT
        {
            return {wasm_v128_load(i)};
        }

        template <unsigned int = 0>
        inline static auto load_unaligned(const ScalarT *const i) noexcept -> VectorT
        {
            return {wasm_v128_load(i)};
        }

        template <unsigned int = 0>
        inline static auto store(ScalarT *i, VectorT v) noexcept -> void
        {
            wasm_v128_store(i, v.v);
        }

        template <unsigned int = 0>
        inline static auto store_unaligned(ScalarT *i, VectorT v) noexcept -> void
        {
            wasm_v128_store(i, v.v);
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return {wasm_v128_bitselect(b.v, a.v, blend_mask.v)};
        }

        template <unsigned int = 0>
        inline static auto mask(VectorT v) noexcept -> unsigned int
        {
            return wasm_i32x4_bitmask(v.v);
        }

        template <typename = void>
        inline static constexpr auto gather(WasmIntVec idxs, const ScalarT *base) noexcept -> VectorT
        {
            alignas(16) int32_t idx_arr[4];
            wasm_v128_store(idx_arr, idxs.v);
            return {wasm_i32x4_make(base[idx_arr[0]], base[idx_arr[1]], base[idx_arr[2]], base[idx_arr[3]])};
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(WasmIntVec idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            auto overlay = gather(idxs, base);
            return blend(overlay, alternative, mask);
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, WasmFloatVec>)
            {
                return {wasm_f32x4_convert_i32x4(v.v)};
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, WasmFloatVec>)
            {
                return {wasm_i32x4_trunc_sat_f32x4(v.v)};
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, WasmFloatVec>)
            {
                return {v.v};  // Reinterpret cast - same bits
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }
    };

    template <>
    struct SIMDVector<WasmFloatVec>
    {
        using VectorT = WasmFloatVec;
        using ScalarT = float;
        using IntT = SIMDVector<WasmIntVec>;
        static constexpr std::size_t VectorWidth = 4;
        static constexpr std::size_t Alignment = 16;

        template <unsigned int = 0>
        inline static auto constant(ScalarT v) noexcept -> VectorT
        {
            return {wasm_f32x4_splat(v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto constant_int(unsigned int v) noexcept -> VectorT
        {
            return {wasm_f32x4_convert_i32x4(wasm_i32x4_splat(v))};
        }

        template <unsigned int = 0>
        inline static auto load(const ScalarT *const f) noexcept -> VectorT
        {
            return {wasm_v128_load(f)};
        }

        template <unsigned int = 0>
        inline static auto load_unaligned(const ScalarT *const f) noexcept -> VectorT
        {
            return {wasm_v128_load(f)};
        }

        template <unsigned int = 0>
        inline static auto store(ScalarT *f, VectorT v) noexcept -> void
        {
            wasm_v128_store(f, v.v);
        }

        template <unsigned int = 0>
        inline static auto store_unaligned(ScalarT *f, VectorT v) noexcept -> void
        {
            wasm_v128_store(f, v.v);
        }

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            alignas(16) float arr[4];
            wasm_v128_store(arr, v.v);
            return arr[idx];
        }

        template <std::size_t idx>
        inline static constexpr auto broadcast_dispatch(VectorT v) noexcept -> VectorT
        {
            return {wasm_i32x4_shuffle(v.v, v.v, idx, idx, idx, idx)};
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

        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return {wasm_v128_not(l.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto xor_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_v128_xor(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto neg(VectorT l) noexcept -> VectorT
        {
            return {wasm_f32x4_neg(l.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_add(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_sub(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_mul(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_le(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_lt(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_ge(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_gt(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_eq(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_not_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_ne(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static auto floor(VectorT v) noexcept -> VectorT
        {
            return {wasm_f32x4_floor(v.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto div(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_f32x4_div(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto rcp(VectorT l) noexcept -> VectorT
        {
            return {wasm_f32x4_div(wasm_f32x4_splat(1.0f), l.v)};
        }

        template <unsigned int = 0>
        inline static auto mask(VectorT v) noexcept -> unsigned int
        {
            return wasm_i32x4_bitmask(v.v);
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return {wasm_f32x4_splat(0.0f)};
        }

        template <unsigned int = 0>
        inline static auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            // Returns true if no sign bits are set (all values non-negative)
            auto andlr = wasm_v128_and(l.v, r.v);
            auto sign_mask = wasm_i32x4_splat(static_cast<int32_t>(0x80000000));
            auto sign_bits = wasm_v128_and(andlr, sign_mask);
            return !wasm_v128_any_true(sign_bits);
        }

        template <unsigned int = 0>
        inline static constexpr auto abs(VectorT v) noexcept -> VectorT
        {
            return {wasm_f32x4_abs(v.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_v128_and(l.v, r.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return {wasm_v128_or(l.v, r.v)};
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
            return {wasm_i32x4_shl(v.v, i)};
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
            return {wasm_u32x4_shr(v.v, i)};
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return rshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <unsigned int = 0>
        inline static constexpr auto sqrt(VectorT v) noexcept -> VectorT
        {
            return {wasm_f32x4_sqrt(v.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto clamp(VectorT v, VectorT lower, VectorT upper) noexcept -> VectorT
        {
            return {wasm_f32x4_min(wasm_f32x4_max(v.v, lower.v), upper.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto max(VectorT v, VectorT other) noexcept -> VectorT
        {
            return {wasm_f32x4_max(v.v, other.v)};
        }

        template <unsigned int = 0>
        inline static constexpr auto hsum(VectorT v) noexcept -> float
        {
            alignas(16) float arr[4];
            wasm_v128_store(arr, v.v);
            return arr[0] + arr[1] + arr[2] + arr[3];
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return {wasm_v128_bitselect(b.v, a.v, blend_mask.v)};
        }

        // Special-case blend for trim() and pack_and_pad
        template <unsigned int blend_mask>
        inline static constexpr auto blend_constant(VectorT a, VectorT b) noexcept -> VectorT
        {
            if constexpr (blend_mask == 8)
            {
                // Blend last element only
                return {wasm_v128_bitselect(
                    b.v, a.v, wasm_i32x4_make(0, 0, 0, static_cast<int32_t>(0xffffffff)))};
            }
            else if constexpr (blend_mask == 12)
            {
                // Blend last two elements
                return {wasm_v128_bitselect(
                    b.v,
                    a.v,
                    wasm_i32x4_make(
                        0, 0, static_cast<int32_t>(0xffffffff), static_cast<int32_t>(0xffffffff)))};
            }
            else if constexpr (blend_mask == 14)
            {
                // Blend last three elements
                return {wasm_v128_bitselect(
                    b.v,
                    a.v,
                    wasm_i32x4_make(
                        0,
                        static_cast<int32_t>(0xffffffff),
                        static_cast<int32_t>(0xffffffff),
                        static_cast<int32_t>(0xffffffff)))};
            }
            else
            {
                static_assert(always_false<blend_mask>, "blend_mask not in allowed value set!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            return {wasm_i32x4_trunc_sat_f32x4(v.v)};
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            return {wasm_f32x4_convert_i32x4(v.v)};
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            return {v.v};  // Reinterpret cast - same bits
        }

        template <typename OtherVectorT>
        inline auto map_to_range(OtherVectorT v) -> VectorT
        {
            const auto v_1 = WasmIntVec{wasm_v128_and(v.v, wasm_i32x4_splat(1))};
            const auto v1_f = wasm_f32x4_convert_i32x4(v_1.v);
            const auto v_scaled = wasm_f32x4_add(wasm_f32x4_convert_i32x4(v.v), v1_f);
            return {wasm_f32x4_mul(
                v_scaled,
                wasm_f32x4_splat(1.f / static_cast<float>(std::numeric_limits<unsigned int>::max())))};
        }

        template <typename = void>
        inline static auto gather(WasmIntVec idxs, const ScalarT *base) noexcept -> VectorT
        {
            alignas(16) int32_t idx_arr[4];
            wasm_v128_store(idx_arr, idxs.v);
            return {wasm_f32x4_make(base[idx_arr[0]], base[idx_arr[1]], base[idx_arr[2]], base[idx_arr[3]])};
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(WasmIntVec idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            auto overlay = gather(idxs, base);
            return blend(overlay, alternative, mask);
        }
    };
}  // namespace vamp
