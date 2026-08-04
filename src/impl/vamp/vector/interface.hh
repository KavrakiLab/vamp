#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

#include <vamp/constants.hh>
#include <vamp/vector/utils.hh>

namespace vamp
{
    template <typename SimdT, std::size_t num_rows, std::size_t num_scalars_per_row>
    struct Vector;

    template <typename S>
    inline constexpr void print_vector(std::ostream &out, typename S::VectorT vec) noexcept
    {
        for (auto i = 0ul; i < S::VectorWidth - 1; ++i)
        {
            out << S::extract(vec, i) << ", ";
        }

        out << S::extract(vec, S::VectorWidth - 1);
    }

    template <typename SimdT_, std::size_t num_rows_, std::size_t num_scalars_per_row_>
    struct VectorSignature
    {
        using S = SimdT_;
        inline static constexpr std::size_t num_rows = num_rows_;
        inline static constexpr std::size_t num_scalars_per_row = num_scalars_per_row_;
        inline static constexpr std::size_t num_scalars = num_rows * num_scalars_per_row;
        inline static constexpr std::size_t num_vectors_per_row =
            fit_scalars_to_vectors(num_scalars_per_row, S::VectorWidth);
        inline static constexpr std::size_t num_scalars_per_row_rounded =
            num_vectors_per_row * S::VectorWidth;
        inline static constexpr std::size_t num_scalars_rounded = num_rows * num_scalars_per_row_rounded;
        // BUG: This calculation can be wrong, e.g. if num_scalars_per_row < VectorWidth, and leads
        // to underpacking
        // NOTE: After some discussion, we've decided to leave this - the parameters are clearly
        // named, and packing tightly would make row access/extraction operations more expensive
        // (since they would need to parts of multiple vectors in weird overlaps)
        inline static constexpr std::size_t num_vectors = num_rows * num_vectors_per_row;
        using DataT = std::array<typename S::VectorT, num_vectors>;
    };

    template <typename DerivedT, typename Sig>
    struct VectorInterface
    {
        using D = DerivedT;
        using S = typename Sig::S;
        inline static constexpr std::size_t num_scalars = Sig::num_scalars;
        inline static constexpr std::size_t num_scalars_rounded = Sig::num_scalars_rounded;
        inline static constexpr std::size_t num_vectors = Sig::num_vectors;
        inline static constexpr std::size_t num_scalars_per_row = Sig::num_scalars_per_row;
        inline static constexpr std::size_t num_rows = Sig::num_rows;
        using DataT = typename Sig::DataT;

        inline constexpr auto to_array() const noexcept
            -> std::array<typename S::ScalarT, num_scalars_rounded>
        {
            alignas(S::Alignment) std::array<typename S::ScalarT, num_scalars_rounded> result = {};
            to_array(result);
            return result;
        }

        inline constexpr void
        to_array(std::array<typename S::ScalarT, num_scalars_rounded> &buf) const noexcept
        {
            to_array(buf.data());
        }

        inline constexpr void to_array(typename S::ScalarT *buf) const noexcept
        {
            store_vector(buf, std::make_index_sequence<num_vectors>());
        }

        inline constexpr void to_array_unaligned(typename S::ScalarT *buf) const noexcept
        {
            store_vector_unaligned(buf, std::make_index_sequence<num_vectors>());
        }

        inline static constexpr auto fill(typename S::ScalarT f) noexcept -> D
        {
            return D(make_array(f, std::make_index_sequence<num_vectors>()));
        }

        inline constexpr auto rcp() const noexcept -> D
        {
            return D(apply<S::template rcp<0>>(d()->data));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto add(T o) const noexcept -> D
        {
            return add(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto add(T o) const noexcept -> D
        {
            return D(apply<S::template add<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto sub(T o) const noexcept -> D
        {
            return sub(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto sub(T o) const noexcept -> D
        {
            return D(apply<S::template sub<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto mul(T o) const noexcept -> D
        {
            return mul(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto mul(T o) const noexcept -> D
        {
            return D(apply<S::template mul<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto div(T o) const noexcept -> D
        {
            return div(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto div(T o) const noexcept -> D
        {
            return D(apply<S::template div<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto equal(T o) const noexcept -> D
        {
            return equal(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto equal(T o) const noexcept -> D
        {
            return D(apply<S::template cmp_equal<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto not_equal(T o) const noexcept -> D
        {
            return D(apply<S::template cmp_not_equal<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto not_equal(T o) const noexcept -> D
        {
            return not_equal(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto less_equal(T o) const noexcept -> D
        {
            return D(apply<S::template cmp_less_equal<0>>(d()->data, o));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto greater_equal(T o) const noexcept -> D
        {
            return greater_equal(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto greater_equal(T o) const noexcept -> D
        {
            if constexpr (std::is_same_v<T, DataT>)
            {
                return D{{apply<S::template cmp_greater_equal<0>>(d()->data, o)}};
            }
            else
            {
                return D(apply<S::template cmp_greater_equal<0>>(d()->data, o));
            }
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto greater_than(T o) const noexcept -> D
        {
            return greater_than(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto greater_than(T o) const noexcept -> D
        {
            if constexpr (std::is_same_v<T, DataT>)
            {
                return D{{apply<S::template cmp_greater_than<0>>(d()->data, o)}};
            }
            else
            {
                return D(apply<S::template cmp_greater_than<0>>(d()->data, o));
            }
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto less_than(T o) const noexcept -> D
        {
            return less_than(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto less_than(T o) const noexcept -> D
        {
            if constexpr (std::is_same_v<T, DataT>)
            {
                return D{{apply<S::template cmp_less_than<0>>(d()->data, o)}};
            }
            else
            {
                return D(apply<S::template cmp_less_than<0>>(d()->data, o));
            }
        }

        inline constexpr auto equal(typename S::ScalarT s) const noexcept -> D
        {
            return equal(broadcast_scalar(s));
        }

        inline constexpr auto less_equal(typename S::ScalarT s) const noexcept -> D
        {
            return less_equal(broadcast_scalar(s));
        }

        inline constexpr auto greater_equal(typename S::ScalarT s) const noexcept -> D
        {
            return greater_equal(broadcast_scalar(s));
        }

        inline constexpr auto greater_than(typename S::ScalarT s) const noexcept -> D
        {
            return greater_than(broadcast_scalar(s));
        }

        inline constexpr auto less_than(typename S::ScalarT s) const noexcept -> D
        {
            return greater_than(broadcast_scalar(s));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto test_zero(T o) const noexcept -> bool
        {
            return test_zero(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto test_zero(T o) const noexcept -> bool
        {
            return unpack::and_(apply<S::template test_zero<0>>(d()->data, o));
        }

        [[nodiscard]] inline constexpr auto test_zero() const noexcept -> bool
        {
            return test_zero(d()->data);
        }

        inline constexpr auto test_zero(typename S::ScalarT s) -> bool
        {
            return test_zero(broadcast_scalar(s));
        }

        inline constexpr auto test_any_equal(D o) const noexcept -> bool
        {
            return (o == *d()).any();
        }

        inline constexpr auto test_any_less_equal(D o) const noexcept -> bool
        {
            return (*d() <= o).any();
        }

        inline constexpr auto test_any_greater_equal(D o) const noexcept -> bool
        {
            return (*d() >= o).any();
        }

        inline constexpr auto test_all_equal(D o) const noexcept -> bool
        {
            return (*d() == o).all();
        }

        inline constexpr auto test_all_less_equal(D o) const noexcept -> bool
        {
            return (*d() <= o).all();
        }

        inline constexpr auto test_all_greater_equal(D o) const noexcept -> bool
        {
            return (*d() >= o).all();
        }

        inline constexpr auto and_(D o) const noexcept -> D
        {
            return and_(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto and_(T o) const noexcept -> D
        {
            return D(apply<S::template and_<0>>(d()->data, o));
        }

        inline constexpr auto or_(D o) const noexcept -> D
        {
            return or_(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto or_(T o) const noexcept -> D
        {
            return D(apply<S::template or_<0>>(d()->data, o));
        }

        inline constexpr auto xor_(D o) const noexcept -> D
        {
            return xor_(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto xor_(T o) const noexcept -> D
        {
            return D(apply<S::template xor_<0>>(d()->data, o));
        }

        template <
            typename T,
            typename allow_types<DataT, typename S::VectorT, unsigned int>::template check<T> = true>
        inline constexpr auto shift_left(T c) const noexcept -> D
        {
            return D(apply<S::template shift_left<0>>(d()->data, c));
        }

        template <
            typename T,
            typename allow_types<DataT, typename S::VectorT, unsigned int>::template check<T> = true>
        inline constexpr auto shift_right(T c) const noexcept -> D
        {
            return D(apply<S::template shift_right<0>>(d()->data, c));
        }

        inline constexpr auto abs() const noexcept -> D
        {
            return D(apply<S::template abs<0>>(d()->data));
        }

        inline constexpr auto sqrt() const noexcept -> D
        {
            return D(apply<S::template sqrt<0>>(d()->data));
        }

        inline constexpr auto floor() const noexcept -> D
        {
            return D(apply<S::template floor<0>>(d()->data));
        }

        inline constexpr auto clamp(typename S::VectorT lower, typename S::VectorT upper) const noexcept -> D
        {
            return D(apply<S::template clamp<0>>(d()->data, lower, upper));
        }

        // HACK: Because making shape-matched clamping work in general is a pain
        template <std::size_t n_v = num_vectors, typename = std::enable_if_t<n_v == 1, bool>>
        inline constexpr auto clamp(const D &lower, const D &upper) const noexcept -> D
        {
            return clamp(lower.data[0], upper.data[0]);
        }

        inline constexpr auto clamp(typename S::ScalarT lower, typename S::ScalarT upper) const noexcept -> D
        {
            return D(
                apply<S::template clamp<0>>(d()->data, broadcast_scalar(lower), broadcast_scalar(upper)));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto max(T o) const noexcept -> D
        {
            return max(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto max(T other) const noexcept -> D
        {
            return D(apply<S::template max<0>>(d()->data, other));
        }

        inline constexpr auto max(typename S::ScalarT other) const noexcept -> D
        {
            return D(apply<S::template max<0>>(d()->data, broadcast_scalar(other)));
        }

        template <typename T, typename allow_types<D>::template check<T> = true>
        inline constexpr auto min(T o) const noexcept -> D
        {
            return min(o.data);
        }

        template <typename T, typename allow_types<DataT, typename S::VectorT>::template check<T> = true>
        inline constexpr auto min(T other) const noexcept -> D
        {
            return D(apply<S::template min<0>>(d()->data, other));
        }

        inline constexpr auto min(typename S::ScalarT other) const noexcept -> D
        {
            return D(apply<S::template min<0>>(d()->data, broadcast_scalar(other)));
        }

        inline constexpr auto hsum() const noexcept -> typename S::ScalarT
        {
            return S::hsum(unpack::sum_(d()->data));
        }

        inline constexpr auto l2_norm() const noexcept -> typename S::ScalarT
        {
            return std::sqrt(squared_l2_norm());
        }

        inline constexpr auto squared_l2_norm() const noexcept -> typename S::ScalarT
        {
            return (D(apply<S::template mul<0>>(d()->data, d()->data))).hsum();
        }

        inline constexpr auto blend(D other, D blend_mask) const noexcept -> D
        {
            return D(apply<S::template blend<0>>(d()->data, other.data, blend_mask.data));
        }

        inline constexpr auto distance(D other) const noexcept -> typename S::ScalarT
        {
            return (other - *d()).l2_norm();
        }

        inline constexpr auto interpolate(D other, typename S::ScalarT alpha) const noexcept -> D
        {
            return *d() + (other - *d()) * alpha;
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename = std::enable_if_t<std::is_same_v<ScalarT, float>, bool>>
        inline auto log() const noexcept -> D
        {
            using Scalar = typename S::ScalarT;
            using IntVec = Vector<typename S::IntT, num_rows, num_scalars_per_row>;
            using IntScalar = typename IntVec::S::ScalarT;

            D x = *d();
            const D invalid_mask = x <= Scalar(0);

            x = x.max(D(std::numeric_limits<Scalar>::min()));
            IntVec emm0 = x.template as<IntVec>() >> 23U;

            const D mantissa_mask = IntVec::fill(static_cast<IntScalar>(0x807FFFFF)).template as<D>();
            x = x & mantissa_mask;
            x = x | D(Scalar(0.5F));

            emm0 = emm0 - IntVec::fill(0x7F);
            D e = D::template from<IntVec>(emm0) + Scalar(1);

            const D small = x < Scalar(0.707106781186547524F);
            const D delta = x & small;
            x = x - Scalar(1) + delta;
            e = e - (D(Scalar(1)) & small);

            const D z = x * x;
            D y = D(Scalar(7.0376836292E-2F));
            y = y * x + Scalar(-1.1514610310E-1F);
            y = y * x + Scalar(+1.1676998740E-1F);
            y = y * x + Scalar(-1.2420140846E-1F);
            y = y * x + Scalar(+1.4249322787E-1F);
            y = y * x + Scalar(-1.6668057665E-1F);
            y = y * x + Scalar(+2.0000714765E-1F);
            y = y * x + Scalar(-2.4999993993E-1F);
            y = y * x + Scalar(+3.3333331174E-1F);
            y = y * x * z;
            y = y + e * Scalar(-2.12194440E-4F);
            y = y - z * Scalar(0.5F);
            x = x + y + e * Scalar(0.693359375F);

            return x | invalid_mask;
        }

        inline constexpr auto remove_corrupted() const noexcept -> D
        {
            auto mask = sub(*d());
            return blend(zero_vector(), mask);
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename =
                std::enable_if_t<std::is_same_v<ScalarT, float> or std::is_same_v<ScalarT, double>, bool>>
        inline auto sin() const noexcept -> D
        {
            using Scalar = typename S::ScalarT;
            using IntVec = Vector<typename S::IntT, num_rows, num_scalars_per_row>;

            D x = *d();
            D sign_bit = x & Scalar(-0.0F);

            x = x.abs();
            D y = x * Scalar(1.27323954473516F);
            IntVec emm2 = IntVec::template from<D>(y);
            emm2 = (emm2 + IntVec::fill(1)) & IntVec::fill(~1);
            y = D::template from<IntVec>(emm2);

            const IntVec emm0 = (emm2 & IntVec::fill(4)) << 29U;
            const IntVec poly_select = (emm2 & IntVec::fill(2)).equal(IntVec::fill(0));

            const D swap_sign_bit = emm0.template as<D>();
            const D poly_mask = poly_select.template as<D>();
            sign_bit = sign_bit ^ swap_sign_bit;

            x = x + y * Scalar(-0.78515625F);
            x = x + y * Scalar(-2.4187564849853515625E-4F);
            x = x + y * Scalar(-3.77489497744594108E-8F);

            const D z = x * x;
            D poly_a = D(Scalar(2.443315711809948E-5F));
            poly_a = poly_a * z + Scalar(-1.388731625493765E-3F);
            poly_a = poly_a * z + Scalar(4.166664568298827E-2F);
            poly_a = poly_a * z * z - z * Scalar(0.5F) + Scalar(1.0F);

            D poly_b = D(Scalar(-1.9515295891E-4F));
            poly_b = poly_b * z + Scalar(8.3321608736E-3F);
            poly_b = poly_b * z + Scalar(-1.6666654611E-1F);
            poly_b = poly_b * z * x + x;

            D y_out = poly_a.blend(poly_b, poly_mask);
            return y_out ^ sign_bit;
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename =
                std::enable_if_t<std::is_same_v<ScalarT, float> or std::is_same_v<ScalarT, double>, bool>>
        inline constexpr auto cos() const noexcept -> D
        {
            constexpr float PI = 3.14159265359;
            const auto v_sq = *d() + static_cast<typename S::ScalarT>(PI / 2.);
            const auto vsq_sq = v_sq - ((v_sq >= static_cast<typename S::ScalarT>(PI)) &
                                        static_cast<typename S::ScalarT>(2 * PI));
            return vsq_sq.sin();
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename =
                std::enable_if_t<std::is_same_v<ScalarT, float> or std::is_same_v<ScalarT, double>, bool>>
        inline auto asin() const noexcept -> D
        {
            using Scalar = typename S::ScalarT;
            const D x = *d();
            const D a = x.abs();
            const auto gt_half = a > Scalar(0.5);

            const D zz_high = (D(Scalar(1)) - a) * Scalar(0.5);
            const D zz = (a * a).blend(zz_high, gt_half);
            const D x_branch = a.blend(zz_high.sqrt(), gt_half);

            D z = D(Scalar(4.2163199048E-2));
            z = z * zz + Scalar(2.4181311049E-2);
            z = z * zz + Scalar(4.5470025998E-2);
            z = z * zz + Scalar(7.4953002686E-2);
            z = z * zz + Scalar(1.6666752422E-1);
            z = z * zz * x_branch + x_branch;

            const D z_large = Scalar(1.5707963267948966) - (z + z);
            const D res = z.blend(z_large, gt_half);

            return res.blend(-res, x < Scalar(0));
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename =
                std::enable_if_t<std::is_same_v<ScalarT, float> or std::is_same_v<ScalarT, double>, bool>>
        inline auto acos() const noexcept -> D
        {
            using Scalar = typename S::ScalarT;
            const D x = *d();
            const auto gt_half = x > Scalar(0.5);

            const D arg1 = Scalar(0.5) - x * Scalar(0.5);
            const D z = arg1.sqrt().asin() * Scalar(2);

            constexpr Scalar PI4 = 7.85398163397448309616E-1;
            constexpr Scalar MOREBITS = 6.123233995736765886130E-17;
            const D z2 = (Scalar(PI4) - x.asin()) + Scalar(MOREBITS) + Scalar(PI4);

            return z2.blend(z, gt_half);
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename =
                std::enable_if_t<std::is_same_v<ScalarT, float> or std::is_same_v<ScalarT, double>, bool>>
        inline auto atan() const noexcept -> D
        {
            using Scalar = typename S::ScalarT;
            const D x = *d();
            D a = x.abs();

            const auto gt_3pi8 = a > Scalar(2.41421356237309504880);
            const auto gt_pi8 = a > Scalar(0.41421356237309504880);
            const auto mid_mask = gt_pi8 & ~gt_3pi8;

            const D reduced_big = D(Scalar(-1)) / a;
            const D reduced_mid = (a - Scalar(1)) / (a + Scalar(1));

            a = a.blend(reduced_big, gt_3pi8).blend(reduced_mid, mid_mask);

            D y = D(Scalar(0));
            y = y.blend(D(Scalar(1.57079632679489661923)), gt_3pi8);
            y = y.blend(D(Scalar(0.785398163397448309616)), mid_mask);

            const D zz = a * a;
            D p = D(Scalar(-8.750608600031904122785E-1));
            p = p * zz + Scalar(-1.615753718733365076637E1);
            p = p * zz + Scalar(-7.500855792314704667340E1);
            p = p * zz + Scalar(-1.228866684490136173410E2);
            p = p * zz + Scalar(-6.485021904942025371773E1);
            p = p * zz * a;

            D q = zz + Scalar(2.485846490142306297962E1);
            q = q * zz + Scalar(1.650270098316988542046E2);
            q = q * zz + Scalar(4.328810604912902668951E2);
            q = q * zz + Scalar(4.853903996359136964868E2);
            q = q * zz + Scalar(1.945506571482613964425E2);

            const D z = p / q + a + y;
            return z.blend(-z, x < Scalar(0));
        }

        template <
            typename ScalarT = typename S::ScalarT,
            typename =
                std::enable_if_t<std::is_same_v<ScalarT, float> or std::is_same_v<ScalarT, double>, bool>>
        inline auto atan2(D x) const noexcept -> D
        {
            using Scalar = typename S::ScalarT;
            constexpr Scalar PI = 3.14159265358979323846;
            constexpr Scalar PI_2 = 1.57079632679489661923;

            const D y = *d();
            D z = (y / x).atan();

            const auto x_lt_0 = x < Scalar(0);
            const auto y_lt_0 = y < Scalar(0);
            const auto y_ge_0 = y >= Scalar(0);

            // x < 0, y >= 0 -> +pi; x < 0, y < 0 -> -pi
            z = z.blend(z + Scalar(PI), x_lt_0 & y_ge_0);
            z = z.blend(z - Scalar(PI), x_lt_0 & y_lt_0);

            // x == 0: +/- pi/2 depending on sign of y
            const auto x_eq_0 = x == Scalar(0);
            z = z.blend(D(Scalar(PI_2)), x_eq_0 & (y > Scalar(0)));
            z = z.blend(D(Scalar(-PI_2)), x_eq_0 & (y < Scalar(0)));

            // x == 0 && y == 0 -> 0
            z = z.blend(D(Scalar(0)), x_eq_0 & (y == Scalar(0)));
            return z;
        }

        template <typename OtherT, typename BoundsT>
        inline static constexpr auto map_to_range(OtherT v, BoundsT min_v, BoundsT max_v) noexcept -> D
        {
            constexpr typename S::ScalarT lo = -0.5F;
            constexpr typename S::ScalarT hi = 0.5F;

            // maps [-INT_MAX, INT_MAX] to [-0.5, 0.5]
            const auto normalized = D(apply<S::template map_to_range<typename OtherT::S::VectorT>>(v.data));

            // adjust to desired range
            return min_v + ((normalized - lo) / (hi - lo)) * (max_v - min_v);
        }

        template <
            typename T,
            typename allow_types<DataT, typename S::VectorT, unsigned int>::template check<T> = true>
        inline constexpr auto operator<<(T c) const noexcept -> D
        {
            return shift_left(c);
        }

        template <
            typename T,
            typename allow_types<DataT, typename S::VectorT, unsigned int>::template check<T> = true>
        inline constexpr auto operator>>(T c) const noexcept -> D
        {
            return shift_right(c);
        }

        inline constexpr auto operator~() const noexcept -> D
        {
            return D(apply<S::template bitneg<0>>(d()->data));
        }

        inline constexpr auto operator-() const noexcept -> D
        {
            return D(apply<S::template neg<0>>(d()->data));
        }

        inline constexpr auto operator+(D o) const noexcept -> D
        {
            return add(o.data);
        }

        inline constexpr auto operator-(D o) const noexcept -> D
        {
            return sub(o.data);
        }

        inline constexpr auto operator*(D o) const noexcept -> D
        {
            return mul(o.data);
        }

        inline constexpr auto operator/(D o) const noexcept -> D
        {
            return div(o.data);
        }

        inline constexpr auto operator|(D o) const noexcept -> D
        {
            return or_(o.data);
        }

        inline constexpr auto operator&(D o) const noexcept -> D
        {
            return and_(o.data);
        }

        inline constexpr auto operator^(D o) const noexcept -> D
        {
            return xor_(o.data);
        }

        inline constexpr auto operator==(D o) const noexcept -> D
        {
            return equal(o.data);
        }

        inline constexpr auto operator!=(D o) const noexcept -> D
        {
            return not_equal(o.data);
        }

        inline constexpr auto operator<=(D o) const noexcept -> D
        {
            return less_equal(o.data);
        }

        inline constexpr auto operator>=(D o) const noexcept -> D
        {
            return greater_equal(o.data);
        }

        inline constexpr auto operator<(D o) const noexcept -> D
        {
            return less_than(o.data);
        }

        inline constexpr auto operator>(D o) const noexcept -> D
        {
            return greater_than(o.data);
        }

        inline constexpr auto operator+(typename S::ScalarT f) const noexcept -> D
        {
            return add(broadcast_scalar(f));
        }

        inline constexpr auto operator-(typename S::ScalarT f) const noexcept -> D
        {
            return sub(broadcast_scalar(f));
        }

        inline constexpr auto operator*(typename S::ScalarT f) const noexcept -> D
        {
            return mul(broadcast_scalar(f));
        }

        inline constexpr auto operator/(typename S::ScalarT f) const noexcept -> D
        {
            return div(broadcast_scalar(f));
        }

        inline constexpr auto operator|(typename S::ScalarT o) const noexcept -> D
        {
            return or_(broadcast_scalar(o));
        }

        inline constexpr auto operator&(typename S::ScalarT o) const noexcept -> D
        {
            return and_(broadcast_scalar(o));
        }

        inline constexpr auto operator==(typename S::ScalarT f) const noexcept -> D
        {
            return equal(broadcast_scalar(f));
        }

        inline constexpr auto operator!=(typename S::ScalarT f) const noexcept -> D
        {
            return not_equal(broadcast_scalar(f));
        }

        inline constexpr auto operator<=(typename S::ScalarT f) const noexcept -> D
        {
            return less_equal(broadcast_scalar(f));
        }

        inline constexpr auto operator>=(typename S::ScalarT f) const noexcept -> D
        {
            return greater_equal(broadcast_scalar(f));
        }

        inline constexpr auto operator>(typename S::ScalarT f) const noexcept -> D
        {
            return greater_than(broadcast_scalar(f));
        }

        [[nodiscard]] inline constexpr auto any() const noexcept -> bool
        {
            return unpack::or_(apply<S::template mask<0>>(d()->data));
        }

        [[nodiscard]] inline constexpr auto none() const noexcept -> bool
        {
            return !unpack::or_(apply<S::template mask<0>>(d()->data));
        }

        [[nodiscard]] inline constexpr auto all() const noexcept -> bool
        {
            return unpack::and_(apply<all_true>(apply<S::template mask<0>>(d()->data)));
        }

        inline static constexpr auto zero_vector() -> D
        {
            return D(apply<S::template zero_vector<0>, DataT>());
        }

        template <typename OtherT, typename std::enable_if_t<not std::is_same_v<OtherT, D>, bool> = true>
        inline constexpr auto to() const noexcept -> OtherT
        {
            return OtherT{apply<S::template to<typename OtherT::S::VectorT>>(d()->data)};
        }

        template <typename OtherT, typename std::enable_if_t<std::is_same_v<OtherT, D>, bool> = true>
        inline constexpr auto to() const noexcept -> OtherT
        {
            return *d();
        }

        template <typename OtherT, typename std::enable_if_t<not std::is_same_v<OtherT, D>, bool> = true>
        inline static constexpr auto from(OtherT v) noexcept -> D
        {
            return D(apply<S::template from<typename OtherT::S::VectorT>, typename OtherT::DataT>(v.data));
        }

        template <typename OtherT, typename std::enable_if_t<not std::is_same_v<OtherT, D>, bool> = true>
        inline constexpr auto as() const noexcept -> OtherT
        {
            return OtherT{apply<S::template as<typename OtherT::S::VectorT>>(d()->data)};
        }

        inline constexpr auto trim() const noexcept -> D
        {
            if constexpr (num_scalars % S::VectorWidth)
            {
                constexpr auto mask = generate_mask(
                    std::make_index_sequence<S::VectorWidth - (num_scalars % S::VectorWidth)>());
                D masked(d()->data);
                const auto ZERO = S::template zero_vector<0>();
                masked.d()->data.back() = S::template blend_constant<mask>(masked.d()->data.back(), ZERO);
                return masked;
            }
            else
            {
                return *d();
            }
        }

        template <typename IndexT, typename = std::enable_if_t<same_num_scalars<D, IndexT>::value>>
        inline static constexpr auto gather(const typename S::ScalarT *base, const IndexT &idxs) noexcept -> D
        {
            return D(apply<S::template gather<void>>(idxs.data, base));
            // return D(apply_indexed<S::gather>(idxs.data, base));
        }

        template <
            typename IndexT,
            typename MaskT,
            typename = std::enable_if_t<same_num_scalars<D, IndexT>::value>,
            typename = std::enable_if_t<same_num_scalars<D, MaskT>::value>>
        inline static constexpr auto gather_select(
            const typename S::ScalarT *base,
            const IndexT &idxs,
            const MaskT &mask,
            const D &alternative) noexcept -> D
        {
            return D(
                apply_indexed<S::template gather_select<void>>(
                    idxs.data, mask.template to<D>().data, alternative.data, base));
        }

    protected:
        template <std::size_t... I>
        inline static constexpr auto generate_mask(std::index_sequence<I...>) noexcept -> unsigned int
        {
            return (... | (1 << (S::VectorWidth - 1 - I)));
        }

        inline constexpr void
        broadcast_array(std::array<typename S::ScalarT, num_vectors> scalar_data) noexcept
        {
            d()->data = apply<S::template constant<0>>(scalar_data);
        }

        inline void broadcast_vector(std::vector<typename S::ScalarT> scalar_data) noexcept
        {
            assert(scalar_data.size() == num_vectors);
            d()->data = apply<S::template constant<0>>(
                *reinterpret_cast<std::array<typename S::ScalarT, num_vectors> *>(scalar_data.data()));
        }

        inline constexpr auto broadcast_scalar(typename S::ScalarT s) const noexcept -> typename S::VectorT
        {
            return S::template constant<0>(s);
        }

        template <bool is_aligned = true>
        inline constexpr void pack(const typename S::ScalarT *const scalar_data) noexcept
        {
            load_vector<is_aligned>(scalar_data, std::make_index_sequence<num_vectors>());
        }

        template <auto fn, std::size_t stride = 1, std::size_t... I>
        inline static constexpr void
        scalar_stride(typename S::ScalarT *base, DataT data, std::index_sequence<I...>) noexcept
        {
            (..., fn(base + I * stride, std::get<I>(data)));
        }

        template <auto fn, std::size_t stride = 1, std::size_t... I>
        inline static constexpr void
        scalar_stride(const typename S::ScalarT *base, DataT data, std::index_sequence<I...>) noexcept
        {
            (..., fn(base + I * stride, std::get<I>(data)));
        }

        template <bool is_aligned, std::size_t... I>
        inline constexpr void
        load_vector(const typename S::ScalarT *const scalar_array, std::index_sequence<I...>) noexcept
        {
            // TODO: This might segfault if we had to over-allocate vectors and the scalar data isn't
            // full for the over-allocated size
            if constexpr (is_aligned)
            {
                (..., (std::get<I>(d()->data) = S::template load<0>(scalar_array + I * S::VectorWidth)));
            }
            else
            {
                (...,
                 (std::get<I>(d()->data) = S::template load_unaligned<0>(scalar_array + I * S::VectorWidth)));
            }
        }

        template <std::size_t... I>
        inline constexpr void
        store_vector(typename S::ScalarT *scalar_array, std::index_sequence<I...>) const noexcept
        {
            // TODO: This might segfault if we had to over-allocate vectors and the scalar data isn't
            // full for the over-allocated size
            (..., (S::template store<0>(scalar_array + I * S::VectorWidth, std::get<I>(d()->data))));
        }

        template <std::size_t... I>
        inline constexpr void
        store_vector_unaligned(typename S::ScalarT *scalar_array, std::index_sequence<I...>) const noexcept
        {
            // TODO: This might segfault if we had to over-allocate vectors and the scalar data isn't
            // full for the over-allocated size
            (...,
             (S::template store_unaligned<0>(scalar_array + I * S::VectorWidth, std::get<I>(d()->data))));
        }

        inline constexpr auto d() const noexcept -> const D *
        {
            return static_cast<const D *>(this);
        }

        inline constexpr auto d() noexcept -> D *
        {
            return static_cast<D *>(this);
        }

        inline static constexpr auto any_true(unsigned int v) noexcept -> bool
        {
            return v != 0;
        }

        inline static constexpr auto all_true(unsigned int v) noexcept -> bool
        {
            return v == (static_cast<unsigned int>(1) << S::VectorWidth) - 1;
        }

        template <std::size_t... I>
        inline static constexpr auto
        make_array(typename S::ScalarT f, std::index_sequence<I...> /* indices */) noexcept -> DataT
        {
            return {(static_cast<void>(I), S::constant(f))...};
        }
    };

    template <typename SimdT, std::size_t num_rows, std::size_t num_scalars_per_row>
    struct Vector : public VectorInterface<
                        Vector<SimdT, num_rows, num_scalars_per_row>,
                        VectorSignature<SimdT, num_rows, num_scalars_per_row>>
    {
        using S = SimdT;
        using Sig = VectorSignature<S, num_rows, num_scalars_per_row>;
        using Interface = VectorInterface<Vector<S, num_rows, num_scalars_per_row>, Sig>;
        inline static constexpr std::size_t num_scalars = Sig::num_scalars;
        inline static constexpr std::size_t num_vectors_per_row = Sig::num_vectors_per_row;
        inline static constexpr std::size_t num_vectors = Sig::num_vectors;
        using DataT = typename Sig::DataT;
        using RowT = Vector<S, 1, num_scalars_per_row>;

        DataT data{0};

        constexpr Vector() noexcept = default;

        constexpr Vector(DataT data_) noexcept : data(std::move(data_))
        {
        }

        // TODO: Enable unaligned load for other constructors too
        constexpr Vector(const typename S::ScalarT *const scalar_data, bool is_aligned) noexcept
        {
            // NOTE: assumes that scalar_data is a multiple of VectorWidth of valid data
            if (is_aligned)
            {
                Interface::pack(scalar_data);
            }
            else
            {
                Interface::template pack<false>(scalar_data);
            }
        }

        constexpr Vector(const typename S::ScalarT *const scalar_data) noexcept
        {
            // NOTE: assumes that scalar_data is a multiple of VectorWidth of valid data
            Interface::pack(scalar_data);
        }

        constexpr Vector(std::array<typename S::ScalarT, num_scalars> scalar_data) noexcept
        {
            if constexpr (num_scalars % S::VectorWidth)
            {
                alignas(S::Alignment) std::array<
                    typename S::ScalarT,

                    num_scalars + (S::VectorWidth - (num_scalars % S::VectorWidth))>
                    rounded_size_buffer{0};

                for (auto i = 0U; i < scalar_data.size(); ++i)
                {
                    rounded_size_buffer[i] = scalar_data[i];
                }

                Interface::pack(rounded_size_buffer.data());
            }
            else
            {
                alignas(S::Alignment) std::array<typename S::ScalarT, num_scalars> aligned_buffer =
                    scalar_data;
                Interface::pack(aligned_buffer.data());
            }
        }

        template <std::size_t n_s = num_scalars, typename std::enable_if_t<n_s != 1>>
        constexpr Vector(std::array<typename S::ScalarT, num_rows> scalar_data) noexcept
        {
            Interface::broadcast_array(std::move(scalar_data));
        }

        constexpr Vector(std::vector<typename S::ScalarT> scalar_data, bool broadcast_ = false) noexcept
        {
            if (broadcast_)
            {
                assert(scalar_data.size() == num_rows);
                Interface::broadcast_vector(std::move(scalar_data));
            }
            else
            {
                assert(scalar_data.size() == num_scalars);
                if constexpr (num_scalars % S::VectorWidth)
                {
                    alignas(S::Alignment) std::array<
                        typename S::ScalarT,
                        num_scalars + (S::VectorWidth - (num_scalars % S::VectorWidth))>
                        rounded_size_buffer{0};
                    std::copy(scalar_data.begin(), scalar_data.end(), rounded_size_buffer.begin());
                    Interface::pack(rounded_size_buffer.data());
                }
                else
                {
                    Interface::pack(scalar_data.data());
                }
            }
        }

        inline static constexpr auto
        pack_and_pad(std::vector<typename S::ScalarT> scalar_data, std::size_t pad_idx = 0) noexcept
        {
            // TODO: Consider de-duplication with trim()
            auto result = Vector(std::forward<std::vector<typename S::ScalarT>>(scalar_data));
            if constexpr (num_scalars % S::VectorWidth)
            {
                constexpr auto mask = Interface::generate_mask(
                    std::make_index_sequence<S::VectorWidth - (num_scalars % S::VectorWidth)>());
                S::template blend_constant<mask>(result.d()->data.back(), S::constant(scalar_data[pad_idx]));
            }

            return result;
        }

        // TODO: Make sure we always want filling behavior here
        constexpr Vector(typename S::ScalarT scalar_data) noexcept : Vector(Interface::fill(scalar_data))
        {
        }

        // TODO: Make sure we always want filling behavior here
        template <typename DT = typename S::ScalarT, typename = std::enable_if_t<not std::is_same_v<DT, int>>>
        explicit constexpr Vector(int scalar_data) noexcept
          : Vector(Interface::fill(typename S::ScalarT(scalar_data)))
        {
        }

        [[nodiscard]] inline constexpr auto row(std::size_t idx) const noexcept -> RowT
        {
            return RowT(*reinterpret_cast<const std::array<typename S::VectorT, num_vectors_per_row> *>(
                data.data() + idx));
        }

        [[nodiscard]] inline constexpr auto row(std::size_t idx) noexcept -> RowT &
        {
            return *reinterpret_cast<RowT *>(
                reinterpret_cast<std::array<typename S::VectorT, num_vectors_per_row> *>(data.data() + idx));
        }

        [[nodiscard]] inline constexpr auto element(std::size_t idx) const noexcept -> typename S::ScalarT
        {
            std::size_t row_idx = idx / num_scalars_per_row;
            std::size_t col_idx = idx % num_scalars_per_row;
            return S::extract(data[row_idx], col_idx);
        }

        inline constexpr auto operator[](std::size_t idx) const noexcept -> RowT
        {
            return row(idx);
        }

        inline constexpr auto operator[](std::pair<std::size_t, std::size_t> idx) const noexcept ->
            typename S::ScalarT
        {
            auto [row_idx, col_idx] = idx;
            return S::extract(data[row_idx], col_idx);
        }

        inline constexpr auto operator[](std::size_t idx) noexcept -> RowT &
        {
            return row(idx);
        }

        inline constexpr auto broadcast(std::size_t idx) const noexcept -> Vector<S, 1, S::VectorWidth>
        {
            auto [v_idx, v_offset] = utils::c_div(idx, S::VectorWidth);
            return Vector<S, 1, S::VectorWidth>({S::broadcast(data[v_idx], v_offset)});
        }
    };

    namespace
    {
        template <typename T>
        struct is_vector
        {
        private:
            template <typename D, typename S>
            static auto check_convert(const VectorInterface<D, S> &)
                -> decltype(static_cast<const VectorInterface<D, S> &>(std::declval<T>()), std::true_type{});

            static auto check_convert(...) -> std::false_type;

        public:
            static constexpr bool value = decltype(is_vector::check_convert(std::declval<T>()))::value;
        };
    }  // namespace

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator/(typename VectorIT::S::ScalarT s, VectorIT v) noexcept -> VectorIT
    {
        // TODO: Make a broadcast_scalar-based version of this
        return VectorIT{VectorIT::fill(s)} / v;
    }

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator*(typename VectorIT::S::ScalarT s, VectorIT v) noexcept -> VectorIT
    {
        return v * s;
    }

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator+(typename VectorIT::S::ScalarT s, VectorIT v) noexcept -> VectorIT
    {
        return v + s;
    }

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator-(typename VectorIT::S::ScalarT s, VectorIT v) noexcept -> VectorIT
    {
        // TODO: Make a broadcast_scalar-based version of this
        return VectorIT{VectorIT::fill(s)} - v;
    }

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator|(typename VectorIT::S::ScalarT s, VectorIT v) noexcept -> VectorIT
    {
        return v | s;
    }

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator&(typename VectorIT::S::ScalarT s, VectorIT v) noexcept -> VectorIT
    {
        return v & s;
    }

    template <typename VectorIT, typename = std::enable_if_t<is_vector<VectorIT>::value, bool>>
    inline constexpr auto operator<<(std::ostream &o, VectorIT v) noexcept -> std::ostream &
    {
        o << "[";

        for (auto i = 0ul; i < v.data.size() - 1; ++i)
        {
            o << " [";
            print_vector<typename VectorIT::S>(o, v.data[i]);
            o << "],";
        }

        o << " [";

        print_vector<typename VectorIT::S>(o, v.data.back());
        o << "] ]";

        return o;
    }

    template <typename VectorT>
    struct SIMDVector;
}  // namespace vamp
