#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

template <typename DataT>
inline constexpr auto sin(const DataT &v) -> DataT
{
    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::sin(v);
    }
    else
    {
        return v.sin();
    }
}

template <typename DataT>
inline constexpr auto cos(const DataT &v) -> DataT
{
    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::cos(v);
    }
    else
    {
        return v.cos();
    }
}

template <typename DataT>
inline constexpr auto sqrt(const DataT &v) -> DataT
{

    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::sqrt(v);
    }
    else
    {
        return v.sqrt();
    }

}


template <typename DataT, typename DataTB>
inline constexpr auto max(const DataT &x, const DataTB &y) -> DataT
{
    return x.max(y);
}

template <typename DataT, typename DataTB>
inline constexpr auto min(const DataT &x, const DataTB &y) -> DataT
{
    return x.min(y);
}

template <typename DataA, typename DataB, typename DataC>
inline constexpr auto blend(const DataA &a, const DataB &b, const DataC &mask ) -> DataC
{
    if constexpr (std::is_arithmetic_v<DataC>)
    {
        return (mask >=0) ? a:b;
    }
    else
    {
        DataC a_vec(a);
        DataC b_vec(b);
        return a_vec.blend(b_vec, mask);
    }
}

template <typename DataT>
inline constexpr auto asin(const DataT &v) -> DataT
{

    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::asin(v);
    }
    else
    {
        return v.asin();
    }

}

template <typename DataT>
inline constexpr auto acos(const DataT &v) -> DataT
{

    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::acos(v);
    }
    else
    {
        return v.acos();
    }

}

template <typename DataT>
inline constexpr auto atan(const DataT &v) -> DataT
{

    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::atan(v);
    }
    else
    {
        return v.atan();
    }

}

template <typename DataT>
inline constexpr auto atan2(const DataT &y, const DataT &x) -> DataT
{

    if constexpr (std::is_arithmetic_v<DataT>)
    {
        return std::atan2(y, x);
    }
    else
    {
        return y.atan2(x);
    }

}


template <typename DataT>
inline static auto to_isometry(const DataT *buf) -> Eigen::Transform<DataT, 3, Eigen::Isometry>
{
    Eigen::Transform<DataT, 3, Eigen::Isometry> out;
    const Eigen::Map<const Eigen::Matrix<DataT, 3, 3>> R(&buf[3]);

    out.translation()[0] = buf[0];
    out.translation()[1] = buf[1];
    out.translation()[2] = buf[2];
    out.linear() = R;

    return out;
}