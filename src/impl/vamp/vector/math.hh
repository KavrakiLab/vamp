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
    return v.sqrt();
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

template <size_t N, typename DataT>
inline static auto to_isometries(const DataT *buf) -> std::array<Eigen::Transform<DataT, 3, Eigen::Isometry>, N>
{
    std::array<Eigen::Transform<DataT, 3, Eigen::Isometry>, N> out;

    for(auto i=0; i < N; i++)
    {
        auto offset = i * 12;
        const Eigen::Map<const Eigen::Matrix<DataT, 3, 3>> R(&buf[offset + 3]);

        out[i].translation()[0] = buf[offset + 0];
        out[i].translation()[1] = buf[offset + 1];
        out[i].translation()[2] = buf[offset + 2];
        out[i].linear() = R;
    }

    return out;
}