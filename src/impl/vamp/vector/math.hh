#pragma once

template <typename DataT>
inline constexpr auto sin(const DataT &v) -> DataT
{
    return v.sin();
}

template <typename DataT>
inline constexpr auto cos(const DataT &v) -> DataT
{
    return v.cos();
}

template <typename DataT>
inline constexpr auto sqrt(const DataT &v) -> DataT
{
    return v.sqrt();
}
