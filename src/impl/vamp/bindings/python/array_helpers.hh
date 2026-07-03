#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vamp::binding
{
    namespace nb = nanobind;

    template <typename NDArray, std::size_t Ndim>
    inline auto make_ndarray(const float *data, std::array<std::size_t, Ndim> shape) -> NDArray
    {
        std::size_t total = 1;
        for (auto s : shape)
        {
            total *= s;
        }
        auto *buf = new float[total];
        std::memcpy(buf, data, total * sizeof(float));
        nb::capsule owner(buf, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
        return NDArray(buf, Ndim, shape.data(), owner);
    }

    template <std::size_t Ndim>
    inline auto make_ndarray(const float *data, std::array<std::size_t, Ndim> shape)
        -> nb::ndarray<nb::numpy, float, nb::device::cpu>
    {
        return make_ndarray<nb::ndarray<nb::numpy, float, nb::device::cpu>, Ndim>(data, shape);
    }

    template <typename NDArray, std::size_t Ndim, typename FillFn>
    inline auto
    make_ndarray_filled(std::array<std::size_t, Ndim> shape, FillFn &&fill, std::size_t extra_slop = 0)
        -> NDArray
    {
        std::size_t total = 1;
        for (auto s : shape)
        {
            total *= s;
        }
        auto *buf = new float[total + extra_slop];
        fill(buf);
        nb::capsule owner(buf, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
        return NDArray(buf, Ndim, shape.data(), owner);
    }

    template <std::size_t Ndim, typename FillFn>
    inline auto
    make_ndarray_filled(std::array<std::size_t, Ndim> shape, FillFn &&fill, std::size_t extra_slop = 0)
        -> nb::ndarray<nb::numpy, float, nb::device::cpu>
    {
        return make_ndarray_filled<nb::ndarray<nb::numpy, float, nb::device::cpu>, Ndim>(
            shape, std::forward<FillFn>(fill), extra_slop);
    }

    template <typename NDArray>
    inline auto as_flat_1d(const NDArray &a, std::size_t dim, std::vector<float> &scratch, const char *what)
        -> const float *
    {
        if (a.shape(0) != dim)
        {
            throw std::runtime_error(std::string(what) + " has wrong dimension");
        }
        if (a.stride(0) != 1)
        {
            scratch.resize(dim);
            for (std::size_t i = 0; i < dim; ++i)
            {
                scratch[i] = a(i);
            }
            return scratch.data();
        }
        return a.data();
    }

    template <typename NDArray>
    inline auto
    as_flat_2d(const NDArray &a, std::size_t inner_dim, std::vector<float> &scratch, const char *what)
        -> std::pair<const float *, std::uint64_t>
    {
        if (a.shape(1) != inner_dim)
        {
            throw std::runtime_error(std::string(what) + " has wrong inner dimension");
        }
        const std::uint64_t n = a.shape(0);
        const bool contiguous = (a.stride(0) == static_cast<int64_t>(inner_dim) and a.stride(1) == 1);
        if (contiguous)
        {
            return {a.data(), n};
        }
        scratch.resize(n * inner_dim);
        for (std::uint64_t i = 0; i < n; ++i)
        {
            for (std::size_t j = 0; j < inner_dim; ++j)
            {
                scratch[i * inner_dim + j] = a(i, j);
            }
        }
        return {scratch.data(), n};
    }
}  // namespace vamp::binding