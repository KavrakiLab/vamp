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

// Small nanobind-ndarray helpers shared between the static robot bindings
// (vamp/bindings/python/robot_helper.hh) and the JIT/dynamic bindings
// (vamp/bindings/python/dynamic.cc). The shapes / scalar types vary across
// call sites (static: nb::shape<dim>, dynamic: nb::ndim<N>), so these are
// templated on the ndarray flavor and use ordinary scalar access (`a(i)`,
// `a.data()`) that works for both.

namespace vamp::binding
{
    namespace nb_ = nanobind;

    // Wrap a flat float buffer in a capsule-owned numpy ndarray. The buffer
    // is freshly heap-allocated and copied from `data`; the capsule's
    // deleter frees it when Python drops the array.
    //
    // Two overloads:
    //  - make_ndarray<Ndim>(data, shape) — returns the generic
    //    nb::ndarray<numpy, float, cpu>, used by the dynamic bindings.
    //  - make_ndarray<NDArray, Ndim>(data, shape) — returns the explicit
    //    NDArray flavour (e.g. with nb::shape<Robot::dimension>) used by
    //    the static bindings, where Configuration shape is template-known.
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
        nb_::capsule owner(buf, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
        // nanobind's ndarray ctor takes (data, ndim, shape_ptr, owner).
        return NDArray(buf, Ndim, shape.data(), owner);
    }

    template <std::size_t Ndim>
    inline auto make_ndarray(const float *data, std::array<std::size_t, Ndim> shape)
        -> nb_::ndarray<nb_::numpy, float, nb_::device::cpu>
    {
        return make_ndarray<nb_::ndarray<nb_::numpy, float, nb_::device::cpu>, Ndim>(data, shape);
    }

    // Read a 1-D nanobind ndarray as a const float*. If the array is
    // unit-stride we return the underlying pointer (zero-copy view);
    // otherwise we flatten into `scratch`. Caller keeps `scratch` alive
    // for as long as it uses the returned pointer.
    template <typename NDArray>
    inline auto as_flat_1d(
        const NDArray &a,
        std::size_t dim,
        std::vector<float> &scratch,
        const char *what) -> const float *
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

    // Read a 2-D nanobind ndarray as (const float*, n_rows). C-contiguous
    // input returns a zero-copy view; otherwise the rows are packed into
    // `scratch` row-major.
    template <typename NDArray>
    inline auto as_flat_2d(
        const NDArray &a,
        std::size_t inner_dim,
        std::vector<float> &scratch,
        const char *what) -> std::pair<const float *, std::uint64_t>
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
