#include <vamp/collision/factory.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/bindings/init.hh>

#include <nanobind/stl/string.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
namespace vc = vamp::collision;
namespace vf = vamp::collision::factory;
using namespace nb::literals;

void vamp::binding::init_environment(nanobind::module_ &pymodule)
{
    nb::class_<vc::Sphere<float>>(pymodule, "Sphere")
        .def(
            "__init__",
            [](vc::Sphere<float> *q, const std::array<float, 3> &center, float radius) noexcept
            { new (q) vc::Sphere<float>(vf::sphere::array(center, radius)); },
            "Constructor from center and radius.")
        .def_static("make_flat", &vf::sphere::flat)
        .def_static("make", &vf::sphere::array)
        .def_ro("x", &vc::Sphere<float>::x)
        .def_ro("y", &vc::Sphere<float>::y)
        .def_ro("z", &vc::Sphere<float>::z)
        .def_ro("r", &vc::Sphere<float>::r)
        .def_ro("min_distance", &vc::Sphere<float>::min_distance)
        .def_rw("name", &vc::Sphere<float>::name);

    nb::class_<vc::Cylinder<float>>(pymodule, "Cylinder")
        .def(
            "__init__",
            [](vc::Cylinder<float> *q,
               const std::array<float, 3> &center,
               const std::array<float, 3> &euler_xyz,
               float radius,
               float length) noexcept
            { new (q) vc::Cylinder<float>(vf::cylinder::center::array(center, euler_xyz, radius, length)); },
            "Constructor from center, Euler XYZ orientation, radius, and length.")
        .def(
            "__init__",
            [](vc::Cylinder<float> *q,
               const std::array<float, 3> &endpoint1,
               const std::array<float, 3> &endpoint2,
               float radius) noexcept { *q = vf::cylinder::endpoints::array(endpoint1, endpoint2, radius); },
            "Constructor from endpoints and radius.")
        .def_ro("x1", &vc::Cylinder<float>::x1)
        .def_ro("y1", &vc::Cylinder<float>::y1)
        .def_ro("z1", &vc::Cylinder<float>::z1)
        .def_prop_ro("x2", &vc::Cylinder<float>::x2)
        .def_prop_ro("y2", &vc::Cylinder<float>::y2)
        .def_prop_ro("z2", &vc::Cylinder<float>::z2)
        .def_ro("xv", &vc::Cylinder<float>::xv)
        .def_ro("yv", &vc::Cylinder<float>::yv)
        .def_ro("zv", &vc::Cylinder<float>::zv)
        .def_ro("r", &vc::Cylinder<float>::r)
        .def_ro("rdv", &vc::Cylinder<float>::rdv)
        .def_ro("min_distance", &vc::Cylinder<float>::min_distance)
        .def_rw("name", &vc::Cylinder<float>::name);

    nb::class_<vc::Cuboid<float>>(pymodule, "Cuboid")
        .def(
            "__init__",
            [](vc::Cuboid<float> *q,
               const std::array<float, 3> &center,
               const std::array<float, 3> &euler_xyz,
               const std::array<float, 3> &half_extents) noexcept
            { new (q) vc::Cuboid<float>(vf::cuboid::array(center, euler_xyz, half_extents)); },
            "Constructor from center, Euler XYZ orientation, and XYZ half-extents.")
        .def_ro("x", &vc::Cuboid<float>::x)
        .def_ro("y", &vc::Cuboid<float>::y)
        .def_ro("z", &vc::Cuboid<float>::z)
        .def_ro("axis_1_x", &vc::Cuboid<float>::axis_1_x)
        .def_ro("axis_1_y", &vc::Cuboid<float>::axis_1_y)
        .def_ro("axis_1_z", &vc::Cuboid<float>::axis_1_z)
        .def_ro("axis_2_x", &vc::Cuboid<float>::axis_2_x)
        .def_ro("axis_2_y", &vc::Cuboid<float>::axis_2_y)
        .def_ro("axis_2_z", &vc::Cuboid<float>::axis_2_z)
        .def_ro("axis_3_x", &vc::Cuboid<float>::axis_3_x)
        .def_ro("axis_3_y", &vc::Cuboid<float>::axis_3_y)
        .def_ro("axis_3_z", &vc::Cuboid<float>::axis_3_z)
        .def_ro("axis_1_r", &vc::Cuboid<float>::axis_1_r)
        .def_ro("axis_2_r", &vc::Cuboid<float>::axis_2_r)
        .def_ro("axis_3_r", &vc::Cuboid<float>::axis_3_r)
        .def_ro("min_distance", &vc::Cuboid<float>::min_distance)
        .def_rw("name", &vc::Cuboid<float>::name);

    pymodule.def("make_heightfield", &vf::heightfield::array);

    nb::class_<vc::HeightField<float>>(pymodule, "HeightField")
        .def_ro("x", &vc::HeightField<float>::x)
        .def_ro("y", &vc::HeightField<float>::y)
        .def_ro("z", &vc::HeightField<float>::z)
        .def_ro("xs", &vc::HeightField<float>::xs)
        .def_ro("ys", &vc::HeightField<float>::ys)
        .def_ro("zs", &vc::HeightField<float>::zs)
        .def_ro("data", &vc::HeightField<float>::data);

    nb::class_<vc::Environment<float>>(pymodule, "Environment")
        .def(nb::init<>())
        .def(
            "add_sphere",
            [](vc::Environment<float> &e, const vc::Sphere<float> &s)
            {
                e.spheres.emplace_back(s);
                e.sort();
            })
        .def(
            "add_cuboid",
            [](vc::Environment<float> &e, const vc::Cuboid<float> &s)
            {
                if (s.axis_3_z == 1.)
                {
                    e.z_aligned_cuboids.emplace_back(s);
                }
                else
                {
                    e.cuboids.emplace_back(s);
                }
                e.sort();
            })
        .def(
            "add_capsule",
            [](vc::Environment<float> &e, const vc::Cylinder<float> &s)
            {
                if (s.xv == 0. and s.yv == 0.)
                {
                    e.z_aligned_capsules.emplace_back(s);
                }
                else
                {
                    e.capsules.emplace_back(s);
                }
                e.sort();
            })
        .def(
            "add_heightfield",
            [](vc::Environment<float> &e, const vc::HeightField<float> &s)
            { e.heightfields.emplace_back(s); });
}
