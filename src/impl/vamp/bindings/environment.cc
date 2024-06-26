#include <vamp/collision/filter.hh>
#include <vamp/collision/capt.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/bindings/init.hh>

#include <nanobind/stl/string.h>
#include <nanobind/stl/pair.h>
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
        .def_prop_ro(
            "position",
            [](vc::Sphere<float> &sphere) {
                return std::array<float, 3>{sphere.x, sphere.y, sphere.z};
            })
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
            { e.heightfields.emplace_back(s); })
        .def(
            "add_pointcloud",
            [](vc::Environment<float> &e,
               const std::vector<collision::Point> &pc,
               float r_min,
               float r_max,
               float r_point)
            {
                auto start_time = std::chrono::steady_clock::now();
                e.pointclouds.emplace_back(pc, r_min, r_max, r_point);
                return vamp::utils::get_elapsed_nanoseconds(start_time);
            })
        .def(
            "attach",
            [](vc::Environment<float> &e, const vc::Attachment<float> &a) { e.attachments.emplace(a); })
        .def("detach", [](vc::Environment<float> &e) { e.attachments.reset(); });

    pymodule.def(
        "filter_pointcloud",
        [](const std::vector<collision::Point> &pc,
           float min_dist,
           float max_range,
           const collision::Point &origin,
           const collision::Point &workcell_min,
           const collision::Point &workcell_max,
           bool cull) -> std::pair<std::vector<collision::Point>, std::size_t>
        {
            auto start_time = std::chrono::steady_clock::now();
            auto filtered =
                vc::filter_pointcloud(pc, min_dist, max_range, origin, workcell_min, workcell_max, cull);
            return {filtered, vamp::utils::get_elapsed_nanoseconds(start_time)};
        });

    nb::class_<vc::Attachment<float>>(pymodule, "Attachment")
        .def(
            "__init__",
            [](vc::Attachment<float> *q,
               const std::array<float, 3> &center,
               const std::array<float, 4> &quaternion_xyzw) noexcept
            {
                new (q) vc::Attachment<float>(
                    center[0],
                    center[1],
                    center[2],
                    quaternion_xyzw[0],
                    quaternion_xyzw[1],
                    quaternion_xyzw[2],
                    quaternion_xyzw[3]);
            },
            "Constructor for an attachment centered at a relative position and XYZW quaternion from the "
            "end-effector.")
        .def_prop_ro(
            "relative_frame",
            [](vc::Attachment<float> &a)
            {
                std::array<float, 3> position = {a.tf_tx, a.tf_ty, a.tf_tz};
                std::array<float, 4> quaternion_xyzw = {a.tf_rx, a.tf_ry, a.tf_rz, a.tf_rw};

                return std::pair{position, quaternion_xyzw};
            })
        .def(
            "add_sphere",
            [](vc::Attachment<float> &a, collision::Sphere<float> &sphere)
            { a.spheres.emplace_back(sphere); })
        .def(
            "add_spheres",
            [](vc::Attachment<float> &a, std::vector<collision::Sphere<float>> &spheres)
            { a.spheres.insert(a.spheres.end(), spheres.cbegin(), spheres.cend()); })
        .def(
            "set_ee_pose",
            [](vc::Attachment<float> &a,
               const std::array<float, 3> &position,
               const std::array<float, 4> &quaternion_xyzw)
            {
                a.pose(
                    position[0],
                    position[1],
                    position[2],
                    quaternion_xyzw[0],
                    quaternion_xyzw[1],
                    quaternion_xyzw[2],
                    quaternion_xyzw[3]);
            },
            "position"_a,
            "quaternion_xyzw"_a)
        .def_ro("posed_spheres", &vc::Attachment<float>::posed_spheres);
}
