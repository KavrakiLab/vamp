#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/sphere.hh>

void vamp::binding::init_sphere(nanobind::module_ &pymodule)
{
    auto submodule = vamp::binding::init_robot<vamp::robots::Sphere>(pymodule);
    submodule.def("set_lows", &vamp::robots::sphere::set_lows);
    submodule.def("set_highs", &vamp::robots::sphere::set_highs);
    submodule.def("set_radius", &vamp::robots::sphere::set_radius);
}
