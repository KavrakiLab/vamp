#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/ur5.hh>

void vamp::binding::init_ur5(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::UR5>(pymodule);
}
