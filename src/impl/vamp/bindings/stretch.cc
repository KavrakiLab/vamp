#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/stretch.hh>

void vamp::binding::init_stretch(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Stretch>(pymodule);
}
