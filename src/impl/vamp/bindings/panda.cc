#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/panda.hh>

void vamp::binding::init_panda(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Panda>(pymodule);
}
