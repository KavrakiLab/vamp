#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/baxter.hh>

void vamp::binding::init_baxter(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Baxter>(pymodule);
}
