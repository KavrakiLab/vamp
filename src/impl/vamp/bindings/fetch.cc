#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/fetch.hh>

void vamp::binding::init_fetch(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::Fetch>(pymodule);
}
