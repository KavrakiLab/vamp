#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/panda_attachment.hh>

void vamp::binding::init_panda_attachment(nanobind::module_ &pymodule)
{
    vamp::binding::init_robot<vamp::robots::PandaAttachment>(pymodule);
}
