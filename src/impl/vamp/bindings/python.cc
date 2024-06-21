#include <vamp/bindings/init.hh>

namespace vb = vamp::binding;

NB_MODULE(_core_ext, pymodule)
{
    vb::init_settings(pymodule);
    vb::init_environment(pymodule);
    vb::init_sphere(pymodule);
    vb::init_ur5(pymodule);
    vb::init_panda(pymodule);
    vb::init_fetch(pymodule);
    vb::init_baxter(pymodule);
}
