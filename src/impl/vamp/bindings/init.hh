#include <nanobind/nanobind.h>

namespace vamp::binding
{
    void init_environment(nanobind::module_ &pymodule);
    void init_settings(nanobind::module_ &pymodule);
    void init_sphere(nanobind::module_ &pymodule);
    void init_ur5(nanobind::module_ &pymodule);
    void init_panda(nanobind::module_ &pymodule);
    void init_fetch(nanobind::module_ &pymodule);
    void init_baxter(nanobind::module_ &pymodule);
}  // namespace vamp::binding
