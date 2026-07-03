#ifdef __clang__
#pragma clang diagnostic ignored "-Wpragma-once-outside-header"
#endif

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/collision/filter.hh>
#include <vamp/collision/validity.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/random/halton.hh>
#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#endif
#include <vamp/vector.hh>

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
