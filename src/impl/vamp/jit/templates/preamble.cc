// Concatenated to the front of every JIT'd stub TU. Must come BEFORE the
// caller's robot_source because some robot headers (e.g. vamp/robots/sphere.hh)
// reference vamp::collision / Eigen types without including those headers
// themselves. No runtime substitution — pure includes.
#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/xorshift.hh>
#include <vamp/vector.hh>
// Per-planner headers (<vamp/planning/{planner}.hh>) are pulled in by the
// individual planner stubs, not here.

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>
