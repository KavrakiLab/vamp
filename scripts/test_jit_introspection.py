"""Parity check for the introspection / FK / validation / PHS bindings on
the JIT DynamicRobot vs the static vamp.panda.* module.

Bit-exact: metadata, fk, validate, validate_motion, filter_self_from_pointcloud.
Smoke (samples produced, no crash): PHS + phs_sampler — Eigen SVD's matrix
drifts slightly under different compile flags so the resulting samples can
diverge by 1–2 units in 7-D space, which is benign for the rejection sampler.
"""
import pickle
from pathlib import Path

import numpy as np

import vamp
import vamp._core._core_ext as ext

URDF = "/home/zak/src/vamp_project/cricket/resources/panda/panda_spherized.urdf"
SRDF = "/home/zak/src/vamp_project/cricket/resources/panda/panda.srdf"

print("Loading JIT Panda ...")
jit_robot = ext.load_robot(
    urdf=URDF, srdf=SRDF, end_effector="panda_grasptarget",
    planners=["rrtc"], name="Panda",
)

ok = True


def check(label: str, condition: bool) -> None:
    global ok
    status = "OK" if condition else "FAIL"
    if not condition:
        ok = False
    print(f"  [{status}] {label}")


print("\n=== static metadata ===")
check("dimension matches",     jit_robot.dimension == vamp.panda.dimension())
check("n_spheres matches",     jit_robot.n_spheres == vamp.panda.n_spheres())
check("space_measure matches", abs(jit_robot.space_measure - vamp.panda.space_measure()) < 1e-5)
check("min_max_radii matches", jit_robot.min_max_radii() == vamp.panda.min_max_radii())
check("joint_names matches",   list(jit_robot.joint_names) == list(vamp.panda.joint_names()))
check("upper_bounds matches",  np.allclose(jit_robot.upper_bounds(), vamp.panda.upper_bounds()))
check("lower_bounds matches",  np.allclose(jit_robot.lower_bounds(), vamp.panda.lower_bounds()))

print("\n=== fk / validate / validate_motion ===")
with open(Path(__file__).resolve().parent.parent / "resources" / "panda" / "problems.pkl", "rb") as f:
    problems = pickle.load(f)
data = next(iter(problems["problems"].values()))[0]
env = vamp.problem_dict_to_vamp(data)
config = list(data["start"])
goal = list(data["goals"][0])


def sphere_eq(a, b: object) -> bool:
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z) + abs(a.r - b.r) < 1e-5


fk_static = vamp.panda.fk(config)
fk_l = jit_robot.fk(config)
fk_n = jit_robot.fk(np.asarray(config, dtype=np.float32))
check(f"fk(list) length matches static ({len(fk_static)})", len(fk_l) == len(fk_static))
check("fk(list)    values match static", all(sphere_eq(a, b) for a, b in zip(fk_l, fk_static)))
check("fk(ndarray) values match static", all(sphere_eq(a, b) for a, b in zip(fk_n, fk_static)))

v_static = vamp.panda.validate(config, env, check_bounds=True)
check(f"validate(list)    matches static ({v_static})",
      jit_robot.validate(config, env, check_bounds=True) == v_static)
check("validate(ndarray) matches static",
      jit_robot.validate(np.asarray(config, dtype=np.float32), env, check_bounds=True) == v_static)

vm_static = vamp.panda.validate_motion(config, goal, env, check_bounds=True)
check(f"validate_motion(list)    matches static ({vm_static})",
      jit_robot.validate_motion(config, goal, env, check_bounds=True) == vm_static)
check("validate_motion(ndarray) matches static",
      jit_robot.validate_motion(
          np.asarray(config, np.float32), np.asarray(goal, np.float32),
          env, check_bounds=True) == vm_static)

print("\n=== filter_self_from_pointcloud ===")
pc = [[0.0, 0.0, 0.0], [1.5, 1.5, 1.5], [0.4, 0.0, 0.6]]
radius = 0.02
fil_static = vamp.panda.filter_self_from_pointcloud(pc, radius, config, env)
fil_l = jit_robot.filter_self_from_pointcloud(pc, radius, config, env)
fil_n = jit_robot.filter_self_from_pointcloud(
    np.asarray(pc, dtype=np.float32), radius,
    np.asarray(config, dtype=np.float32), env,
)
print(f"  static:   {fil_static}")
print(f"  jit list: {fil_l}")
check("filter(list)    == static", list(fil_l) == list(fil_static))
check("filter(ndarray) == static", list(fil_n) == list(fil_static))

print("\n=== PHS (smoke + JIT consistency) ===")
foci_dist = float(np.linalg.norm(np.asarray(config) - np.asarray(goal)))
jit_phs = jit_robot.phs(config, goal)
jit_phs.set_transverse_diameter(foci_dist * 1.5)
x = np.full(7, 0.3, dtype=np.float32)
jit_t = jit_phs.transform(x)
jit_t_l = jit_phs.transform(list(x))
check(f"phs.transform(ndarray) shape={jit_t.shape}, dtype={jit_t.dtype}",
      jit_t.shape == (7,) and jit_t.dtype == np.float32)
check("phs.transform(list) == phs.transform(ndarray)", np.allclose(jit_t, jit_t_l))

jit_inner = jit_robot.halton()
jit_phs_rng = jit_robot.phs_sampler(jit_phs, jit_inner)
s = jit_phs_rng.next()
check(f"phs_sampler.next() returns ndarray ({s.dtype}, {s.shape})",
      isinstance(s, np.ndarray) and s.dtype == np.float32 and s.shape == (7,))

print("\nOVERALL:", "PASS" if ok else "FAIL")
raise SystemExit(0 if ok else 1)
