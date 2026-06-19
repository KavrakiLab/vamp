import pickle
import time
from pathlib import Path
from typing import List, Union

from fire import Fire
from tabulate import tabulate
from tqdm import tqdm
import pandas as pd

import vamp
import vamp._core._core_ext as ext


# Mapping from a vamp robot name to the cricket inputs needed to JIT it.
# (cricket's resources/ dir layout is the source of truth.)
_CRICKET_ROBOTS = {
    "panda": dict(
        urdf="panda/panda_spherized.urdf",
        srdf="panda/panda.srdf",
        end_effector="panda_grasptarget",
        name="Panda",
    ),
    "ur5": dict(
        urdf="ur5/ur5_spherized.urdf",
        srdf="ur5/ur5.srdf",
        end_effector="robotiq_85_base_link",
        name="UR5",
    ),
    "fetch": dict(
        urdf="fetch/fetch_spherized.urdf",
        srdf="fetch/fetch.srdf",
        end_effector="gripper_link",
        name="Fetch",
    ),
    "baxter": dict(
        urdf="baxter/baxter_spherized.urdf",
        srdf="baxter/baxter.srdf",
        end_effector="right_gripper",
        name="Baxter",
    ),
}


def _cricket_resources_dir() -> Path:
    # Sibling-of-vamp layout in the dev tree.
    return Path(__file__).resolve().parent.parent.parent / "cricket" / "resources"


def main(
    robot: str = "panda",
    dataset: str = "problems.pkl",
    problem: Union[str, List[str]] = (),
    trials: int = 1,
    skip_rng_iterations: int = 0,
    print_failures: bool = False,
    rake: int = 8,
    resolution: int = 32,
    ):
    if robot not in _CRICKET_ROBOTS:
        raise SystemExit(f"No cricket recipe for robot '{robot}'. Known: {list(_CRICKET_ROBOTS)}")

    cfg = _CRICKET_ROBOTS[robot]
    resources = _cricket_resources_dir()
    urdf = resources / cfg["urdf"]
    srdf = resources / cfg["srdf"]
    if not urdf.exists():
        raise SystemExit(f"Missing URDF at {urdf}")

    print(f"[jit] loading {cfg['name']} from {urdf.relative_to(resources)} ...")
    t0 = time.perf_counter()
    jit_robot = ext.load_robot(
        urdf=str(urdf),
        srdf=str(srdf),
        end_effector=cfg["end_effector"],
        planners=["rrtc"],
        rake=rake,
        resolution=resolution,
        name=cfg["name"],
    )
    t1 = time.perf_counter()
    print(f"[jit] load_robot took {(t1 - t0):.2f}s  (dim={jit_robot.dimension}, rake={jit_robot.rake})")

    # MBM problem set ships with vamp.
    problems_path = Path(__file__).resolve().parent.parent / "resources" / robot / dataset
    with open(problems_path, "rb") as f:
        problems = pickle.load(f)

    problem_names = list(problems["problems"].keys())
    if isinstance(problem, str):
        problem = [problem] if problem else []
    if not problem:
        problem = problem_names

    settings = vamp.RRTCSettings()
    # Match the per-robot defaults that configure_robot_and_planner_with_kwargs
    # applies for the static path — otherwise we compare apples to oranges.
    if robot in vamp.ROBOT_RRT_RANGES:
        settings.range = vamp.ROBOT_RRT_RANGES[robot]
    # Static path bumps the iteration/sample caps to 1M; C++ struct default is
    # 100k, which causes harder problems (Fetch in particular) to time out.
    settings.max_iterations = 1_000_000
    settings.max_samples = 1_000_000

    total = 0
    valid = 0
    failed = 0
    rows = []

    tick = time.perf_counter()
    for name, pset in problems["problems"].items():
        if name not in problem:
            continue
        print(f"Evaluating {robot} (JIT) on {name}:")
        failures = []
        for i, data in tqdm(list(enumerate(pset))):
            total += 1
            if not data["valid"]:
                continue
            valid += 1

            env = vamp.problem_dict_to_vamp(data)
            start = list(data["start"])
            # MBM problems list multiple goals; JIT API takes one goal — use the
            # first, matching how RRTConnect handles goal sets in practice.
            goal = list(data["goals"][0])

            for trial in range(trials):
                seed = skip_rng_iterations + 1000 * i + trial
                sampler = jit_robot.halton()
                if seed:
                    sampler.skip(seed)
                result = jit_robot.rrtc(start, goal, env, settings, sampler)
                if not result.solved:
                    failures.append(i)
                    break
                rows.append({
                    "problem_set": name,
                    "problem_idx": i,
                    "trial": trial,
                    "waypoints": len(result.path),
                    "planning_time_us": result.nanoseconds / 1000.0,
                    "iterations": result.iterations,
                    "cost": result.path.cost(),
                })

        failed += len(failures)
        if print_failures and failures:
            print(f"  Failed on {failures}")
    tock = time.perf_counter()

    if not rows:
        print("No successful plans recorded.")
        return

    df = pd.DataFrame(rows)
    time_stats = df[["planning_time_us", "iterations", "cost"]].describe(
        percentiles=[0.25, 0.5, 0.75, 0.95]
    )
    time_stats.drop(index=["count"], inplace=True)

    print()
    print(tabulate(
        time_stats,
        headers=["Planning Time (μs)", "Iterations", "Cost (L2)"],
        tablefmt="github",
    ))
    print()
    print(f"Solved / Valid / Total: {valid - failed} / {valid} / {total}")
    print(f"Total planning time: {df['planning_time_us'].sum() / 1000:.3f} ms")
    print(f"Wall time including Python overhead: {(tock - tick) * 1000:.3f} ms")


if __name__ == "__main__":
    Fire(main)
