import pickle
import time
from pathlib import Path
from typing import List, Union

from fire import Fire
from tabulate import tabulate
from tqdm import tqdm
import pandas as pd

import cricket
import vamp

CRICKET_ROBOTS = {
    "panda": dict(end_effector="panda_grasptarget", name="Panda"),
    "ur5": dict(end_effector="robotiq_85_base_link", name="UR5"),
    "fetch": dict(end_effector="gripper_link", name="Fetch"),
    "baxter": dict(end_effector="right_gripper", name="Baxter"),
}


def main(
    robot: str = "panda",
    dataset: str = "problems.pkl",
    problem: Union[str, List[str]] = (),
    trials: int = 1,
    print_failures: bool = False,
    rake: int = 8,
    resolution: int = 32,
):
    if robot not in CRICKET_ROBOTS:
        raise RuntimeError( f"No cricket recipe for robot '{robot}'.")

    cfg = CRICKET_ROBOTS[robot]
    base = cricket.resources_dir() / robot
    urdf, srdf = base / f"{robot}_spherized.urdf", base / f"{robot}.srdf"
    if not urdf.exists():
        raise SystemExit(f"Missing URDF at {urdf}")

    print(f"[jit] loading {cfg['name']} from {urdf} ...")
    t0 = time.perf_counter()
    jit_robot = vamp.jit.load_robot(
        urdf=str(urdf),
        srdf=str(srdf),
        end_effector=cfg["end_effector"],
        planners=["rrtc"],
        rake=rake,
        resolution=resolution,
        name=cfg["name"],
    )
    t1 = time.perf_counter()
    print(
        f"[jit] load_robot took {(t1 - t0):.2f}s"
    )

    problems_dir = Path(
        __file__).parent.parent / 'resources' / robot / 'problems'
    with open(problems_dir.parent / dataset, 'rb') as f:
        problems = pickle.load(f)

    problem_names = list(problems['problems'].keys())
    if isinstance(problem, str):
        problem = [problem]

    if not problem:
        problem = problem_names
    else:
        for problem_name in problem:
            if problem_name not in problem_names:
                raise RuntimeError(
                    f"Problem `{problem_name}` not available! Available problems: {problem_names}"
                )

    settings = vamp.RRTCSettings()
    if robot in vamp.ROBOT_RRT_RANGES:
        settings.range = vamp.ROBOT_RRT_RANGES[robot]
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
        print(f"Evaluating {robot} on {name}:")
        failures = []
        for i, data in tqdm(list(enumerate(pset))):
            total += 1
            if not data["valid"]:
                continue
            valid += 1

            env = vamp.problem_dict_to_vamp(data)
            start = list(data["start"])
            goals = list(data["goals"])

            for trial in range(trials):
                sampler = jit_robot.halton()
                result = jit_robot.rrtc(start, goals, env, settings, sampler)
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
    time_stats = df[["planning_time_us", "iterations",
                     "cost"]].describe(percentiles=[0.25, 0.5, 0.75, 0.95])
    time_stats.drop(index=["count"], inplace=True)

    print()
    print(
        tabulate(
            time_stats,
            headers=["Planning Time (μs)", "Iterations", "Cost (L2)"],
            tablefmt="github",
        ))
    print()
    print(f"Solved / Valid / Total: {valid - failed} / {valid} / {total}")
    print(f"Total planning time: {df['planning_time_us'].sum() / 1000:.3f} ms")
    print(
        f"Wall time including Python overhead: {(tock - tick) * 1000:.3f} ms")


if __name__ == "__main__":
    Fire(main)
