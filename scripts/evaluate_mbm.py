import pickle
import time
from tabulate import tabulate
from tqdm import tqdm
from pathlib import Path
import pandas as pd
from typing import Union, List

from fire import Fire
import vamp


def main(
    robot: str = "panda",
    planner: str = "rrtc",
    dataset: str = "problems.pkl",
    problem: Union[str, List[str]] = [],
    trials: int = 1,
    **kwargs,
    ):

    if robot not in vamp.ROBOT_JOINTS:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    problems_dir = Path(__file__).parent.parent / 'resources' / robot / 'problems'
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

    vamp_module, planner_func, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    total_problems = 0
    valid_problems = 0
    failed_problems = 0

    tick = time.perf_counter()
    results = []
    for name, pset in problems['problems'].items():
        if name not in problem:
            continue

        failures = []
        invalids = []
        print(f"Evaluating {robot} on {name}: ")
        for i, data in tqdm(enumerate(pset)):
            total_problems += 1

            if not data['valid']:
                invalids.append(i)
                continue

            valid_problems += 1

            env = vamp.problem_dict_to_vamp(data)
            for _ in range(trials):
                result = planner_func(data['start'], data['goals'], env, plan_settings)
                if not result.solved:
                    failures.append(i)
                    break

                simple = vamp_module.simplify(result.path, env, simp_settings)
                results.append(vamp.results_to_dict(result, simple))

        failed_problems += len(failures)

        if invalids:
            print(f"  Invalid problems: {invalids}")

        if failures:
            print(f"  Failed on {failures}")

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to microseconds
    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds
    df["total_time"] = df["total_time"].dt.microseconds
    df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]

    # Get summary statistics
    time_stats = df[[
        "planning_time",
        "simplification_time",
        "total_time",
        "planning_iterations",
        "avg_time_per_iteration",
        ]].describe(percentiles = [0.25, 0.5, 0.75, 0.95])
    time_stats.drop(index = ["count"], inplace = True)

    cost_stats = df[[
        "initial_path_cost",
        "simplified_path_cost",
        ]].describe(percentiles = [0.25, 0.5, 0.75, 0.95])
    cost_stats.drop(index = ["count"], inplace = True)

    print()
    print(
        tabulate(
            time_stats,
            headers = [
                'Planning Time (μs)',
                'Simplification Time (μs)',
                'Total Time (μs)',
                'Planning Iters.',
                'Time per Iter. (μs)',
                ],
            tablefmt = 'github'
            )
        )

    print(
        tabulate(
            cost_stats, headers = [
                ' Initial Cost (L2)',
                '    Simplified Cost (L2)',
                ], tablefmt = 'github'
            )
        )

    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
