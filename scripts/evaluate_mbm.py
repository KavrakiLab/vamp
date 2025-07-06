import pickle
import time
from tabulate import tabulate
from tqdm import tqdm
from pathlib import Path
import pandas as pd
from typing import Union, List

from fire import Fire
import vamp
from vamp import pointcloud as vpc


def main(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    sampler: str = "halton",               # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    print_failures: bool = False,          # Print out failures and invalid problems
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    samples_per_object: int = 10000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = True,              # Cull pointcloud around robot by maximum distance
    **kwargs,
    ):

    if robot not in vamp.robots:
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

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    sampler = getattr(vamp_module, sampler)()

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

            if pointcloud:
                r_min, r_max = vamp_module.min_max_radii()
                (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
                    robot,
                    r_min,
                    r_max,
                    data,
                    samples_per_object,
                    filter_radius,
                    filter_cull,
                    )

                pointcloud_results = {
                    'original_pointcloud_size': len(original_pc),
                    'filtered_pointcloud_size': len(filtered_pc),
                    'filter_time': pd.Timedelta(nanoseconds = filter_time),
                    'capt_build_time': pd.Timedelta(nanoseconds = build_time)
                    }
            else:
                env = vamp.problem_dict_to_vamp(data)

            sampler.reset()
            sampler.skip(skip_rng_iterations)
            for _ in range(trials):
                result = planner_func(data['start'], data['goals'], env, plan_settings, sampler)
                if not result.solved:
                    failures.append(i)
                    break

                simple = vamp_module.simplify(result.path, env, simp_settings, sampler)

                trial_result = vamp.results_to_dict(result, simple)
                if pointcloud:
                    trial_result.update(pointcloud_results)

                results.append(trial_result)

        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if failures:
                print(f"  Failed on {failures}")

    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to microseconds
    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds
    df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]

    # Pointcloud data
    if pointcloud:
        df["total_build_and_plan_time"] = df["total_time"] + df["filter_time"] + df["capt_build_time"]
        df["filter_time"] = df["filter_time"].dt.microseconds / 1e3
        df["capt_build_time"] = df["capt_build_time"].dt.microseconds / 1e3
        df["total_build_and_plan_time"] = df["total_build_and_plan_time"].dt.microseconds / 1e3

    df["total_time"] = df["total_time"].dt.microseconds

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

    if pointcloud:
        pointcloud_stats = df[[
            "filter_time",
            "capt_build_time",
            "total_build_and_plan_time",
            ]].describe(percentiles = [0.25, 0.5, 0.75, 0.95])
        pointcloud_stats.drop(index = ["count"], inplace = True)

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

    if pointcloud:
        print(
            tabulate(
                pointcloud_stats,
                headers = [
                    '  Filter Time (ms)',
                    '    CAPT Build Time (ms)',
                    'Total Time (ms)',
                    ],
                tablefmt = 'github'
                )
            )

    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
