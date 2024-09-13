import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from fire import Fire
import matplotlib.pyplot as plt

pd.set_option('display.max_columns', None)

# Starting configuration
start = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
goal = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]


def main(
    radius: float = 0.2,
    planner: str = "rrt_star",
    force_max_iters: bool = True,
    range: float = 3.0,
    plot: bool = False,
    save_path: str = 'plot.png',
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs)

    random.seed(0)
    np.random.seed(0)

    all_results = []
    spheres = [np.array(sphere) for sphere in problem]
    max_iters = np.arange(1, 100000, 1000)

    env = vamp.Environment()
    for sphere in spheres:
        env.add_sphere(vamp.Sphere(sphere, radius))

    plan_settings.force_max_iters = force_max_iters
    plan_settings.range = range
    for iters in max_iters:
        plan_settings.max_iterations = iters
        plan_settings.max_samples = iters
        if vamp.panda.validate(start, env) and vamp.panda.validate(goal, env):
            result = planner_func(start, goal, env, plan_settings)
            # print(result.solved)
            if not result.solved: continue
            simple = vamp_module.simplify(result.path, env, simp_settings)
            results = vamp.results_to_dict(result, simple)
            results["iterations"] = iters
            all_results.append(results)

    df = pd.DataFrame.from_dict(all_results)

    # Convert to microseconds
    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds

    print(df)

    # Create the plot
    if plot:
        plt.figure(figsize = (10, 6))
        plt.plot(df['planning_iterations'], df['initial_path_cost'], marker = 'o')
        plt.title('Iterations vs Initial Path Cost')
        plt.xlabel('Iterations')
        plt.ylabel('Initial Path Cost')
        plt.grid(True)
        plt.savefig(save_path, dpi = 300, bbox_inches = 'tight')


if __name__ == "__main__":
    Fire(main)
