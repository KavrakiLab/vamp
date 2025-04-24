import numpy as np
import pandas as pd
import vamp
from fire import Fire
import matplotlib.pyplot as plt

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
        radius: float = 0.2,           # Radius of obstacles
        planner: str = "fcit",         # Planner to use.
        sampler: str = "halton",       # Sampler to use.
        plot: bool = True,             # Plot intermediate results of planning.
        save_path: str = 'plot.png',   # Where to save plot.
        iterations: int = 1000,        # Number of iterations. Note that FCIT* has very different iterations than RRT*.
        samples: int = 100000,         # Max. number of samples.
        **kwargs,                      # Use `batch_size` to control FCIT* behavior
    ):

    if planner not in ["fcit", "rrt_star"]:
        print(f"{planner} is not an ASAO planner!")
        return

    (vamp_module, planner_func, plan_settings,
     _) = vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs)

    spheres = [np.array(sphere) for sphere in problem]

    env = vamp.Environment()
    for sphere in spheres:
        env.add_sphere(vamp.Sphere(sphere, radius))

    if not (vamp.panda.validate(start, env) and vamp.panda.validate(goal, env)):
        return

    sampler = getattr(vamp_module, sampler)()
    plan_settings.optimize = True
    plan_settings.max_iterations = iterations
    plan_settings.max_samples = samples

    result = planner_func(start, goal, env, plan_settings, sampler)
    if not result.solved:
        return

    data = vamp.results_to_dict(result, include_intermediate_results = True)
    df = pd.DataFrame.from_dict(data["intermediate_results"])

    df["planning_time"] = df['planning_time'].astype('int64') / 1e9

    # Create the plot
    if plot:
        plt.figure(figsize = (10, 6))
        plt.plot(df["planning_time"], df["path_cost"], marker = 'o')
        plt.title('Seconds vs Initial Path Cost')
        plt.xlabel('Seconds')
        plt.ylabel('Initial Path Cost')
        plt.grid(True)
        plt.savefig(save_path, dpi = 300, bbox_inches = 'tight')


if __name__ == "__main__":
    Fire(main)
