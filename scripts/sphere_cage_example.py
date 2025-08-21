import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from fire import Fire
import time

# Starting configuration
a = [0., 0., 0., 0.4, 0., 0., 0., 0., 0., 0., 0., 0., 0.]

# Goal configuration
#b = [1., 1., 2.0, 0.95, 0.1, 0.1, 0.1, 0.1, 0., 0., 0., 0., 0.]
b = [1., 0., 0., 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
# Problem specification: a list of sphere centers
problem = [
    #[0.55, 0, 0.25],
    #[0.35, 0.35, 0.25],
    #[0, 0.55, 0.25],
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
    #[0.35, -0.35, 0.8],
    ]

problem = [[0.5, 0, 0.2], [-0.5, 0, 0.2]]


def main(
    variation: float = 0.01,
    benchmark: bool = False,
    n_trials: int = 100,
    radius: float = 0.2,
    visualize: bool = True,
    planner: str = "rrtc",
    sampler_name: str = "halton",  # Sampler to use.
    skip_rng_iterations: int = 0,  # Skip a number of RNG iterations
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("stretch", planner, **kwargs)

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    if benchmark:
        random.seed(0)
        np.random.seed(0)

        results = []
        spheres = [np.array(sphere) for sphere in problem]
        for _ in range(n_trials):
            random.shuffle(spheres)
            spheres_copy = copy.deepcopy(spheres)

            e = vamp.Environment()
            for sphere in spheres_copy:
                sphere += np.random.uniform(low = -variation, high = variation, size = (3, ))
                e.add_sphere(vamp.Sphere(sphere, radius))

            if vamp.panda.validate(a, e) and vamp.panda.validate(b, e):
                result = planner_func(a, b, e, plan_settings, sampler)
                simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
                results.append(vamp.results_to_dict(result, simple))

        df = pd.DataFrame.from_dict(results)

        # Convert to microseconds
        df["planning_time"] = df["planning_time"].dt.microseconds
        df["simplification_time"] = df["simplification_time"].dt.microseconds

        # Get summary statistics
        stats = df[[
            "planning_time",
            "simplification_time",
            "initial_path_cost",
            "simplified_path_cost",
            "planning_iterations"
            ]].describe()

        print(stats)

    if visualize:
        from vamp import pybullet_interface as vpb

        robot_dir = Path(__file__).parent.parent / 'resources' / 'stretch'
        sim = vpb.PyBulletSimulator(
            str(robot_dir / f"stretch_spherized.urdf"), vamp.ROBOT_JOINTS['stretch'], True
            )
        
        sim.set_joint_positions(a)
        print(sim.in_collision())

        e = vamp.Environment()
        print(vamp.stretch.validate(a, e))
        for sphere in problem:
            e.add_sphere(vamp.Sphere(sphere, radius))
            sim.add_sphere(radius, sphere)

        print(vamp.stretch.validate(a, e))
        result = planner_func(a, b, e, plan_settings, sampler)
        print("OBTAINED RESULT:")
        print(result.size)
        simple = vamp_module.simplify(result.path, e, simp_settings, sampler)

        simple.path.interpolate(vamp.panda.resolution())
        print(len(simple.path))
        for c in simple.path:
            if isinstance(c, list):
                c_list = c
            elif isinstance(c, np.ndarray):
                c_list = c.tolist()
            else:
                c_list = c.to_list()
            print(c_list)

        # Check if you can turn, go, turn
        # Check if you can follow a reed-shepp path between points

        sim.animate(simple.path)


if __name__ == "__main__":
    Fire(main)
