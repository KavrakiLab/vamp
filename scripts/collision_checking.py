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

# A middle point with free motion from a
m = [0.020, 0.022, 0.0005, 0.4, 0.0, 0.0008, 0.0008, 0.0007, 0.0085, -0.0132, -0.00584, -0.0075, -0.01]


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
    n_trials: int = 100,
    radius: float = 0.2,
    planner: str = "rrtc",
    sampler_name: str = "halton",  # Sampler to use.
    skip_rng_iterations: int = 0,  # Skip a number of RNG iterations
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("stretch", planner, **kwargs)

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    

    

    from vamp import pybullet_interface as vpb

    robot_dir = Path(__file__).parent.parent / 'resources' / 'stretch'
    sim = vpb.PyBulletSimulator(
        str(robot_dir / f"stretch_spherized.urdf"), vamp.ROBOT_JOINTS['stretch'], True
        )
    
    sim.set_joint_positions(a)
    print(sim.in_collision())


    # TESTING 
    # COLLISION CHECKING OF ONE CONFIGURATIONS
    e = vamp.Environment()
    print("BEFORE ADDING OBSTACLES: ", vamp.stretch.validate(a, e))
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, radius))
        sim.add_sphere(radius, sphere)
    print("AFTER ADDING OBSTACLES: ", vamp.stretch.validate(a, e))

    print("MOTION FROM A TO M: ", vamp.stretch.validate(a, m, e))
    print("MOTION FROM A TO B: ", vamp.stretch.validate(a, b, e))

    # PLANNING...
    result = planner_func(a, b, e, plan_settings, sampler)
    print("OBTAINED RESULT:")
    print(result.size)
    simple = vamp_module.simplify(result.path, e, simp_settings, sampler)

    simple.path.interpolate(vamp.panda.resolution())
    print(len(simple.path))
    # for c in simple.path:
    #     if isinstance(c, list):
    #         c_list = c
    #     elif isinstance(c, np.ndarray):
    #         c_list = c.tolist()
    #     else:
    #         c_list = c.to_list()
    #     print(c_list)

    # Check if you can turn, go, turn
    # Check if you can follow a reed-shepp path between points

    sim.animate(simple.path)


if __name__ == "__main__":
    Fire(main)
