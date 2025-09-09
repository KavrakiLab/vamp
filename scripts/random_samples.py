import pickle
from pathlib import Path
import numpy as np
import time

from fire import Fire

import vamp
from vamp import pybullet_interface as vpb


def sample_valid(vamp_module, rng):
    while True:
        config = rng.next()
        if vamp_module.validate(config):
            return config


def main(
    robot: str = "panda",          # Robot to plan for
    planner: str = "rrtc",         # Planner name to use
    sampler_name: str = "halton",  # Sampler to use.
    skip_rng_iterations: int = 0,  # Skip a number of RNG iterations
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / robot

    print(str(robot_dir / f"{robot}_spherized.urdf"))

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    sim = vpb.PyBulletSimulator(str(robot_dir / f"{robot}_spherized.urdf"), vamp_module.joint_names(), True)

    while True:
        config = sampler.next()
        print(vamp_module.validate(config))
        sim.play_once([config])
        input()


if __name__ == "__main__":

    Fire(main)
