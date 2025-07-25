import pickle
from pathlib import Path
import numpy as np

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
    env = vamp.Environment()

    sim = vpb.PyBulletSimulator(str(robot_dir / f"{robot}_spherized.urdf"), vamp_module.joint_names(), True)

    start = sample_valid(vamp_module, sampler)

    while True:
        goal = sample_valid(vamp_module, sampler)
        result = planner_func(start, goal, env, plan_settings, sampler)
        solved = result.solved
        print(solved)

        if solved:
            simplify = vamp_module.simplify(result.path, env, simp_settings, sampler)
            stats = vamp.results_to_dict(result, simplify)
            print(
                f"""
Planning Time: {stats['planning_time'].microseconds:8d}μs
Simplify Time: {stats['simplification_time'].microseconds:8d}μs
   Total Time: {stats['total_time'].microseconds:8d}μs

Planning Iters: {stats['planning_iterations']}
n Graph States: {stats['planning_graph_size']}

Path Length:
   Initial: {stats['initial_path_cost']:5.3f}
Simplified: {stats['simplified_path_cost']:5.3f}"""
                )

            plan = simplify.path
            plan.interpolate_to_resolution(vamp_module.resolution())

            sim.play_once(plan)

            start = goal

        else:
            print("Failed to solve, trying again!")


if __name__ == "__main__":

    Fire(main)
