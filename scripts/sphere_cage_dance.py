from pathlib import Path

from fire import Fire

import vamp
from vamp import pybullet_interface as vpb


def sample_valid(vamp_module, rng):
    while True:
        config = rng.next()
        print(config)
        if vamp_module.validate(config):
            return config


# Problem specification: a list of sphere centers
problem = [
    [0.65, 0, 0.15],
    [0.4, 0.4, 0.15],
    [0, 0.65, 0.15],
    [-0.65, 0, 0.15],
    [-0.4, -0.4, 0.15],
    [0, -0.65, 0.15],
    [0.4, -0.4, 0.15],
    [0.4, 0.4, 0.7],
    [0, 0.65, 0.7],
    [-0.4, 0.4, 0.7],
    [-0.65, 0, 0.7],
    [-0.4, -0.4, 0.7],
    [0, -0.65, 0.7],
    [0.4, -0.4, 0.7],
    ]


def main(
    robot: str = "panda",
    planner: str = "rrtc",
    sampler_name: str = "halton",  # Sampler to use.
    skip_rng_iterations: int = 0,  # Skip a number of RNG iterations
    radius: float = 0.1,
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    robot_dir = Path(__file__).parent.parent / 'resources' / robot
    sim = vpb.PyBulletSimulator(str(robot_dir / f"{robot}_spherized.urdf"), vamp_module.joint_names(), True)

    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, radius))
        sim.add_sphere(radius, sphere)

    while True:
        a = sample_valid(vamp_module, sampler).tolist()
        b = sample_valid(vamp_module, sampler).tolist()
        result = planner_func(a, b, e, plan_settings, sampler)

        solved = result.solved
        print(solved)

        if solved:
            simplify = vamp_module.simplify(result.path, e, simp_settings, sampler)
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
        else:
            print("Failed to solve!")


if __name__ == "__main__":
    Fire(main)
