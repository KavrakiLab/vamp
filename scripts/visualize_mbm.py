import pickle
from pathlib import Path
import numpy as np

from fire import Fire

import vamp
from vamp import pybullet_interface as vpb
from vamp import pointcloud as vpc


def main(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: str = "",                     # Problem name
    index: int = 1,                        # Problem index
    sampler_name: str = "halton",          # Sampler to use.
    skip_rng_iterations: int = 0,          # Skip a number of RNG iterations
    display_object_names: bool = False,    # Display object names over geometry
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    samples_per_object: int = 10000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = True,              # Cull pointcloud around robot by maximum distance
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / robot
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    (vamp_module, planner_func, plan_settings, simp_settings) = vamp.configure_robot_and_planner_with_kwargs(
        robot,
        planner,
        **kwargs,
        )

    if not problem:
        problem = list(data['problems'].keys())[0]

    if problem not in data['problems']:
        raise RuntimeError(
            f"""No problem with name {problem}!
Existing problems: {list(data['problems'].keys())}"""
            )

    problems = data['problems'][problem]
    try:
        problem_data = next(problem for problem in problems if problem['index'] == index)
    except StopIteration:
        raise RuntimeError(f"No problem in {problem} with index {index}!")

    if pointcloud:
        r_min, r_max = vamp_module.min_max_radii()
        (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
            robot,
            r_min,
            r_max,
            problem_data,
            samples_per_object,
            filter_radius,
            filter_cull,
            )

        print(
            f"""
Original Pointcloud size: {len(original_pc)}
Filtered Pointcloud size: {len(filtered_pc)}

        Filtering Time: {filter_time * 1e-6:5.3f}ms
CAPT Construction Time: {build_time * 1e-6:5.3f}ms
            """
            )

    else:
        env = vamp.problem_dict_to_vamp(problem_data)

    start = problem_data['start']
    goals = problem_data['goals']
    valid = problem_data['valid']

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    if valid:
        result = planner_func(start, goals, env, plan_settings, sampler)
        solved = result.solved
    else:
        print("Problem is invalid!")
        solved = False

    if valid and solved:
        print("Solved problem!")
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

    if valid and not solved:
        print("Failed to solve problem! Displaying start and goals.")
        print(start)
        for goal in goals:
            print(goal)

        if problem_data['valid']:
            print(
                f"""
Planning Time: {int(result.nanoseconds / 1000):8d}μs
Planning Iters: {result.iterations}
n Graph States: {result.size}
"""
                )

    if not solved:
        plan = vamp_module.Path()
        plan.append(start)
        for goal in goals:
            plan.append(goal)

    sim = vpb.PyBulletSimulator(str(robot_dir / f"{robot}_spherized.urdf"), vamp_module.joint_names(), True)
    sim.add_environment_from_problem_dict(problem_data, display_object_names)

    if pointcloud:
        sim.draw_pointcloud(filtered_pc)

    if not valid:
        for state in [start, *goals]:
            if not vamp_module.validate(state, env):
                print(f"Displaying colliding spheres for first invalid state: {state}")
                debug = vamp_module.debug(state, env)
                invalid = set([x[0] for x in filter(lambda x: x[1], enumerate(debug[0]))])

                for (a, b) in debug[1]:
                    invalid.add(a)
                    invalid.add(b)

                spheres = vamp_module.fk(state)
                for i in range(len(spheres)):
                    sphere = spheres[i]
                    if i in invalid:
                        sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 0., 0., 1.])
                    else:
                        sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 1., 1., 1.])

                break

    sim.animate(plan)


if __name__ == "__main__":

    Fire(main)
