import pickle
from pathlib import Path

from fire import Fire

import vamp
from vamp import pybullet_interface as vpb


def main(
    robot: str = "panda",
    planner: str = "rrtc",
    dataset: str = "problems.pkl",
    problem: str = "",
    index: int = 1,
    display_object_names: bool = False,
    **kwargs,
    ):

    if robot not in vamp.ROBOT_JOINTS:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / robot
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

    vamp_module, planner_func, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(robot, planner, **kwargs)

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

    env = vamp.problem_dict_to_vamp(problem_data)

    start = problem_data['start']
    goals = problem_data['goals']
    valid = problem_data['valid']

    if valid:
        result = planner_func(start, goals, env, plan_settings)
        solved = result.solved
    else:
        print("Problem is invalid!")
        solved = False

    if valid and solved:
        print("Solved problem!")
        simplify = vamp_module.simplify(result.path, env, simp_settings)

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
        plan.interpolate(vamp_module.resolution())

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
        plan.append(vamp_module.Configuration(start))
        for goal in goals:
            plan.append(vamp_module.Configuration(goal))

    sim = vpb.PyBulletSimulator(str(robot_dir / f"{robot}_spherized.urdf"), vamp.ROBOT_JOINTS[robot], True)
    sim.add_environment_from_problem_dict(problem_data, display_object_names)

    if not valid:
        for state in [start, *goals]:
            if not vamp_module.validate(state, env):
                print(f"Displaying colliding spheres for first invalid state: {state}")

                validity = vamp_module.sphere_validity(state, env)
                invalid = [x[0] for x in filter(lambda x: x[1], enumerate(validity))]

                spheres = vamp_module.fk(state)
                for inv in invalid:
                    sphere = spheres[inv]
                    sim.add_sphere(sphere.r, [sphere.x, sphere.y, sphere.z], color = [1., 0., 0., 1.])

                break

    sim.animate(plan)


if __name__ == "__main__":
    Fire(main)
