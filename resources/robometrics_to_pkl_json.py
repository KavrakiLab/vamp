import pickle
from pathlib import Path
from fire import Fire
import json
from tqdm import tqdm

import vamp
from vamp.transformations import euler_from_quaternion
from robometrics.datasets import demo, motion_benchmaker, mpinets

datasets = {
    'demo': demo,
    'mbm': motion_benchmaker,
    'mpinets': mpinets,
    }


def test_problem(problem):
    start = problem['start']
    goals = problem['goals']
    env = vamp.problem_dict_to_vamp(problem)

    start_valid = vamp.panda.validate(start, env)

    any_goal_valid = False
    for goal in goals:
        if vamp.panda.validate(goal, env):
            any_goal_valid = True
            break

    return start_valid and any_goal_valid


def robometric_to_vamp(problem):
    start = problem['start'].tolist()
    goals = [goal.tolist() for goal in problem['goal_ik']]

    data = {'start': start, 'goals': goals, 'sphere': [], 'cylinder': [], 'box': []}

    obstacles = problem['obstacles']

    for name, cylinder in obstacles['cylinder'].items():
        center = cylinder.center.tolist()
        radius = cylinder.radius
        height = cylinder.height
        quat_wxyz = cylinder.pose.so3.q.tolist()
        quat_xyzw = quat_wxyz[1:] + [quat_wxyz[0]]
        rotation = euler_from_quaternion(quat_xyzw, 'sxyz')

        data['cylinder'].append(
            {
                'name': name,
                'length': height,
                'radius': radius,
                'position': center,
                'orientation_euler_xyz': rotation,
                'orientation_quat_xyzw': quat_xyzw,
                }
            )

    for name, cuboid in obstacles['cuboid'].items():
        center = cuboid.center.tolist()
        half_extents = cuboid.half_extents.tolist()
        quat_wxyz = cuboid.quaternion.tolist()
        quat_xyzw = quat_wxyz[1:] + [quat_wxyz[0]]
        rotation = euler_from_quaternion(quat_xyzw, 'sxyz')

        data['box'].append(
            {
                'name': name,
                'half_extents': half_extents,
                'position': center,
                'orientation_euler_xyz': rotation,
                'orientation_quat_xyzw': quat_xyzw,
                }
            )

    return data


def main(dataset: str = "mpinets"):
    problem_dir = Path(__file__).parent / 'panda'

    print(f"Loading dataset {dataset}...")
    robometric_data = datasets[dataset]()
    print("Loaded!")

    joints = vamp.panda.joint_names

    data = {'robot': 'panda', 'joints': joints, 'problems': {}}
    for problem_name, problems in robometric_data.items():
        print(f'Processing {problem_name}')
        if problem_name not in data['problems']:
            data['problems'][problem_name] = []

        for i, problem in tqdm(enumerate(problems)):
            problem_data = {'index': i, 'problem': problem_name}
            problem_data.update(robometric_to_vamp(problem))
            problem_data['valid'] = test_problem(problem_data)
            data['problems'][problem_name].append(problem_data)

    with open(problem_dir / f'robometric_{dataset}_problems.pkl', 'wb') as f:
        f.write(pickle.dumps(data))

    with open(problem_dir / f'robometric_{dataset}_problems.json', 'w') as f:
        f.write(json.dumps(data))


if __name__ == "__main__":
    Fire(main)
