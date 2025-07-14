import pickle
from pathlib import Path

from fire import Fire
import vamp
from vamp import pybullet_interface as vpb

try:
    import ompl.base as ob
    import ompl.geometric as og
except:
    raise ImportError("Failed to import OMPL Python bindings.")

try:
    import pandas as pd
except ImportError:
    raise RuntimeError("pandas is not installed!")


def solve(
        start,
        goals,
        simulator: vpb.PyBulletSimulator,
        planning_time: float = 30.,
        planner: str = "RRTConnect",
        **kwargs
    ):
    space = ob.RealVectorStateSpace()
    for l, h in zip(simulator.lows, simulator.highs):
        space.addDimension(l, h)
    space.setup()

    n_dims = space.getDimension()

    def set_state(state, state_list):
        for i in range(n_dims):
            state[i] = state_list[i]

    def get_state(state):
        return [state[i] for i in range(n_dims)]

    def valid(state):
        simulator.set_joint_positions(get_state(state))
        return not simulator.in_collision()

    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(valid))

    si.setStateValidityCheckingResolution(0.001)

    start_state = ob.State(space)
    set_state(start_state, start)
    ss.setStartState(start_state)

    goal_states = ob.GoalStates(si)
    for goal in goals:
        goal_state = ob.State(space)
        set_state(goal_state, goal)
        goal_states.addState(goal_state)

    ss.setGoal(goal_states)
    planner = eval(f"og.{planner}(si)")
    ss.setPlanner(planner)

    for k, v in kwargs.items():
        eval(f"planner.{k}({v})")

    ss.setup()
    result = ss.solve(planning_time)

    if result and ss.haveExactSolutionPath():
        ss.simplifySolution()
        path = ss.getSolutionPath()
        path.interpolate(100)

        states = []
        for state in path.getStates():
            states.append(get_state(state))

        return states

    return None


def main(
    robot: str = "panda",
    dataset: str = "problems.pkl",
    problem: str = "",
    index: int = 1,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    robot_dir = Path(__file__).parent.parent / 'resources' / robot
    with open(robot_dir / dataset, 'rb') as f:
        data = pickle.load(f)

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
    except:
        raise RuntimeError(f"No problem in {problem} with index {index}!")

    start = problem_data['start']
    goals = problem_data['goals']

    sim = vpb.PyBulletSimulator(
        str(robot_dir / f"{robot}_spherized.urdf"), getattr(vamp, robot).joint_names(), True
        )
    sim.add_environment_from_problem_dict(problem_data, False)

    path = solve(start, goals, sim, planner = "RRTConnect", setRange = vamp.ROBOT_RRT_RANGES[robot])
    if not path:
        print("Failed to find path, visualizing start/goals!")
        path = [start] + goals

    sim.animate(path)


if __name__ == "__main__":
    Fire(main)
