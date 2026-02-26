import numpy as np
from viser_utils import setup_viser_with_robot, add_new_robot_to_server, add_spheres
from pathlib import Path

import vamp
from fire import Fire

# Starting configuration
a = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Goal configuration
b = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]

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
    obstacle_radius: float = 0.2,
    attachment_radius: float = 0.07,
    attachment_offset: float = 0.02,
    planner: str = "rrtc",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = (vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot_original = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
    robot_projected = add_new_robot_to_server(
        server,
        robot_dir,
        "panda_spherized.urdf",
        root_node_name = "/robot_projected",
        mesh_color_override = (0, 255, 0, 0.5)
        )

    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))

    _problem_sphere_handles = add_spheres(
        server, np.array(problem), np.array([obstacle_radius] * len(problem))
        )

    sampler = vamp_module.halton()
    # Add button to sample and project
    sample_button = server.gui.add_button("Sample and Project")

    @sample_button.on_click
    def _(event):
        q = sampler.next()
        q_projected = vamp.panda.project_to_valid(q, e)
        robot_original.update_cfg(q)
        robot_projected.update_cfg(q_projected)

    while True:
        continue


if __name__ == "__main__":
    Fire(main)
