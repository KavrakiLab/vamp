import numpy as np
from viser import transforms as tf
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path

import vamp
from fire import Fire


# Starting configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785, 2.35, 1., 0., -0.8, 0, 2.5, 0.785]

# Goal configuration
b = [-2.35, 1., 0., -0.8, 0, 2.5, 0.785, 0., -0.785, 0., -2.356, 0., 1.571, 0.785]


# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.15],
    [0.35, 0.45, 0.15],
    [0, 0.65, 0.15],
    [-0.55, 0, 0.15],
    [-0.35, -0.55, 0.15],
    [0, -0.65, 0.15],
    [0.35, -0.55, 0.15],
    [0.35, 0.6, 0.8],
    [0, 0.6, 0.8],
    [-0.35, 0.5, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.5, 0.8],
    [0, -0.65, 0.8],
    [0.35, -0.65, 0.8],
    ]



def main(
    obstacle_radius: float = 0.2,
    attachment_radius: float = 0.07,
    attachment_offset: float = 0.04,
    planner: str = "rrtc",
    **kwargs,
):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("bimanualpanda", planner, **kwargs)

    # Create an attachment offset on the Z-axis from the end-effector frame
    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, attachment_offset])
    attachment = vamp.Attachment(tf)

    attachment2 = vamp.Attachment(tf)

    # Add a single sphere to the attachment - spheres are added in the attachment's local frame
    attachment.add_spheres([vamp.Sphere([0, 0, 0], attachment_radius)])
    attachment2.add_spheres([vamp.Sphere([0, 0, 0], attachment_radius)])


    robot_dir = Path(__file__).parents[1] / 'resources' / 'panda'

    server, robot = setup_viser_with_robot(robot_dir, "bipanda_spherized.urdf")
    robot.update_cfg(a)

    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))

    _problem_sphere_handles = add_spheres(
        server, np.array(problem), np.array([obstacle_radius] * len(problem))
    )

    # Add the attchment to the VAMP environment
    e.attach(attachment, 0)
    e.attach(attachment2, 1)

    # Add attachment sphere to visualization
    attachment_sph = add_spheres(server, np.zeros((2, 3)), np.array([attachment_radius] * 2), colors=[[0, 255, 0], [0, 0, 255]])

    # Update attachment sphere positions corresponding to the waypoints.
    # this could also be made into a callable that can be called during trajectory viz
    def get_attachment_pos(configuration):
        for idx, attach in enumerate([attachment, attachment2]):
            attach.set_ee_pose(vamp_module.eefk(configuration)[idx])
        return np.array([attachment.posed_spheres[0].position, attachment2.posed_spheres[0].position])


    # Plan and display
    sampler = vamp_module.halton()
    result = planner_func(a, b, e, plan_settings, sampler)
    simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())

    attachment_positions = [get_attachment_pos(pos) for pos in simple.path.numpy()]

    add_trajectory(
        server, simple.path.numpy(), robot, attachment_sph, attachment_positions
    )

    # display
    while True:
        continue


if __name__ == "__main__":
    Fire(main)
