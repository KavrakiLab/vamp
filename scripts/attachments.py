from pathlib import Path
import numpy as np

import vamp
from vamp import pybullet_interface as vpb

from fire import Fire

# Starting configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

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
    attachment_offset: float = 0.14,
    planner: str = "rrtc",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs)

    # Create an attachment offset on the Z-axis from the end-effector frame
    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, attachment_offset])
    attachment = vamp.Attachment(tf)

    # Add a single sphere to the attachment - spheres are added in the attachment's local frame
    attachment.add_spheres([vamp.Sphere([0, 0, 0], attachment_radius)])

    robot_dir = Path(__file__).parents[1] / 'resources' / 'panda'
    sim = vpb.PyBulletSimulator(str(robot_dir / "panda_spherized.urdf"), vamp_module.joint_names(), True)

    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))
        sim.add_sphere(obstacle_radius, sphere)

    # Add the attchment to the VAMP environment
    e.attach(attachment)

    # Add attachment sphere to visualization
    attachment_sphere = sim.add_sphere(attachment_radius, [0, 0, 0])

    # Callback to update sphere's location in PyBullet visualization
    def callback(configuration):
        attachment.set_ee_pose(vamp_module.eefk(configuration))
        sphere = attachment.posed_spheres[0]

        sim.update_object_position(attachment_sphere, sphere.position)

    # Plan and display
    sampler = vamp_module.halton()
    result = planner_func(a, b, e, plan_settings, sampler)
    simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())

    sim.animate(simple.path, callback)


if __name__ == "__main__":
    Fire(main)
