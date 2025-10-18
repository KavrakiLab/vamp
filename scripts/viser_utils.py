from viser.extras import ViserUrdf
import viser
import yourdfpy
from typing import Sequence, Union


def setup_viser_with_robot(robot_dir, robot_urdf_name):
    server = viser.ViserServer()
    # change the robot here
    urdf = yourdfpy.URDF.load(str(robot_dir / robot_urdf_name))
    robot = ViserUrdf(
        server,
        urdf,
        load_meshes=True,
        load_collision_meshes=False,
        root_node_name="/robot",
    )

    return server, robot


def add_spheres(
    server: viser.ViserServer,
    sphere_positions: Sequence,
    sphere_radii: Sequence,
    colors: Union[Sequence[int], Sequence[Sequence[int]]] = [],
    prefix: str = "my_sphere",
):
    """
    Add spheres to the env/
    Sphere positions are (N,3) and sphere radii are (N)
    """
    sphere_handles = [None] * len(sphere_positions)
    if len(colors) == 0:
        colors = [[255, 0, 0]] * len(sphere_positions)
    elif len(colors) == 1:
        colors = colors * len(sphere_positions)
    else:
        assert len(colors) == len(sphere_positions)
    for i, (sphere_pos, sphere_rad) in enumerate(zip(sphere_positions, sphere_radii)):
        sphere_handles[i] = server.scene.add_icosphere(
            name=f"{prefix}_{i}",
            radius=sphere_rad,
            position=tuple(sphere_pos[:3]),
            color=tuple(colors[i]),
        )
    return sphere_handles


def add_trajectory(server, waypoints, robot, attachment_handles, attachment_positions):
    """
    Adds a slider to step through waypoints of a trajectory also allows for auto step through
    using play/pause button

    Args:
        server (ViserServer): ViserServer instance
        waypoints (numpy.array): A 2D numpy array (shape: (N,7)) with N waypoints of joint poses
        robot (ViserUrdf): ViserUrdf instance of the robot

        attachment_handles (numpy.array) - this is a P element list of attachment handles, spheres here
        attachment_positions (numpy.array) - this is a (N, P, 3) array of the position of each attachment handle at each waypoint pos.

    Returns:
        return_type: None.
    """
    if len(waypoints) < 1:
        return
    assert len(attachment_handles) == len(attachment_positions[0])
    traj_slider = server.gui.add_slider(
        "Current Waypoint", min=0, max=len(waypoints) - 1, step=1, initial_value=0
    )

    @traj_slider.on_update
    def update_robot_pose(event):
        waypoint_idx = int(event.target.value)
        joint_config = waypoints[waypoint_idx]
        robot.update_cfg(joint_config)

        for attach_idx, attachment_handle in enumerate(attachment_handles):
            attachment_handle.position = attachment_positions[waypoint_idx][attach_idx]
