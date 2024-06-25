from dataclasses import dataclass
from pathlib import Path

import sys
import time
import numpy as np
import xmltodict
from typing import Any, Dict, List, Optional
from zlib import crc32
from colorsys import hsv_to_rgb

import pybullet as pb
from pybullet_utils.bullet_client import BulletClient
from .redirect_stream import RedirectStream
from .disable_rendering import DisableRendering
from .typing import *


def string_to_01(b: str) -> float:
    return float(crc32(b.encode('utf-8')) & 0xffffffff) / 2**32


def name_to_color(name: str) -> List[float]:
    return [*hsv_to_rgb(string_to_01(name), 0.5, 0.9), 1.]


def handle_color(name: Optional[str], color: Optional[Union[List[float], str]]) -> List[float]:
    if not color and name:
        return name_to_color(name)
    elif isinstance(color, str):
        return name_to_color(color)
    elif not color:
        return [0.5, 0.5, 0.5, 1.]
    else:
        return color


@dataclass
class PyBulletSimulator:
    client: BulletClient

    def __init__(self, urdf: str, joints: List[str], visualize: bool = True):
        with RedirectStream(sys.stdout):
            if visualize:
                self.client = BulletClient(connection_mode = pb.GUI)
                self.client.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
                self.client.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)
            else:
                self.client = BulletClient(connection_mode = pb.DIRECT)

        self.client.setRealTimeSimulation(0)
        self.objects = []

        if urdf:
            self.urdf = urdf
            with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
                self.skel_id = self.client.loadURDF(
                    urdf,
                    basePosition = (0, 0, 0),
                    baseOrientation = (0, 0, 0, 1),
                    useFixedBase = True,
                    flags = pb.URDF_MAINTAIN_LINK_ORDER | pb.URDF_USE_SELF_COLLISION
                    )

        if urdf and joints:

            jtu = [
                [
                    i.decode() if isinstance(i, bytes) else i
                    for i in self.client.getJointInfo(self.skel_id, j)
                    ]
                for j in range(self.client.getNumJoints(self.skel_id))
                ]
            jt = sorted(filter(lambda ji: ji[1] in joints, jtu), key = lambda ji: joints.index(ji[1]))

            self.joints = [ji[0] for ji in jt]
            self.lows = [ji[8] for ji in jt]
            self.highs = [ji[9] for ji in jt]

            self.link_map = {ji[12]: ji[0] for ji in jtu}

            if urdf:
                srdffile = list(Path(urdf).parent.glob('*.srdf'))[0]
                if srdffile:
                    with open(srdffile, 'r') as f:
                        srdf = xmltodict.parse(f.read())

                    for disabled in srdf['robot']['disable_collisions']:
                        link1, link2 = disabled['@link1'], disabled['@link2']
                        l1x = self.link_map[link1] if link1 in self.link_map else -1
                        l2x = self.link_map[link2] if link2 in self.link_map else -1
                        self.client.setCollisionFilterPair(0, 0, l1x, l2x, False)

    def set_joint_positions(self, positions: List[float]):
        for joint, value in zip(self.joints, positions):
            self.client.resetJointState(self.skel_id, joint, value, targetVelocity = 0)

    def in_collision(self) -> bool:
        self.client.performCollisionDetection()
        dists = list(point[8] for point in self.client.getContactPoints())
        return bool(dists) and min(dists) < 0

    def set_camera(self, position: Position, look_at: Position):
        dx = position[0] - look_at[0]
        dy = position[1] - look_at[1]
        dz = position[2] - look_at[2]

        import math

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw = math.atan2(dz, dx)
        pitch = math.atan2(math.sqrt(dz * dz + dx * dx), dy) + math.pi

        self.client.resetDebugVisualizerCamera(
            cameraDistance = distance,
            cameraYaw = math.degrees(yaw),
            cameraPitch = math.degrees(pitch),
            cameraTargetPosition = look_at
            )

    def add_capsule(
        self,
        radius: float,
        length: float,
        position: Position,
        rot_xyzw: XYZWQuaternion,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):
        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_CAPSULE,
                radius = radius,
                length = length,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(pb.GEOM_CAPSULE, radius = radius, height = length)

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                baseOrientation = rot_xyzw,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

                return multibody_id

    def add_cylinder(
        self,
        radius: float,
        length: float,
        position: Position,
        rot_xyzw: XYZWQuaternion,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):
        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_CYLINDER,
                radius = radius,
                length = length,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(
                pb.GEOM_CYLINDER, radius = radius, height = length
                )

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                baseOrientation = rot_xyzw,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

                return multibody_id

    def add_cuboid(
        self,
        half_extents: Vector3,
        position: Position,
        rot_xyzw: XYZWQuaternion,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):

        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_BOX,
                halfExtents = half_extents,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(pb.GEOM_BOX, halfExtents = half_extents)

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                baseOrientation = rot_xyzw,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

                return multibody_id

    def add_sphere(
        self,
        radius: float,
        position: Position,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):

        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_SPHERE,
                radius = radius,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(pb.GEOM_SPHERE, radius = radius)

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

            return multibody_id

    def update_object_position(
            self, multibody_id: int, position: Position, rot_xywz: XYZWQuaternion = [0, 0, 0, 1]
        ):
        self.client.resetBasePositionAndOrientation(multibody_id, position, rot_xywz)

    def add_height_map(
            self,
            height_file: Path,
            texture_file: Optional[Path] = None,
            scale: Vector3 = [1, 1, 1],
            center: Position = [0., 0., 0.]
        ):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            shape_id = self.client.createCollisionShape(
                shapeType = pb.GEOM_HEIGHTFIELD, meshScale = scale, fileName = str(height_file)
                )

            terrain = self.client.createMultiBody(baseCollisionShapeIndex = shape_id, basePosition = center)

            if texture_file:
                texture_id = self.client.loadTexture(str(texture_file))
                self.client.changeVisualShape(terrain, -1, textureUniqueId = texture_id)

            self.client.changeVisualShape(terrain, -1, rgbaColor = [1, 1, 1, 1])

    def add_environment_from_problem_dict(self, problem: Dict[str, Any], add_names: bool = True):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            for obj in problem['sphere']:
                self.add_sphere(
                    obj['radius'],
                    obj['position'],
                    obj['name'] if add_names and 'name' in obj else None,
                    obj['name'],
                    )

            for obj in problem['cylinder']:
                self.add_capsule(
                    obj['radius'],
                    obj['length'],
                    obj['position'],
                    obj['orientation_quat_xyzw'],
                    obj['name'] if add_names and 'name' in obj else None,
                    obj['name'],
                    )

            for obj in problem['box']:
                self.add_cuboid(
                    obj['half_extents'],
                    obj['position'],
                    obj['orientation_quat_xyzw'],
                    obj['name'] if add_names and 'name' in obj else None,
                    obj['name'],
                    )

    def draw_roadmap(self, fk_function, roadmap):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            for i, edge_list in enumerate(roadmap.edges):
                for edge in edge_list:
                    a = fk_function(roadmap[i].to_list())[-1]
                    b = fk_function(roadmap[edge].to_list())[-1]
                    self.client.addUserDebugLine([a.x, a.y, a.z], [b.x, b.y, b.z])

    def draw_pointcloud(self, pc, lifetime: float = 0.):
        maxes = np.max(pc, axis = 0)
        colors = 0.8 * (pc / maxes)
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            self.client.addUserDebugPoints(pc, colors, pointSize = 3, lifeTime = lifetime)

    def clear_pointcloud(self):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            self.client.removeAllUserDebugItems()

    def animate(self, plan, callback = None):
        plan_idx = 0
        playing = False
        moved = True
        left = self.client.B3G_LEFT_ARROW
        right = self.client.B3G_RIGHT_ARROW
        space_code = ord(' ')

        print(
            """Press `space` to start/stop animation.
Use left/right arrow keys to move through individual states."""
            )

        while True:
            c = plan[plan_idx]
            if isinstance(c, list):
                c_list = c
            elif isinstance(c, np.ndarray):
                c_list = c.tolist()
            else:
                c_list = c.to_list()

            self.set_joint_positions(c_list)

            if callable(callback):
                callback(c_list)

            moved = False
            keys = self.client.getKeyboardEvents()

            if space_code in keys and keys[space_code] & self.client.KEY_WAS_TRIGGERED:
                playing = not playing

            elif not playing and left in keys and keys[left] & self.client.KEY_WAS_TRIGGERED:
                plan_idx -= 1
                if plan_idx < 0:
                    plan_idx = len(plan) - 1

                moved = True

            elif not playing and right in keys and keys[right] & self.client.KEY_WAS_TRIGGERED:
                plan_idx += 1
                if plan_idx >= len(plan):
                    plan_idx = 0

                moved = True

            elif playing:
                plan_idx = min(len(plan), plan_idx + 1)
                moved = True

            if moved:
                if plan_idx >= len(plan):
                    plan_idx = 0

            time.sleep(0.016)
