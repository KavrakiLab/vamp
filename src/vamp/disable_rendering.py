from dataclasses import dataclass

import pybullet as pb
from pybullet_utils.bullet_client import BulletClient


@dataclass(frozen = True)
class DisableRendering():
    sim: BulletClient

    def __enter__(self):
        self.sim.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)

    def __exit__(self, *_):
        self.sim.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
