"""Open the wooden pencil case. Turn and hold the orange release knob with one arm while the other lifts out the pen. Hold the pen clear of the case."""

from dataclasses import replace
import math
from pathlib import Path
import tempfile

import numpy as np

from resist_isaac.state import pose_state, unit_quaternion

from .scene import HINGE, PEN_CENTER, RELEASE, RELEASE_KNOB_HEIGHT, export_assets, geometry
from .scoring import ExtractionScore

CONDITIONS = ('recoverable',)


def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return WoodenPencilCase()


def create_assets(directory, condition='recoverable'):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return export_assets(Path(directory))


class WoodenPencilCase:
    hold_s = .75
    robot_contact_offset = .001

    def __init__(self):
        from yam_arena.station import top_camera

        bearing = math.sqrt(.5)
        self.overview_camera = replace(top_camera(), pos=(.275, -.10, 1.54),
                                       rot_xyzw=(0., 0., -bearing, bearing))
        self.descriptions = {'pencil_case': geometry()}

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.assets.object_reference import ObjectReference

        self._temporary = tempfile.TemporaryDirectory(prefix='wooden-pencil-case-')
        path = export_assets(Path(self._temporary.name))['pencil_case']
        case = Object(name='pencil_case', object_type=ObjectType.BASE, usd_path=str(path))
        return [case, *[ObjectReference(parent_asset=case, name=f'pencil_case_{part}',
                                        prim_path=f'{case.get_prim_path()}/{part}',
                                        object_type=ObjectType.RIGID) for part in ('lid', 'pen', 'release')]]

    @staticmethod
    def initial_action():
        from yam_arena.constants import READY

        return np.array([*READY, 1., *READY, 1.])

    def reset(self, base):
        import omni.usd
        from isaaclab.app.settings_manager import get_settings_manager
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics

        # The simulator disables force collection until a contact sensor opts in.
        get_settings_manager().set_bool('/physics/disableContactProcessing', False)
        env = '/World/envs/env_0'
        stage = omni.usd.get_context().get_stage()
        paths = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        fingers = []
        for arm in ('Left', 'Right'):
            for link in ('tip_left', 'tip_right'):
                matches = [path for path in paths if path.startswith(f'{env}/{arm}Arm/')
                           and path.endswith(f'/{link}')]
                if len(matches) != 1:
                    raise RuntimeError(f'Expected one native contact link for {arm}Arm/{link}: {matches}')
                fingers.extend(matches)
        simulation = PhysxManager.get_physics_sim_view()
        paths = [f'{env}/pencil_case/{part}' for part in ('pen', 'release')]
        self.contacts = simulation.create_rigid_contact_view(paths, filter_patterns=[fingers, fingers])
        self.rubric = ExtractionScore()

    @staticmethod
    def pose_state(base, name, part):
        return pose_state(base.scene[f'{name}_{part}'])

    def _state(self, base, part):
        from yam_arena.urdf import quat_xyzw_to_matrix

        point, quat, velocity = (v.cpu().numpy() for v in self.pose_state(base, 'pencil_case', part))
        if not unit_quaternion(quat) or not np.isfinite(np.r_[point, velocity]).all():
            raise RuntimeError(f'Invalid native state for {part}')
        return point, quat_xyzw_to_matrix(quat), velocity

    def validate_reset(self, base):
        for part, expected in (('lid', HINGE), ('pen', PEN_CENTER), ('release', RELEASE)):
            point, rotation, velocity = self._state(base, part)
            if (math.dist(point, expected) > .002 or np.max(np.abs(rotation - np.eye(3))) > .04
                    or np.linalg.norm(velocity[:3]) > .01 or np.linalg.norm(velocity[3:]) > .1):
                raise RuntimeError(f'{part} failed to settle at its reset pose {expected}: {point.tolist()}')

    after_settle = validate_reset

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world

        _, lid_rotation, lid_velocity = self._state(base, 'lid')
        pen_point, pen_rotation, pen_velocity = self._state(base, 'pen')
        release_point, release_rotation, _ = self._state(base, 'release')
        forces = self.contacts.get_contact_force_matrix(dt=base.physics_dt).numpy()
        if forces.shape != (2, 4, 3):
            raise RuntimeError(f'Unexpected native contact matrix shape: {forces.shape}')
        lid_angle = math.degrees(math.atan2(-lid_rotation[2, 1], lid_rotation[1, 1]))
        release_angle = math.degrees(math.atan2(release_rotation[1, 0], release_rotation[0, 0]))
        knob_center = release_point + RELEASE_KNOB_HEIGHT * release_rotation[:, 2]
        knob_distances = [math.dist(tcp_world(base.scene[f'{arm}_robot']), knob_center)
                          for arm in ('left', 'right')]
        return self.rubric.evaluate(lid_angle, float(np.linalg.norm(lid_velocity[3:])),
                                    release_angle, knob_distances, pen_point, pen_rotation[2, 0],
                                    float(np.linalg.norm(pen_velocity[:3])), *forces.reshape(2, 2, 2, 3))
