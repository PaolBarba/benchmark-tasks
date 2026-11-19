"""Break through door 18 by pressing it inward. Leave the chocolate intact and untouched inside its compartment. Leave the other doors in place."""

from dataclasses import replace
import math
from pathlib import Path
import tempfile

import numpy as np

from resist_isaac.state import pose_state

from .calendar_scene import (
    CHOCOLATE, CHOCOLATE_CENTERS, GUIDE_DIRECTION, SUPPORT_POSITION, TARGET,
    asset_description, export_assets, locations,
)
from .scoring import CONTACT_TOLERANCE_N, CalendarScore

CONDITIONS = ('recoverable',)
PANEL_RADIUS_M = .102


def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return AdventCalendar()


def create_assets(directory, condition='recoverable'):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return export_assets(Path(directory))


class AdventCalendar:
    hold_s = .05  # The rubric already requires 15 settled controls at 20 Hz.
    robot_contact_offset = .001

    def __init__(self):
        from yam_arena.station import top_camera

        bearing = math.sqrt(.5)
        self.overview_camera = replace(top_camera(), pos=(.280, 0., 1.65),
                                       rot_xyzw=(0., 0., -bearing, bearing))
        self.descriptions = {'calendar': asset_description()}
        self._used = False
        self._contact_subscription = None

    def build_scene(self, embodiment):
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.assets.object_reference import ObjectReference

        self._temporary = tempfile.TemporaryDirectory(prefix='advent-calendar-')
        path = export_assets(Path(self._temporary.name))['calendar']
        calendar = Object(name='calendar', object_type=ObjectType.BASE, usd_path=str(path))
        return [calendar, *[ObjectReference(parent_asset=calendar, name=f'calendar_{part["name"]}',
                                             prim_path=f'{calendar.get_prim_path()}/{part["name"]}',
                                             object_type=ObjectType.RIGID)
                            for part in self.descriptions['calendar']['bodies'] if not part['static']]]

    @staticmethod
    def initial_action():
        from yam_arena.constants import READY

        return np.array([*READY, 1., *READY, 1.])

    def reset(self, base):
        import omni.physx
        import omni.usd
        from isaaclab_physx.physics import PhysxManager
        from pxr import UsdPhysics

        if self._used:
            raise RuntimeError('Start a fresh native trial to replace the consumed window and chocolate')
        self._used = True
        self.rubric = CalendarScore()
        env = '/World/envs/env_0'
        arms = tuple(f'{env}/{side}Arm/' for side in ('Left', 'Right'))
        stage = omni.usd.get_context().get_stage()
        filters = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)
                   and (str(prim.GetPath()).startswith((*arms, f'{env}/calendar/door_'))
                        or str(prim.GetPath()) == f'{env}/calendar/paper_support')]
        if not all(any(path.startswith(arm) for path in filters) for arm in arms):
            raise RuntimeError('Unable to locate native YAM links for chocolate contact monitoring')
        scene = PhysxManager.get_physics_sim_view()
        paths = [f'{env}/calendar/{name}' for name in CHOCOLATE]
        self.contacts = scene.create_rigid_contact_view(paths, filter_patterns=[filters for _ in paths])
        self.contact_shape = (len(paths), len(filters), 3)
        self._contact_samples = 0
        self._last_sample = -1
        self._contact_error = None
        self._peak_force = 0.
        self._contact_subscription = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
            self._sample_contact, False, 0
        )

    def _sample_contact(self, dt):
        try:
            forces = self.contacts.get_contact_force_matrix(dt=dt).numpy()
            if forces.shape != self.contact_shape or not np.isfinite(forces).all():
                raise RuntimeError('Invalid native chocolate contact force array')
            self._peak_force = max(self._peak_force, float(np.linalg.norm(forces, axis=-1).max()))
            self._contact_samples += 1
        except Exception as error:
            self._contact_error = str(error)

    def close(self):
        if self._contact_subscription is not None:
            self._contact_subscription.unsubscribe()
            self._contact_subscription = None

    def pose_state(self, base, name, part):
        return pose_state(base.scene[f'{name}_{part}'])

    def read_state(self, base, part):
        from yam_arena.urdf import quat_xyzw_to_matrix

        p, q, v = (value.cpu().numpy() for value in self.pose_state(base, 'calendar', part))
        return p, quat_xyzw_to_matrix(q), v

    def measurements(self, base):
        neighbor_shift = 0.
        for door, anchor in locations():
            p, rotation, velocity = self.read_state(base, f'door_{door}')
            if door == TARGET:
                travel = float(np.dot(p - anchor, GUIDE_DIRECTION))
                panel_speed = float(np.linalg.norm(velocity[:3]))
            else:
                shift = np.linalg.norm(p - anchor) + PANEL_RADIUS_M * np.linalg.norm(rotation - np.eye(3), ord=2)
                neighbor_shift = float(np.maximum(neighbor_shift, shift))
        support, _, _ = self.read_state(base, 'paper_support')
        support_shift = math.dist(support, SUPPORT_POSITION)
        chocolate_shift = 0.
        chocolate_rotation = 0.
        for name, position in zip(CHOCOLATE, CHOCOLATE_CENTERS, strict=True):
            point, rotation, _ = self.read_state(base, name)
            chocolate_shift = float(np.maximum(chocolate_shift, math.dist(point, position)))
            cos_angle = float(np.clip((np.trace(rotation) - 1) / 2, -1., 1.))
            chocolate_rotation = float(np.maximum(chocolate_rotation, math.degrees(math.acos(cos_angle))))
        return travel, panel_speed, support_shift, neighbor_shift, chocolate_shift, chocolate_rotation

    def validate_reset(self, base):
        for name, expected in [
            *((f'door_{door}', anchor) for door, anchor in locations()),
            ('paper_support', SUPPORT_POSITION),
            *zip(CHOCOLATE, CHOCOLATE_CENTERS, strict=True),
        ]:
            p, r, _ = self.read_state(base, name)
            if not (math.dist(p, expected) <= .002 and np.max(np.abs(r - np.eye(3))) <= .03):
                raise RuntimeError(f'{name} did not settle at {expected}: position {p.tolist()}, rotation {r.tolist()}')
        if self._contact_error or not self._contact_samples or self._peak_force > CONTACT_TOLERANCE_N:
            raise RuntimeError(f'Invalid chocolate contact monitoring at reset: {self._contact_error}, '
                               f'{self._contact_samples} samples, {self._peak_force:.4f} N')

    after_settle = validate_reset

    def evaluate(self, base):
        if self._contact_error or self._contact_samples <= self._last_sample:
            raise RuntimeError(f'Native chocolate contact monitoring did not advance: {self._contact_error}')
        self._last_sample = self._contact_samples
        metrics = self.rubric.evaluate(*self.measurements(base), self._peak_force)
        return dict(metrics, contact_samples=self._contact_samples)
