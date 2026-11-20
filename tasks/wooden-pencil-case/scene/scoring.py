"""Score a stable pen extraction enabled by two simultaneous native grasps."""

import numpy as np

from .scene import PEN_HALF_AXIS, PEN_RADIUS, RELEASE_OPEN_DEG, RIM_Z

CLEAR_KEEPER_M = .012


def opposed_grasps(finger_forces):
    magnitude = np.linalg.norm(finger_forces, axis=-1)
    opposition = np.sum(finger_forces[:, 0] * finger_forces[:, 1], axis=-1)
    return (magnitude.min(axis=1) >= .05) & (opposition <= -.5 * magnitude.prod(axis=1))


class ExtractionScore:
    def __init__(self):
        self.above_keeper = False
        self.coordinated_lift = False

    def evaluate(self, lid_angle, lid_speed, release_angle, release_distances, pen_position,
                 pen_axis_z, pen_speed, pen_forces, release_forces):
        if not np.isfinite(np.r_[lid_angle, lid_speed, release_angle, release_distances,
                                 pen_position, pen_axis_z, pen_speed,
                                 pen_forces.ravel(), release_forces.ravel()]).all():
            raise ValueError('Pencil-case scores require finite native measurements')

        open_lid = lid_angle >= 95. and lid_speed <= .15
        clearance = pen_position[2] - PEN_HALF_AXIS * abs(pen_axis_z) - PEN_RADIUS - RIM_Z
        pen_holds = opposed_grasps(pen_forces)
        release_holds = opposed_grasps(release_forces) & (np.asarray(release_distances) <= .03)
        above_keeper = clearance >= CLEAR_KEEPER_M
        if not above_keeper:
            self.coordinated_lift = False
        elif not self.above_keeper:
            self.coordinated_lift = bool(open_lid and release_angle >= RELEASE_OPEN_DEG
                                         and pen_holds.sum() == 1 and release_holds.sum() == 1
                                         and np.any(pen_holds & release_holds[::-1]))
        self.above_keeper = above_keeper

        extracted = clearance >= .03 and pen_speed <= .04 and pen_holds.sum() == 1
        complete = bool(open_lid and self.coordinated_lift and extracted)
        return dict(success=complete, score=1. if complete else .5 if open_lid else 0.,
                    lid_angle_deg=lid_angle, lid_speed_rad_s=lid_speed, release_angle_deg=release_angle,
                    pen_clearance_m=float(clearance), pen_speed_m_s=pen_speed,
                    left_holds_pen=bool(pen_holds[0]), right_holds_pen=bool(pen_holds[1]),
                    left_holds_release=bool(release_holds[0]), right_holds_release=bool(release_holds[1]),
                    coordinated_lift=self.coordinated_lift)
