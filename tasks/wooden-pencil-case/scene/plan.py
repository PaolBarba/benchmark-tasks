"""Build the bimanual demonstration and matched interlock controls."""

import json
import math
from pathlib import Path

import numpy as np

from yam_arena.ik import matrix_to_quat_xyzw

from scene import HINGE, KNOB, PEN_CENTER, RELEASE, RELEASE_KNOB_HEIGHT

LID_DOWN = np.diag([1., -1., -1.])
PEN_DOWN = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])


def stage(phase, position, seconds, grip, orientation=LID_DOWN, arm='right'):
    return dict(phase=phase, seconds=seconds, **{arm: dict(position=list(position), grip=grip,
                rotation_xyzw=matrix_to_quat_xyzw(orientation).tolist())})


def knob_tcp(angle_deg):
    phi = -math.radians(angle_deg)
    c, s = math.cos(phi), math.sin(phi)
    rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    return np.array(HINGE) + rotation @ KNOB + [0., 0., .008]


def plans():
    knob = knob_tcp(0.)
    opened = knob_tcp(109.)
    retreat = opened + [-.040, .008, .041]
    open_lid = [
        stage('approach_lid_knob', knob + [0., 0., .085], 1.6, .55),
        stage('lower_onto_knob', knob, 1.1, .55),
        stage('pinch_knob', knob, .7, .26),
        *[stage(f'open_lid_to_{angle}_degrees', knob_tcp(angle), .45, .26)
          for angle in (12, 24, 36, 48, 60, 72, 84, 96, 109)],
        stage('release_open_lid', opened, .55, .60),
        stage('lift_off_knob', opened + [0., 0., .026], .7, .60),
        stage('clear_lid', retreat, .85, .60),
        stage('clear_the_release_approach', [.320, -.225, .960], 1.2, .60, PEN_DOWN),
        dict(phase='settle_at_open_stop', seconds=1.2),
    ]
    pinch = np.array(PEN_CENTER) + [0., 0., .007]
    above = pinch + [0., 0., .112]
    clear = above + [0., -.095, 0.]
    extract = [
        stage('approach_pen_recess', above, 1.25, .43, PEN_DOWN),
        stage('lower_into_finger_relief', pinch, 1.2, .43, PEN_DOWN),
        stage('pinch_pen_barrel', pinch, .75, .145, PEN_DOWN),
        stage('lift_pen_out_of_case', above, 1.15, .145, PEN_DOWN),
        stage('carry_pen_clear_of_case', clear, .95, .145, PEN_DOWN),
        dict(phase='hold_pen_clear', seconds=1.5),
    ]
    release = np.array(RELEASE) + [0., 0., RELEASE_KNOB_HEIGHT + .008]
    approach = release + [0., 0., .060]
    turn = [
        stage('approach_release', approach, 1.5, .52, PEN_DOWN, 'left'),
        stage('lower_onto_release', release, 1.15, .52, PEN_DOWN, 'left'),
        stage('grasp_release', release, .65, .179, PEN_DOWN, 'left'),
    ]
    for degrees in (20, 38, 52):
        theta = math.radians(degrees)
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ PEN_DOWN
        turn.append(stage(f'turn_release_{degrees}', release, .55, .179, rotation, 'left'))
    let_go = [
        stage('let_release_spring_return', release, .65, .68, rotation, 'left'),
        stage('retract_left_hand', approach, .7, .68, rotation, 'left'),
        dict(phase='wait_for_relock', seconds=.6),
    ]
    return dict(recovery=[*open_lid, *turn, *extract, *let_go],
                locked_pull=[*open_lid, *extract],
                early_release=[*open_lid, *turn, *let_go, *extract],
                hold=[dict(phase='hold_at_ready', seconds=2.)])


if __name__ == '__main__':
    Path(__file__).with_name('trajectory.json').write_text(json.dumps(plans(), indent=2) + '\n')
