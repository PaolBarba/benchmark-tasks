"""Generate scripted inward pressing and two controls for the opening."""

import json
from pathlib import Path

import numpy as np

from yam_arena.ik import matrix_to_quat_xyzw

from calendar_scene import CHOCOLATE_CENTERS, TARGET, locations

DOWN = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])


def stage(phase, position, seconds, grip):
    return dict(phase=phase, seconds=seconds, right=dict(position=list(position), grip=grip,
                rotation_xyzw=matrix_to_quat_xyzw(DOWN).tolist()))


if __name__ == '__main__':
    panel = np.array(dict(locations())[TARGET]) + [-.049, 0., 0.]
    ready = panel + [0., 0., .095]
    breakthrough = panel + [0., -.012, -.007]
    above_chocolate = np.array(CHOCOLATE_CENTERS[0]) + [0., 0., .095]
    press = [
        stage('approach_door', ready, 1.5, .04),
        stage('lower_to_panel', panel + [0., 0., .020], 1.2, .04),
        stage('break_support_inward', breakthrough, 1.6, .04),
        stage('retract_from_window', breakthrough + [0., 0., .096], 1.2, .04),
        dict(phase='settle_debris', seconds=1.2),
    ]
    grasp = panel + [0., .002, .0055]
    plans = dict(
        recovery=press,
        hold=[dict(phase='hold_ready', seconds=3.)],
        pull=[stage('approach_edges', ready, 1.5, .88),
              stage('lower_around_panel', grasp, 1.5, .88),
              stage('grasp_edges', grasp, 1.4, .60),
              stage('pull_outward', grasp + [0., .012, .065], 1.7, .60),
              stage('release', ready, .8, .90)],
        touch=[*press, stage('approach_exposed_chocolate', above_chocolate, 1.3, .04),
               stage('touch_chocolate', above_chocolate - [0., 0., .102], 1.4, .04),
               stage('retract_after_contact', above_chocolate, 1.2, .04)],
    )
    Path(__file__).with_name('trajectory.json').write_text(json.dumps(plans, indent=2) + '\n')
