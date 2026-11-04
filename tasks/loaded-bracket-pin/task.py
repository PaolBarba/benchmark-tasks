"""Extract the retaining pin while maintaining the orange carriage's height."""
from pathlib import Path
import numpy as np
from resist_isaac.clearance_recovery import ClearanceRecovery, export_assets

CONDITIONS = ('recoverable',)


def score_state(pin, carriage, hands, unsafe=False):
    p, r, v = pin
    c, _, cv = carriage
    height_error = abs(float(c[2]) - .92)
    violation = height_error > .010 or cv[2] < -.35 or p[2] < .7725
    for pos in (p, c):
        violation |= not (-.19 < pos[0] < .91 and abs(pos[1]) < .65 and .7425 < pos[2] < 1.4)
    unsafe = bool(unsafe or violation)
    ends = np.array([[0, -.175, 0], [0, -.02, 0]]) @ r.T + p
    # Include shaft radius when testing clearance of the outer bracket faces.
    cleared = bool(ends[:,1].min() - .006 > .041 or ends[:,1].max() + .006 < -.081)
    retained = min(np.linalg.norm(p - hand) for hand in hands) < .08
    stable = max(np.linalg.norm(v[:3]), np.linalg.norm(cv[:3])) < .02
    success = bool(cleared and retained and stable and height_error < .008 and not unsafe)
    return dict(success=success, score=0. if unsafe else 1. if success else .5 if cleared and retained else 0.,
        unsafe_event=unsafe, pin_clear=cleared, pin_retained=bool(retained),
        pin_y_m=float(p[1]), carriage_height_m=float(c[2]),
        carriage_height_error_m=height_error, downward_speed_m_s=max(0.,float(-cv[2])))


class LoadedPin(ClearanceRecovery):
    def validate_reset(self, base):
        super().validate_reset(base)
        states = self.states(base)
        if abs(float(states['bracket/carriage'][0][2]) - .92) > .005:
            raise RuntimeError('Carriage did not settle on the retaining pin')
        if np.linalg.norm(states['pin'][0] - np.array(self.cfg['poses']['pin'])) > .005:
            raise RuntimeError('Pin moved during preparation')

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world
        state = self.states(base)
        result = score_state(state['pin'], state['bracket/carriage'],
            [tcp_world(base.scene[s+'_robot']) for s in ('left','right')], self._unsafe)
        self._unsafe = result['unsafe_event']
        return result


def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return LoadedPin(Path(__file__).parent)


def create_assets(directory, condition='recoverable'):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    return export_assets(Path(__file__).parent, directory)
