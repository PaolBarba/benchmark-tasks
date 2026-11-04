"""Scorer edge cases and actual YAM URDF preflight; no physics validation."""
import json
from pathlib import Path
import numpy as np
from resist_isaac.runtime import _task_module
from tools.run_recovery_pilot import planned_actions

folder = Path(__file__).parent.resolve()
m, fingerprint = _task_module(folder)
task = m.make_task('recoverable')
def state(p):
    return np.array(p), np.eye(3), np.zeros(6)
carriage = state([.35,-.02,.92])
pin = state([.35,.32,.92])
hands = [np.array([.35,.32,.924]), np.array([.35,-.12,.822])]
assert m.score_state(pin, carriage, hands)['success']
assert not m.score_state(state([.35,.1,.92]), carriage, hands)['success']
assert m.score_state(pin, state([.35,-.02,.94]), hands)['score'] == 0
assert m.score_state(pin, carriage, hands, True)['score'] == 0
assert m.score_state(pin, carriage, hands[::-1])['success']
print('Scorer edge cases passed', flush=True)
for policy, plan in json.loads((folder/'trajectory.json').read_text()).items():
    count = sum(1 for _ in planned_actions(task.initial_action(),plan,1200))
    assert count == round(sum(s['seconds'] for s in plan)*20)
    print(policy,count,'URDF reachable',flush=True)
print('fingerprint',fingerprint)
