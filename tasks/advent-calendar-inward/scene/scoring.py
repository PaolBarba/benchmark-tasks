"""Score an inward break while leaving the chocolate untouched and in place."""

import math

MIN_BREAK_TRAVEL_M = .012
MIN_OPEN_TRAVEL_M = .073
CONTACT_TOLERANCE_N = .01
MAX_CHOCOLATE_SHIFT_M = .003
MAX_CHOCOLATE_ROTATION_DEG = 5.


class CalendarScore:
    def __init__(self):
        self.invalid = False
        self.chocolate_disturbed = False
        self.stable_steps = 0

    def evaluate(self, travel, panel_speed, support_shift, neighbor_shift, chocolate_shift,
                 chocolate_rotation, peak_force):
        measurements = dict(inward_travel_m=travel, panel_speed_m_s=panel_speed,
                            support_shift_m=support_shift, neighbor_panel_shift_m=neighbor_shift,
                            chocolate_shift_m=chocolate_shift, chocolate_rotation_deg=chocolate_rotation,
                            peak_chocolate_contact_force_n=peak_force)
        finite = all(map(math.isfinite, measurements.values()))
        self.chocolate_disturbed |= (chocolate_shift > MAX_CHOCOLATE_SHIFT_M
                                     or chocolate_rotation > MAX_CHOCOLATE_ROTATION_DEG
                                     or peak_force > CONTACT_TOLERANCE_N)
        self.invalid |= not finite or neighbor_shift > .005 or self.chocolate_disturbed
        broken = support_shift >= .006 and travel >= MIN_BREAK_TRAVEL_M
        clear = broken and travel >= MIN_OPEN_TRAVEL_M
        self.stable_steps = self.stable_steps + 1 if clear and panel_speed <= .04 and not self.invalid else 0
        success = self.stable_steps >= 15
        score = 0. if self.invalid else 1. if success else .5 if broken else 0.
        return dict(success=success, score=score, unsafe_event=self.invalid, window_clear=clear,
                    chocolate_disturbed=self.chocolate_disturbed,
                    **{key: value if math.isfinite(value) else 0. for key, value in measurements.items()})
