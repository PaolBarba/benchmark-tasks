"""Dimensions and native rigid geometry for a fitted wooden pen case."""

import math

from resist_isaac.recovery import body, box, description
from resist_isaac.usd_assets import create_asset

BASE = (.310, -.023, .7915)
HINGE = (.310, -.023, .7975)
KNOB = (0., -.065, .024)
STOP_DEG = 112.
RELEASE = (.178, -.045, .79225)
RELEASE_KNOB_HEIGHT = .036
RELEASE_STOP_DEG = 55.
RELEASE_OPEN_DEG = 35.
PEN_CENTER = (.310, -.062, .7805)
PEN_HALF_AXIS = .065
PEN_RADIUS = .008
LID_HALF_THICKNESS = .004
RIM_Z = .7905

WOOD = (.46, .235, .098, 1.)
LID = (.56, .315, .155, 1.)
FELT = (.09, .18, .135, 1.)
FIXTURE = (.125, .15, .17, 1.)
BRASS = (.66, .45, .17, 1.)
INK = (.035, .07, .205, 1.)
SILVER = (.71, .72, .76, 1.)
ORANGE = (.88, .26, .038, 1.)


def geometry():
    base = [
        box((.104, .047, .003), (0., -.039, -.041), FIXTURE),
        box((.096, .039, .0035), (0., -.039, -.0345), WOOD),
        *[box((.096, .004, .015), (0., y, -.016), WOOD) for y in (-.074, -.004)],
        *[box((.004, .031, .015), (x, -.039, -.016), WOOD) for x in (-.092, .092)],
        *[box((.004, .036, .005), (x, -.039, -.033), FIXTURE) for x in (-.100, .100)],
        *[dict(type='cylinder', size=[.0045, .009],
               fromto=[x - .009, 0., .006, x + .009, 0., .006], color=BRASS, collision=False)
          for x in (-.063, .063)],
        box((.026, .015, .003), (-.116, -.022, -.041), FIXTURE),
        dict(type='cylinder', size=[.012, .0175], position=[-.132, -.022, -.0205], color=FIXTURE),
    ]
    # Two fitted saddles leave the middle 72 mm clear for the jaws.
    for x in (-.057, .057):
        base.append(box((.021, .016, .006), (x, -.039, -.025), FELT, friction=(.65, .5)))
        base.extend(box((.021, .004, .004), (x, -.039 + y, -.015), FELT, friction=(.65, .5))
                    for y in (-.0135, .0135))
    lid = [
        # The rear edge stays outside the case wall once it swings below the pin.
        box((.096, .039, LID_HALF_THICKNESS), (0., -.039, LID_HALF_THICKNESS), LID),
        dict(type='cylinder', size=[.007, .006], position=[0., KNOB[1], .013], color=BRASS),
        dict(type='capsule', size=[.014, .003], position=KNOB, color=BRASS, friction=(.85, .7)),
    ]
    pen = [
        dict(type='capsule', size=[PEN_RADIUS, PEN_HALF_AXIS],
             fromto=[-PEN_HALF_AXIS, 0., 0., PEN_HALF_AXIS, 0., 0.], color=INK, friction=(.85, .7)),
        dict(type='cylinder', size=[.0084, .0015], fromto=[-.0415, 0., 0., -.0385, 0., 0.],
             color=SILVER, collision=False),
        box((.019, .0016, .00065), (-.044, 0., .00815), SILVER, collision=False),
    ]
    # Upward pull on the pen loads the two steel tabs across the vertical pivot.
    release = [
        box((.011, .0145, .001), (.004, .0105, 0.), SILVER),
        box((.101, .004, .001), (.096, .021, 0.), SILVER),
        *[box((.008, .0245, .001), (x, -.0035, 0.), SILVER) for x in (.077, .187)],
        dict(type='cylinder', size=[.006, .014], position=[0., 0., .015], color=SILVER),
        box((.016, .010, .008), (0., 0., RELEASE_KNOB_HEIGHT), ORANGE, friction=(.85, .7)),
    ]
    return description(
        'WoodenPencilCase',
        [body('base', base, static=True, position=BASE),
         body('lid', lid, mass=.106, position=HINGE),
         body('pen', pen, mass=.018, position=PEN_CENTER),
         body('release', release, mass=.043, position=RELEASE)],
        joints=[dict(name='hinge', type='revolute', parent='base', child='lid', axis='X',
                     position_parent=[0., 0., .006],
                     lower=-math.radians(STOP_DEG), upper=0., damping=.018, collision=False),
                dict(name='release_spring', type='revolute', parent='base', child='release', axis='Z',
                     position_parent=[a-b for a, b in zip(RELEASE, BASE, strict=True)],
                     lower=0., upper=math.radians(RELEASE_STOP_DEG), stiffness=.09, damping=.007,
                     collision=False)],
        solver_position_iterations=32, solver_velocity_iterations=8,
    )


def export_assets(directory):
    from pxr import PhysxSchema, Usd

    path = create_asset(geometry(), directory / 'pencil_case.usda')
    stage = Usd.Stage.Open(str(path))
    for part in ('pen', 'release'):
        prim = stage.GetPrimAtPath(stage.GetDefaultPrim().GetPath().AppendChild(part))
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    stage.GetRootLayer().Save()
    return {'pencil_case': path}
