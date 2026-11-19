"""Inward-break paperboard window and a molded chocolate in a fixed tray."""

import math

from resist_isaac.recovery import body, box
from resist_isaac.usd_assets import create_asset

TABLE = .7475
PANEL_Z = .8175
LAYOUT = ((7, 3, 22), (12, 24, 18))
DOORS = tuple(number for row in LAYOUT for number in row)
TARGET = 18
CHOCOLATE = ('chocolate_a', 'chocolate_b')
CHOCOLATE_CENTERS = ((.271, -.112, TABLE + .018), (.2914, -.112, TABLE + .018))
GUIDE_DIRECTION = (0., -12 / 13, -5 / 13)
GUIDE_TRAVEL = .081
SUPPORT_POSITION = (.320, -.114, PANEL_Z - .0081)
SUPPORT_BREAK_N = 5.
SEAL_BREAK_N = 5.
SEAL_BREAK_NM = .45
CHOCOLATE_BREAK_N = 3.0
CHOCOLATE_BREAK_NM = .022

NUMBERS = {
    '1': ('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
    '2': ('01110', '10001', '00001', '00110', '01000', '10000', '11111'),
    '3': ('11110', '00001', '00001', '01110', '00001', '00001', '11110'),
    '4': ('00010', '00110', '01010', '10010', '11111', '00010', '00010'),
    '7': ('11111', '00001', '00010', '00010', '00100', '00100', '00100'),
    '8': ('01110', '10001', '10001', '01110', '10001', '10001', '01110'),
}
RED = (.32, .023, .042, 1)
GREEN = (.023, .18, .107, 1)
CREAM = (.93, .81, .58, 1)
TRAY = (.59, .29, .11, 1)
DARK_CHOCOLATE = (.11, .036, .018, 1)


def locations():
    for x, row in zip((.453, .330), LAYOUT, strict=True):
        for number, y in zip(row, (.112, 0., -.112), strict=True):
            yield number, (x, y, PANEL_Z)


def digits(number, x, z):
    label = str(number)
    pixel = .0049
    half_width = (6 * len(label) - 1) * pixel / 2
    points, faces = [], []
    for index, digit in enumerate(label):
        for row, line in enumerate(NUMBERS[digit]):
            for col, filled in enumerate(line):
                if filled == '0':
                    continue
                x_top = x + (3.5 - row) * pixel
                y_left = half_width - (6 * index + col) * pixel
                faces.append(list(range(len(points), len(points) + 4)))
                points.extend(((x_top, y_left, z), (x_top - pixel, y_left, z),
                               (x_top - pixel, y_left - pixel, z), (x_top, y_left - pixel, z)))
    return dict(type='mesh', mesh=dict(points=points, faces=faces), color=CREAM, collision=False)


def asset_description():
    frame = [box((.128, .174, .006), (.333, 0., TABLE + .006), TRAY)]
    for y in (-.056, .056, .171):
        frame.append(box((.124, .004, .0335), (.333, y, TABLE + .0335), RED))
    frame.append(box((.004, .172, .0335), (.457, 0., TABLE + .0335), RED))
    frame.extend([
        box((.061, .004, .0335), (.396, -.171, TABLE + .0335), RED),
        # A molded rib keeps the falling support on its own side of the chocolate well.
        box((.002, .054, .011), (.306, -.112, TABLE + .023), TRAY),
        box((.066, .053, .006), (.272, -.226, TABLE + .006), TRAY),
        box((.066, .004, .026), (.272, -.278, TABLE + .026), RED),
        box((.004, .054, .026), (.209, -.225, TABLE + .026), RED),
        box((.004, .054, .026), (.335, -.225, TABLE + .026), RED),
    ])
    for x in (.210, .333):
        # The top-facing cardboard frame has open thumb recesses at the free edges.
        for low, high in ((-.171, -.156), (-.068, -.044), (.044, .068), (.156, .171)):
            frame.append(box((.004, (high - low) / 2, .0335), (x, (low + high) / 2, TABLE + .0335), RED))
        frame.append(box((.005, .168, .012), (x, 0., TABLE + .012), RED))
    parts = []
    joints = []
    for number, anchor in locations():
        name = f'door_{number}'
        collision = dict(friction=[1.1, .9], contact_offset=.00015)
        parts.append(body(name, [
            box((.047, .031, .002), (-.049, 0., 0.), GREEN, **collision),
            digits(number, -.049, .0022),
        ], mass=.008, position=anchor))
        x, y, z = anchor
        if number == TARGET:
            for side in (-1, 1):
                frame.append(box((.049, .006, .004), (x - .049, y + side * .033, z + .0063), RED))
                frame.append(box((.004, .031, .004), (x - .049 + side * .046, y, z + .0063), RED))
            # The outward stop carries tension without loading the separate support below the panel.
            tilt = math.pi + math.atan2(5, 12)
            joint_rotation = [math.cos(tilt / 2), math.sin(tilt / 2), 0., 0.]
            joints.append(dict(name='press_guide', type='prismatic', child=name, axis='Y',
                               position_parent=anchor, orientation_parent=joint_rotation,
                               orientation_child=joint_rotation, lower=0., upper=GUIDE_TRAVEL,
                               damping=.18))
            continue
        # The molded rim supports the card at its ends while leaving both long edges accessible.
        for offset in (-.088, -.010):
            frame.append(box((.006, .030, .006), (x + offset, y, z - .0083), TRAY))
            for side in (-1, 1):
                frame.append(box((.006, .003, .008), (x + offset, y + side * .0355, z - .007), TRAY))
        # A narrow, die-cut strip stays attached to the box after the window tears out.
        frame.append(box((.003, .031, .002), (x + .003, y, z), GREEN, collision=False))
        for offset in (-.026, -.013, 0., .013, .026):
            frame.append(box((.0015, .0035, .001), (x - .0005, y + offset, z + .001),
                             CREAM, collision=False))
        joints.append(dict(name=f'perforation_{number}', type='fixed', child=name,
                           position_parent=anchor))
    parts.append(body('paper_support', [box((.010, .018, .006), color=CREAM)],
                      mass=.003, position=SUPPORT_POSITION))
    joints.append(dict(name='support_seam', type='fixed', child='paper_support',
                       position_parent=SUPPORT_POSITION))
    for name, center in zip(CHOCOLATE, CHOCOLATE_CENTERS, strict=True):
        parts.append(body(name, [box((.010, .015, .006), color=DARK_CHOCOLATE, friction=[.6, .5]),
                                 box((.008, .012, .0006), (0., 0., .0063),
                                     (.15, .055, .028, 1), collision=False)], .011, center))
    joints.append(dict(name='chocolate_fracture', type='fixed', parent=CHOCOLATE[0], child=CHOCOLATE[1],
                       position_parent=[.0102, 0., 0.], position_child=[-.0102, 0., 0.]))
    return dict(name='AdventCalendar', bodies=[body('tray', frame, static=True), *parts], joints=joints,
                solver_position_iterations=32, solver_velocity_iterations=8)


def export_assets(directory):
    from pxr import PhysxSchema, Usd, UsdPhysics

    path = create_asset(asset_description(), directory / 'calendar.usda')
    stage = Usd.Stage.Open(str(path))
    root = stage.GetDefaultPrim().GetPath()
    for number in DOORS:
        if number == TARGET:
            continue
        seam = UsdPhysics.FixedJoint.Get(stage, root.AppendChild(f'perforation_{number}'))
        seam.CreateBreakForceAttr(SEAL_BREAK_N)
        seam.CreateBreakTorqueAttr(SEAL_BREAK_NM)
    support = UsdPhysics.FixedJoint.Get(stage, root.AppendChild('support_seam'))
    support.CreateBreakForceAttr(SUPPORT_BREAK_N)
    chocolate = UsdPhysics.FixedJoint.Get(stage, root.AppendChild('chocolate_fracture'))
    chocolate.CreateBreakForceAttr(CHOCOLATE_BREAK_N)
    chocolate.CreateBreakTorqueAttr(CHOCOLATE_BREAK_NM)
    for name in CHOCOLATE:
        report = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(root.AppendChild(name)))
        report.CreateThresholdAttr(0.)
    stage.GetRootLayer().Save()
    return {'calendar': path}
