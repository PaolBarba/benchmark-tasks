"""Author nominal passive bracket apparatus; this script does not run physics."""
import json
from pathlib import Path
from resist_isaac.recovery import box, body, description

ROOT = Path(__file__).parent
STEEL = (.55, .59, .64, 1)
ORANGE = (.90, .33, .06, 1)

def ring(y, color):
    # Four separate walls preserve the bore; never a convexified solid hole.
    return [box((.009, .006, .022), (x, y, 0), color, friction=[.8, .6]) for x in (-.016, .016)] + [
        box((.007, .006, .0075), (0, y, z), color, friction=[.8, .6]) for z in (-.0145, .0145)]

fixed = []
for y in (-.04, .04):
    fixed += ring(y, STEEL)
    fixed += [box((.009, .006, .06725), (x, y, -.08925), STEEL) for x in (-.09, .09)]
    fixed += [box((.0325,.006,.008),(x,y,0),STEEL) for x in (-.0575,.0575)]
fixed += [box((.10, .11, .008), (0, -.035, -.1645), STEEL)]
load = ring(0, ORANGE) + [
    box((.025, .006, .014), (0, 0, -.036), ORANGE),
    box((.025, .055, .006), (0, -.05, -.050), ORANGE),
    box((.013, .025, .02), (0, -.10, -.040), ORANGE, friction=[1.,.8]),
]
frame = body('frame', fixed, 1)
carriage = body('carriage', load, .8)
shaft = dict(type='cylinder', size=[.006], fromto=[0,-.175,0,0,-.02,0], color=list(STEEL), friction=[.8,.6])
pin = body('pin', [shaft, box((.013,.018,.015),color=(.13,.35,.75,1),friction=[1.0,.8])], .09)
assets = {
    'fixture': description('Fixture',[body('fixture',[box((.04,.04,.001),(.35,.30,.7485),(.08,.65,.18,1))],static=True)]),
    'bracket': description('Bracket',[frame,carriage],articulation=True,fixed_base='frame',
        solver_position_iterations=32,solver_velocity_iterations=8,
        joints=[dict(name='vertical_guide',type='prismatic',parent='frame',child='carriage',axis='Z',
            lower=-.06,upper=.012,collision=True)]),
    'pin': description('Pin',[pin],solver_position_iterations=32,solver_velocity_iterations=8),
}
assert len(ring(0, STEEL)) == 4
assert assets['bracket']['joints'][0].get('stiffness',0) == 0
(ROOT/'geometry.json').write_text(json.dumps(assets,indent=2)+'\n')
