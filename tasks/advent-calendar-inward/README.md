# Break through day 18 and leave the chocolate untouched

Break through door 18 by pressing it inward. Leave the chocolate intact and untouched inside its compartment. Leave the other doors in place.

Day 18 sits on a disposable paperboard support. Pressing inward breaks the support, sending the captive panel into an empty side pocket. An outward stop carries pulling loads. See the [design review](design-review.md).

## Real-world basis

Chocolate Advent calendars use numbered, perforated paperboard windows over individual compartments. Pressing inward to rupture a perforated tab is also a documented opening step for paperboard food cartons. The scene combines these features to test selective opening around fragile contents. The captive guide and side pocket make the panel's clearance measurable in simulation. [Glerup Revere Advent Calendar](https://glerup.com/advent-calendar/) · [Quaker Oats food packaging carton, US8959877B2, Fig. 1 and description of tab 25](https://patents.google.com/patent/US8959877B2/en)

## Fixed setup and reset

The two YAM arms stand at (0, ±0.305, 0.750) m with open grippers. The 1,100 × 1,300 × 60 mm tabletop is at z = 0.7475 m. Station +x points forward, +y robot-left, and +z up.

- The fixed red face and molded plastic tray span about 256 × 456 × 80 mm, including the empty debris pocket extending to the right of day 18. Each green numbered panel measures 94 × 62 × 4 mm. The rows, read left to right, are **7, 3, 22** and **12, 24, 18**.
- A raised rim shields the edges of day 18. The panel is captive on an 81 mm passive diagonal guide. At its travel stop, its center has moved about 75 mm to the right and 31 mm down. A 20 × 36 × 12 mm paperboard support rests beneath the far edge. Its idealized breakable attachment has a 5 N force threshold. A 4 × 108 × 22 mm molded plastic rib separates the support's drop channel from the chocolate well.
- Day 18 contains one 40.4 × 30 × 12 mm piece of molded chocolate. Its segment centers start at (0.2710, −0.1120, 0.7655) and (0.2914, −0.1120, 0.7655) m. The rest of the wells are empty.
- A camera fixed directly overhead supplies all reported video. Dream-1 also sees the native wrist camera observations.

Use seed 0 and a fresh simulator process for each trial. The reset verifies the settled parts within 2 mm and starts chocolate contact monitoring before the robot moves.

## Scoring

- **0.5** after the breakable support moves at least 6 mm and the panel travels at least 12 mm inward.
- **1.0** after the support breaks and the panel travels at least 73 mm along its inward guide at a speed of 40 mm/s or less for 0.75 seconds. The normal contact force on either chocolate segment from any robot link, door or loose paperboard support must stay at or below the 0.01 N solver tolerance.
- **0** if either chocolate segment moves over 3 mm or rotates over 5°, a neighboring panel moves over 5 mm, or the chocolate contact tolerance is exceeded. A violation latches zero even after the objects settle.

An overhead opening of at least 2 mm beyond the original panel footprint follows from the 73 mm travel along the guide. Normal contact is sampled at every native physics step and evaluated as a peak over the trial. The scene approximates paper tearing with a breakable joint and the paperboard door with a rigid captive panel. Fibers, crumbs and surface markings are outside the simulation. The thresholds describe the simulator.

## Run and score

Start in the [challenge YAM station environment](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux with an NVIDIA GPU, Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Activate its Python environment, set `OMNI_KIT_ACCEPT_EULA=YES` after accepting the NVIDIA license, and configure the challenge Dream credentials. From this task directory, install the bundled embodiment and run each trial in a fresh process.

```bash
python -m pip install --no-deps -e ./native
export PYTHONPATH="$PWD/native"
python native/tools/run_recovery_pilot.py scene /tmp/advent-inward-scripted \
  --policy recovery --seed 0 --camera top_cam --return-to-ready
python native/tools/run_advent_calendar_dream.py scene /tmp/advent-inward-dream
```

The task limit is 800 controls at 20 Hz (40 simulated seconds). Rebuild the deterministic waypoints with `python scene/plan.py`. The `hold`, `pull` and `touch` scripted controls check the sealed reset, outward stop and contact penalty. Each scripted trial records executed controls and measurements in `trace.jsonl`. The Dream-1 receipt points to its evaluation log and executed action sidecar. Use fresh output directories, separate from earlier Advent calendar versions.
