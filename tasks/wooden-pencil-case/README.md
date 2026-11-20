# Open a wooden pencil case and release the pen

Open the wooden pencil case. Turn and hold the orange release knob with one arm while the other lifts out the pen. Hold the pen clear of the case.

Hinged wooden presentation cases hold a pen in a fitted insert. This case adds a spring-return keeper based on rotary latches. The arm on the release must keep the pen unlocked as the other arm lifts. [Hinged maple pen case](https://signaturelaserdesigns.com/product/engraved-pen-and-wooden-gift-box-set-personalized-maple-pen-case/) · [Southco E5 spring-loaded cam latches](https://media.southco.com/media/static/Literature/e5-t.en.pdf) · [Design review](design-review.md)

## Fixed setup

Use the calibrated YAM station. Its 1,100 × 1,300 × 60 mm worktop is at z = 0.7475 m. The arms start at (0, ±0.305, 0.750) m in their open-gripper ready poses. Station +x points forward, +y robot-left and +z up.

| Object | Size, material and starting pose |
| --- | --- |
| Bench fixture | Dark 208 × 94 × 6 mm steel plate centered at (0.310, −0.062, 0.7505) m with two end retainers, plus a steel pedestal for the knob pivot. |
| Pencil case | 192 × 78 mm wooden base with a 7 mm floor and 30 mm walls. A 192 × 78 × 8 mm wooden lid sits on a brass hinge along x at y = −0.023 m and z = 0.7975 m. Its 28 mm brass lifting knob is 65 mm from the hinge. The open stop is 112°. |
| Insert and pen | Two 42 mm long felt-lined saddles with a 19 mm wide slot and 72 mm of central finger relief. The blue plastic pen with a metal clip is 146 mm long and 16 mm across, along x, centered at (0.310, −0.062, 0.7805) m. |
| Pen release | An orange 32 × 20 × 16 mm knob on a vertical steel shaft at (0.178, −0.045, 0.79225) m, 36 mm above the pivot. The 202 × 8 × 2 mm steel rail runs over the back rim and carries two 16 × 49 × 2 mm retaining tabs. The shaft turns up to 55° counterclockwise when seen from above. A torsional spring returns it to 0° when released. |

Use seed 0 and zero object jitter. The directly overhead camera supplies the 640 × 360 screenshot and reported videos. Dream-1 also receives its wrist-camera observations. The solver uses rigid colliders, a passive spring drive and native contact forces.

## Scoring

- **0** while the lid is less than 95° open or moving faster than 0.15 rad/s.
- **0.5** when the lid is at least 95° open and moving no faster than 0.15 rad/s.
- **1.0** when a coordinated extraction is followed by holding the whole pen at least 30 mm above the case rim, moving at most 40 mm/s, for 0.75 seconds with the lid open. At the moment the pen clears the keeper by crossing 12 mm above the rim, the orange knob must be turned at least 35° and pinched by the arm opposite the one pinching the pen. Each pinch requires two opposing fingertip contacts of at least 0.05 N with force directions at least 120° apart. The knob may return once the pen is clear.

## Run and score

Use the [calibrated YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux with Python 3.12 and an NVIDIA GPU for rendering. The recorded trials used Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, the Isaac Lab-Arena release/0.3.1 checkout (package version 0.3.0), Inspect Robots 0.60.0, Inspect Robots Dream 0.1.0, and Dreamlabs Client 1.0.14. The task uses native CPU PhysX and GPU rendering. Activate the station environment, accept the NVIDIA license and configure the challenge Dream credentials for the model trial. Run each trial in a fresh process from a Git checkout.

```bash
cd tasks/wooden-pencil-case
python -m pip install --no-deps -e ./native
export PYTHONPATH="$PWD/native"
export OMNI_KIT_ACCEPT_EULA=YES

python native/tools/run_recovery_pilot.py scene /tmp/pencil-case-demo \
  --policy recovery --seed 0 --camera top_cam
python native/tools/run_dream.py scene /tmp/pencil-case-dream
```

The run limit is 800 controls at 20 Hz (40 simulated seconds). The scripted demo first scored 1.0 at control 458 (22.9 simulated seconds), then released the knob while keeping the pen clear. Its full 513-control recording took about 105 seconds of wall time including setup. Dream-1 completed all 800 controls at score 0.0 in about 174 seconds of wall time. Both runs used seed 0 and the same scene.

The scripted runner writes the overhead MP4, setup PNG, execution receipt and per-control score and joint targets in `trace.jsonl`. The Dream-1 runner records the overhead MP4 and a receipt that points to the Inspect Robots result and executed action trajectory. Use fresh output directories. To reproduce the locked-pull and early-release controls, select `--policy locked_pull` or `--policy early_release`. Both leave the pen below the rim and score 0.5 for the lid opening. Rebuild the scripted waypoints with `python scene/plan.py`.
