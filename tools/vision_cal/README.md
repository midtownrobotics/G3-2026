# Camera offset characterization

Finds out whether the `robotToCamera` transforms in `Robot.java` are wrong, and in
which direction — using a wall of AprilTags at known relative positions and **no
objective pose reference**.

## How it works

Every camera independently produces a field→camera pose. Each one implies a robot
pose through its own mounting transform. If all the transforms were right, every
camera would imply the *same* robot pose at the same instant. They don't, and the
difference is a direct measurement of the offset error.

The solver fits, jointly:

- one unknown robot pose per station (where the robot was parked), and
- one 6-DOF correction per camera,

so that `measured[c] ≈ X_station · nominal[c] · exp(δ_c)` holds for every camera in
every capture.

### What this cannot tell you

- **Only relative corrections are observable.** One camera is held fixed as the
  gauge reference; everything is reported relative to it. An error shared by all
  cameras — including a mis-measured tag wall — is invisible here.
- **A turret zero error and a turret camera mount error are provably the same
  thing** to this data, at every turret angle. The report gives both readings of the
  same number and you pick. Zeroing the turret against a hard stop is how you
  actually settle it.
- **The fit will absorb camera intrinsics errors into "offsets."** That is what the
  before/after residual is for: if disagreement doesn't collapse, the mounting
  transforms were not your problem.

---

## 1. Measure the wall

Edit [`tag_wall.json`](tag_wall.json). Pick one tag as the origin, and give every
tag's position relative to it:

- `y` — left along the wall, as seen by a robot facing it
- `z` — height above the floor, to the **center** of the tag

Then generate the layout:

```bash
python3 tools/vision_cal/make_layout.py
```

This writes `src/main/deploy/apriltag/tagwall.json`.

> **Upload that same file to every PhotonVision coprocessor** (Settings → AprilTag
> Field Layout), all three of them. The coprocessor runs the multi-tag solve against
> *its own* copy of the layout — the RIO's copy is only used for logging. Skipping
> this is the single easiest way to get confident, meaningless numbers out of this
> whole procedure.

## 2. Set up the robot

On the dashboard:

- `Toggles/TagWallMode` → **true** (resolves tags against the wall, not the field)
- `Toggles/UseVisionObservations` → **false** (keeps wall poses out of the pose estimator)

Press `TuningModes/VisionCal/ResetStations`.

## 3. Collect

At each station:

1. Park the robot facing the wall and let it come to a complete stop.
2. Press `TuningModes/VisionCal/Capture` — it records a 2-second window.
3. Rotate the turret, press `Capture` again. Repeat for ~3 turret angles.
4. Press `TuningModes/VisionCal/NextStation`, then physically move the robot.

Requirements, in rough order of how much they matter:

- **Vary the distance to the wall.** Roughly 1.5 m, 3 m, and 5 m at minimum. A
  mounting *rotation* error produces a pose error that grows with range; a mounting
  *translation* error produces a constant one. At a single distance the two are
  indistinguishable and the solver will report a confident wrong answer. It warns
  when your range spread is under 1.5 m.
- **At least 6 stations**, ideally 10, with lateral and heading variation too.
- **Every camera you care about must see ≥2 tags** in the same capture as the
  others. Cameras that never overlap cannot be tied together at all.
- **Hold still.** Latency error also produces camera disagreement, but it scales
  with velocity and would be misread as a mounting error.

## 4. Live check (optional, but do it first)

While parked, watch `VisionCal/Pairs/<A>-<B>/` in AdvantageScope:

- `mean/dTranslationNormInches` — systematic bias, i.e. real offset error
- `stdDev/*` — per-frame noise

**A mean well above the stddev on some pair is the signal that a transform is
genuinely wrong** and the full capture is worth doing. If every mean is buried in
its own noise, your offsets are already fine.

`dTranslationNormInches` is also logged ungated next to `chassisSpeedMetersPerSec`:
disagreement that grows with speed is a latency problem, not a mounting problem.

## 5. Solve

Pull the log off the RIO (`/home/lvuser/logs`), then:

```bash
python3 -m venv tools/vision_cal/.venv
tools/vision_cal/.venv/bin/pip install -r tools/vision_cal/requirements.txt
tools/vision_cal/.venv/bin/python tools/vision_cal/solve_offsets.py path/to/log.wpilog
```

Useful flags: `--reference Rear` to pin the gauge camera, `--trans-sigma` /
`--rot-sigma` to match your actual measurement noise (these set the reported σ).

The report has four parts:

1. **Fit quality** — pairwise disagreement before vs. after. If it doesn't drop a
   lot, stop; the mounting transforms aren't the issue.
2. **Per-camera corrections** with 1σ. *A correction smaller than its own σ is not
   evidence of anything* — don't chase it.
3. **Observability** — which DOFs your station set genuinely constrained, and what
   to collect more of if some are degenerate.
4. **Corrected transforms**, ready to paste into `Robot.java`.

## 6. Confirm

Apply the corrections, re-run a capture, and check that the `VisionCal/Pairs/*`
means drop toward zero. Then set `Toggles/TagWallMode` back to false.

---

## Self-test

`test_solver.py` synthesizes a log with known mounting errors injected and asserts
that the solver recovers them. It validates frame conventions, composition order,
the turret model, and the fit — none of which real data can check, since real data
has no answer key.

```bash
tools/vision_cal/.venv/bin/python tools/vision_cal/test_solver.py
```

Run it after touching anything in `se3.py` or `solve_offsets.py`.

## Files

| file | role |
| --- | --- |
| `tag_wall.json` | your measurements of the wall — **edit this** |
| `make_layout.py` | → `src/main/deploy/apriltag/tagwall.json` |
| `solve_offsets.py` | the solver and report |
| `se3.py` | SE(3) math (exp/log, quaternion averaging) |
| `datalog.py` | minimal `.wpilog` reader |
| `test_solver.py` | end-to-end self-test against injected ground truth |

Robot-side pieces: `sensors/VisionCalibration.java` (live disagreement + capture
labelling), `sensors/Camera.java` (logs the raw `fieldToCameraPose`),
`constants/FieldConstants.java` (`Toggles/TagWallMode`).

The solver reads the nominal transforms out of the log rather than keeping its own
copy, so it cannot drift out of sync with `Robot.java`.
