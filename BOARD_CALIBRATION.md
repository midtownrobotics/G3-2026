# Board-camera calibration branch

This branch is for supervised calibration, not match play. `Constants.kBoardCalibrationMode`
is enabled: vision results are **not** fused into localization, the field-layout
pose-reset dashboard command is unavailable, and the pose estimator's field-wall
clamp is disabled. Odometry still uses the swerve encoders **and the gyro**.

## Preparation

- Secure the board; clear a radius around the robot for a full 360-degree turn
  plus its bumpers and mechanisms. Don't let the robot push on the tag board.
- Configure each PhotonVision instance with the measured **board** AprilTag layout
  and a `tag16h5` multitag pipeline at **index 3**. The dashboard chooser defaults
  to index 3 on this branch; verify it is actually configured on every camera. This replaces that instance's global field
  layout; restore the 2026 layout afterward. Select the pipeline on each camera.
- Keep the turret fixed at a known angle (log `Turret/angle`) for the first run.
  Verify each camera sees at least two board tags at the intended distances.
- Place the robot facing the board at the measured starting location. The
  configured positions are relative to the **starting robot center**, not the
  board surface; the routine does not detect the board or reset its pose.
- Confirm the gyro is connected. Test the motion in a clear space before using
  the board. Keep a hand on the controller and be ready to disable the robot.

## Ordered motion plan

Edit `src/main/deploy/board_calibration.json` and redeploy the robot program
before each experiment. The command loads and validates the entire file every
time Xbox controller 0 **Back** is pressed. The `steps` array is executed in
exactly the order written. Only these step types are accepted:

- `{"type":"move", "awayMeters":1.5, "leftMeters":0.35}` drives to an **absolute**
  planar position relative to the starting robot center: positive `awayMeters`
  backs away from the board along the starting heading; positive `leftMeters`
  moves left along the starting robot's left axis. It holds the heading measured
  on entering that move. Limits: away 0–2.75 m; left −1.25–1.25 m.
- `{"type":"rotate", "headingDegrees":30}` turns in place to a heading relative
  to the starting orientation, **not** relative to the previous step. Signed
  headings up to ±720 degrees preserve full turns (e.g. 330 then 360).
- `{"type":"wait", "seconds":5}` stops the wheels and collects stationary data.
  Limits: 0.5–10 seconds. Add a wait after any motion where you want settled
  frames; motion steps do not insert a wait automatically.

The checked-in file is a short example that visits center and ±0.35 m lateral
positions with a couple of headings. It is **not** the old three-distance
360-degree sweep; adjust it to match the cameras' visibility windows and the
space available. Steps, including backward and lateral moves, must be checked
for obstacles. The routine does not perceive the board or reset odometry.
Movement is limited to 0.15 m/s translation and 0.2 rad/s rotation, with
step-specific timeouts. Invalid or missing files stop without driving.

In teleop, **hold Back throughout the run**. Releasing Back, disabling the
robot, losing the gyro, or a motion timeout stops the drive. Start with a short
plan in an empty space; keep a hand on the controller and be ready to disable.
The robot's **starting center** is the position origin, not the board surface.

## Manual driving

The default Xbox joystick drive is scaled to at most **0.15 m/s translation**
(combined X/Y magnitude) and **0.2 rad/s rotation** in calibration mode. The
same caps are enforced in `Drive.runVelocity()` for other commands using that
path, including the scripted plan. The calibration log runs continuously, so
you can manually drive to a pose, stop, and record data. `Calibration/capturing`
marks only scripted `wait` steps; identify manual stationary intervals from
odometry/chassis speeds and discard settling frames. These are commanded-speed
caps, **not** obstacle avoidance or guarantees against wheel slip; direct
module-control characterization paths are not covered. Supervise the robot and
keep the disable control available.

## Logs

On the RoboRIO, AdvantageKit writes WPILOG files (including `/home/lvuser/logs`).
Each `Calibration/<camera>/` update contains aligned arrays of
`timestampsSeconds`, `boardToCamera` (Pose3d in meters/radians), and
`tagIdsUsed` (stringified ID sets). Empty arrays mean no multitag frame in that
loop. The raw poses do **not** apply the existing robot-to-camera guess.
`Calibration/odometryTimestampsSeconds` and `Calibration/odometryPoses` are
likewise paired arrays of encoder/gyro odometry observations. `Calibration/phase`,
`capturing`, `stepIndex`, `unwrappedYawRadians`, and `startPose` mark the
run. `Calibration/plan` records the validated steps used at the start of a run.
Use the camera capture timestamp to interpolate odometry, not the time at
which the array was written. Images are **not** saved by this branch; configure
PhotonVision snapshots separately if corner-level/image analysis is needed.

The board must be surveyed relative to the robot's starting frame to obtain
absolute extrinsics. Wheel slip and gyro/board measurement errors remain; the
stops are not independent ground truth. Validate fitted transforms on separate
poses with lateral offsets and oblique viewpoints. Most fixed cameras will see
only a subset of headings at any one board location; a stationary turret must
be calibrated separately from its pivot/encoder zero.
