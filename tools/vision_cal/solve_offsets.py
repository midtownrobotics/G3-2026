#!/usr/bin/env python3
"""Solve per-camera mounting-offset corrections from inter-camera disagreement.

There is no ground truth for robot pose, so this works purely off the fact that
every camera should imply the SAME robot pose at the same instant. For each
capture the robot pose is an unknown; each camera's measurement must be explained
by that shared pose composed with its own (possibly wrong) mounting transform.

    measured_field_to_camera[c]  ~=  X_station @ nominal_robot_to_camera[c] @ exp(delta_c)

We fit the X's and the delta's jointly by nonlinear least squares. One camera is
held fixed as the gauge reference -- only RELATIVE corrections are observable, so
an error common to every camera (or a mis-measured tag wall) cannot be seen here
and will not show up in the results.

Usage:
    python3 tools/vision_cal/solve_offsets.py path/to/log.wpilog
    python3 tools/vision_cal/solve_offsets.py log.wpilog --reference Rear
"""

import argparse
import math
import sys
from collections import defaultdict

try:
    import numpy as np
    from scipy.optimize import least_squares
except ImportError:
    sys.exit(
        "numpy and scipy are required.\n"
        "  python3 -m venv tools/vision_cal/.venv\n"
        "  tools/vision_cal/.venv/bin/pip install -r tools/vision_cal/requirements.txt\n"
        "  tools/vision_cal/.venv/bin/python tools/vision_cal/solve_offsets.py <log>"
    )

import datalog
import se3

M_TO_IN = 1 / 0.0254

# Per-measurement noise used to weight translation against rotation in the cost.
# These only set the relative scaling and the absolute size of reported sigmas.
DEFAULT_TRANS_SIGMA_M = 0.02
DEFAULT_ROT_SIGMA_RAD = math.radians(0.6)

DOF_NAMES = ["x", "y", "z", "roll", "pitch", "yaw"]


# --------------------------------------------------------------------------- #
# Log extraction
# --------------------------------------------------------------------------- #

def _strip(name):
    """Reduce an AdvantageKit log key to its VisionCal-relative path."""
    idx = name.find("VisionCal/")
    return name[idx:] if idx >= 0 else None


def _sample_at(series, t):
    """Value of a step-function series at time t (most recent sample at or before t)."""
    lo, hi = 0, len(series) - 1
    if not series or series[0][0] > t:
        return None
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return series[lo][1]


def load_log(path):
    raw = datalog.read_entries(path, name_filter=lambda n: "VisionCal/" in n)
    entries = {}
    for name, (typ, samples) in raw.items():
        key = _strip(name)
        if key and samples:
            entries[key] = samples
    if not entries:
        sys.exit(
            f"no VisionCal/* entries found in {path}.\n"
            "Was the robot code with VisionCalibration deployed when this log was taken?"
        )
    return entries


def find_capture_windows(entries):
    """Time windows where captureActive was true, with their station id."""
    active = entries.get("VisionCal/captureActive")
    if not active:
        sys.exit("log contains no VisionCal/captureActive -- no captures were taken.")

    station_series = entries.get("VisionCal/stationId", [(0.0, 0)])
    turret_series = entries.get("VisionCal/turretAngleDeg", [(0.0, 0.0)])
    capture_series = entries.get("VisionCal/captureId", [(0.0, 0)])

    windows = []
    start = None
    for t, value in active:
        if value and start is None:
            start = t
        elif not value and start is not None:
            mid = (start + t) / 2.0
            windows.append(
                {
                    "t0": start,
                    "t1": t,
                    "station": _sample_at(station_series, mid),
                    "turret_deg": _sample_at(turret_series, mid),
                    "capture": _sample_at(capture_series, mid),
                }
            )
            start = None
    return windows


def camera_names(entries):
    names = set()
    for key in entries:
        parts = key.split("/")
        if len(parts) == 3 and parts[2] == "fieldToCameraPose":
            names.add(parts[1])
    return sorted(names)


def build_captures(entries, min_samples):
    """One record per capture window, holding each camera's averaged measurement."""
    windows = find_capture_windows(entries)
    cams = camera_names(entries)
    if not cams:
        sys.exit("log contains no VisionCal/<camera>/fieldToCameraPose entries.")

    captures = []
    dropped = defaultdict(int)
    for window in windows:
        measurements = {}
        nominals = {}
        for cam in cams:
            poses = entries.get(f"VisionCal/{cam}/fieldToCameraPose", [])
            in_window = [
                se3.pose_to_matrix(v) for t, v in poses if window["t0"] <= t <= window["t1"]
            ]
            if len(in_window) < min_samples:
                if in_window:
                    dropped[cam] += 1
                continue
            measurements[cam] = se3.average_poses(in_window)

            nominal_series = entries.get(f"VisionCal/{cam}/robotToCameraNominal", [])
            nominal = _sample_at(nominal_series, window["t1"])
            if nominal is None:
                sys.exit(f"missing VisionCal/{cam}/robotToCameraNominal in log")
            nominals[cam] = se3.pose_to_matrix(nominal)

        if len(measurements) >= 2:
            captures.append({**window, "measurements": measurements, "nominals": nominals})

    if dropped:
        for cam, n in sorted(dropped.items()):
            print(
                f"  note: {cam} had fewer than {min_samples} frames in {n} capture(s); "
                "those were skipped for that camera."
            )
    return captures, cams


def load_turret_model(entries):
    names = entries.get("VisionCal/TurretModel/cameraName")
    pivot = entries.get("VisionCal/TurretModel/robotToTurretPivot")
    mount = entries.get("VisionCal/TurretModel/turretToCamera")
    if not (names and pivot and mount):
        return None
    return {
        "name": names[-1][1],
        "pivot": se3.pose_to_matrix(pivot[-1][1]),
        "mount": se3.pose_to_matrix(mount[-1][1]),
    }


# --------------------------------------------------------------------------- #
# Model
# --------------------------------------------------------------------------- #

class Problem:
    """Parameter packing and the residual function."""

    def __init__(self, captures, cams, reference, turret, trans_sigma, rot_sigma):
        self.captures = captures
        self.cams = cams
        self.reference = reference
        self.turret = turret
        self.trans_w = 1.0 / trans_sigma
        self.rot_w = 1.0 / rot_sigma

        self.stations = sorted({c["station"] for c in captures})
        self.station_index = {s: i for i, s in enumerate(self.stations)}
        # The reference camera's delta is held at zero to fix the gauge freedom.
        self.free_cams = [c for c in cams if c != reference]
        self.cam_index = {c: i for i, c in enumerate(self.free_cams)}

        self.n_station = len(self.stations)
        self.n_cam = len(self.free_cams)
        self.n_params = 6 * self.n_station + 6 * self.n_cam

    # -- parameter access --------------------------------------------------- #

    def station_pose(self, params, station):
        i = self.station_index[station]
        return se3.exp(params[6 * i : 6 * i + 6])

    def delta(self, params, cam):
        if cam == self.reference:
            return np.zeros(6)
        i = self.cam_index[cam]
        base = 6 * self.n_station + 6 * i
        return params[base : base + 6]

    # -- model -------------------------------------------------------------- #

    def nominal(self, capture, cam):
        """Nominal robot -> camera for this capture, as the robot code believed it.

        Taken straight from the log. For the turret camera this already includes the
        turret angle at capture time, since DynamicCamera refreshes its transform
        every loop.
        """
        return capture["nominals"][cam]

    def predict(self, params, capture, cam):
        station = self.station_pose(params, capture["station"])
        return station @ self.nominal(capture, cam) @ se3.exp(self.delta(params, cam))

    def residuals(self, params):
        out = []
        for capture in self.captures:
            for cam, measured in capture["measurements"].items():
                err = se3.log(se3.inv(self.predict(params, capture, cam)) @ measured)
                out.append(err[:3] * self.trans_w)
                out.append(err[3:] * self.rot_w)
        return np.concatenate(out)

    def initial_guess(self):
        params = np.zeros(self.n_params)
        for station in self.stations:
            # Seed each station pose from whichever camera in that capture is closest to
            # the gauge reference, so the starting point is already near the solution.
            capture = next(c for c in self.captures if c["station"] == station)
            cam = self.reference if self.reference in capture["measurements"] else next(
                iter(capture["measurements"])
            )
            pose = capture["measurements"][cam] @ se3.inv(self.nominal(capture, cam))
            i = self.station_index[station]
            params[6 * i : 6 * i + 6] = se3.log(pose)
        return params


# --------------------------------------------------------------------------- #
# Diagnostics
# --------------------------------------------------------------------------- #

def pairwise_disagreement(problem, params=None):
    """Per-pair translation/rotation disagreement in implied robot pose.

    With params=None the nominal (uncorrected) transforms are used, which is the
    'before' number.
    """
    stats = defaultdict(lambda: {"trans": [], "rot": []})
    for capture in problem.captures:
        implied = {}
        for cam, measured in capture["measurements"].items():
            nominal = problem.nominal(capture, cam)
            correction = se3.exp(problem.delta(params, cam)) if params is not None else np.eye(4)
            implied[cam] = measured @ se3.inv(nominal @ correction)
        names = sorted(implied)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                d = se3.inv(implied[a]) @ implied[b]
                stats[(a, b)]["trans"].append(np.linalg.norm(d[:3, 3]))
                stats[(a, b)]["rot"].append(np.linalg.norm(se3.so3_log(d[:3, :3])))
    return stats


def rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else float("nan")


def delta_covariance(problem, jac, residuals):
    """Covariance of the camera-correction block, via Schur complement.

    Marginalizing out the station poses is what makes these sigmas honest: a camera
    correction that is only weakly separable from 'the robot was somewhere else'
    should come out with a large sigma, not a small one.
    """
    n_s = 6 * problem.n_station
    j_x, j_d = jac[:, :n_s], jac[:, n_s:]
    if j_d.shape[1] == 0:
        return None, None

    hxx = j_x.T @ j_x
    hxd = j_x.T @ j_d
    hdd = j_d.T @ j_d
    # Tikhonov term keeps the solve finite if a station is seen by one camera only.
    schur = hdd - hxd.T @ np.linalg.solve(hxx + 1e-12 * np.eye(hxx.shape[0]), hxd)

    dof = max(len(residuals) - problem.n_params, 1)
    scale = float(residuals @ residuals) / dof  # residuals are already sigma-normalized
    cov = scale * np.linalg.pinv(schur)
    return cov, schur


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #

def report_corrections(problem, params, cov):
    print("\n" + "=" * 78)
    print("PER-CAMERA CORRECTIONS  (relative to reference camera "
          f"'{problem.reference}')")
    print("=" * 78)
    print("A correction smaller than its own sigma is not evidence of anything.\n")

    n_s = 6 * problem.n_station
    for cam in problem.cams:
        if cam == problem.reference:
            print(f"{cam}: reference (held fixed by definition)")
            continue
        d = problem.delta(params, cam)
        i = problem.cam_index[cam]
        block = cov[6 * i : 6 * i + 6, 6 * i : 6 * i + 6] if cov is not None else None
        sigma = np.sqrt(np.clip(np.diag(block), 0, None)) if block is not None else np.zeros(6)

        print(f"{cam}:")
        for k in range(3):
            val, sig = d[k] * M_TO_IN, sigma[k] * M_TO_IN
            flag = "" if abs(val) > sig else "   (within noise)"
            print(f"  d{DOF_NAMES[k]:<6} {val:+8.3f} in   +/- {sig:6.3f}{flag}")
        for k in range(3, 6):
            val, sig = math.degrees(d[k]), math.degrees(sigma[k])
            flag = "" if abs(val) > sig else "   (within noise)"
            print(f"  d{DOF_NAMES[k]:<6} {val:+8.3f} deg  +/- {sig:6.3f}{flag}")
        print()

    report_turret_ambiguity(problem, params)


def turret_zero_direction(mount):
    """How a small turret-angle-zero error appears as a mount correction.

    A turret zero error eps gives  P @ Rz(theta + eps) @ M  ==  P @ Rz(theta) @ M @ exp(d),
    with d = log(M^-1 @ Rz(eps) @ M). Because that rearrangement holds for EVERY theta,
    a turret zero error and a turret camera mount error are exactly the same thing as
    far as this data is concerned -- collecting more turret angles does not separate
    them. This returns d/d(eps), so the two readings can at least be reported.
    """
    h = 1e-6
    d = se3.log(se3.inv(mount) @ se3.make(np.zeros(3), se3.rot_z(h)) @ mount)
    return d / h


def report_turret_ambiguity(problem, params):
    if problem.turret is None or problem.turret["name"] not in problem.cams:
        return
    name = problem.turret["name"]
    if name == problem.reference:
        return

    d = problem.delta(params, name)
    g = turret_zero_direction(problem.turret["mount"])
    w = np.array([problem.trans_w] * 3 + [problem.rot_w] * 3)
    eps = float((g * w * w) @ d / ((g * w * w) @ g))
    leftover = d - eps * g

    print("-" * 78)
    print(f"Turret camera ({name}) -- zero error vs. mount error are NOT separable")
    print("-" * 78)
    print("A turret angle zero error and a turret camera mounting error produce identical")
    print("measurements at every turret angle, so vision alone cannot tell them apart.")
    print("The correction above is the mount interpretation. The equivalent reading is:\n")
    print(f"  turret zero error   {math.degrees(eps):+8.3f} deg")
    print(f"  residual mount error left over after removing it:")
    for k in range(3):
        print(f"    d{DOF_NAMES[k]:<6} {leftover[k] * M_TO_IN:+8.3f} in")
    for k in range(3, 6):
        print(f"    d{DOF_NAMES[k]:<6} {math.degrees(leftover[k]):+8.3f} deg")
    print("\nIf the leftover is small, a turret zero adjustment alone explains it -- prefer")
    print("that, since it also fixes aiming. Otherwise fix the mount transform.")
    print("To decide for real, zero the turret against a hard stop and re-measure.\n")


def report_java(problem, params):
    print("=" * 78)
    print("CORRECTED TRANSFORMS  (paste into Robot.java)")
    print("=" * 78)
    for cam in problem.cams:
        if problem.turret is not None and cam == problem.turret["name"]:
            continue
        capture = next(c for c in problem.captures if cam in c["nominals"])
        corrected = capture["nominals"][cam] @ se3.exp(problem.delta(params, cam))
        t = corrected[:3, 3] * M_TO_IN
        roll, pitch, yaw = se3.rpy_degrees(corrected[:3, :3])
        print(f"""
    Camera {cam.lower()}Camera = new Camera(
        "{cam}",
        new Transform3d(
            new Translation3d(Inches.of({t[0]:.3f}), Inches.of({t[1]:.3f}), Inches.of({t[2]:.3f})),
            new Rotation3d(Degrees.of({roll:.3f}), Degrees.of({pitch:.3f}), Degrees.of({yaw:.3f}))));"""
              )

    if problem.turret is not None and problem.turret["name"] in problem.cams:
        corrected = problem.turret["mount"] @ se3.exp(problem.delta(params, problem.turret["name"]))
        t = corrected[:3, 3] * M_TO_IN
        roll, pitch, yaw = se3.rpy_degrees(corrected[:3, :3])
        print(f"""
    // Constants.kTurretToCamera
    public static final Transform3d kTurretToCamera = new Transform3d(
        Inches.of({t[0]:.3f}), Inches.of({t[1]:.3f}), Inches.of({t[2]:.3f}),
        new Rotation3d(Degrees.of({roll:.3f}), Degrees.of({pitch:.3f}), Degrees.of({yaw:.3f})));"""
              )
    print()


def report_observability(problem, schur):
    print("=" * 78)
    print("OBSERVABILITY")
    print("=" * 78)
    if schur is None:
        print("Nothing to report: no free cameras.\n")
        return

    distances = []
    for capture in problem.captures:
        for measured in capture["measurements"].values():
            distances.append(abs(measured[0, 3]))  # wall plane is x = 0
    spread = (max(distances) - min(distances)) if distances else 0.0

    print(f"Stations: {problem.n_station}    captures: {len(problem.captures)}")
    print(f"Range to wall: {min(distances):.2f} .. {max(distances):.2f} m (spread {spread:.2f} m)")
    if spread < 1.5:
        print("  WARNING: range spread under 1.5 m. A mounting rotation error and a mounting")
        print("  translation error look nearly identical at a single distance -- collect more")
        print("  stations, close to the wall AND far from it, or these numbers will be")
        print("  confidently wrong.")
    if problem.n_station < 4:
        print(f"  WARNING: only {problem.n_station} station(s); 6 or more is recommended.")

    eigvals, eigvecs = np.linalg.eigh(schur)
    worst = max(eigvals.max(), 1e-30)
    print("\nWeakest constrained directions:")
    for idx in range(min(3, len(eigvals))):
        ratio = eigvals[idx] / worst
        vec = eigvecs[:, idx]
        contributors = np.argsort(-np.abs(vec))[:3]
        parts = []
        for c in contributors:
            cam = problem.free_cams[c // 6]
            parts.append(f"{cam}.{DOF_NAMES[c % 6]} ({vec[c]:+.2f})")
        marker = "  <-- effectively unobservable" if ratio < 1e-6 else ""
        print(f"  rel. strength {ratio:.2e}: {', '.join(parts)}{marker}")
    print()


def report_fit(before, after):
    print("=" * 78)
    print("FIT QUALITY")
    print("=" * 78)
    print("If disagreement does not drop substantially, the mounting transforms are not")
    print("the problem -- suspect camera intrinsics or the tag wall measurements.\n")
    print(f"{'pair':<22}{'translation (in)':>24}{'rotation (deg)':>22}")
    print(f"{'':<22}{'before':>11}{'after':>13}{'before':>11}{'after':>11}")
    all_before_t, all_after_t = [], []
    for pair in sorted(before):
        bt = rms(before[pair]["trans"]) * M_TO_IN
        at = rms(after[pair]["trans"]) * M_TO_IN
        br = math.degrees(rms(before[pair]["rot"]))
        ar = math.degrees(rms(after[pair]["rot"]))
        all_before_t.extend(before[pair]["trans"])
        all_after_t.extend(after[pair]["trans"])
        print(f"{pair[0] + '-' + pair[1]:<22}{bt:>11.3f}{at:>13.3f}{br:>11.3f}{ar:>11.3f}")
    print("-" * 68)
    print(
        f"{'OVERALL RMS':<22}{rms(all_before_t) * M_TO_IN:>11.3f}"
        f"{rms(all_after_t) * M_TO_IN:>13.3f}"
    )
    print()


# --------------------------------------------------------------------------- #

def solve(
    log_path,
    reference=None,
    min_samples=3,
    trans_sigma=DEFAULT_TRANS_SIGMA_M,
    rot_sigma=DEFAULT_ROT_SIGMA_RAD,
):
    """Run the full solve. Returns (problem, params, cov, schur, before, after)."""
    entries = load_log(log_path)
    turret = load_turret_model(entries)
    captures, cams = build_captures(entries, min_samples)

    if not captures:
        sys.exit(
            "no usable captures: every window had fewer than 2 cameras with a multi-tag solve.\n"
            "Cameras can only be tied together when they see the wall at the same time."
        )

    counts = defaultdict(int)
    for capture in captures:
        for cam in capture["measurements"]:
            counts[cam] += 1

    print(f"Loaded {len(captures)} capture(s) across "
          f"{len({c['station'] for c in captures})} station(s)")
    for cam in cams:
        marker = "  [turret]" if turret and cam == turret["name"] else ""
        print(f"  {cam:<10} {counts[cam]:>3} capture(s){marker}")

    if reference is None:
        fixed = [c for c in cams if not (turret and c == turret["name"])] or cams
        reference = max(fixed, key=lambda c: counts[c])
    if reference not in cams:
        sys.exit(f"reference camera {reference!r} is not in the log (have: {', '.join(cams)})")
    if turret and reference == turret["name"]:
        print("\nWARNING: using the turret camera as the reference makes every other camera's")
        print("correction depend on the turret angle being right. Prefer a fixed camera.")

    problem = Problem(captures, cams, reference, turret, trans_sigma, rot_sigma)

    before = pairwise_disagreement(problem, None)

    result = least_squares(
        problem.residuals,
        problem.initial_guess(),
        loss="soft_l1",  # one bad capture should not steer the whole fit
        f_scale=3.0,
        xtol=1e-12,
        ftol=1e-12,
    )
    if not result.success:
        print(f"\nWARNING: optimizer reported failure: {result.message}")

    after = pairwise_disagreement(problem, result.x)
    cov, schur = delta_covariance(problem, result.jac, result.fun)
    return problem, result.x, cov, schur, before, after


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("log", help="path to a .wpilog captured during the calibration run")
    parser.add_argument(
        "--reference",
        help="camera held fixed as the gauge reference "
        "(default: the fixed camera present in the most captures)",
    )
    parser.add_argument("--min-samples", type=int, default=3,
                        help="minimum frames a camera needs in a window to be used (default 3)")
    parser.add_argument("--trans-sigma", type=float, default=DEFAULT_TRANS_SIGMA_M,
                        help="per-measurement translation noise, meters (default %(default)s)")
    parser.add_argument("--rot-sigma", type=float, default=math.degrees(DEFAULT_ROT_SIGMA_RAD),
                        help="per-measurement rotation noise, degrees (default %(default).2f)")
    args = parser.parse_args()

    problem, params, cov, schur, before, after = solve(
        args.log,
        reference=args.reference,
        min_samples=args.min_samples,
        trans_sigma=args.trans_sigma,
        rot_sigma=math.radians(args.rot_sigma),
    )

    report_fit(before, after)
    report_corrections(problem, params, cov)
    report_observability(problem, schur)
    report_java(problem, params)


if __name__ == "__main__":
    main()
