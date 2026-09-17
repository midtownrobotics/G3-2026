#!/usr/bin/env python3
"""End-to-end self-test: synthesize a log with KNOWN mounting errors, then check
that solve_offsets.py recovers them.

This is the real correctness check for the whole chain -- frame conventions,
transform composition order, the turret model, and the solver. Sign errors here
are easy to make and nearly impossible to notice from real data, where there is
nothing to check the answer against.

    tools/vision_cal/.venv/bin/python tools/vision_cal/test_solver.py
"""

import math
import os
import struct
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import se3  # noqa: E402
import solve_offsets  # noqa: E402

# Nominal transforms, mirroring Robot.java. The robot code BELIEVES these.
IN = 0.0254
NOMINAL = {
    "Rear": se3.make(
        np.array([-9.394, -12.564, 20.659]) * IN,
        se3.so3_exp([0, 0, math.radians(160)]) @ se3.so3_exp([0, math.radians(-14), 0]),
    ),
    "Right": se3.make(
        np.array([-5.587, -13.648, 20.6695]) * IN,
        se3.so3_exp([0, 0, math.radians(-60)]) @ se3.so3_exp([0, math.radians(-14), 0]),
    ),
    "Left": se3.make(
        np.array([1.054, 14.429, 10.172]) * IN,
        se3.so3_exp([0, 0, math.radians(70)]) @ se3.so3_exp([0, math.radians(-12), 0]),
    ),
}

TURRET_PIVOT = se3.make(np.array([-3.75, 7.25, 13.25]) * IN, np.eye(3))
TURRET_MOUNT = se3.make(
    np.array([6.359, 0.0, 1.792]) * IN, se3.so3_exp([0, math.radians(-24), 0])
)
TURRET_CAM = "Turret"

# The truth the solver has to find. Rear is the gauge reference, so its error is
# deliberately zero -- errors are only ever recoverable relative to the reference.
TRUE_DELTA = {
    "Rear": np.zeros(6),
    "Right": np.array([0.0, 0.0, 0.0, 0.0, 0.0, math.radians(3.0)]),  # 3 deg yaw
    "Left": np.array([0.5 * IN, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 0.5 in along x
    "Turret": np.array([0.0, 0.0, 0.0, 0.0, math.radians(1.5), 0.0]),  # 1.5 deg pitch
}
TRUE_TURRET_ZERO_DEG = 0.75

TRANS_NOISE_M = 0.004
ROT_NOISE_RAD = math.radians(0.12)


# --------------------------------------------------------------------------- #
# Minimal WPILOG writer (exercises datalog.py on a real file)
# --------------------------------------------------------------------------- #

class LogWriter:
    def __init__(self, path):
        self.f = open(path, "wb")
        self.f.write(b"WPILOG" + struct.pack("<HI", 0x0100, 0))
        self.next_entry = 1

    def _str(self, s):
        b = s.encode("utf-8")
        return struct.pack("<I", len(b)) + b

    # 2-byte entry id, 4-byte payload size, 8-byte timestamp: (2-1) | (4-1)<<2 | (8-1)<<4
    _HEADER = 1 | (3 << 2) | (7 << 4)

    def _record(self, entry, timestamp_us, payload):
        self.f.write(bytes([self._HEADER]))
        self.f.write(struct.pack("<HIQ", entry, len(payload), timestamp_us))
        self.f.write(payload)

    def start(self, name, typ):
        entry = self.next_entry
        self.next_entry += 1
        payload = (
            bytes([0])
            + struct.pack("<I", entry)
            + self._str(name)
            + self._str(typ)
            + self._str("")
        )
        self._record(0, 0, payload)
        return entry

    def write(self, entry, timestamp_us, payload):
        self._record(entry, timestamp_us, payload)

    def close(self):
        self.f.close()


def pose_payload(matrix):
    q = se3.matrix_to_quat(matrix[:3, :3])
    return struct.pack("<7d", *matrix[:3, 3], *q)


# --------------------------------------------------------------------------- #

def true_nominal(cam, turret_deg, turret_zero_deg):
    """The transform the robot code believes, for this capture."""
    if cam == TURRET_CAM:
        theta = math.radians(turret_deg + turret_zero_deg)
        return TURRET_PIVOT @ se3.make(np.zeros(3), se3.rot_z(theta)) @ TURRET_MOUNT
    return NOMINAL[cam]


def build_log(path, rng):
    cams = list(NOMINAL) + [TURRET_CAM]

    writer = LogWriter(path)
    prefix = "NT:/AdvantageKit/RealOutputs/"
    entries = {
        "captureActive": writer.start(prefix + "VisionCal/captureActive", "boolean"),
        "stationId": writer.start(prefix + "VisionCal/stationId", "int64"),
        "captureId": writer.start(prefix + "VisionCal/captureId", "int64"),
        "turretAngleDeg": writer.start(prefix + "VisionCal/turretAngleDeg", "double"),
        "turretName": writer.start(prefix + "VisionCal/TurretModel/cameraName", "string"),
        "turretPivot": writer.start(
            prefix + "VisionCal/TurretModel/robotToTurretPivot", "struct:Pose3d"
        ),
        "turretMount": writer.start(
            prefix + "VisionCal/TurretModel/turretToCamera", "struct:Pose3d"
        ),
    }
    for cam in cams:
        entries[f"{cam}/pose"] = writer.start(
            prefix + f"VisionCal/{cam}/fieldToCameraPose", "struct:Pose3d"
        )
        entries[f"{cam}/nominal"] = writer.start(
            prefix + f"VisionCal/{cam}/robotToCameraNominal", "struct:Pose3d"
        )

    # A "string" entry's payload is raw UTF-8; only the names inside CONTROL records
    # carry a length prefix.
    writer.write(entries["turretName"], 0, TURRET_CAM.encode())
    writer.write(entries["turretPivot"], 0, pose_payload(TURRET_PIVOT))
    writer.write(entries["turretMount"], 0, pose_payload(TURRET_MOUNT))

    # Stations at a spread of distances from the wall (x = 0), which is what makes a
    # rotation error separable from a translation error.
    stations = []
    for distance in (1.5, 2.2, 3.0, 3.8, 4.6, 5.4):
        for lateral, heading in ((0.0, 180.0), (1.2, 195.0), (-1.0, 165.0)):
            stations.append((distance, 3.0 + lateral, math.radians(heading)))

    t_us = 1_000_000
    for station_id, (x, y, heading) in enumerate(stations):
        robot = se3.make([x, y, 0.0], se3.rot_z(heading))
        for turret_deg in (-40.0, 0.0, 55.0):
            writer.write(entries["stationId"], t_us, station_id.to_bytes(8, "little", signed=True))
            writer.write(entries["turretAngleDeg"], t_us, struct.pack("<d", turret_deg))
            writer.write(entries["captureActive"], t_us, bytes([1]))
            t_us += 10_000

            for frame in range(8):
                for cam in cams:
                    nominal = true_nominal(cam, turret_deg, 0.0)
                    writer.write(entries[f"{cam}/nominal"], t_us, pose_payload(nominal))
                    # Ground truth: the real mounting differs from the nominal by TRUE_DELTA,
                    # and the reported turret angle is off by TRUE_TURRET_ZERO_DEG.
                    actual = true_nominal(cam, turret_deg, TRUE_TURRET_ZERO_DEG) @ se3.exp(
                        TRUE_DELTA[cam]
                    )
                    measured = robot @ actual
                    noise = np.concatenate(
                        [
                            rng.normal(0, TRANS_NOISE_M, 3),
                            rng.normal(0, ROT_NOISE_RAD, 3),
                        ]
                    )
                    writer.write(
                        entries[f"{cam}/pose"], t_us, pose_payload(measured @ se3.exp(noise))
                    )
                t_us += 5_000

            writer.write(entries["captureActive"], t_us, bytes([0]))
            t_us += 10_000

    writer.close()
    return len(stations)


def main():
    rng = np.random.default_rng(20260914)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "synthetic.wpilog")
        n_stations = build_log(path, rng)
        print(f"synthesized {n_stations} stations x 3 turret angles -> {path}")
        print("\ninjected ground truth (relative to Rear):")
        for cam, delta in TRUE_DELTA.items():
            t = np.array(delta[:3]) / IN
            r = np.degrees(delta[3:])
            print(
                f"  {cam:<8} dx/dy/dz = {t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f} in   "
                f"droll/dpitch/dyaw = {r[0]:+.3f} {r[1]:+.3f} {r[2]:+.3f} deg"
            )
        print(f"  turret zero error = {TRUE_TURRET_ZERO_DEG:+.3f} deg")
        print("\n" + "-" * 78)

        problem, params, cov, schur, before, after = solve_offsets.solve(
            path, reference="Rear"
        )
        solve_offsets.report_fit(before, after)
        solve_offsets.report_corrections(problem, params, cov)
        solve_offsets.report_observability(problem, schur)
        solve_offsets.report_java(problem, params)

        return check(problem, params)


# Tolerances: generous enough not to be flaky, tight enough that a sign error or a
# swapped composition order cannot pass.
TRANS_TOL_IN = 0.05
ROT_TOL_DEG = 0.05


def check(problem, params):
    failures = []

    def expect(label, actual, expected, tol, unit):
        ok = abs(actual - expected) <= tol
        status = "ok  " if ok else "FAIL"
        print(f"  [{status}] {label:<22} got {actual:+8.3f} {unit}, want {expected:+8.3f}")
        if not ok:
            failures.append(label)

    print("=" * 78)
    print("SELF-TEST: recovered vs. injected")
    print("=" * 78)

    for cam in ("Left", "Right"):
        truth = TRUE_DELTA[cam]
        got = problem.delta(params, cam)
        for k in range(3):
            expect(f"{cam}.d{solve_offsets.DOF_NAMES[k]}", got[k] / IN, truth[k] / IN,
                   TRANS_TOL_IN, "in")
        for k in range(3, 6):
            expect(f"{cam}.d{solve_offsets.DOF_NAMES[k]}", math.degrees(got[k]),
                   math.degrees(truth[k]), ROT_TOL_DEG, "deg")

    # The turret camera's mount error and the turret zero error are provably
    # inseparable, so the only checkable quantities are the zero error implied by the
    # projection and the mount error left after removing it.
    d = problem.delta(params, TURRET_CAM)
    g = solve_offsets.turret_zero_direction(problem.turret["mount"])
    w = np.array([problem.trans_w] * 3 + [problem.rot_w] * 3)
    eps = float((g * w * w) @ d / ((g * w * w) @ g))
    leftover = d - eps * g
    expect("turret zero", math.degrees(eps), TRUE_TURRET_ZERO_DEG, ROT_TOL_DEG, "deg")
    for k in range(3, 6):
        expect(f"Turret.leftover.d{solve_offsets.DOF_NAMES[k]}", math.degrees(leftover[k]),
               math.degrees(TRUE_DELTA[TURRET_CAM][k]), ROT_TOL_DEG, "deg")

    print()
    if failures:
        print(f"FAILED ({len(failures)}): {', '.join(failures)}")
        return 1
    print("PASS: solver recovered every injected error.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
