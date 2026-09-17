#!/usr/bin/env python3
"""Convert tag_wall.json (human-measured tag positions) into a WPILib
AprilTagFieldLayout JSON.

Output goes to src/main/deploy/apriltag/tagwall.json, which is deployed to the
roboRIO automatically. The SAME file must also be uploaded to every PhotonVision
coprocessor (Settings -> AprilTag Field Layout), because the coprocessor runs
multi-tag PnP against its own copy of the layout. Skipping that step produces
plausible-looking but meaningless poses.

Usage:
    python3 tools/vision_cal/make_layout.py [tag_wall.json] [-o output.json]

No third-party dependencies.
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

DEFAULT_INPUT = os.path.join(HERE, "tag_wall.json")
DEFAULT_OUTPUT = os.path.join(REPO, "src", "main", "deploy", "apriltag", "tagwall.json")

UNIT_TO_METERS = {
    "inches": 0.0254,
    "inch": 0.0254,
    "in": 0.0254,
    "meters": 1.0,
    "meter": 1.0,
    "m": 1.0,
    "millimeters": 0.001,
    "mm": 0.001,
    "centimeters": 0.01,
    "cm": 0.01,
    "feet": 0.3048,
    "ft": 0.3048,
}

# Margin kept between the wall and the edge of the synthetic "field", so the
# whole thing renders sanely in AdvantageScope.
Y_MARGIN_METERS = 1.0
FIELD_LENGTH_METERS = 16.54  # room in +X for the robot to back away from the wall
FIELD_WIDTH_PAD_METERS = 2.0


def quaternion_from_pitch_yaw(pitch_rad, yaw_rad):
    """WPILib Rotation3d(roll=0, pitch, yaw) as a (W, X, Y, Z) quaternion.

    WPILib composes intrinsic Z-Y-X, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll).
    With roll = 0 the half-angle product collapses to the form below.
    """
    cy, sy = math.cos(yaw_rad / 2.0), math.sin(yaw_rad / 2.0)
    cp, sp = math.cos(pitch_rad / 2.0), math.sin(pitch_rad / 2.0)
    return (cy * cp, -sy * sp, cy * sp, sy * cp)


def load_config(path):
    with open(path) as f:
        cfg = json.load(f)

    units = cfg.get("units", "inches").lower()
    if units not in UNIT_TO_METERS:
        raise SystemExit(
            f"unknown units {units!r}; expected one of {sorted(set(UNIT_TO_METERS))}"
        )
    scale = UNIT_TO_METERS[units]

    tags = cfg.get("tags")
    if not tags:
        raise SystemExit(f"{path} has no 'tags' entries")

    ids = [t["id"] for t in tags]
    duplicates = {i for i in ids if ids.count(i) > 1}
    if duplicates:
        raise SystemExit(f"duplicate tag ids in {path}: {sorted(duplicates)}")
    if len(tags) < 2:
        raise SystemExit(
            "at least 2 tags are required for multi-tag PnP; "
            f"{path} only defines {len(tags)}"
        )

    origin_id = cfg.get("originTagId", ids[0])
    if origin_id not in ids:
        raise SystemExit(f"originTagId {origin_id} is not present in 'tags'")

    # z is an absolute height above the floor, so only y is expected to be zeroed
    # on the origin tag.
    origin = next(t for t in tags if t["id"] == origin_id)
    if abs(float(origin.get("y", 0.0))) > 1e-9:
        print(
            f"note: origin tag {origin_id} is not at y=0 "
            "-- positions are used as given, not re-zeroed.",
            file=sys.stderr,
        )

    return cfg, scale, tags


def build_layout(tags, scale):
    ys = [float(t.get("y", 0.0)) * scale for t in tags]
    zs = [float(t.get("z", 0.0)) * scale for t in tags]

    # Shift Y so every tag lands at a positive coordinate; the wall itself stays
    # at x = 0 with the robot operating in +X. Any consistent frame works for the
    # solve -- this one just renders nicely.
    y_shift = Y_MARGIN_METERS - min(ys)
    field_width = (max(ys) - min(ys)) + 2 * Y_MARGIN_METERS + FIELD_WIDTH_PAD_METERS

    entries = []
    for tag in sorted(tags, key=lambda t: t["id"]):
        y = float(tag.get("y", 0.0)) * scale + y_shift
        z = float(tag.get("z", 0.0)) * scale
        yaw = math.radians(float(tag.get("yawDeg", 0.0)))
        pitch = math.radians(float(tag.get("pitchDeg", 0.0)))
        w, qx, qy, qz = quaternion_from_pitch_yaw(pitch, yaw)
        entries.append(
            {
                "ID": int(tag["id"]),
                "pose": {
                    "translation": {"x": 0.0, "y": y, "z": z},
                    "rotation": {"quaternion": {"W": w, "X": qx, "Y": qy, "Z": qz}},
                },
            }
        )

    layout = {
        "tags": entries,
        "field": {"length": FIELD_LENGTH_METERS, "width": field_width},
    }
    return layout, y_shift, zs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", default=DEFAULT_INPUT)
    parser.add_argument("-o", "--output", default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    _, scale, tags = load_config(args.input)
    layout, y_shift, zs = build_layout(tags, scale)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(layout, f, indent=2)
        f.write("\n")

    print(f"wrote {len(layout['tags'])} tags -> {args.output}")
    print(f"  tag ids:      {[t['ID'] for t in layout['tags']]}")
    print(f"  wall plane:   x = 0, tags facing +X (robot operates at x > 0)")
    print(f"  y shift:      +{y_shift:.4f} m applied to keep coordinates positive")
    print(f"  height range: {min(zs):.3f} .. {max(zs):.3f} m")
    print(f"  field size:   {layout['field']['length']:.2f} x {layout['field']['width']:.2f} m")
    print()
    print("Next: upload this same file to EVERY PhotonVision coprocessor")
    print("      (Settings -> AprilTag Field Layout), then enable Toggles/TagWallMode.")


if __name__ == "__main__":
    main()
