#!/usr/bin/env python3
"""Compare migrated control interface bodies with a ROS2_SALUS checkout."""

import argparse
from pathlib import Path
import sys


CONTRACTS = {
    "msg": (
        "BatteryMissionGuard.msg", "CmdVelFinal.msg", "DriveTelemetry.msg",
        "NavEvent.msg", "NavTelemetry.msg", "PathHealth.msg",
    ),
    "srv": (
        "BrakeNav.srv", "CancelNavGoal.srv", "GetNavState.srv", "GetZonesState.srv", "SetManualMode.srv", "SetNavGoalLL.srv",
        "SetZonesGeoJson.srv",
        "SetSimBatteryPreset.srv", "SetSimBatteryState.srv",
    ),
}


def normalized(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "reference",
        type=Path,
        help="Path to the ROS2_SALUS src/interfaces directory",
    )
    args = parser.parse_args()
    migrated = Path(__file__).resolve().parents[1] / "src" / "salus_interfaces"
    errors = []
    for kind, names in CONTRACTS.items():
        for name in names:
            old_path = args.reference / kind / name
            new_path = migrated / kind / name
            if not old_path.is_file():
                errors.append(f"missing reference: {old_path}")
            elif normalized(old_path) != normalized(new_path):
                errors.append(f"contract differs: {kind}/{name}")
    if errors:
        print("Interface compatibility failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("Interface compatibility passed: 14 contracts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
