#!/usr/bin/env python3
"""Planner-only regression from #307's second real loop incident.

Run inside an isolated Humble container, never against a hardware ROS graph.
The C++ harness configures only a costmap and a native Smac plugin. It has no
controller/actuator, uses no sensor subscriptions and makes no action requests.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess

import yaml
from salus_navigation.route_model import RouteWaypoint
from salus_navigation.route_preparation import dispatch_yaws, resolve_yaws


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--costmap-dir", type=Path,
                        help="optional initial/replan lower/upper .bin exports")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    fixture = json.loads((root / "src/salus_evaluation/test/fixtures/loop_corner_replan.json").read_text())
    points = [RouteWaypoint(0, 0, float("nan"), index,
                            map_x=xy[0], map_y=xy[1])
              for index, xy in enumerate((fixture["incoming_point"],
                                          fixture["first_point"],
                                          fixture["next_point"]))]
    resolved = resolve_yaws(points, False, curve_tangent=True)
    current = dispatch_yaws(tuple(resolved[1:]),
                            approach_xy=tuple(fixture["initial_start"][:2]),
                            approach_heading_deg=fixture["captured_yaws_deg"][0],
                            curve_tangent=True)
    # Hold terminal heading equal to the captured value to isolate the first
    # waypoint change, rather than changing two constraints in the experiment.
    modes = {"captured": fixture["captured_yaws_deg"],
             "current": [current[0], fixture["captured_yaws_deg"][1]]}
    config = yaml.safe_load((root / "src/salus_navigation/config/nav2_core_real.yaml").read_text())
    planner = dict(config["planner_server"]["ros__parameters"]["GridBased"])
    planner.pop("plugin")
    cases = []
    for phase in ("initial", "replan"):
        maps = [("empty", None)]
        if args.costmap_dir:
            maps += [(mode, str((args.costmap_dir / f"{phase}-{mode}.bin").resolve()))
                     for mode in ("lower", "upper")]
        for map_name, cost_file in maps:
            for mode, yaws in modes.items():
                case = dict(name=f"{phase}-{map_name}-{mode}",
                            map=fixture[f"{phase}_map"],
                            start=fixture[f"{phase}_start"],
                            goals=[[xy[0], xy[1], math.radians(yaw)]
                                   for xy, yaw in zip((fixture["first_point"],
                                                       fixture["next_point"]), yaws)])
                if cost_file:
                    case["cost_file"] = cost_file
                cases.append(case)
    sharp = (RouteWaypoint(0, 0, -90.0, 0, map_x=6.0, map_y=0.0),
             RouteWaypoint(0, 0, 0.0, 1, map_x=20.0, map_y=-14.0))
    join_yaws = dispatch_yaws(sharp, approach_xy=(0.0, 0.0),
                              approach_heading_deg=0.0, curve_tangent=True)
    assert join_yaws == [0.0, 0.0], "sharp join protection regressed"
    lateral = (RouteWaypoint(0, 0, 90.0, 0, map_x=0.0, map_y=6.0),
               RouteWaypoint(0, 0, 45.0, 1, map_x=8.0, map_y=6.0))
    lateral_yaws = dispatch_yaws(lateral, approach_xy=(0.0, 0.0),
                                 approach_heading_deg=0.0, curve_tangent=True)
    assert lateral_yaws == [0.0, 45.0], "lateral join protection regressed"
    cases.append(dict(name="sharp-join-control", map=fixture["initial_map"],
                      start=[0.0, 0.0, 0.0],
                      goals=[[6.0, 0.0, 0.0], [20.0, -14.0, 0.0]]))
    inputs = output / "inputs.json"
    inputs.write_text(json.dumps(dict(planner=planner, cases=cases), indent=2))
    build = output / "build"
    subprocess.run(["cmake", "-S", str(root / "tools/offline_smac_replay"),
                    "-B", str(build)], check=True)
    subprocess.run(["cmake", "--build", str(build), "-j2"], check=True)
    results_file = output / "results.json"
    subprocess.run([str(build / "reproduce"), str(inputs), str(results_file)], check=True)
    results = json.loads(results_file.read_text())
    measured = {}
    direct = math.hypot(*fixture["replan_start"][:2])
    for result in results:
        assert result["ok"], result
        measured[result["name"]] = [segment["length"] for segment in result["segments"]]
        if result["name"].startswith("replan-"):
            ratio = measured[result["name"]][0] / direct
            if result["name"].endswith("-captured"):
                assert ratio > 3.0, "baseline no longer reproduces the loop"
            else:
                assert ratio < 1.5, "current request still forces a loop"
    assert measured["initial-empty-current"][0] < 1.5 * math.hypot(*fixture["initial_start"][:2])
    assert measured["sharp-join-control"][0] < 8.0
    summary = dict(source=fixture["source"], first_yaw_before=fixture["captured_yaws_deg"][0],
                   first_yaw_after=current[0], lengths_m=measured,
                   scope="planner-only; not executed robot trajectory; reconstructed costs are lossy")
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
