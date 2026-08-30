"""Integrated simulation checkpoint for the subsystems migrated so far."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _include(package: str, launch_file: str, arguments=None, condition=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    rviz = LaunchConfiguration("rviz")
    launch_navigation = LaunchConfiguration("launch_navigation")
    launch_zones = LaunchConfiguration("launch_zones")
    use_keepout = LaunchConfiguration("use_keepout")
    zones_runtime_dir = LaunchConfiguration("zones_runtime_dir")
    patrol_runtime_dir = LaunchConfiguration("patrol_runtime_dir")
    launch_routes = LaunchConfiguration("launch_routes")
    launch_patrol = LaunchConfiguration("launch_patrol")
    launch_web = LaunchConfiguration("launch_web")
    launch_camera = LaunchConfiguration("launch_camera")
    web_ws_port = LaunchConfiguration("web_ws_port")
    web_waypoints_file = LaunchConfiguration("web_waypoints_file")
    web_telemetry_profile = LaunchConfiguration("web_telemetry_profile")
    patrol_battery_guard_topic = LaunchConfiguration("patrol_battery_guard_topic")
    patrol_battery_state_topic = LaunchConfiguration("patrol_battery_state_topic")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_no_obstacles_params_file = LaunchConfiguration("nav2_no_obstacles_params_file")
    vehicle_io_profile = LaunchConfiguration("vehicle_io_profile")
    compare_legacy_odometry = LaunchConfiguration("compare_legacy_odometry")
    command_input_mode = LaunchConfiguration("command_input_mode")
    capability_profile = LaunchConfiguration("capability_profile")
    imu_source = LaunchConfiguration("imu_source")
    orientation_source = LaunchConfiguration("orientation_source")
    obstacle_detection_enabled = PythonExpression([
        "'", capability_profile, "' == 'obstacle_detection'",
    ])
    no_obstacle_detection = PythonExpression([
        "'", capability_profile, "' == 'no_obstacle_detection'",
    ])

    common = {"use_sim_time": use_sim_time}
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "vehicle_io_profile",
                default_value="legacy",
                description="Vehicle measurement/odometry profile: legacy or canonical.",
            ),
            DeclareLaunchArgument(
                "compare_legacy_odometry",
                default_value="false",
                description="Run legacy odometry on shadow topics under canonical profile.",
            ),
            DeclareLaunchArgument(
                "command_input_mode",
                default_value="legacy_cmd_vel",
                description=(
                    "Exclusive control input: legacy_cmd_vel or "
                    "canonical_vehicle_command."
                ),
            ),
            DeclareLaunchArgument(
                "capability_profile",
                default_value="obstacle_detection",
                choices=["obstacle_detection", "no_obstacle_detection"],
                description=(
                    "Explicit capability profile. It never changes automatically "
                    "after a sensor failure."
                ),
            ),
            DeclareLaunchArgument(
                "imu_source",
                default_value="imu_primary",
                choices=["imu_primary", "imu_secondary"],
                description="Exclusive local motion IMU source; no automatic fallback.",
            ),
            DeclareLaunchArgument(
                "orientation_source",
                default_value="course_over_ground",
                choices=["course_over_ground", "external_heading"],
                description="Exclusive global orientation source; no automatic fallback.",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -s",
                description="Gazebo arguments; use '-r' for the graphical client.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    Path(get_package_share_directory("salus_simulation"))
                    / "worlds"
                    / "empty.world"
                ),
                description="Gazebo world used by this composed simulation.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz diagnostics with guarded 2D goal control.",
            ),
            DeclareLaunchArgument(
                "launch_navigation",
                default_value="true",
                description="Start the Nav2 autonomous navigation core.",
            ),
            DeclareLaunchArgument(
                "launch_zones",
                default_value="true",
                description=(
                    "Start zones manager and keepout map servers. Diagnostic false "
                    "removes that workload; production/default behavior is unchanged."
                ),
            ),
            DeclareLaunchArgument(
                "use_keepout",
                default_value="true",
                description="Require dynamic GeoJSON keepout readiness in Nav2 startup.",
            ),
            DeclareLaunchArgument(
                "zones_runtime_dir",
                default_value="runtime/zones",
                description="Runtime directory for the dynamic keepout mask.",
            ),
            DeclareLaunchArgument(
                "patrol_runtime_dir",
                default_value="runtime/patrol",
                description="Runtime directory for structured patrol persistence.",
            ),
            DeclareLaunchArgument(
                "launch_routes", default_value="false",
                description="Start the optional route executor.",
            ),
            DeclareLaunchArgument(
                "launch_patrol", default_value="false",
                description="Start the optional structured patrol/HOME coordinator.",
            ),
            DeclareLaunchArgument(
                "launch_web", default_value="false",
                description="Start Cockpit WebSocket bridge and snapshot service.",
            ),
            DeclareLaunchArgument(
                "launch_camera", default_value="false",
                description="Start the simulated PTZ control service without a video stream.",
            ),
            DeclareLaunchArgument(
                "camera_presets_file",
                default_value="runtime/camera/presets.json",
                description="Writable PTZ preset store for the simulated camera.",
            ),
            DeclareLaunchArgument("web_ws_port", default_value="8766"),
            DeclareLaunchArgument(
                "web_waypoints_file", default_value="runtime/web/waypoints.yaml"
            ),
            DeclareLaunchArgument(
                "web_telemetry_profile", default_value="compact",
                description="Cockpit telemetry profile: compact or full.",
            ),
            DeclareLaunchArgument(
                "patrol_battery_guard_topic", default_value="/battery_mission_guard",
                description="Battery guard consumed by structured patrol.",
            ),
            DeclareLaunchArgument(
                "patrol_battery_state_topic", default_value="/battery_state",
                description="SOC fallback consumed by structured patrol.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(
                    Path(get_package_share_directory("salus_navigation"))
                    / "config" / "nav2_core_sim.yaml"
                ),
                description="Obstacle-detection Nav2 parameter file.",
            ),
            DeclareLaunchArgument(
                "nav2_no_obstacles_params_file",
                default_value=str(
                    Path(get_package_share_directory("salus_navigation"))
                    / "config" / "nav2_core_no_obstacles_sim.yaml"
                ),
                description=(
                    "No-obstacle Nav2 parameter file; overridable for controlled "
                    "diagnostic compositions."
                ),
            ),
            _include(
                "salus_simulation",
                "motion_sim.launch.py",
                {"use_sim_time": use_sim_time, "gz_args": gz_args, "world": world},
            ),
            _include(
                "salus_control",
                "control_sim.launch.py",
                {**common, "command_input_mode": command_input_mode},
            ),
            _include(
                "salus_bringup",
                "vehicle_io_sim.launch.py",
                common,
                condition=IfCondition(
                    PythonExpression(["'", vehicle_io_profile, "' == 'canonical'"])
                ),
            ),
            _include(
                "salus_localization",
                "localization_sim.launch.py",
                {
                    **common,
                    "odometry_backend": vehicle_io_profile,
                    "compare_legacy_odometry": compare_legacy_odometry,
                    "imu_source": imu_source,
                },
            ),
            _include(
                "salus_localization",
                "global_localization_sim.launch.py",
                {**common, "orientation_source": orientation_source},
            ),
            _include(
                "salus_hardware", "capability_profile.launch.py",
                {
                    "profile": capability_profile,
                    "imu_source": imu_source,
                    "orientation_source": orientation_source,
                },
            ),
            _include(
                "salus_perception", "lidar_sim.launch.py",
                condition=IfCondition(obstacle_detection_enabled),
            ),
            _include(
                "salus_navigation", "safety_arbitration_sim.launch.py", common,
                condition=IfCondition(obstacle_detection_enabled),
            ),
            _include(
                "salus_navigation", "safety_arbitration_no_obstacles_sim.launch.py",
                common,
                condition=IfCondition(no_obstacle_detection),
            ),
            _include(
                "salus_navigation",
                "navigation_zones_sim.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "use_keepout": use_keepout,
                    "runtime_dir": zones_runtime_dir,
                },
                condition=IfCondition(PythonExpression([
                    "'", launch_navigation, "'.lower() == 'true' and '",
                    launch_zones, "'.lower() == 'true'",
                ])),
            ),
            _include(
                "salus_navigation",
                "navigation_core_sim.launch.py",
                {"use_sim_time": use_sim_time, "use_keepout": use_keepout,
                 "nav2_params_file": nav2_params_file},
                condition=IfCondition(PythonExpression([
                    "'", launch_navigation, "'.lower() == 'true' and ",
                    obstacle_detection_enabled,
                ])),
            ),
            _include(
                "salus_navigation",
                "navigation_core_sim.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "use_keepout": use_keepout,
                    "obstacle_detection_required": "false",
                    "nav2_params_file": nav2_no_obstacles_params_file,
                },
                condition=IfCondition(PythonExpression([
                    "'", launch_navigation, "'.lower() == 'true' and ",
                    no_obstacle_detection,
                ])),
            ),
            _include(
                "salus_navigation", "route_executor_sim.launch.py", common,
                condition=IfCondition(launch_routes),
            ),
            _include(
                "salus_navigation", "patrol_mission_sim.launch.py", {
                    **common,
                    "runtime_dir": patrol_runtime_dir,
                    "battery_guard_topic": patrol_battery_guard_topic,
                    "battery_state_topic": patrol_battery_state_topic,
                },
                condition=IfCondition(launch_patrol),
            ),
            _include(
                "salus_navigation",
                "navigation_snapshot_sim.launch.py",
                common,
                condition=IfCondition(launch_web),
            ),
            _include(
                "salus_web",
                "web_bridge.launch.py",
                {
                    "ws_port": web_ws_port,
                    "waypoints_file": web_waypoints_file,
                    "telemetry_profile": web_telemetry_profile,
                    "scan_preview_enabled": obstacle_detection_enabled,
                    "require_camera_service": launch_camera,
                },
                condition=IfCondition(launch_web),
            ),
            _include(
                "salus_hardware",
                "camera_sim.launch.py",
                {"camera_presets_file": LaunchConfiguration("camera_presets_file")},
                condition=IfCondition(launch_camera),
            ),
            _include(
                "salus_perception",
                "lidar_diagnostics.launch.py",
                condition=IfCondition(PythonExpression([
                    "'", rviz, "'.lower() == 'true' and ",
                    obstacle_detection_enabled,
                ])),
            ),
        ]
    )
