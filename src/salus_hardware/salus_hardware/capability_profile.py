"""Pure capability declarations for explicitly selected hardware profiles."""

from __future__ import annotations

from dataclasses import dataclass
import math


PROFILE_OBSTACLE_DETECTION = "obstacle_detection"
PROFILE_NO_OBSTACLE_DETECTION = "no_obstacle_detection"
VALID_PROFILES = (PROFILE_OBSTACLE_DETECTION, PROFILE_NO_OBSTACLE_DETECTION)
VALID_IMU_SOURCES = ("imu_primary", "imu_secondary")
VALID_ORIENTATION_SOURCES = ("course_over_ground", "external_heading")


@dataclass(frozen=True)
class CapabilityDeclaration:
    """Configured capability, distinct from runtime sensor measurements."""

    capability_id: str
    state: int
    required: bool
    enabled: bool
    source_ids: tuple[str, ...]
    detail: str


def observed_state(
    *,
    now_s: float,
    last_sample_s: float | None,
    timeout_s: float,
    unavailable_state: int,
    stale_state: int,
    ready_state: int,
) -> int:
    """Classify availability without ever selecting another source."""

    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("sensor freshness timeout must be positive and finite")
    if last_sample_s is None:
        return unavailable_state
    age_s = now_s - last_sample_s
    if not math.isfinite(age_s) or age_s < 0.0:
        return unavailable_state
    return ready_state if age_s <= timeout_s else stale_state


def normalize_profile(value: object) -> str:
    """Return a supported explicit profile or reject ambiguous values."""

    profile = str(value).strip().lower()
    if profile not in VALID_PROFILES:
        raise ValueError(
            "capability profile must be obstacle_detection or "
            "no_obstacle_detection"
        )
    return profile


def normalize_selection(value: object, *, field: str, choices: tuple[str, ...]) -> str:
    selected = str(value).strip().lower()
    if selected not in choices:
        raise ValueError(f"{field} must be one of: {', '.join(choices)}")
    return selected


def declarations_for_profile(
    profile: object,
    *,
    ready_state: int,
    disabled_state: int,
    imu_source: object = "imu_primary",
    orientation_source: object = "course_over_ground",
) -> tuple[CapabilityDeclaration, ...]:
    """Build the immutable declared state for the selected profile."""

    selected = normalize_profile(profile)
    selected_imu = normalize_selection(
        imu_source, field="imu_source", choices=VALID_IMU_SOURCES
    )
    selected_orientation = normalize_selection(
        orientation_source,
        field="orientation_source",
        choices=VALID_ORIENTATION_SOURCES,
    )
    obstacle_enabled = selected == PROFILE_OBSTACLE_DETECTION
    state = ready_state if obstacle_enabled else disabled_state
    detail = (
        "local obstacle detection enabled and required; runtime health not asserted"
        if obstacle_enabled
        else "local obstacle detection deliberately disabled by explicit profile"
    )
    return (
        CapabilityDeclaration(
            capability_id="local_obstacle_detection",
            state=state,
            required=obstacle_enabled,
            enabled=obstacle_enabled,
            source_ids=("lidar_primary",),
            detail=detail,
        ),
        CapabilityDeclaration(
            capability_id="lidar_primary",
            state=state,
            required=obstacle_enabled,
            enabled=obstacle_enabled,
            source_ids=("sim_lidar",),
            detail=(
                "primary LiDAR pipeline enabled; runtime health not asserted"
                if obstacle_enabled
                else "primary LiDAR pipeline not started by profile"
            ),
        ),
        CapabilityDeclaration(
            capability_id="local_motion_imu",
            state=ready_state,
            required=True,
            enabled=True,
            source_ids=(selected_imu,),
            detail=(
                f"{selected_imu} selected exclusively; logical output is monitored"
            ),
        ),
        CapabilityDeclaration(
            capability_id="global_orientation",
            state=ready_state,
            required=True,
            enabled=True,
            source_ids=(selected_orientation,),
            detail=(
                f"{selected_orientation} selected exclusively; logical output is "
                "monitored with no automatic fallback"
            ),
        ),
    )
