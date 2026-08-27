"""Pure capability declarations for explicitly selected hardware profiles."""

from __future__ import annotations

from dataclasses import dataclass


PROFILE_OBSTACLE_DETECTION = "obstacle_detection"
PROFILE_NO_OBSTACLE_DETECTION = "no_obstacle_detection"
VALID_PROFILES = (PROFILE_OBSTACLE_DETECTION, PROFILE_NO_OBSTACLE_DETECTION)


@dataclass(frozen=True)
class CapabilityDeclaration:
    """Configured capability, distinct from runtime sensor measurements."""

    capability_id: str
    state: int
    required: bool
    enabled: bool
    source_ids: tuple[str, ...]
    detail: str


def normalize_profile(value: object) -> str:
    """Return a supported explicit profile or reject ambiguous values."""

    profile = str(value).strip().lower()
    if profile not in VALID_PROFILES:
        raise ValueError(
            "capability profile must be obstacle_detection or "
            "no_obstacle_detection"
        )
    return profile


def declarations_for_profile(
    profile: object, *, ready_state: int, disabled_state: int,
) -> tuple[CapabilityDeclaration, ...]:
    """Build the immutable declared state for the selected profile."""

    selected = normalize_profile(profile)
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
    )
