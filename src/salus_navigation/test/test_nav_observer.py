"""Unit tests for compact Nav2 observability without a running graph."""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from salus_navigation.nav_observer import PlanReplanTracker, plan_signature


def path(points):
    message = Path()
    message.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        message.poses.append(pose)
    return message


def test_plan_signature_ignores_timestamp_and_tracks_geometry():
    first, repeated, changed = path([(0.0, 0.0), (1.0, 0.0)]), path([(0.0, 0.0), (1.0, 0.0)]), path([(0.0, 0.0), (1.0, 1.0)])
    first.header.stamp.sec, repeated.header.stamp.sec = 1, 2
    assert plan_signature(first) == plan_signature(repeated)
    assert plan_signature(first) != plan_signature(changed)


def test_replan_tracker_only_reports_changes_for_active_goal():
    tracker, first, changed = PlanReplanTracker(), path([(0.0, 0.0), (1.0, 0.0)]), path([(0.0, 0.0), (1.0, 1.0)])
    assert not tracker.observe(first, goal_active=False)
    assert not tracker.observe(first, goal_active=True)
    assert tracker.observe(changed, goal_active=True)
    assert not tracker.observe(changed, goal_active=False)
