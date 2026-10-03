"""Differential-drive kinematics shared by the simulator and its tests."""


def wheel_speeds(linear, angular, separation, radius):
    """
    Return ``(left, right)`` wheel speeds in rad/s for a body twist.

    ``linear`` is m/s along +X and ``angular`` is rad/s about +Z (counter-
    clockwise), as in ``geometry_msgs/Twist``.
    """
    half = 0.5 * separation * angular
    return (linear - half) / radius, (linear + half) / radius


def slew(current, target, max_delta):
    """Move ``current`` towards ``target`` by at most ``max_delta``."""
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target
