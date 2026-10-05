"""
Scenario metrics for the DATN T-junction track, computed from /odom samples.

Track geometry (same world frame as datn_track.xml): the robot starts on the
stem centred on y = 0, heading +X; the route turns left at the junction into
the branch centred on x = 1.5, heading +Y. Tape runs 0.13 m either side of a
centre line, so the lane is 0.26 m wide.
"""

from dataclasses import asdict, dataclass
import math

STEM_CENTER_Y = 0.0
BRANCH_CENTER_X = 1.5
LANE_HALF_WIDTH = 0.13
# Segments where the lane is straight and the error is meaningful (junction
# and branch end excluded).
STEM_X_RANGE = (0.0, 1.0)
BRANCH_Y_RANGE = (0.7, 2.95)
# Centre further than this from the lane centre means a wheel is on the tape.
KEEP_LANE_LIMIT = 0.10
# Route complete once the robot is this far up the branch, inside the lane.
COMPLETE_Y = 2.9
# Where the speed-limit and stop signs act along the branch.
SLOW_Y_RANGE = (0.6, 1.5)
STOP_Y_RANGE = (2.3, 2.95)
STOPPED_SPEED = 0.005   # m/s
MIN_STOP_TIME = 4.0     # s, STOP holds 5 s
SLOW_SPEED_RANGE = (0.03, 0.075)  # m/s, the 20 km/h sign speed in simulation
# The route ends where the branch tape stops; the robot then halts for good.
END_Y = 2.95
END_STILL_TIME = 3.0    # s


@dataclass
class Sample:
    t: float
    x: float
    y: float
    yaw: float
    speed: float


def lane_error(x, y):
    """Return ``(segment, signed error in m)`` or ``(None, None)`` off-segment."""
    if STEM_X_RANGE[0] <= x <= STEM_X_RANGE[1] and abs(y) < 0.5:
        return 'stem', y - STEM_CENTER_Y
    if BRANCH_Y_RANGE[0] <= y <= BRANCH_Y_RANGE[1] and abs(x - BRANCH_CENTER_X) < 0.5:
        return 'branch', x - BRANCH_CENTER_X
    return None, None


def end_reached(samples):
    """Return True once the robot has stood still past the end of the route."""
    still_since = None
    for s in reversed(samples):
        if s.y <= END_Y or s.speed >= STOPPED_SPEED:
            break
        still_since = s.t
    return still_since is not None and samples[-1].t - still_since >= END_STILL_TIME


def _longest_duration(samples, predicate):
    best = 0.0
    start = None
    for s in samples:
        if predicate(s):
            start = s.t if start is None else start
            best = max(best, s.t - start)
        else:
            start = None
    return best


@dataclass
class ScenarioMetrics:
    duration: float
    finish_time_s: float
    stem_mean_error_mm: float
    stem_max_error_mm: float
    branch_mean_error_mm: float
    branch_max_error_mm: float
    kept_lane: bool
    took_left_branch: bool
    completed: bool
    stop_hold_s: float
    slow_hold_s: float

    def as_dict(self):
        return asdict(self)


def _stats_mm(errors):
    if not errors:
        return math.nan, math.nan
    abs_errors = [abs(e) for e in errors]
    return 1000 * sum(abs_errors) / len(abs_errors), 1000 * max(abs_errors)


def evaluate(samples):
    """Return ScenarioMetrics for a run given time-ordered samples."""
    errors = {'stem': [], 'branch': []}
    for s in samples:
        segment, error = lane_error(s.x, s.y)
        if segment:
            errors[segment].append(error)
    stem_mean, stem_max = _stats_mm(errors['stem'])
    branch_mean, branch_max = _stats_mm(errors['branch'])
    all_errors = errors['stem'] + errors['branch']
    in_branch = [
        s for s in samples
        if abs(s.x - BRANCH_CENTER_X) < LANE_HALF_WIDTH and s.y > BRANCH_Y_RANGE[0]]
    return ScenarioMetrics(
        duration=samples[-1].t - samples[0].t if samples else 0.0,
        finish_time_s=next(
            (s.t - samples[0].t for s in in_branch if s.y >= COMPLETE_Y), math.nan),
        stem_mean_error_mm=stem_mean,
        stem_max_error_mm=stem_max,
        branch_mean_error_mm=branch_mean,
        branch_max_error_mm=branch_max,
        kept_lane=bool(all_errors) and max(abs(e) for e in all_errors) < KEEP_LANE_LIMIT,
        took_left_branch=bool(in_branch),
        completed=any(s.y >= COMPLETE_Y for s in in_branch),
        stop_hold_s=_longest_duration(
            samples,
            lambda s: STOP_Y_RANGE[0] <= s.y <= STOP_Y_RANGE[1] and s.speed < STOPPED_SPEED),
        slow_hold_s=_longest_duration(
            samples,
            lambda s: SLOW_Y_RANGE[0] <= s.y <= SLOW_Y_RANGE[1]
            and SLOW_SPEED_RANGE[0] <= s.speed <= SLOW_SPEED_RANGE[1]),
    )
