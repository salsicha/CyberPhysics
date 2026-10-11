"""RACECAR Neo SLAM Toolbox settings that bound the map -> odom corrections.

SLAM Toolbox corrects map -> odom only when it matches a scan. The odometry
over-reports distance by 7-10% (applications/racecarneo/README.md), so the
stack's pose runs ahead of the car between matches, and Nav2's goal check
uses the correction from when its plan was made. With the 0.5 m default and
the imprecise distance check, matches came every ~0.9 m and moved the car by
up to 0.19 m; cars stopped up to 0.07 m outside the 0.25 m goal tolerance.
"""
import yaml

from conftest import REPO

CONFIG = REPO / 'systems/racecarneo/config'
SLAM = yaml.safe_load((CONFIG / 'slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
NAV2 = yaml.safe_load((CONFIG / 'nav2.yaml').read_text())
GOAL_CHECKER = NAV2['controller_server']['ros__parameters']['general_goal_checker']
MAX_SPEED = NAV2['velocity_smoother']['ros__parameters']['max_velocity'][0]

ODOMETRY_DISTANCE_BIAS = 0.10   # upper end of the measured over-report


def test_scans_are_matched_every_fifth_of_a_metre():
    assert SLAM['minimum_travel_distance'] <= 0.25
    # Without the precise check the node passes a scan after 0.89 x the
    # distance and the mapper drops it as too close, doubling the spacing.
    assert SLAM['check_min_dist_and_heading_precisely'] is True


def test_odometry_drift_between_matches_is_a_small_part_of_the_tolerance():
    drift = ODOMETRY_DISTANCE_BIAS * SLAM['minimum_travel_distance']
    assert drift <= 0.1 * GOAL_CHECKER['xy_goal_tolerance']


def test_distance_not_time_or_throttle_sets_the_cadence():
    assert SLAM['throttle_scans'] == 1
    assert SLAM['minimum_time_interval'] < SLAM['minimum_travel_distance'] / MAX_SPEED
