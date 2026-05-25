#!/usr/bin/env python
import math


def calc_distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def normalize_angle_deg(angle):
    """Normalize angle to [-180, 180] degrees."""
    return (angle + 180) % 360 - 180


def normalize_angle_rad(angle):
    """Normalize angle to [-pi, pi] radians."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
