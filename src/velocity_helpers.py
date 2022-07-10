#!/usr/bin/env python3

# python
from math import isclose

# ROS
from geometry_msgs.msg import Twist

def is_zero_velocity(velocity: Twist) -> bool:
    return isclose(velocity.linear.x, 0.0, rel_tol=1e-6, abs_tol=0) and \
        isclose(velocity.linear.y, 0.0, rel_tol=1e-6, abs_tol=0) and \
        isclose(velocity.linear.z, 0.0, rel_tol=1e-6, abs_tol=0) and \
        isclose(velocity.angular.x, 0.0, rel_tol=1e-6, abs_tol=0) and \
        isclose(velocity.angular.y, 0.0, rel_tol=1e-6, abs_tol=0) and \
        isclose(velocity.angular.z, 0.0, rel_tol=1e-6, abs_tol=0)

def is_not_zero_velocity(velocity: Twist) -> bool:
    return not isclose(velocity.linear.x, 0.0, rel_tol=1e-6, abs_tol=0) or \
        not isclose(velocity.linear.y, 0.0, rel_tol=1e-6, abs_tol=0) or \
        not isclose(velocity.linear.z, 0.0, rel_tol=1e-6, abs_tol=0) or \
        not isclose(velocity.angular.x, 0.0, rel_tol=1e-6, abs_tol=0) or \
        not isclose(velocity.angular.y, 0.0, rel_tol=1e-6, abs_tol=0) or \
        not isclose(velocity.angular.z, 0.0, rel_tol=1e-6, abs_tol=0)
