# python

# ROS
from geometry_msgs.msg import Twist

# locals
from velocity_helpers import is_zero_velocity, is_not_zero_velocity

def test_should_return_false_when_is_zero_velocity_with_nonzero_twist():
    twist = Twist()
    twist.linear.x = 2e6

    assert not is_zero_velocity(twist)

    twist.linear.x = 0.0
    twist.linear.y = 2e6

    assert not is_zero_velocity(twist)

    twist.linear.y = 0.0
    twist.linear.z = 2e6

    assert not is_zero_velocity(twist)

    twist.linear.z = 0.0
    twist.angular.x = 2e6

    assert not is_zero_velocity(twist)

    twist.angular.x = 0.0
    twist.angular.y = 2e6

    assert not is_zero_velocity(twist)

    twist.angular.y = 0.0
    twist.angular.z = 2e6

    assert not is_zero_velocity(twist)

def test_should_return_true_when_is_zero_velocity_with_zero_twist():
    twist = Twist()
    assert is_zero_velocity(twist)

def test_should_return_true_when_is_non_zero_velocity_with_nonzero_twist():
    twist = Twist()
    twist.linear.x = 2e6

    assert is_not_zero_velocity(twist)

    twist.linear.x = 0.0
    twist.linear.y = 2e6

    assert is_not_zero_velocity(twist)

    twist.linear.y = 0.0
    twist.linear.z = 2e6

    assert is_not_zero_velocity(twist)

    twist.linear.z = 0.0
    twist.angular.x = 2e6

    assert is_not_zero_velocity(twist)

    twist.angular.x = 0.0
    twist.angular.y = 2e6

    assert is_not_zero_velocity(twist)

    twist.angular.y = 0.0
    twist.angular.z = 2e6

    assert is_not_zero_velocity(twist)

def test_should_return_false_when_is_not_zero_velocity_with_zero_twist():
    twist = Twist()
    assert not is_not_zero_velocity(twist)
