#!/usr/bin/env python3

# Typing
from __future__ import annotations

# Python
import sys
from math import acos, atan2, copysign, isclose, pi, pow, sqrt
from typing import Callable

# ROS
import rospy
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

# Local
from state_machine import StateMachine, State
from velocity_helpers import is_zero_velocity, is_not_zero_velocity

#
# The ScuttleBumperNavigator runs a Finite State Machine (FSM) that determines what the next
# navigation action is for the robot.
#
# The states are:
# - Stopped: The robot isn't moving
# - Moving: The robot is moving towards a goal
# - Reverting: The robot is backing up after hitting an obstacle
#
# The transitions are:
#
# Stopped   -> Moving: When a velocity command comes in
#           -> Reversing: When the bumper is pressed. This state occurs briefly when the robot hits
#              an object big enough to stop the robot
#
# Moving    -> Stopped: When a velocity command with zero velocity comes in
#           -> Reversing: When the bumper sends a bumper event message
#
# Reversing -> Stopped:
#           -> Moving:
#

class ScuttleStoppedState(State):
    state_name = 'stopped'

    def __init__(self, bumper_state: int, bumper_location: int, bumper_pressed: Callable[[int], bool]):
        super().__init__(bumper_state, bumper_location)
        self.is_bumper_pressed = bumper_pressed

    @property
    def name(self):
        return self.state_name

    def enter(self, machine: StateMachine):
        super().enter(machine)

    def exit(self, machine: StateMachine):
        super().exit(machine)

    def update(self, machine: StateMachine):
        if super().update(machine):
            if self.is_bumper_pressed(self.bumper_state):
                machine.go_to_state(ScuttleBumperObstacleAvoidingState.state_name)
            else:
                if self.velocity and is_not_zero_velocity(self.velocity):
                    machine.go_to_state(ScuttleMovingState.state_name)

class ScuttleMovingState(State):
    state_name = 'moving'

    def __init__(self, bumper_state: int, bumper_location: int, bumper_pressed: Callable[[int], bool]):
        super().__init__(bumper_state, bumper_location)
        self.is_bumper_pressed = bumper_pressed

    @property
    def name(self):
        return self.state_name

    def enter(self, machine: StateMachine):
        super().enter(machine)

    def exit(self, machine: StateMachine):
        super().exit(machine)

    def update(self, machine: StateMachine):
        if super().update(machine):
            if self.is_bumper_pressed(self.bumper_state):
                machine.go_to_state(ScuttleBumperObstacleAvoidingState.state_name)
            else:
                if not self.velocity or is_zero_velocity(self.velocity):
                    machine.go_to_state(ScuttleStoppedState.state_name)

class ScuttleBumperObstacleAvoidingState(State):
    state_name = 'avoiding_bumper_obstacle'

    def __init__(
            self,
            bumper_state: int,
            bumper_location: int,
            max_linear_velocity: float,
            min_linear_velocity: float,
            max_linear_acceleration: float,
            rate_in_hz: int,
            distance_tolerance: float,
            publish_velocity: Callable[[Twist], None],
            transform_from_base_to_odom: Callable[[Pose], Pose],
            log: Callable[[str], None]
        ):
        super().__init__(bumper_state, bumper_location)
        self.publish_velocity = publish_velocity
        self.log = log

        self.max_linear_acceleration = max_linear_acceleration
        self.rate_in_hz = rate_in_hz
        self.distance_tolerance = distance_tolerance
        self.max_linear_velocity = max_linear_velocity
        self.min_linear_velocity = min_linear_velocity

        self.transform_from_base_to_odom = transform_from_base_to_odom

        # Store the default message fields
        # self.is_bigendian = sys.byteorder == 'big'
        # self.fields = [
        #         PointField(
        #             name='x',
        #             offset=0,
        #             datatype=PointField.Float32,
        #             count=1),
        #         PointField(
        #             name='y',
        #             offset=4,
        #             datatype=PointField.Float32,
        #             count=1),
        #         PointField(
        #             name='z',
        #             offset=8,
        #             datatype=PointField.Float32,
        #             count=1)
        #     ]

    @property
    def name(self):
        return self.state_name

    def enter(self, machine: StateMachine):
        super().enter(machine)

        # Indicate that we're in obstacle avoidance mode before we do anything else
        self.avoiding_obstacle = True

        # Hit an obstacle, so stop moving so we don't make things worse
        self.publish_velocity(Twist())

        if self.odometry is None:
            # Uh oh, we don't know where we are ..
            self.odometry = Odometry()

        # From the current position, move backwards by 1 robot length
        # If we use the chassis link frame, then the new position is easy to calculate
        initial_pose = Pose()
        initial_pose.position.x = -0.3

        # Now translate to the odom frame
        self.target_pose_in_odom_frame = self.transform_from_base_to_odom(initial_pose)

        # Send an obstacle to the map so that we know for next time where it is
        #self.publish_obstacle(self.bumper_location)

    def exit(self, machine: StateMachine):
        super().exit(machine)
        self.avoiding_obstacle = False
        self.target_pose_in_odom_frame = None

    def update(self, machine: StateMachine):
        if super().update(machine):
            self.log('Updating reversing state ...')

            if self.avoiding_obstacle:
                self.log('Updating reversing state: Avoiding obstacle...')

                # Move backwards until we reach the requested distance moved
                if not self.has_reached_target():
                    # Calculate the velocity to get to the target location. The velocity is
                    # calculated based on the distance to the targer, further away == higher
                    # velocity, closer == lower velocity. Acceleration is limited to provide
                    # a nice velocity ramp
                    linear_velocity = self.linear_vel(self.target_pose_in_odom_frame, self.position)
                    angular_velocity = 0.0 # self.angular_vel(self.target_pose)

                    # We want to go backwards.
                    # The velocity we get is always a positive
                    # number as it's based on the absolute distance to the target. Thus
                    # negate the linear velocity.
                    twist = Twist()
                    twist.linear.x = linear_velocity
                    twist.linear.y = 0
                    twist.linear.z = 0

                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = angular_velocity

                    self.log('Reversing state: Updating velocity: [{0}, {1}]'.format(linear_velocity, angular_velocity))
                    self.publish_velocity(twist)
                else:
                    # Backed up far enough. Stop the movement
                    self.avoiding_obstacle = False
                    self.target_pose_in_odom_frame = None

                    self.log('Reversing state: Updating velocity to zero')
                    twist = Twist()
                    self.publish_velocity(twist)
            else:
                if self.velocity and is_not_zero_velocity(self.velocity):
                    machine.go_to_state(ScuttleMovingState.state_name)
                else:
                    machine.go_to_state(ScuttleStoppedState.state_name)

    def distance(self, target_position: Pose, current_position: Pose):
        dx = target_position.position.x - current_position.position.x
        dy = target_position.position.y - current_position.position.y
        return sqrt(pow(dx, 2) + pow(dy, 2))

    def has_reached_target(self):
        distance = self.distance(self.target_pose_in_odom_frame, self.position)

        self.log('Obstacle avoiding state: Distance to target: {0}. Maximum distance from target: {1}'.format(distance, self.distance_tolerance))
        if distance < self.distance_tolerance:
            self.log('Obstacle avoiding state: back up distance reached. Distance to target: {0} < {1}'.format(distance, self.distance_tolerance))
            return True

        if self.is_target_in_front():
            self.log('Obstacle avoiding state: Passed backup target. Distance to target: {0} > {1}'.format(distance, self.distance_tolerance))
            return True

        return False

    # Determine if we have driven backwards past our target point by any chance.
    # This can happen when we move either fast enough that we move past our point between two time steps, or if our
    # required accuracy is set to high
    #
    # We will assume that if the target is in front of the robot between 270 (right hand side of the robot) and 90
    # (left hand side) then we have passed it.
    def is_target_in_front(self):
        angle_of_direction_vector_with_odom_x_axis = atan2(
            self.target_pose_in_odom_frame.position.y - self.position.position.y,
            self.target_pose_in_odom_frame.position.x - self.position.position.x)

        (_, _, yaw) = euler_from_quaternion(
            [
                self.position.orientation.x,
                self.position.orientation.y,
                self.position.orientation.z,
                self.position.orientation.w,
            ])

        angle_between_orientation_and_target_direction = angle_of_direction_vector_with_odom_x_axis - yaw

        if abs(angle_between_orientation_and_target_direction) > (0.5 * pi):
            return False
        else:
            return True

    def linear_vel(self, target_position: Pose, current_position: Pose, constant=0.5):
        desired_velocity = constant * self.distance(target_position, current_position)

        # Assume that we drive backwards in the robot x-direction, so the velocity becomes
        desired_velocity = -desired_velocity

        # Take into account maximum accelerations and deccellerations so that we don't strip
        # gears etc.
        current_velocity = self.velocity.linear.x
        return self.velocity_with_ramp(current_velocity, desired_velocity)

    def publish_obstacle(self, last_bumper_location: int):
        # Send obstacle message with coordinates of the obstacles if there are any
        msg = PointCloud2()
        #msg.header.stamp = time_of_recording
        msg.header.frame_id = 'base_frame'

        msg.is_bigendian = self.is_bigendian
        msg.is_dense = False

        msg.fields = self.fields

        # CREATE THE MESSAGE TYPE HERE
        # self.obstacle_pub.publish(msg)

    def velocity_with_ramp(self, current_velocity: float, desired_velocity: float) -> float:
        if abs(desired_velocity) < self.min_linear_velocity:
            desired_velocity = copysign(self.min_linear_velocity, desired_velocity)

        if abs(desired_velocity) > self.max_linear_velocity:
            desired_velocity = copysign(self.max_linear_velocity, desired_velocity)

        if desired_velocity == current_velocity:
            return desired_velocity
        else:
            if desired_velocity > current_velocity:
                # accelerating
                achievable_velocity = current_velocity + self.max_linear_acceleration / self.rate_in_hz
                if achievable_velocity > desired_velocity:
                    # desired acceleration is less than the possible acceleration
                    return desired_velocity
                else:
                    # desired acceleration is more than the possible acceleration
                    return achievable_velocity
            else:
                achievable_velocity = current_velocity - self.max_linear_acceleration / self.rate_in_hz
                if achievable_velocity < desired_velocity:
                    # desired deceleration is less than the possible deceleration
                    return desired_velocity
                else:
                    # desired deceleration is more than the possible decelaration
                    return achievable_velocity
