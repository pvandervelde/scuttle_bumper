#!/usr/bin/env python3

# Typing
from __future__ import annotations

# Python
import sys
from math import atan2, copysign, isclose, pow, sqrt
from typing import Callable

# ROS
import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import tf2_geometry_msgs
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

    def __init__(self, bumper_pressed: Callable[[int], bool]):
        self.is_bumper_pressed = bumper_pressed

    @property
    def name(self):
        return self.state_name

    def enter(self, machine: StateMachine):
        State.enter(self, machine)

    def exit(self, machine: StateMachine):
        State.exit(self, machine)

    def update(self, machine: StateMachine):
        if State.update(self, machine):
            if self.is_bumper_pressed(self.bumper_state):
                machine.go_to_state(ScuttleBumperObstacleAvoidingState.state_name)
            else:
                if self.velocity and is_not_zero_velocity(self.velocity):
                    machine.go_to_state(ScuttleMovingState.state_name)

class ScuttleMovingState(State):
    state_name = 'moving'

    def __init__(self, bumper_pressed: Callable[[int], bool]):
        self.is_bumper_pressed = bumper_pressed

    @property
    def name(self):
        return self.state_name

    def enter(self, machine: StateMachine):
        State.enter(self, machine)

    def exit(self, machine: StateMachine):
        State.exit(self, machine)

    def update(self, machine: StateMachine):
        if State.update(self, machine):
            if self.is_bumper_pressed(self.bumper_state):
                machine.go_to_state(ScuttleBumperObstacleAvoidingState.state_name)
            else:
                if not self.velocity or is_zero_velocity(self.velocity):
                    machine.go_to_state(ScuttleStoppedState.state_name)

class ScuttleBumperObstacleAvoidingState(State):
    state_name = 'avoiding_bumper_obstacle'

    def __init__(self, publish_velocity: Callable[[Twist], None], log: Callable[[str], None]):
        self.publish_velocity = publish_velocity
        self.log = log

        self.frame_id = rospy.get_param('~robot_frame_id')

        self.tf_buffer = Buffer(rospy.Duration(100.0))
        self.tf_listener = TransformListener(self.tf_buffer)

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
        State.enter(self, machine)

        if self.odometry is None:
            # Uh oh, we don't know where we are ..
            self.odometry = Odometry()

        self.target_pose = self.calculate_target_position(self.odometry)
        self.avoiding_obstacle = True

        # Send an obstacle to the map so that we know for next time where it is
        self.publish_obstacle(self.bumper_location)

    def exit(self, machine: StateMachine):
        State.exit(self, machine)
        self.avoiding_obstacle = False
        self.target_pose = None

    def update(self, machine: StateMachine):
        if State.update(self, machine):
            self.log('Updating reversing state ...')

            if self.avoiding_obstacle:
                self.log('Updating reversing state: Avoiding obstacle...')

                # Move backwards until we reach the requested distance moved
                current_distance = self.distance(self.target_pose)
                if current_distance > self.distance_tolerance:
                    # Not far enough away, keep going backwards
                    linear_velocity = self.linear_vel(self.target_pose)
                    angular_velocity = self.angular_vel(self.target_pose)

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
                    self.target_pose = None

                    self.log('Reversing state: Updating velocity to zero')
                    twist = Twist()
                    self.publish_velocity(twist)
            else:
                if self.velocity and is_not_zero_velocity(self.velocity):
                    machine.go_to_state(ScuttleMovingState.state_name)
                else:
                    machine.go_to_state(ScuttleStoppedState.state_name)

    def calculate_target_position(self, current_pose: Odometry) -> Pose:
        # The pose is in the odometry frame. We need to migrate that to the robot chassis frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                current_pose.header.frame_id,
                current_pose.header.stamp,
                rospy.Duration(1))
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Bad stuff happens here
            pass

        initial_pose = Pose()
        initial_pose.position.x = -0.3
        return tf2_geometry_msgs.do_transform_pose(initial_pose, transform)

    def distance(self, pose: Pose):
        dx = pose.position.x - self.pose.linear.x
        dy = pose.position.y - self.pose.linear.y
        return sqrt(pow(dx, 2) + pow(dy, 2))

    def velocity_with_ramp(self, current_velocity: float, desired_velocity: float, acceleration: float) -> float:
        if desired_velocity == current_velocity:
            return desired_velocity
        else:
            if desired_velocity > current_velocity:
                # accelerating
                achievable_velocity = current_velocity + acceleration / self.rate_in_hz
                if achievable_velocity > desired_velocity:
                    # desired acceleration is less than the possible acceleration
                    return desired_velocity
                else:
                    # desired acceleration is more than the possible acceleration
                    return achievable_velocity
            else:
                achievable_velocity = current_velocity - acceleration / self.rate_in_hz
                if achievable_velocity < desired_velocity:
                    # desired deceleration is less than the possible deceleration
                    return desired_velocity
                else:
                    # desired deceleration is more than the possible decelaration
                    return achievable_velocity

    def linear_vel(self, pose: Pose, constant=0.5):
        # We want to drive at this velocity
        desired_velocity = constant * self.distance(pose)

        # But we have maximum accelerations and deccellerations
        # If we don't have those SCUTTLE will stand on its rear wheels (which
        # isn't possible in real life)
        velocity = self.velocity_with_ramp(self.vx, desired_velocity, self.max_linear_acceleration)
        if abs(velocity) > self.max_linear_velocity:
            return copysign(self.max_linear_velocity, velocity)
        else:
            return velocity

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
