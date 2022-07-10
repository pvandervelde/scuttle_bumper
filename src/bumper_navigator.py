#!/usr/bin/env python3

# python
from math import atan2, copysign, isclose, pow, sqrt
from threading import Lock

# ROS
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

# SCUTTLE
from scuttle_ros_msgs.msg import BumperEvent

# LOCAL
from state_machine import StateMachine
from scuttle_bumper_response_states import ScuttleStoppedState, ScuttleMovingState, ScuttleBumperObstacleAvoidingState

class ScuttleBumperNavigator(object):
    def __init__(self):
        rospy.init_node('scuttle_bumper_navigator')

        # Define the locks for setting object level fields. We're assuming that the subscription and
        # the publisher could run on different threads, especially if the subscription comes in
        # via a hardware interrupt (via the network stack)
        #
        # Also have separate locsk for the obstacle subscription and the odometry subscription
        # because we don't want these one subscriber to block the other subscriber.
        self.bumper_lock = Lock()
        self.velocity_lock = Lock()

        # Create the state machine
        self.states = [
            ScuttleStoppedState(self.is_bumper_pressed),
            ScuttleMovingState(self.is_bumper_pressed),
            ScuttleBumperObstacleAvoidingState(self.publish_move_command, self.log)
        ]

        self.machine = StateMachine(self.log)
        for state in self.states:
            self.machine.add_state(state)

        self.machine.go_to_state(ScuttleStoppedState.state_name)

        # Listen for the bumper events
        self.obstacle_sub = rospy.Subscriber('/scuttle/sensor/bumper/events', BumperEvent, self.monitor_obstacle_callback)

        # Keep track of the position of the robot
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.monitor_odometry)

        # Publish velocity commands in case we hit an obstacle
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.costmap_pub = rospy.Publisher('/obstacles', PointCloud2, queue_size=10)

        # Publish at the given rate
        sample_frequency_in_hz = rospy.get_param('~update_frequency_in_hz', 15)
        self.rate = rospy.Rate(sample_frequency_in_hz)

    def is_bumper_pressed(self, state: int):
        return state == BumperEvent.PRESSED

    def log(self, msg: str):
        rospy.logdebug(msg)

    def monitor_obstacle_callback(self, msg: BumperEvent):
        bumper_state = msg.state
        bumper_location = msg.bumper

        for state in self.states:
            state.set_bumper_state(bumper_state, bumper_location)

    def monitor_odometry(self, msg: Odometry):
        for state in self.states:
            state.set_odometry(msg)

    def publish(self):
        while not rospy.is_shutdown():
            self.machine.update()

            self.rate.sleep()

    def publish_move_command(self, velocity: Twist):
        self.cmd_vel_pub.publish(velocity)

def main():
    try:
        navigator = ScuttleBumperNavigator()
        navigator.publish()
    except rospy.ROSInterruptException:
        # Do we log stuff here
        pass

if __name__ == '__main__':
    main()