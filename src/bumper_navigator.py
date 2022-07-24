#!/usr/bin/env python3

# python
from math import atan2, copysign, isclose, pow, sqrt
from threading import Lock

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import tf2_geometry_msgs
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

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

        self.chassis_frame_id = rospy.get_param('~robot_frame_id')
        self.odom_frame_id = rospy.get_param('~odom_frame_id')

        # Get the TF buffer so that we can translate between the odometry frame and the robot frame
        self.tf_buffer = Buffer(rospy.Duration(100.0))
        self.tf_listener = TransformListener(self.tf_buffer)

        # Publish at the given rate
        sample_frequency_in_hz = rospy.get_param('~update_frequency_in_hz', 15)
        self.rate = rospy.Rate(sample_frequency_in_hz)

        # Create the state machine
        max_linear_velocity = rospy.get_param('~max_linear_velocity')
        min_linear_velocity = rospy.get_param('~min_linear_velocity')
        max_linear_acceleration = rospy.get_param('~max_linear_acceleration')
        distance_tolerance = rospy.get_param('~distance_tolerance')
        self.states = [
            ScuttleStoppedState(BumperEvent.RELEASED, BumperEvent.NONE, self.is_bumper_pressed),
            ScuttleMovingState(BumperEvent.RELEASED, BumperEvent.NONE, self.is_bumper_pressed),
            ScuttleBumperObstacleAvoidingState(
                BumperEvent.RELEASED,
                BumperEvent.NONE,
                max_linear_velocity,
                min_linear_velocity,
                max_linear_acceleration,
                sample_frequency_in_hz,
                distance_tolerance,
                self.chassis_frame_id,
                self.get_time,
                self.publish_move_command,
                self.transform_from_base_to_odom,
                self.log)
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

    def get_time(self) -> rospy.Time:
        return rospy.Time.now()

    def is_bumper_pressed(self, state: int) -> bool:
        return state == BumperEvent.PRESSED

    def log(self, msg: str):
        rospy.logdebug(msg)

    def log_last_exception(self, msg: str):
        rospy.logerr(msg, exc_info=True)

    def transform_from_base_to_odom(self, current_pose: PoseStamped) -> Pose:
        # The pose is in the base frame. We need to migrate that to the odometry frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame_id,
                self.chassis_frame_id,
                rospy.Time.now(),
                rospy.Duration(1))

            return tf2_geometry_msgs.do_transform_pose(current_pose, transform)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.log_last_exception('Transform of pose from base to odom failed.')

        return None

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
        rospy.logerr('bumper navigator node failed.', exc_info=True)

if __name__ == '__main__':
    main()