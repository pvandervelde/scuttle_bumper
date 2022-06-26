#!/usr/bin/env python3

# ROS
import rospy
from std_msgs.msg import Int16

# Gazebo
from gazebo_msgs.msg import ContactsState, ContactState
from geometry_msgs.msg import Point, Polygon

# SCUTTLE
from scuttle_ros_msgs.msg import BumperEvent
from debounce import Debounce

# Translate the Gazebo contact sensor messages to the scuttle ContactSwitch messages
class GazeboContactSensorTranslator(object):
    def __init__(self):
        rospy.init_node('gazebo_contact_sensor_translator')

        self.bumper_frame_id = rospy.get_param('~bumper_frame_id')
        self.bumper_name = rospy.get_param('~bumper_name')
        self.bumper_width = rospy.get_param('~bumper_section_width')
        self.bumper_height = rospy.get_param('~bumper_section_height')

        # Setup debouncing for the Gazebo contact switch. It turns out that the Gazebo contact switch
        # occasionally reports a non-contact during a period of contact. It's often only a single
        # loss of contact, possibly due to the fact that all Gazebo sensors have noise
        #
        # Default debounce time for switches is about 20 milliseconds, but we'll use a bit more
        # time here so that we get at least a few messages before saying that we have a bounce,
        # see: https://www.eejournal.com/article/ultimate-guide-to-switch-debounce-part-4/
        # Default contact frequency is about 15 Hz
        debounce_time_in_seconds = rospy.get_param('~gazebo_contact_debounce_time_in_seconds', 0.02)
        sample_frequency_in_hz = rospy.get_param('~gazebo_contact_sample_frequency_in_hz', 15)

        # On start-up assume the bumper isn't touching anything. If it is then we'll be notified
        # soon enough.
        self.left_debounce = Debounce(
            debounce_time_in_seconds,
            sample_frequency_in_hz,
            BumperEvent.RELEASED,
            BumperEvent.PRESSED,
            BumperEvent.RELEASED)

        self.right_debounce = Debounce(
            debounce_time_in_seconds,
            sample_frequency_in_hz,
            BumperEvent.RELEASED,
            BumperEvent.PRESSED,
            BumperEvent.RELEASED)

        # Publish at the same rate Gazebo publishes
        self.rate = rospy.Rate(sample_frequency_in_hz)

        # Publish the scuttle relevant bumper state
        self.bumper_pub = rospy.Publisher('/scuttle/sensor/bumper/events', BumperEvent, queue_size=10)

        # Listen for the gazebo bumper state
        self.gz_left_sub = rospy.Subscriber('/scuttle/sensor/bumper/gazebo/front/left', ContactsState, self.monitor_left_bumper_callback)
        self.gz_right_sub = rospy.Subscriber('/scuttle/sensor/bumper/gazebo/front/right', ContactsState, self.monitor_right_bumper_callback)

    def monitor_left_bumper_callback(self, msg: ContactsState):
        rospy.logdebug("Received contact message from left bumper with state %s", msg)

        # If there are states, then we have a contact, i.e. the bumper hit something. We don't
        # care what we hit, at which angle or force etc. etc.. We just care we hit something
        #
        # Note setting an integer should be thread safe in Python so no need for a threading lock.
        if msg.states:
            rospy.logdebug("Gazebo message indicates left bumper hit")
            self.left_debounce.record_high_value()
        else:
            rospy.logdebug("Gazebo message indicates left bumper released")
            self.left_debounce.record_low_value()

    def monitor_right_bumper_callback(self, msg: ContactsState):
        rospy.logdebug("Received contact message from right bumper with state %s", msg)

        # If there are states, then we have a contact, i.e. the bumper hit something. We don't
        # care what we hit, at which angle or force etc. etc.. We just care we hit something
        #
        # Note setting an integer should be thread safe in Python so no need for a threading lock.
        if msg.states:
            rospy.logdebug("Gazebo message indicates right bumper hit")
            self.right_debounce.record_high_value()
        else:
            rospy.logdebug("Gazebo message indicates right bumper released")
            self.right_debounce.record_low_value()

    def publish(self):
        while not rospy.is_shutdown():
            left_bumper_state = self.left_debounce.current_value()
            right_bumper_state = self.right_debounce.current_value()

            location = 0
            state = BumperEvent.RELEASED
            if left_bumper_state == BumperEvent.PRESSED:
                if right_bumper_state == BumperEvent.PRESSED:
                    # Both left and right pressed -> obstacle probably in the middle
                    location = BumperEvent.CENTER
                    state = BumperEvent.PRESSED
                else:
                    # Left only pressed
                    location = BumperEvent.LEFT
                    state = BumperEvent.PRESSED
            else:
                if right_bumper_state == BumperEvent.PRESSED:
                    # right only pressed
                    location = BumperEvent.RIGHT
                    state = BumperEvent.PRESSED
                else:
                    # Neither pressed
                    location = BumperEvent.NONE
                    state = BumperEvent.RELEASED

            msg = BumperEvent()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.bumper_frame_id # coordinate frame of the bumper

            msg.bumper = location
            msg.state = state

            self.bumper_pub.publish(msg)

            self.rate.sleep()

def main():
    try:
        translator = GazeboContactSensorTranslator()
        translator.publish()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()