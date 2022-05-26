#!/usr/bin/env python3

# ROS
import rospy
from std_msgs.msg import Int16

# Gazebo
from gazebo_msgs.msg import ContactsState, ContactState

# SCUTTLE
from scuttle_ros_msgs.msg import ContactSwitch
from debounce import Debounce

# Translate the Gazebo contact sensor messages to the scuttle ContactSwitch messages
class GazeboContactSensorTranslator(object):
    def __init__(self):
        rospy.init_node('gazebo_contact_sensor_translator')

        self.bumper_id = rospy.get_param('~bumper_id')
        self.bumper_name = rospy.get_param('~bumper_name')

        # Listen for the gazebo bumper state
        self.gz_sub = rospy.Subscriber('/scuttle/sensor/bumper/gazebo', ContactsState, self.monitor_bumper_callback)

        # Publish the scuttle relevant bumper state
        self.bumper_pub = rospy.Publisher('/scuttle/sensor/bumper/states', ContactSwitch, queue_size=10)
        self.bumper_gazebo_pub = rospy.Publisher('/scuttle/sensor/bumper/states_gz_{name}'.format(name=self.bumper_name), Int16, queue_size=10)

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
        self.debounce = Debounce(
            debounce_time_in_seconds,
            sample_frequency_in_hz,
            ContactSwitch.SWITCH_OPEN,
            ContactSwitch.SWITCH_CLOSED,
            ContactSwitch.SWITCH_OPEN)

        # Publish at the same rate Gazebo publishes
        self.rate = rospy.Rate(sample_frequency_in_hz)

    def monitor_bumper_callback(self, msg):
        rospy.logdebug("Received gazebo contact message with state %s [%d]", msg, len(msg.states))

        # If there are states, then we have a contact, i.e. the bumper hit something. We don't
        # care what we hit, at which angle or force etc. etc.. We just care we hit something
        #
        # Note setting an integer should be thread safe in Python so no need for a threading lock.
        if msg.states:
            rospy.logdebug("Gazebo message indicates switch closed")
            self.debounce.record_high_value()
            self.bumper_gazebo_pub.publish(ContactSwitch.SWITCH_CLOSED)
        else:
            rospy.logdebug("Gazebo message indicates switch open")
            self.debounce.record_low_value()
            self.bumper_gazebo_pub.publish(ContactSwitch.SWITCH_OPEN)

    def publish(self):
        while not rospy.is_shutdown():
            msg = ContactSwitch()
            msg.id = self.bumper_id
            msg.name = self.bumper_name
            msg.state = self.debounce.current_value()

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