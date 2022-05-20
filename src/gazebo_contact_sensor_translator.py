#!/usr/bin/env python3

# ROS
import rospy

# Gazebo
from gazebo_msgs.msg import ContactsState, ContactState

# SCUTTLE
from scuttle_ros_msgs.msg import ContactSwitch

# Translate the Gazebo contact sensor messages to the scuttle ContactSwitch messages
class GazeboContactSensorTranslator(object):
    def __init__(self):
        rospy.init_node('gazebo_contact_sensor_translator')

        # On start-up assume the bumper isn't touching anything. If it is then we'll be notified
        # soon enough.
        self.gazebo_bumper_state = ContactSwitch.SWITCH_OPEN

        self.bumper_id = rospy.get_param('~bumper_id')
        self.bumper_name = rospy.get_param('~bumper_name')

        # Listen for the gazebo bumper state
        self.gz_sub = rospy.Subscriber('/scuttle/sensor/bumper/gazebo', ContactsState, self.monitor_bumper_callback)

        # Publish the scuttle relevant bumper state
        self.bumper_pub = rospy.Publisher('/scuttle/sensor/bumper/states', ContactSwitch)

        self.rate = rospy.Rate(10)

    def monitor_bumper_callback(self, msg):
        rospy.logdebug("Received gazebo contact message with state %s [%d]", msg, len(msg.states))
        # If there are states, then we have a contact, i.e. the bumper hit something. We don't
        # care what we hit, at which angle or force etc. etc.. We just care we hit something
        #
        # Note setting an integer should be thread safe in Python so no need for a threading lock.
        if msg.states:
            rospy.logdebug("Gazebo message indicates switch closed")
            self.gazebo_bumper_state = ContactSwitch.SWITCH_CLOSED # Should this be depressed / engaged?
        else:
            rospy.logdebug("Gazebo message indicates switch open")
            self.gazebo_bumper_state = ContactSwitch.SWITCH_OPEN # Should this be unpressed / disengaged?


    def publish(self):

        while not rospy.is_shutdown():
            msg = ContactSwitch()
            msg.id = self.bumper_id
            msg.name = self.bumper_name
            msg.state = self.gazebo_bumper_state

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