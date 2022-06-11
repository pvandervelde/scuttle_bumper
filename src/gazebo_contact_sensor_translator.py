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

        # Listen for the gazebo bumper state
        self.gz_sub = rospy.Subscriber('/scuttle/sensor/bumper/gazebo', ContactsState, self.monitor_bumper_callback)

        # Publish the scuttle relevant bumper state
        self.bumper_pub = rospy.Publisher('/scuttle/sensor/bumper/events', BumperEvent, queue_size=10)

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
            BumperEvent.NO_CONTACT,
            BumperEvent.CONTACT,
            BumperEvent.NO_CONTACT)

        # Publish at the same rate Gazebo publishes
        self.rate = rospy.Rate(sample_frequency_in_hz)

    def monitor_bumper_callback(self, msg: ContactsState):
        rospy.logdebug("Received gazebo contact message with state %s", msg)

        # If there are states, then we have a contact, i.e. the bumper hit something. We don't
        # care what we hit, at which angle or force etc. etc.. We just care we hit something
        #
        # Note setting an integer should be thread safe in Python so no need for a threading lock.
        if msg.states:
            rospy.logdebug("Gazebo message indicates switch closed")
            self.debounce.record_high_value()
        else:
            rospy.logdebug("Gazebo message indicates switch open")
            self.debounce.record_low_value()

    def publish(self):
        while not rospy.is_shutdown():
            msg = BumperEvent()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.bumper_frame_id # coordinate frame of the bumper

            msg.state = self.debounce.current_value()

            # We assume that the object is placed in contact with the bumper
            # when the bumper is not compressed. This means that the contact is
            # several mm further in the x-direction than it actually is.
            #
            # Note that the bumper section for which we get signals is half the
            # full bumper.
            #
            # We will use a line type obstacle that is about 1/3 the size of the
            # bumper section. We don't exactly know where the object is, it
            # could be in the middle of the total bumper, right on top of
            # the contacts or even out on the edge. There is no way to figure it
            # out, so we just assume the bumper roughly got hit near the contacts.
            #
            # We assume that the bumpers are placed perpendicular to the driving
            # axes.
            msg.contacted_object = Polygon(
                [
                    Point(x=0.0, y=-1/6 * self.bumper_width, z=0.0),
                    Point(x=0.0, y=1/6 * self.bumper_width, z=0.0)
                ]
            )

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