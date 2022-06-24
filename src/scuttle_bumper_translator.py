#!/usr/bin/env python3

# python
from copy import deepcopy
from threading import Lock

# ROS
import rospy
from geometry_msgs.msg import Polygon

# SCUTTLE
from scuttle_ros_msgs.msg import BumperEvent, Obstacle, Obstacles

class ScuttleBumperTranslator(object):
    """Process messages from the bumpers and translate those into obstacle locations

    Receives messages on the '/scuttle/sensor/bumper/states' topic and publishes
    'Obstacle' messages on the '/scuttle/sensor/obstacle' topic.
    """

    def __init__(self):
        rospy.init_node('scuttle_bumper_translator')

        self.lock = Lock()
        self.bumper_state = BumperEvent.RELEASED
        self.bumper_location = BumperEvent.NONE

        # Listen for the bumper state changes
        self.bumper_sub = rospy.Subscriber('/scuttle/sensor/bumper/events', BumperEvent, self.monitor_bumper_callback)

        # Publish the obstacles
        self.obstacle_pub = rospy.Publisher('/scuttle/sensor/obstacle', Obstacles, queue_size=10)

        # Publish at the given rate
        sample_frequency_in_hz = rospy.get_param('~update_frequency_in_hz', 15)
        self.rate = rospy.Rate(sample_frequency_in_hz)

    def monitor_bumper_callback(self, msg: BumperEvent):
        rospy.logdebug("Received BumperEvent message for %s with state %d", msg.header.frame_id, msg.state)

        self.lock.acquire()
        try:
            self.bumper_state = msg.state
            self.bumper_location = msg.bumper
        finally:
            self.lock.release()

    def publish(self):
        while not rospy.is_shutdown():
            # Grab a copy of the dictionary because we don't want to block while we're
            # figuring out what is going on.
            self.lock.acquire()
            try:
                state = self.bumper_state
                location = self.bumper_location
            finally:
                self.lock.release()

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

            # Record obstacle

            # Reverse current movement


            time_of_recording = rospy.Time.now()
            obstacles = []
            for k,v in bumper_state.items():
                if v.state == BumperEvent.CONTACT:
                    obstacle = Obstacle()
                    obstacle.header.stamp = time_of_recording
                    obstacle.header.frame_id = k

                    obstacle.object = v.contacted_object
                    obstacle.object_type = Obstacle.OBJECT_TYPE.OBSTACLE

                    obstacles.append(obstacle)

            # Send obstacle message with coordinates of the obstacles if there are any
            msg = Obstacles()
            msg.header.stamp = time_of_recording
            msg.objects = obstacles

            self.obstacle_pub.publish(msg)

            self.rate.sleep()

def main():
    try:
        translator = ScuttleBumperTranslator()
        translator.publish()
    except rospy.ROSInterruptException:
        # Do we log stuff here?
        pass

if __name__ == '__main__':
    main()