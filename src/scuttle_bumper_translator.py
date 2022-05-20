#!/usr/bin/env python3

# ROS
import rospy

# SCUTTLE
from scuttle_ros_msgs.msg import ContactSwitch

class ScuttleBumperTranslator(object):
    def __init__(self):
        rospy.init_node('scuttle_bumper_translator')

    # Listen for ContactSwitch messages
    # Foreach message
    #    if contact && !last_contact:
    #        RecordObstacleOnMap
    #        SendObstacleMessageToSafetyController
    #     else:
    #         if contact:
    #             # New contact
    #         if !contact:
    #             # disconnected contact?
    #

def main():
    try:
        translator = ScuttleBumperTranslator()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()