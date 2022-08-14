#!/usr/bin/env python3

# ROS
import rospy

# GPIO
import RPi.GPIO as GPIO
GPIO.setwarnings(False)

# SCUTTLE
from scuttle_ros_msgs.msg import BumperEvent
from debounce import Debounce

# Translate the limit switch state to the scuttle ContactSwitch messages
class BumperSwitchTranslator(object):
    def __init__(self):
        rospy.init_node('bumper_switch_translator')

        self.bumper_frame_id = rospy.get_param('~bumper_frame_id')

        # Setup debouncing for the limit switch.
        #
        # Default debounce time for switches is about 20 milliseconds, but we'll use a bit less
        # time here so that we get at least a few messages before saying that we have a bounce,
        # see: https://www.eejournal.com/article/ultimate-guide-to-switch-debounce-part-4/
        # Default contact frequency is about 15 Hz
        debounce_time_in_seconds = rospy.get_param('~switch_debounce_time_in_seconds', 0.02)
        sample_frequency_in_hz = rospy.get_param('~sample_frequency_in_hz', 25)

        # Channels
        # Based on: https://linuxhint.com/gpio-pinout-raspberry-pi/
        # Using Pin 17 as 3.3V
        # Using Pin 20 as Ground
        # Using the following pins for data
        # Pin 19 -- Front left bumper switch
        # Pin 21 -- Front right bumper switch
        # Pin 18 -- Rear left bumper switch
        # Pin 22 -- Rear right bumper switch
        self.left_switch_pin = rospy.get_param('~gpio_pin_left_switch', 19)
        self.right_switch_pin = rospy.get_param('~gpio_pin_right_switch', 21)

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

        # Setup the GPIO pins etc
        self.set_gpio()

        self.rate = rospy.Rate(sample_frequency_in_hz)

        # Publish the scuttle relevant bumper state
        self.bumper_pub = rospy.Publisher('/scuttle/sensor/bumper/events', BumperEvent, queue_size=10)

    def cleanup(self):
        # Remove event detection
        GPIO.remove_event_detect(self.left_switch_pin)
        GPIO.remove_event_detect(self.right_switch_pin)

        # unset all the pins
        GPIO.cleanup()

    def event_callback_switch(self, channel: int):
        if channel == self.left_switch_pin:
            rospy.logdebug("Bumper event - Left")
            self.record_event_switch(self.left_switch_pin, self.left_debounce)

        if channel == self.right_switch_pin:
            rospy.logdebug("Bumper event - Right")
            self.record_event_switch(self.right_switch_pin, self.right_debounce)

    def record_event_switch(self, channel: int, debounce: Debounce):
        if GPIO.input(channel) == GPIO.HIGH:
            rospy.logdebug("Bumper pressed")
            debounce.record_high_value()
        else:
            rospy.logdebug("Bumper released")
            debounce.record_low_value()

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

    def set_gpio(self):
        # Set the the GPIO mode to the easy mode, i.e. the pin ordering number that is for the board
        # The other mode is the one for the processor which is much harder to work with.
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        else:
            pass

        GPIO.setup(self.left_switch_pin, GPIO.IN)
        GPIO.add_event_detect(self.left_switch_pin, GPIO.BOTH)
        GPIO.add_event_callback(self.left_switch_pin, self.event_callback_switch)

        GPIO.setup(self.right_switch_pin, GPIO.IN)
        GPIO.add_event_detect(self.right_switch_pin, GPIO.BOTH)
        GPIO.add_event_callback(self.right_switch_pin, self.event_callback_switch)

def main():
    try:
        translator = BumperSwitchTranslator()
        translator.publish()
        translator.cleanup()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()