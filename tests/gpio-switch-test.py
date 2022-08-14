import datetime
import signal
import sys

import RPi.GPIO as GPIO
GPIO.setwarnings(False)

STATE_RELEASED = 0
STATE_PRESSED = 1

BUTTON_NONE = 0
BUTTON_LEFT = 1
BUTTON_RIGHT = 2
BUTTON_BOTH = 4

class Debounce:
    # Implements a debounce approach as described here: https://www.kennethkuhn.com/electronics/debounce.c
    # Comments from that file duplicated here for ease of use
    #
    # written by Kenneth A. Kuhn
    #
    # This is an algorithm that debounces or removes random or spurious
    # transistions of a digital signal read as an input by a computer.  This is
    # particularly applicable when the input is from a mechanical contact.  An
    # integrator is used to perform a time hysterisis so that the signal must
    # persistantly be in a logical state (0 or 1) in order for the output to change
    # to that state.  Random transitions of the input will not affect the output
    # except in the rare case where statistical clustering is longer than the
    # specified integration time.
    #
    # The following example illustrates how this algorithm works.  The sequence
    # labeled, real signal, represents the real intended signal with no noise.  The
    # sequence labeled, corrupted, has significant random transitions added to the
    # real signal.  The sequence labled, integrator, represents the algorithm
    # integrator which is constrained to be between 0 and 3.  The sequence labeled,
    # output, only makes a transition when the integrator reaches either 0 or 3.
    # Note that the output signal lags the input signal by the integration time but
    # is free of spurious transitions.
    #
    # real signal 0000111111110000000111111100000000011111111110000000000111111100000
    # corrupted   0100111011011001000011011010001001011100101111000100010111011100010
    # integrator  0100123233233212100012123232101001012321212333210100010123233321010
    # output      0000001111111111100000001111100000000111111111110000000001111111000
    #
    # I have been using this algorithm for years and I show it here as a code
    # fragment in C.  The algorithm has been around for many years but does not seem
    # to be widely known.  Once in a rare while it is published in a tech note.  It
    # is notable that the algorithm uses integration as opposed to edge logic
    # (differentiation).  It is the integration that makes this algorithm so robust
    # in the presence of noise.
    # ******************************************************************************/


    # The following parameters tune the algorithm to fit the particular
    # application.  The example numbers are for a case where a computer samples a
    # mechanical contact 10 times a second and a half-second integration time is
    # used to remove bounce.  Note: DEBOUNCE_TIME is in seconds and SAMPLE_FREQUENCY
    # is in Hertz
    def __init__(self, debounce_time_in_seconds: float, sample_frequency_in_hz: int, low_value: int, high_value: int, starting_value: int) -> None:
        self.maximum = debounce_time_in_seconds * sample_frequency_in_hz

        self.low_value = low_value
        self.high_value = high_value

        self.value = starting_value
        self.integrator = 0 if self.value == self.low_value else self.maximum

    def record_low_value(self):
        # Step 1: Update the integrator based on the input signal.  Note that the
        # integrator follows the input, decreasing or increasing towards the limits as
        # determined by the input state (0 or 1).

        if (self.integrator > 0):
            self.integrator -= 1

    def record_high_value(self):
        if (self.integrator < self.maximum):
            self.integrator += 1

    def current_value(self) -> int:
        # Step 2: Update the output state based on the integrator.  Note that the
        # output will only change states if the integrator has reached a limit, either
        # 0 or MAXIMUM.

        if (self.integrator <= 0):
            self.integrator = 0 # defensive code if integrator got corrupted
            self.value = self.low_value
        else:
            if (self.integrator >= self.maximum):
                self.integrator = self.maximum # defensive code if integrator got corrupted
                self.value = self.high_value

        return self.value


class GpioSwitchTest(object):
    def __init__(self):

        # Setup debouncing for the limit switch.
        #
        # Default debounce time for switches is about 20 milliseconds, but we'll use a bit less
        # time here so that we get at least a few messages before saying that we have a bounce,
        # see: https://www.eejournal.com/article/ultimate-guide-to-switch-debounce-part-4/
        # Default contact frequency is about 15 Hz
        debounce_time_in_seconds = 0.02
        sample_frequency_in_hz = 25

        # Channels
        # Based on: https://linuxhint.com/gpio-pinout-raspberry-pi/
        # Using Pin 17 as 3.3V
        # Using Pin 20 as Ground
        # Using the following pins for data
        # Pin 19 -- Front left bumper switch
        # Pin 21 -- Front right bumper switch
        # Pin 18 -- Rear left bumper switch
        # Pin 22 -- Rear right bumper switch
        self.left_switch_pin = 19
        self.right_switch_pin = 21

        # On start-up assume the bumper isn't touching anything. If it is then we'll be notified
        # soon enough.
        self.left_debounce = Debounce(
            debounce_time_in_seconds,
            sample_frequency_in_hz,
            STATE_RELEASED,
            STATE_PRESSED,
            STATE_RELEASED)

        self.right_debounce = Debounce(
            debounce_time_in_seconds,
            sample_frequency_in_hz,
            STATE_RELEASED,
            STATE_PRESSED,
            STATE_RELEASED)

        self.stopping = False

        # Setup the GPIO pins etc
        self.set_gpio()


    def cleanup(self):
        # Remove event detection
        GPIO.remove_event_detect(self.left_switch_pin)
        GPIO.remove_event_detect(self.right_switch_pin)

        # unset all the pins
        GPIO.cleanup()

    def event_callback_switch(self, channel: int):
        if channel == self.left_switch_pin:
            self.record_event_switch(self.left_switch_pin, self.left_debounce)

        if channel == self.right_switch_pin:
            self.record_event_switch(self.right_switch_pin, self.right_debounce)

    def record_event_switch(self, channel: int, debounce: Debounce):
        if GPIO.input(channel) == GPIO.HIGH:
            debounce.record_high_value()
        else:
            debounce.record_low_value()

    def publish(self):
        while not self.stopping:
            left_bumper_state = self.left_debounce.current_value()
            right_bumper_state = self.right_debounce.current_value()

            now = datetime.datetime.now()
            now_string = now.strftime("%Y-%m-%d %H:%M:%S")

            if left_bumper_state == STATE_PRESSED:
                if right_bumper_state == STATE_PRESSED:
                    # Both left and right pressed -> obstacle probably in the middle
                    print(f"{now_string} - Left: 1; Right: 1")
                else:
                    print(f"{now_string} - Left: 1; Right: 0")
            else:
                if right_bumper_state == STATE_PRESSED:
                    print(f"{now_string} - Left: 0; Right: 1")
                else:
                    print(f"{now_string} - Left: 0; Right: 0")

    def signal_handler(self, signum, frame):
        signal.signal(signum, signal.SIG_IGN) # ignore additional signals
        self.stopping = True

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
        translator = GpioSwitchTest()
        translator.publish()
        translator.cleanup()
    except Exception:
        pass

if __name__ == '__main__':
    main()