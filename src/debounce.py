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

    # Do we need a timer to count through. The current design indicates that we need an external
    # caller that calls the methods on a reglar basis
