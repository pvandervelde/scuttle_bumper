#!/usr/bin/env python3

# Python
from __future__ import annotations
from typing import Callable

# ROS
from nav_msgs.msg import Odometry

class State(object):

    def __init__(self):
        pass

    @property
    def name(self):
        return ''

    def enter(self, machine: StateMachine):
        pass

    def exit(self, machine: StateMachine):
        pass

    def update(self, machine: StateMachine):
        pass

    def set_bumper_state(self, bumper_state: int, bumper_location: int):
        self.bumper_state = bumper_state
        self.bumper_location = bumper_location

    def set_odometry(self, odometry: Odometry):
        self.odometry = odometry
        self.velocity = odometry.twist.twist

class StateMachine(object):

    def __init__(self, log: Callable[[str], None]):
        self.state = None
        self.states = {}
        self.log = log

    def add_state(self, state: State):
        self.states[state.name] = state

    def go_to_state(self, state_name: str):
        if self.state:
            self.log('TrajectorySupervisor - StateMachine: Exiting {0}'.format(self.state.name))
            self.state.exit(self)
        self.state = self.states[state_name]
        self.log('TrajectorySupervisor - StateMachine: Entering {0}'.format(self.state.name))
        self.state.enter(self)

    def update(self):
        if self.state:
            self.log('TrajectorySupervisor - StateMachine: Updating {0}'.format(self.state.name))
            self.state.update(self)
