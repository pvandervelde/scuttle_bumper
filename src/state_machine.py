#!/usr/bin/env python3

# Python
from __future__ import annotations
from typing import Callable

# ROS
from nav_msgs.msg import Odometry

class State(object):

    def __init__(self, bumper_state: int, bumper_location: int):
        self.entered = False
        self.exited = False

        self.bumper_state = bumper_state
        self.bumper_location = bumper_location

        self.odometry = None
        self.velocity = None

    @property
    def name(self):
        return ''

    @property
    def has_entered(self):
        return self.entered

    @property
    def has_exited(self):
        return self.exited

    def enter(self, machine: StateMachine):
        self.entered = True
        self.exited = False
        pass

    def exit(self, machine: StateMachine):
        self.entered = False
        self.exited = True
        pass

    def update(self, machine: StateMachine):
        return True

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
