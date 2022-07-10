# python

# ROS
from geometry_msgs.msg import Twist

# locals
from state_machine import State, StateMachine

class MockState(State):
    def __init__(self, name: str):
        self.state_name = name
        self.entered = False
        self.exited = False
        self.updated = False

    @property
    def name(self):
        return self.state_name

    @property
    def has_entered(self):
        return self.entered

    @property
    def has_exited(self):
        return self.exited

    @property
    def has_updated(self):
        return self.updated

    def enter(self, machine: StateMachine):
        self.entered = True
        self.exited = False
        State.enter(self, machine)

    def exit(self, machine: StateMachine):
        self.entered = False
        self.exited = True
        State.exit(self, machine)

    def update(self, machine: StateMachine):
        self.updated = True

def log(msg: str):
    pass

def test_should_exit_state_when_changing_state_in_state_machine():
    sm = StateMachine(log=log)

    state_1 = MockState("a")
    sm.add_state(state_1)

    state_2 = MockState("b")
    sm.add_state(state_2)

    assert not state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited

    sm.go_to_state(state_2.name)

    assert not state_1.has_entered
    assert state_2.has_entered

    assert state_1.has_exited
    assert not state_2.has_exited

def test_should_update_when_state_active():
    sm = StateMachine(log=log)

    state_1 = MockState("a")
    sm.add_state(state_1)

    assert not state_1.has_entered
    assert not state_1.has_exited
    assert not state_1.has_updated
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_1.has_exited
    assert not state_1.has_updated

    sm.update()

    assert state_1.has_entered
    assert not state_1.has_exited
    assert state_1.has_updated

def test_should_not_update_when_state_not_active():
    sm = StateMachine(log=log)

    state_1 = MockState("a")
    sm.add_state(state_1)

    assert not state_1.has_entered
    assert not state_1.has_exited
    assert not state_1.has_updated

    sm.update()

    assert not state_1.has_entered
    assert not state_1.has_exited
    assert not state_1.has_updated
