# python

# ROS
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry

# locals
from state_machine import State, StateMachine
from scuttle_bumper_response_states import ScuttleMovingState, ScuttleBumperObstacleAvoidingState, ScuttleStoppedState

# Constants

BUMPER_LOCATION_NONE = 0
BUMPER_LOCATION_MIDDLE = 4

BUMPER_STATE_RELEASED = 0
BUMPER_STATE_PRESSED = 1

# Helper functions

def log(msg: str):
    pass

def publish_velocity(velocity: Twist):
    pass

def calculate_target_position(odom: Odometry) -> Pose:
    return Pose()

def does_state_indicate_bumper_contact(state: int) -> bool:
    return state == BUMPER_STATE_PRESSED

def odometry_with_linear_velocity(magnitude: float) -> Odometry:
    odom = Odometry()
    odom.twist.twist = Twist(Vector3(magnitude, 0, 0), Vector3(0, 0, 0))

    return odom

# Test ScuttleStoppedState

def test_should_not_change_state_when_velocity_is_zero_during_stopped_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
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

    sm.update()

    assert state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited

def test_should_switch_to_moving_state_when_velocity_is_positive_during_stopped_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    odom = odometry_with_linear_velocity(1.0)
    state_1.set_odometry(odom)

    assert not state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited

    sm.update()

    assert not state_1.has_entered
    assert state_2.has_entered

    assert state_1.has_exited
    assert not state_2.has_exited

def test_should_switch_to_moving_state_when_velocity_is_negative_during_stopped_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    odom = odometry_with_linear_velocity(-1.0)
    state_1.set_odometry(odom)

    assert not state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited

    sm.update()

    assert not state_1.has_entered
    assert state_2.has_entered

    assert state_1.has_exited
    assert not state_2.has_exited

def test_should_switch_to_obstacle_avoiding_state_when_bumper_is_pressed_with_no_velocity_during_stopped_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    state_3 = ScuttleBumperObstacleAvoidingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, publish_velocity=publish_velocity, calculate_target_position=calculate_target_position, log=log)
    sm.add_state(state_3)

    state_1.set_bumper_state(BUMPER_STATE_PRESSED, BUMPER_LOCATION_MIDDLE)

    assert not state_1.has_entered
    assert not state_2.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    assert not state_2.has_exited
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_2.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    assert not state_2.has_exited

    sm.update()

    assert not state_1.has_entered
    assert not state_2.has_entered
    assert state_3.has_entered

    assert state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited

# Test ScuttleMovingState

def test_should_not_change_state_when_velocity_is_not_zero_during_moving_state_update():
    pass

def test_should_switch_to_stopped_state_when_velocity_is_zero_during_moving_state_update():
    pass

def test_should_switch_to_obstacle_avoiding_state_when_bumper_is_pressed_with_no_velocity_during_moving_state_update():
    pass

def test_should_switch_to_obstacle_avoiding_state_when_bumper_is_pressed_with_velocity_during_moving_state_update():
    pass

# Test ScuttleBumperObstacleAvoidingState