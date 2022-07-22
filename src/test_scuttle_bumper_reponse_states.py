# python
from math import isclose, pow
from pytest import approx

# ROS
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from nav_msgs.msg import Odometry

# locals
from state_machine import State, StateMachine
from scuttle_bumper_response_states import ScuttleMovingState, ScuttleBumperObstacleAvoidingState, ScuttleStoppedState

# Constants

BUMPER_LOCATION_NONE = 0
BUMPER_LOCATION_MIDDLE = 4

BUMPER_STATE_RELEASED = 0
BUMPER_STATE_PRESSED = 1

# Global variables == bad

PUBLISHED_VELOCITY : Twist = None
BASE_FRAME_TO_ODOM : Pose = Pose()

# Helper functions

def log(msg: str):
    print(msg)

def publish_velocity(velocity: Twist):
    global PUBLISHED_VELOCITY
    PUBLISHED_VELOCITY = velocity

def clear_globals():
    global PUBLISHED_VELOCITY
    PUBLISHED_VELOCITY = None

def transform_from_base_to_odom(pose_in_base_frame: Pose) -> Pose:
    global BASE_FRAME_TO_ODOM
    result = Pose()
    result.position = Point(
        BASE_FRAME_TO_ODOM.position.x + pose_in_base_frame.position.x,
        BASE_FRAME_TO_ODOM.position.y + pose_in_base_frame.position.y,
        BASE_FRAME_TO_ODOM.position.z + pose_in_base_frame.position.z,
    )

    return result

def does_state_indicate_bumper_contact(state: int) -> bool:
    return state == BUMPER_STATE_PRESSED

def odometry_with_linear_velocity(magnitude: float) -> Odometry:
    odom = Odometry()
    odom.twist.twist = Twist(Vector3(magnitude, 0, 0), Vector3(0, 0, 0))

    return odom

def odometry_with_linear_velocity_and_position(velocity_magnitude: float, x: float, y: float, z: float) -> Odometry:
    odom = odometry_with_linear_velocity(velocity_magnitude)

    pose = Pose()
    pose.position = Point(x, y, z)
    odom.pose.pose = pose

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

    state_3 = ScuttleBumperObstacleAvoidingState(
        BUMPER_STATE_RELEASED,
        BUMPER_LOCATION_NONE,
        max_linear_velocity=0.5,
        min_linear_velocity=0.1,
        max_linear_acceleration=1.0,
        rate_in_hz=10,
        distance_tolerance=0.001,
        publish_velocity=publish_velocity,
        transform_from_base_to_odom=transform_from_base_to_odom,
        log=log)
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
    sm = StateMachine(log=log)

    state_1 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
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

    assert state_1.has_entered
    assert not state_2.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited

def test_should_switch_to_stopped_state_when_velocity_is_zero_during_moving_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    odom = odometry_with_linear_velocity(0.0)
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

def test_should_switch_to_obstacle_avoiding_state_when_bumper_is_pressed_with_no_velocity_during_moving_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    state_3 = ScuttleBumperObstacleAvoidingState(
        BUMPER_STATE_RELEASED,
        BUMPER_LOCATION_NONE,
        max_linear_velocity=0.5,
        min_linear_velocity=0.1,
        max_linear_acceleration=1.0,
        rate_in_hz=10,
        distance_tolerance=0.001,
        publish_velocity=publish_velocity,
        transform_from_base_to_odom=transform_from_base_to_odom,
        log=log)
    sm.add_state(state_3)

    state_1.set_bumper_state(BUMPER_STATE_PRESSED, BUMPER_LOCATION_MIDDLE)

    odom = odometry_with_linear_velocity(0.0)
    state_1.set_odometry(odom)

    assert not state_1.has_entered
    assert not state_2.has_entered
    assert not state_3.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_2.has_entered
    assert not state_3.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited

    sm.update()

    assert not state_1.has_entered
    assert not state_2.has_entered
    assert state_3.has_entered

    assert state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited

def test_should_switch_to_obstacle_avoiding_state_when_bumper_is_pressed_with_velocity_during_moving_state_update():
    sm = StateMachine(log=log)

    state_1 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_1)

    state_2 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    state_3 = ScuttleBumperObstacleAvoidingState(
        BUMPER_STATE_RELEASED,
        BUMPER_LOCATION_NONE,
        max_linear_velocity=0.5,
        min_linear_velocity=0.1,
        max_linear_acceleration=1.0,
        rate_in_hz=10,
        distance_tolerance=0.001,
        publish_velocity=publish_velocity,
        transform_from_base_to_odom=transform_from_base_to_odom,
        log=log)
    sm.add_state(state_3)

    state_1.set_bumper_state(BUMPER_STATE_PRESSED, BUMPER_LOCATION_MIDDLE)

    odom = odometry_with_linear_velocity(1.0)
    state_1.set_odometry(odom)

    assert not state_1.has_entered
    assert not state_2.has_entered
    assert not state_3.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited
    sm.go_to_state(state_1.name)

    assert state_1.has_entered
    assert not state_2.has_entered
    assert not state_3.has_entered

    assert not state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited

    sm.update()

    assert not state_1.has_entered
    assert not state_2.has_entered
    assert state_3.has_entered

    assert state_1.has_exited
    assert not state_2.has_exited
    assert not state_3.has_exited

# Test ScuttleBumperObstacleAvoidingState

# On enter send obstacle message to the map
def test_should_mark_obstacle_on_map_when_obstacle_avoiding_state_is_entered():
    pass

# On update move backwards while not far enough back (with positive starting velocity)
#    while not backed up far enough keep moving backwards
def test_should_move_backwards_until_clear_when_obstacle_avoiding_state_updates_with_positive_starting_velocity():
    global BASE_FRAME_TO_ODOM

    sm = StateMachine(log=log)

    max_velocity = 0.5
    min_velocity = 0.1
    acceleration = 1.0
    rate = 5
    distance_tolerance = 0.01
    state_1 = ScuttleBumperObstacleAvoidingState(
        BUMPER_STATE_RELEASED,
        BUMPER_LOCATION_NONE,
        max_linear_velocity=max_velocity,
        min_linear_velocity=min_velocity,
        max_linear_acceleration=acceleration,
        rate_in_hz=rate,
        distance_tolerance=distance_tolerance,
        publish_velocity=publish_velocity,
        transform_from_base_to_odom=transform_from_base_to_odom,
        log=log)
    sm.add_state(state_1)

    state_1.set_bumper_state(BUMPER_STATE_PRESSED, BUMPER_LOCATION_MIDDLE)

    state_2 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    state_3 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_3)

    start_x = 1.0
    vel = max_velocity
    loc = start_x
    odom = odometry_with_linear_velocity_and_position(vel, start_x, 0.0, 0.0)
    state_1.set_odometry(odom)

    BASE_FRAME_TO_ODOM = odom.pose.pose

    assert not state_1.has_entered
    assert not state_1.has_exited
    sm.go_to_state(state_1.name)

    for t in range(50):
        odom = odometry_with_linear_velocity_and_position(vel, loc, 0.0, 0.0)
        state_1.set_odometry(odom)
        sm.update()

        vel = PUBLISHED_VELOCITY.linear.x
        loc = loc + vel * 1 * 1.0/rate

    # A velocity command should have been sent with velocity X
    assert isclose(PUBLISHED_VELOCITY.linear.x, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.linear.y, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.linear.z, 0.0, rel_tol=1e-6, abs_tol=0)

    assert isclose(PUBLISHED_VELOCITY.angular.x, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.angular.y, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.angular.z, 0.0, rel_tol=1e-6, abs_tol=0)

    assert isclose(loc, 0.70, rel_tol=1e-2, abs_tol=distance_tolerance)


# On update move backwards (with zero starting velocity)
def test_should_move_backwards_until_clear_when_obstacle_avoiding_state_updates_with_zero_starting_velocity():
    global BASE_FRAME_TO_ODOM

    sm = StateMachine(log=log)

    max_velocity = 0.5
    min_velocity = 0.1
    acceleration = 1.0
    rate = 5
    distance_tolerance = 0.01
    state_1 = ScuttleBumperObstacleAvoidingState(
        BUMPER_STATE_RELEASED,
        BUMPER_LOCATION_NONE,
        max_linear_velocity=max_velocity,
        min_linear_velocity=min_velocity,
        max_linear_acceleration=acceleration,
        rate_in_hz=rate,
        distance_tolerance=distance_tolerance,
        publish_velocity=publish_velocity,
        transform_from_base_to_odom=transform_from_base_to_odom,
        log=log)
    sm.add_state(state_1)

    state_1.set_bumper_state(BUMPER_STATE_PRESSED, BUMPER_LOCATION_MIDDLE)

    state_2 = ScuttleMovingState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_2)

    state_3 = ScuttleStoppedState(BUMPER_STATE_RELEASED, BUMPER_LOCATION_NONE, does_state_indicate_bumper_contact)
    sm.add_state(state_3)

    start_x = 1.0
    vel = 0.0
    loc = start_x
    odom = odometry_with_linear_velocity_and_position(vel, start_x, 0.0, 0.0)
    state_1.set_odometry(odom)

    BASE_FRAME_TO_ODOM = odom.pose.pose

    assert not state_1.has_entered
    assert not state_1.has_exited
    sm.go_to_state(state_1.name)

    for t in range(50):
        odom = odometry_with_linear_velocity_and_position(vel, loc, 0.0, 0.0)
        state_1.set_odometry(odom)
        sm.update()

        vel = PUBLISHED_VELOCITY.linear.x
        loc = loc + vel * 1 * 1.0/rate

    # A velocity command should have been sent with velocity X
    assert isclose(PUBLISHED_VELOCITY.linear.x, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.linear.y, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.linear.z, 0.0, rel_tol=1e-6, abs_tol=0)

    assert isclose(PUBLISHED_VELOCITY.angular.x, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.angular.y, 0.0, rel_tol=1e-6, abs_tol=0)
    assert isclose(PUBLISHED_VELOCITY.angular.z, 0.0, rel_tol=1e-6, abs_tol=0)

    assert isclose(loc, 0.70, rel_tol=1e-2, abs_tol=distance_tolerance)
