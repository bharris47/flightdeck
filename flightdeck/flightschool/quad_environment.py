import math
import threading
from time import sleep

import numpy as np
from gym.spaces import Box
from rl.core import Env

from cflib.crazyflie.log import LogConfig
from flightdeck.util import reset_estimator

ROLLOVER_THRESHOLD = 150

STATE_STABILIZER_YAW = 'stabilizer.yaw'
STATE_STABILIZER_PITCH = 'stabilizer.pitch'
STATE_STABILIZER_ROLL = 'stabilizer.roll'
STATE_STABILIZER_THRUST = 'stabilizer.thrust'
STATE_POSITION_X = 'stateEstimate.x'
STATE_POSITION_Y = 'stateEstimate.y'
STATE_POSITION_Z = 'stateEstimate.z'
STATE_FIELDS = [
    STATE_STABILIZER_YAW,
    STATE_STABILIZER_PITCH,
    STATE_STABILIZER_ROLL,
    STATE_STABILIZER_THRUST,
    STATE_POSITION_X,
    STATE_POSITION_Y,
    STATE_POSITION_Z
]
STATE_INDICES = {field: i for i, field in enumerate(STATE_FIELDS)}

ACTION_YAW = 'yaw'
ACTION_PITCH = 'pitch'
ACTION_ROLL = 'roll'
ACTION_THRUST = 'thrust'
ACTIONS = [
    ACTION_YAW,
    ACTION_PITCH,
    ACTION_ROLL,
    ACTION_THRUST
]
ACTION_INDICES = {field: i for i, field in enumerate(ACTIONS)}


class CrazyflieEnvironment(Env):
    def __init__(self, crazyflie, max_pitch=10., max_roll=10., max_thrust=40000, state_update_frequency_ms=10,
                 x_range=(-1.0, 1.0), y_range=(-1.0, 1.0), z_range=(0.5, 1.0)):
        self.action_space = Box(0, 1, shape=(len(ACTIONS),), dtype=np.float32)  # yaw, pitch, roll, thrust
        self.observation_space = Box(-np.inf, np.inf, shape=(10,),
                                     dtype=np.float32)  # yaw, pitch, roll, thrust, x, y, z, goalx, goaly, goalz

        self._cf = crazyflie

        self._max_pitch = max_pitch
        self._max_roll = max_roll
        self._max_thrust = max_thrust

        self._state_update_frequency = state_update_frequency_ms / 1000
        self._state_logger = None
        self._state = None

        self._x_range = x_range
        self._y_range = y_range
        self._z_range = z_range
        self._goal_position = (0, 0, 0)

        self._state_update_event = threading.Event()

    def reset(self):
        if self._state_logger is None:
            self._start_logger()

        # wait for state to not be None
        self._wait_for_state_change(None)

        # wait for human intervention
        while self._is_upside_down():
            sleep(3)

        reset_estimator(self._cf)

        self._goal_position = self._get_new_goal_position(self._get_current_position())
        self._wait_for_state_change()
        print('goal: %s current: %s' % (self._goal_position, self._get_current_position()))
        return self._state

    def _get_new_goal_position(self, current_position):
        x, y, _ = current_position
        # x += self._get_random(*self._x_range)
        # y += self._get_random(*self._y_range)
        z = self._get_random(*self._z_range)  # always assume starting from ground
        return x, y, z

    def _get_random(self, minimum, maximum):
        difference = maximum - minimum
        return np.random.random() * difference + minimum

    def _get_current_position(self):
        return self._get_position(self._state)

    def _get_position(self, state):
        return state[STATE_INDICES[STATE_POSITION_X]:STATE_INDICES[STATE_POSITION_Z] + 1]

    def _get_distance(self, point1, point2):
        x1, y1, z1 = point1
        x2, y2, z2 = point2
        return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2))

    def step(self, action):
        initial_state = self._state

        yaw = max(min(action[ACTION_INDICES[ACTION_YAW]] * 180., 180), -180)
        pitch = max(min(action[ACTION_INDICES[ACTION_PITCH]] * self._max_pitch, self._max_pitch), -self._max_pitch)
        roll = max(min(action[ACTION_INDICES[ACTION_ROLL]] * self._max_roll, self._max_roll), -self._max_roll)

        thrust_factor = (action[ACTION_INDICES[ACTION_THRUST]] + 1.) / 2.0
        thrust = max(min(int(thrust_factor * self._max_thrust), self._max_thrust), 0)

        print('yaw: %f pitch: %f roll: %f thrust: %f' % (yaw, pitch, roll, thrust))

        self._cf.commander.send_setpoint(
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            thrust=thrust
        )

        # only wait 1/10 of update frequency to prevent delay
        self._wait_for_state_change(initial_state)

        done = False
        distance = self._get_distance(self._get_current_position(), self._goal_position)

        if self._is_upside_down():
            reward = -distance * 10
            done = True
        else:
            reward = -distance

        return self._state, reward, done, {}

    def _wait_for_state_change(self, initial_state=None):
        initial_state = initial_state if initial_state is not None else self._state
        self._state_update_event.clear()
        while np.array_equal(self._state, initial_state):
            self._state_update_event.wait()
            self._state_update_event.clear()

    def _is_upside_down(self):
        roll_index = STATE_INDICES[STATE_STABILIZER_ROLL]
        crashed = abs(self._state[roll_index]) >= ROLLOVER_THRESHOLD
        return crashed

    def close(self):
        pass

    def _start_logger(self):
        print('starting logger...')
        self._state_logger = LogConfig(name='State', period_in_ms=self._state_update_frequency * 1000)
        self._state_logger.add_variable(STATE_STABILIZER_YAW, 'float')
        self._state_logger.add_variable(STATE_STABILIZER_PITCH, 'float')
        self._state_logger.add_variable(STATE_STABILIZER_ROLL, 'float')
        self._state_logger.add_variable(STATE_STABILIZER_THRUST, 'uint16_t')
        self._state_logger.add_variable(STATE_POSITION_X, 'float')
        self._state_logger.add_variable(STATE_POSITION_Y, 'float')
        self._state_logger.add_variable(STATE_POSITION_Z, 'float')

        self._cf.log.add_config(self._state_logger)
        self._state_logger.data_received_cb.add_callback(self._on_state_received)
        self._state_logger.error_cb.add_callback(self._on_error_received)

        self._state_logger.start()

    def _on_error_received(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _on_state_received(self, timestamp, data, logconf):
        self._state = np.array([data[field] for field in STATE_FIELDS] + list(self._goal_position), dtype=np.float32)
        self._state_update_event.set()
