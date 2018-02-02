# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the crazyflie at `URI` and runs a figure 8
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
from time import sleep

import numpy as np
from tqdm import tqdm

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Only output errors from the logging framework
from cflib.crazyflie.syncLogger import SyncLogger
from flightdeck.util import reset_estimator

logging.basicConfig(level=logging.ERROR)

ACCEL_X = 'acc.x'
ACCEL_Y = 'acc.y'
ACCEL_Z = 'acc.z'
POSITION_Z = 'stateEstimate.z'


class FallDetector:
    def __init__(self, threshold=0.1):
        self._threshold = threshold

    def is_falling(self, dx, dy, dz):
        return self.get_s_factor(dx, dy, dz) <= self._threshold

    def get_s_factor(self, dx, dy, dz):
        sfactor = np.sqrt(sum(pow(acceleration, 2) for acceleration in [dx, dy, dz]))
        return sfactor


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()

    if available:
        print('Drones found:')
        for i in available:
            print(i[0])

    detector = FallDetector(threshold=0.1)

    with SyncCrazyflie(available[0][0], cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        reset_estimator(cf)

        tracker = tqdm()
        while True:
            state_logger = LogConfig(name='State', period_in_ms=10)
            state_logger.add_variable('acc.x', 'float')
            state_logger.add_variable('acc.y', 'float')
            state_logger.add_variable('acc.z', 'float')
            state_logger.add_variable('stateEstimate.z', 'float')

            with SyncLogger(scf, state_logger) as logger:
                for log_entry in logger:
                    data = log_entry[1]

                    altitude = data[POSITION_Z]
                    dx = data[ACCEL_X]
                    dy = data[ACCEL_Y]
                    dz = data[ACCEL_Z]
                    tracker.set_postfix(altitude=altitude, dx=dx, dy=dy, dz=dz)

                    if detector.is_falling(dx, dy, dz):
                        steps = int(15 * altitude)
                        step = altitude / steps
                        for i in range(steps):
                            altitude -= step
                            cf.commander.send_hover_setpoint(0, 0, 0, altitude)
                            time.sleep(0.1)

                        cf.commander.send_stop_setpoint()
                        sleep(1)
                        break
