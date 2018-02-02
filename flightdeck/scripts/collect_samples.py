from collections import defaultdict

from tqdm import tqdm

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from flightdeck.stop_fall import reset_estimator

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()

    if available:
        print('Drones found:')
        for i in available:
            print(i[0])

    with SyncCrazyflie(available[0][0], cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        reset_estimator(cf)

        tracker = tqdm()
        stats = {}

        state_logger = LogConfig(name='State', period_in_ms=10)
        state_logger.add_variable('stabilizer.yaw', 'float')
        state_logger.add_variable('stabilizer.pitch', 'float')
        state_logger.add_variable('stabilizer.roll', 'float')
        state_logger.add_variable('stabilizer.thrust', 'uint16_t')
        state_logger.add_variable('stateEstimate.x', 'float')
        state_logger.add_variable('stateEstimate.y', 'float')
        state_logger.add_variable('stateEstimate.z', 'float')

        stats = defaultdict(dict)
        with SyncLogger(scf, state_logger) as logger:
            try:
                for log_entry in logger:
                    data = log_entry[1]
                    tracker.set_postfix(data)
                    for key, value in data.items():
                        key_stats = stats[key]
                        key_stats['max'] = value if key_stats.get('max') is None else max(key_stats['max'], value)
                        key_stats['min'] = value if key_stats.get('min') is None else min(key_stats['min'], value)
            except KeyboardInterrupt:
                pass

        for key, key_stats in stats.items():
            print('%s:\t\tmin: %f\t\tmax: %f' % (key, key_stats['min'], key_stats['max']))
