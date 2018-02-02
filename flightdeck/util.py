from time import sleep

import cflib


def find_available_drones():
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()

    if available:
        print('Drones found:')
        for i in available:
            print(i[0])

    return available


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    sleep(2)
