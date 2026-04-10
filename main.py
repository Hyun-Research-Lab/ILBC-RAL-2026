# [1] H. Chang, J. Lane, N.P. Hyun, Learning One-Step Inverse for Performance
# Improvement of Nonlinear Control Systems: Application to Quadrotor Control,
# RA-L 2026

import time
import numpy as np

from hyunlabutils.mocap_thread import MocapThread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

mocap_system_type = 'vicon'

# Host name of the mocap computer
# Make sure you are connected to the same network
host_name = '192.168.1.115'

# URI of the Crazyflie
uri = 'radio://0/80/2M/E7E7E7E707'

# Change the rigid body name to match the mocap object name
rigid_body_name = 'cf07'

cflib.crtp.init_drivers()

mocap_thread = MocapThread(mocap_system_type, host_name, rigid_body_name)

NOMINAL_CONTROLLER_PID = '1' # Built-in PID controller
NOMINAL_CONTROLLER_LQR = '2' # Nominal LQR controller (31) in [1]

LEARNING_TYPE_DISABLED = '0' # Learning is disabled (used for training)
LEARNING_TYPE_LINEAR_MODEL = '1' # Learning based on the linearlized nominal model (30) in [1]
LEARNING_TYPE_NONLINEAR_MODEL = '2' # learning based on the full nonlinear dynamics model (27) in [1]
# LEARNING_TYPE_TRAINING = '3' # Add exploration noise (unused)

# Helper functions for moving the Crazyflie
def moveToPoseLQR(cf: Crazyflie, z, time_s):
    cf.param.set_value('ILBC.nominal_controller', NOMINAL_CONTROLLER_LQR)
    time.sleep(0.1)

    moveToPose(cf, (0.0, 0.0, z, 0.0, 0.0, 0.0), time_s)

def moveToPosePID(cf: Crazyflie, pose: tuple, time_s):
    cf.param.set_value('ILBC.nominal_controller', NOMINAL_CONTROLLER_PID)
    time.sleep(0.1)

    moveToPose(cf, pose, time_s)

def moveToPose(cf: Crazyflie, pose: tuple, time_s):
    x, y, z, roll, pitch, yaw = pose

    for _ in range(int(time_s * 10)):
        cf.commander.send_position_setpoint(x, y, z, np.rad2deg(yaw))
        time.sleep(0.1)
    
    if roll != 0.0 or pitch != 0.0:
        for _ in range(2):
            cf.commander.send_zdistance_setpoint(np.rad2deg(roll), np.rad2deg(pitch), 0.0, z)
            time.sleep(0.1)

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    # Start sending mocap data to the Crazyflie
    mocap_thread.cf = cf

    # Set initial parameters
    cf.param.set_value('stabilizer.estimator', '2') # 2: Kalman 3: UKF
    cf.param.set_value('ILBC.nominal_controller', NOMINAL_CONTROLLER_PID)
    cf.param.set_value('ILBC.learning_type', LEARNING_TYPE_DISABLED)

    reset_estimator(cf)

    # Log battery voltage
    battery_logconf = LogConfig('', 10)
    battery_logconf.add_variable('pm.vbat', 'float')

    with SyncLogger(scf, battery_logconf) as logger:
        log_entry = logger.next()
        vbat = log_entry[1]['pm.vbat']
        print(f'Battery: {vbat:<.2f} V')

    try:
        # Arm the Crazyflie
        cf.platform.send_arming_request(True)
        time.sleep(1)

        # Start logging
        cf.param.set_value('usd.logging', '1')

        # Print console data
        cf.console.receivedChar.add_callback(print)

        # Takeoff to 1.5m using PID
        moveToPosePID(cf, (0.0, 0.0, 1.5, 0.0, 0.0, 0.0), 5.0)

        # Move to an initial condition
        # moveToPosePID(cf, (-1.0, -1.0, 1.25, 0.0, 0.0, 3.0), 5.0) # testing
        moveToPosePID(cf, (-1.97925169, 0.00562378, 1.49577329, 0.0, 0.0, -2.30071692), 5.0) # training

        # Go to the equilibrium with LQR
        cf.param.set_value('ILBC.learning_type', LEARNING_TYPE_DISABLED)
        moveToPoseLQR(cf, 3.0, 8.0)

        # Land using PID
        cf.param.set_value('ILBC.learning_type', LEARNING_TYPE_DISABLED)
        moveToPosePID(cf, (0.0, 0.0, 0.3, 0.0, 0.0, 0.0), 4.0)

        # Soft stop
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)
        cf.commander.send_notify_setpoint_stop()
        time.sleep(0.1)

    except KeyboardInterrupt:
        # press Ctrl+C to immediately send an emergency stop signal
        print('Emergency stop!')
        cf.loc.send_emergency_stop()

    # Stop logging and close log file
    cf.param.set_value('usd.logging', '0')
    time.sleep(0.1)

mocap_thread.close()