import numpy as np
from matplotlib import pyplot as plt
import time

start_time = 0
last_state_change = 0
experiment_state = 0

end_experiment = False
get_start_time = True
start_test = True

tilt_angles = [-20., -10., 0., 10., 20., 30., 40., 50., 60., 70., 80., 90., 100., 110., 120.]
num_tilt_cases = len(tilt_angles)
fraction_max_motor_speed = 0.2
max_motor_speed = 1000
period = 2*np.pi / 10

motor_speed = max_motor_speed * fraction_max_motor_speed

indi_u = np.zeros(8)


def noah_windtunnel_controller(indi_u_local, experiment_state_local, last_state_change_local, start_time_local,
                               end_experiment_local, get_start_time_local, start_test_local):

    if not start_test_local:
        return

    if experiment_state > num_tilt_cases**2:
        end_experiment_local = True
        start_test_local = False

    if get_start_time_local:
        start_time_local = time.time()
        get_start_time_local = False

    current_time = time.time()

    if (current_time - start_time_local) < 15:
        indi_u_local = noah_windtunnel_startup(indi_u_local, start_time_local, current_time)
    else:
        indi_u_local, experiment_state_local, last_state_change_local = noah_windtunnel_matrix(indi_u_local, experiment_state_local,
                                                                                         current_time,
                                                                                         last_state_change_local)

    return indi_u_local, experiment_state_local, last_state_change_local, start_time_local, end_experiment_local, \
        get_start_time_local, start_test_local


def noah_windtunnel_startup(indi_u_local, start_time_local, current_time_local):

    rad_per_second_motors = np.abs(1000 * np.sin(period * (current_time_local - start_time_local)))

    indi_u_local[0] = rad_per_second_motors
    indi_u_local[1] = rad_per_second_motors
    indi_u_local[2] = rad_per_second_motors
    indi_u_local[3] = rad_per_second_motors

    indi_u_local[4] = 0
    indi_u_local[5] = 0
    indi_u_local[6] = 0
    indi_u_local[7] = 0

    return indi_u_local


def noah_windtunnel_matrix(indi_u_local, experiment_state_local, current_time_local, last_state_change_local):

    number_configurations = len(tilt_angles)
    i = int(experiment_state_local / number_configurations)
    j = experiment_state_local % number_configurations

    indi_u_local[0] = motor_speed
    indi_u_local[1] = motor_speed
    indi_u_local[2] = motor_speed
    indi_u_local[3] = motor_speed

    indi_u_local[4] = tilt_angles[i]
    indi_u_local[5] = tilt_angles[i]
    indi_u_local[6] = tilt_angles[j]
    indi_u_local[7] = tilt_angles[j]

    if current_time_local - last_state_change_local >= 5:
        last_state_change_local = current_time_local
        experiment_state_local += 1
        print(tilt_angles[i], tilt_angles[j], current_time_local - start_time)

    return indi_u_local, experiment_state_local, last_state_change_local


if __name__ == "__main__":

    start = time.time()

    while not end_experiment:
        indi_u_local, experiment_state, last_state_change, start_time, end_experiment, get_start_time, start_test = \
            noah_windtunnel_controller(indi_u, experiment_state, last_state_change, start_time, end_experiment,
                                       get_start_time, start_test)

