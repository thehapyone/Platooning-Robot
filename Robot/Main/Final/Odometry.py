
import time

import math
import numpy as np

def motor_encoder(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        theta = old_pose[2]
        x = old_pose[0] + motor_ticks[0] * ticks_to_mm * np.sin(theta)
        y = old_pose[1] + motor_ticks[0] * ticks_to_mm * np.cos(theta)
        return (x, y, theta)
    else:   
        theta = old_pose[2]
        x = old_pose[0] - scanner_displacement * np.sin(theta)
        y = old_pose[1] - scanner_displacement * np.cos(theta)
        alpha = ticks_to_mm * (motor_ticks[1] - motor_ticks[0]) / robot_width
        R = motor_ticks[0] * ticks_to_mm / alpha

        C = [x - (R + robot_width / 2) * np.cos(theta), y + (R + robot_width / 2) * np.sin(theta)]
        theta = (theta + alpha) % (2 * np.pi)
        x = C[0] + (R + robot_width / 2) * np.cos(theta) + scanner_displacement * np.sin(theta)
        y = C[1] - (R + robot_width / 2) * np.sin(theta) + scanner_displacement * np.cos(theta)

    return (x, y, theta)

# the get motor_state returns the position and the covariance matrix
def get_motor_state(previous_state, motor_ticks, initial_covariance):
    global robot_width, scanner_displacement, ticks_to_mm
    control = np.array([motor_ticks[0], motor_ticks[1]]) * ticks_to_mm

    x, y, theta = previous_state
    l, r = control
    w = robot_width
    if r != l:
        alpha = (r - l) / w
        rad = l / alpha
        g1 = x + (rad + w / 2.) * (np.sin(theta + alpha) - np.sin(theta))
        g2 = y + (rad + w / 2.) * (-np.cos(theta + alpha) + np.cos(theta))
        g3 = (theta + alpha + np.pi) % (2 * np.pi) - np.pi
    else:
        g1 = x + l * np.sin(theta)
        g2 = y + l * np.cos(theta)
        g3 = theta

    states = np.array([g1, g2, g3])
    final_state = states + [scanner_displacement * np.cos(states[2]),
                               scanner_displacement * np.sin(states[2]),
                               0.0]
    
    # here we attempt to calculate the covariance
    covariance = get_motor_covariance(initial_covariance, control, final_state)

    return (g1, g2, g3), covariance

# returns the jacobian matrix of both the state and control parameters
def get_jacobian(state, control, w):
    theta = state[2]
    l, r = control

    if r != l:
        alpha = (r - l) / w
        R = l / alpha
        g1 = (R + (w / 2)) * (np.cos(theta + alpha) - np.cos(theta))
        g2 = (R + (w / 2)) * (np.sin(theta + alpha) - np.sin(theta))
        gt = np.array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])

        wr = (w * r) / ((r - l) ** 2)
        wl = (w * l) / ((r - l) ** 2)
        r2l = (r + l) / (2 * (r - l))

        g1_l = wr * (np.sin(theta + alpha) - np.sin(theta)) - r2l * np.cos(theta + alpha)
        g2_l = wr * (-np.cos(theta + alpha) + np.cos(theta)) - r2l * np.sin(theta + alpha)
        g3_l = - (1 / w)

        g1_r = -wl * (np.sin(theta + alpha) - np.sin(theta)) + r2l * np.cos(theta + alpha)
        g2_r = -wl * (-np.cos(theta + alpha) + np.cos(theta)) + r2l * np.sin(theta + alpha)
        g3_r = 1 / w

        Vt = np.array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])
    else: # when r == l
        gt = np.array([[1.0, 0.0, -l * np.sin(theta)], [0.0, 1.0, l * np.cos(theta)], [0.0, 0.0, 1.0]])

        g1_l = .5 * (np.cos(theta) + (l / w) * np.sin(theta))
        g2_l = .5 * (np.sin(theta) - (l / w) * np.cos(theta))
        g3_l = - 1 / w

        g1_r = .5 * ((-l / w) * np.sin(theta) + np.cos(theta))
        g2_r = .5 * ((l / w) * np.cos(theta) + np.sin(theta))
        g3_r = 1 / w

        Vt = np.array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

    # returns the jacobian matrix of the state and control parameters equation
    return gt, Vt

def get_motor_covariance(initial_cov, control, state):
    
    global control_motion_factor, control_turn_factor, robot_width
    # we need to calculate the covariance of the control - Which is diagional matrix of both sigamr and sigaml
    # we will use a control parameter for that.
    left, right = control
    # where E_t_1 = intial convariance
    alpha_1 = control_motion_factor
    alpha_2 = control_turn_factor
    sigma_l = (alpha_1 * left) ** 2 + (alpha_2 * (left - right)) ** 2
    sigma_r = (alpha_1 * right) ** 2 + (alpha_2 * (left - right)) ** 2

    # here we will create a diagional matrix of the control parameter i.e (sigmaL and sigamR)
    control_covariance = np.diag([sigma_l, sigma_r])
    # Convariance equaiton is :
    # E_t = G_t*E_t-1*G_t(tranpose) + V_t*E_control(sigma covariance)*V_t(transpose)
    # Where G_t is the Jacobian matrix of the states.
    G_t, V_t = get_jacobian(state, control, robot_width)

    # calculate the new convariance
    new_convariance = np.dot(G_t, np.dot(initial_cov, G_t.T)) + np.dot(V_t, np.dot(control_covariance, V_t.T))
    return new_convariance

ticks_to_mm = 1.078167
scanner_displacement = 0
robot_width = 160


# Motor constants.
control_motion_factor = 0.13  # Error in motor control.
control_turn_factor = 0.25  # Additional error due to slip when turning.

    
