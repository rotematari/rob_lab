import numpy as np


#######################
######## Lab 4 ########
#######################

def traj_gen_config(q1, q2, t, Tf):
    """
    path plan configuration space

    Args:
        q1: Start configuration (angles) [degrees] --> np.array((6,))
        q2: Goal configuration (angles) [degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: angles positions, angles velocities, angles accelerations
                --> q, dq, ddq --> 3 object of np.array((6,))

    """
    # ### Linear ###
    # q_l = q1 * [1 - t] + q2 * t
    # dq_l = -q1 + q2
    # ddq_l = 0
    #
    # return q_l, dq_l, ddq_l

    ### polynomial ###

    a_0 = q1  # q(0) = qs
    a_1 = np.zeros(6)  # dq(0) = 0 =a1
    a_2 = 3 * (q2 - q1) * Tf ** (-2)
    a_3 = -2 * (q2 - q1) * Tf ** (-3)

    q = a_0 + a_1 * t + a_2 * (t ** 2) + a_3 * (t ** 3)
    dq = a_1 + 2 * a_2 * t + 3 * a_3 * (t ** 2)
    ddq = 2 * a_2 + 6 * a_3 * t

    return q, dq, ddq


def traj_gen_task(x_s, x_g, t, Tf):
    """
    path plan in Task space

    Args:
        x_s: Start end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        x_g: Goal end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: End-effector position, velocity, acceleration
                --> x, dx, ddx --> 3 object of np.array((6,))

    """

    # ### Linear ###
    # x_l = x_s * [1 - t] + x_g * t
    # dx_l = -x_s + x_g
    # ddx_l = 0
    #
    # return x_l, dx_l, ddx_l

    ### polynomial ###

    a_0 = x_s  # x(0) = x_s
    a_1 = np.zeros(6)  # dx(0) = 0 =a1
    a_2 = -3 * (x_s - x_g) * Tf ** (-2)
    a_3 = 2 * (x_s - x_g) * Tf ** (-3)

    x = a_0 + a_1 * t + a_2 * (t ** 2) + a_3 * (t ** 3)
    dx = a_1 + 2 * a_2 * t + 3 * a_3 * (t ** 2)
    ddx = 2 * a_2 + 6 * a_3 * t

    return x, dx, ddx



def generate_x_goals_list():
    """

    Returns: Desired end-effector goals along the planned path --> np.array((4, 6))

    Notes:  1. Position units [m]
            2. Orientation units [degrees]

    """
    x_ = np.array([[0.44, 0.187, 0.419, 96, 1, 150],
                   [0.369, -0.015, 0.21, 178, 3, 177],
                   [0.372, 0.014, 0.01, 178, 4.5, 175],
                   [0.44, 0.187, 0.419, 96, 1, 150]])
    return x_


def generate_q_goals_list():
    """

    Returns: Desired configuration (angle) goals along the planned path --> --> np.array((4, 6))

    Notes: Orientation units [degrees]

    """

    jointPoses = np.array([[0.1, 343, 75, 354, 300, 0.1],
                           [7.5, 337, 80, 271, 287, 10],
                           [7.5, 313, 97, 272, 329, 10],
                           [0.1, 343, 75, 354, 300, 0.1]])

    return jointPoses