
import numpy as np
import transformations
import random as rnd
from scipy.spatial.transform import Rotation
import math
from RRT import *
from matplotlib import pyplot as plt

#
#
#
#
# def valid_p(pp, O, B):
#     min_x = B[0]
#     max_x = B[1]
#     min_y = B[0]
#     max_y = B[1]
#     if (min_x < pp[0] <max_x) and (min_y < pp[1] <max_y):
#         for i in range(len(O)):
#             dx = abs(pp[0]-O[0])
#             dy = abs(pp[1]-O[1])
#             rd = O[2]
#             if dx**2 + dy**2 <= rd:
#                 return False
#
#
#     return True
#
# def generate_step(CP, Pg, O, B, delta):
#     N_path = 50
#     P_best= np.copy(CP)
#     dx0 = abs(CP[0] - Pg[0])
#     dy0 = abs(CP[1] - Pg[1])
#     r0 = np.sqrt(dx0**2 + dy0**2)
#     for i in range(N_path):
#         theta =rnd.random() * 2 * np.pi
#         pp = [CP[0]+delta*np.cos(theta),CP[0]+delta*np.sin(theta)]
#         if valid_p(pp, O, B):
#             dx1 = abs(pp[0] - Pg[0])
#             dy1 = abs(pp[1] - Pg[1])
#             r1 = np.sqrt(dx1**2 + dy1**2)
#             if CP == P_best:
#                 P_best = pp
#             elif (r1 < r0) and (pp != CP):
#                 P_best = pp
#     return P_best
#
# def planner(Pc, Pg, O, B=[-0.05, 0.65], delta=0.02, **args):
#     """
#
#     Args:
#         Pc: start point (x_s, y_s) --> list: len=2 OR np.array(): shape=(2,)
#         Pg: end point (x_g, y_g) --> list: len=2 OR np.array(): shape=(2,)
#         O: [(x_obs_1, y_obs_2, radius_obs_1), ..., (x_obs_N, y_obs_N, radius_obs_N)
#         B: this is the area where you plan the path in. both x and y should be in between these boundaries.
#         delta: Path resolution.
#         **args: add additional arguments as you please such as whether to plot a path or not.
#
#     Returns:
#         path: [[x_1, y_1], [x_2, y_2], ..., [x_M, y_M]] --> List of lists of size 2 (x, y).
#                 Important: Make sure that your output 'path' is in the right order (from start to goal)
#     """
#     list_path = list
#     CP = np.copy(Pc)
#     while CP != Pg:
#         CP = generate_step(CP, Pg, O, B, delta)
#         list_path.append(CP)
#     return list_path
#
#     pass
#
def steering_angle(A_robot_cam, A_base_cam, p_i_base):
    """

    Args:
        A_robot_cam: Homogeneous matrix from car to camera frame
        A_base_cam: Homogeneous matrix from origin to camera frame
        p_i_base: 2d vector of next point in the path with respect to base (origin) frame: (x, y)

    Returns:
        p_i_car: 2d vector of next point in path with respect to car frame: (x, y)
        alpha: Steering angle to next point [degrees].
    """
    A0 = np.linalg.inv(A_robot_cam)
    A1 = A0 @ A_base_cam
    Pb1 = np.hstack((np.array(p_i_base), np.array([0, 1]))).T
    A2 = A1 @ Pb1

    theta = math.decimal(np.rad2deg(np.arctan2(A2[0], A2[1])))
    return A2, theta
    pass

def planner(CP, Pg, o, B=[-0.5,2],delta=0.08, **args):
    rrt = RRT(
        start=CP,
        goal=Pg,
        rand_area=B,
        obstacle_list=o,
        expand_dis = delta)

    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
        return None
    else:
        print("found path!!")
        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-or', ms=4., alpha=0.5)
            plt.grid(True)
            plt.pause(0.01)
            print(path[::-1])
        return path[::-1]

planner([0,0], [1.5,1.5],[[0.3,0.3,0.05]])