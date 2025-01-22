import numpy as np
import transformations
from scipy.spatial.transform import Rotation as R
import modern_robotics


def calculate_error(t_curr, R_curr, target_feature_t, target_feature_R, translation_only=False):
    '''
    Calculate error based on the input pose and the target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Error, [t_err, R_err], 6x1
    '''
    t_del=t_curr-target_feature_t

    err_mat=np.dot(target_feature_R,R_curr.T)

    vec = R.from_matrix(err_mat).as_rotvec()

    theta = np.linalg.norm(vec) + 1e-5
    u = vec / theta

    # theta=np.arccos((np.trace(err_mat)-1)/2)
    # u_x = (err_mat[2][1] - err_mat[1][2]) / np.sqrt(
    #     (err_mat[2][1] - err_mat[1][2]) ** 2 + (err_mat[0][2] - err_mat[2][0]) ** 2 + (
    #                 err_mat[1][0] - err_mat[0][1]) ** 2)
    # u_y = (err_mat[0][2] - err_mat[2][0]) / np.sqrt(
    #     (err_mat[2][1] - err_mat[1][2]) ** 2 + (err_mat[0][2] - err_mat[2][0]) ** 2 + (
    #                 err_mat[1][0] - err_mat[0][1]) ** 2)
    # u_z = (err_mat[1][0] - err_mat[0][1]) / np.sqrt(
    #     (err_mat[2][1] - err_mat[1][2]) ** 2 + (err_mat[0][2] - err_mat[2][0]) ** 2 + (
    #                 err_mat[1][0] - err_mat[0][1]) ** 2)
    # u=np.array([u_x,u_y,u_z])
    # see paragraph above Eq.13
    # of Chaumette, Francois, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
    # https://hal.inria.fr/inria-00350283/document


    if translation_only:
        error = np.hstack((t_del, np.zeros(3)))
    else:
        error = np.hstack((t_del, theta * u))

    return error

def feature_jacobian(t_curr, R_curr, target_feature_R):
    '''
    form interaction matrix / feature jacobian base on current camera pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Interation Matrix (feature Jacobian), 6x6
    '''
    err_mat = np.dot(target_feature_R, R_curr.T)
    vec = R.from_matrix(err_mat).as_rotvec()

    theta = np.linalg.norm(vec) + 1e-5
    u = vec / theta

    #theta = np.arccos((np.trace(err_mat) - 1) / 2)

    # u_x = (err_mat[2][1] - err_mat[1][2]) / np.sqrt(
    #     (err_mat[2][1] - err_mat[1][2]) ** 2 + (err_mat[0][2] - err_mat[2][0]) ** 2 + (
    #             err_mat[1][0] - err_mat[0][1]) ** 2)
    # u_y = (err_mat[0][2] - err_mat[2][0]) / np.sqrt(
    #     (err_mat[2][1] - err_mat[1][2]) ** 2 + (err_mat[0][2] - err_mat[2][0]) ** 2 + (
    #             err_mat[1][0] - err_mat[0][1]) ** 2)
    # u_z = (err_mat[1][0] - err_mat[0][1]) / np.sqrt(
    #     (err_mat[2][1] - err_mat[1][2]) ** 2 + (err_mat[0][2] - err_mat[2][0]) ** 2 + (
    #             err_mat[1][0] - err_mat[0][1]) ** 2)
    #
    # u = np.array([u_x, u_y, u_z])

    S_p_curr=np.array([[0,-t_curr[2],t_curr[1]],[t_curr[2],0,-t_curr[0]],[-t_curr[1],t_curr[0],0]])

    S_u = [[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]]

    #J_thetau=np.identity(3)-(theta/2) @ S_u + (1-(np.sinc(theta)/(np.sinc(theta/2)**2))) * S_u**2
    J_thetau = np.identity(3) - (theta / 2) * np.array(S_u) + (
                     1 - (np.sinc(theta) / ((np.sinc(theta / 2)) ** 2))) * np.dot(np.array(S_u),
                                                                                  np.array(S_u))
    J_top=np.hstack((-np.identity(3),S_p_curr))
    J_bottom=np.hstack((np.zeros(3),J_thetau))

    L_out=np.vstack((J_top,J_bottom))

    return L_out

def control(L, error, _lambda):
    '''
    calculate the twist of camera frame required to reach target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Twist in camera frame
            [nu_c, omg_c], 1x6
    '''

    vel = -_lambda * np.dot(inv(L) , error)

    return vel