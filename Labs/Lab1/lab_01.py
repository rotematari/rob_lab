#! /usr/bin/env python3

import numpy as np
import sys, select, os, time
import threading

if os.name == 'nt':
    import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient


e = """
Communications Failed
"""


def getKey():
    if os.name == 'nt':
        return input().encode('utf-8')
        # return msvcrt.getche()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

stop_rec =False
def listen_for_key():
    global stop_rec
    while not stop_rec:
        key =getKey()
        if str(key) =="b'q'":
            print("Stopping rec...")
            stop_rec =True

def record(base_cyclic):
    global stop_rec
    stop_rec = False

    listener_thread = threading.Thread(target=listen_for_key, daemon=True)
    listener_thread.start()

    cur_joint = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    joint_list = None
    xyz_list = None

    print("Recording started. Press 'q' to stop recording.")
    while not stop_rec:
        try:
            for i in range(len(base_cyclic.RefreshFeedback().actuators)):
                cur_joint[i] = base_cyclic.RefreshFeedback().actuators[i].position
            cur_end_xyz = np.array([
                base_cyclic.RefreshFeedback().base.tool_pose_x,
                base_cyclic.RefreshFeedback().base.tool_pose_y,
                base_cyclic.RefreshFeedback().base.tool_pose_z
            ])

            if joint_list is None:
                joint_list = cur_joint
                xyz_list = cur_end_xyz
            else:
                joint_list = np.vstack((joint_list, cur_joint))
                xyz_list = np.vstack((xyz_list, cur_end_xyz))

            print("TCP position X {}, Y {}, Z {} \n To stop recording press 'q'".format(*cur_end_xyz))
            print("Robot joints q1 {}, q2 {}, q3 {}, q4 {}, q5 {}, q6 {} \n To stop recording press 'q'".format(*cur_joint))

            time.sleep(1)

        except Exception as e:
            print(f"Error during recording: {e}")
            break

    return joint_list, xyz_list

def save():
    global joint_trajectory
    logdir_prefix = 'lab-01'

    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../Lab1/data')

    if not (os.path.exists(data_path)):
        os.makedirs(data_path)

    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)

    print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")

    import pickle
    with open(logdir + '/data' + '.pkl', 'wb') as h:
        pickle.dump(joint_trajectory, h)

if __name__ == "__main__":
    exit_program =False
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        import utilities

        # Parse arguments
        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            input("Connect joystick and press Enter to continue")

            try:
                print("Press s to start recording")
                while not exit_program:
                    key = getKey()
                    if str(key) == "b's'":
                        joint_trajectory = record(base_cyclic)
                        print('Press d to save the data or q to stop and quit')
                    elif str(key) == "b'd'":
                        save()
                        print('Press q to stop and quit')
                    elif str(key) == "b'q'":
                        exit_program =True
            except:
                print(e)
            finally:
                print('\nDone')

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        print("Program interrupted by user.")