#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import yaml
import rospy
import copy
import os
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

class HandEye(object):
    def tf_cb(self,data):
        self.tf = data
        # print(self.tf)

 
if __name__ == '__main__':
    ap = HandEye()
    ap.tf = 0
    count = 0
    while (ap.tf == 0 or count < 100):
        rospy.init_node('listener', anonymous=True)
        tf_result = rospy.Subscriber("/tf", TFMessage, ap.tf_cb)
        if ap.tf != 0:
            count += 1

    

    with open(os.path.expanduser("~/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/cfg/handeye_params.yaml"),'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        para = yaml.load(file, Loader=yaml.FullLoader)

    M_1_position = np.array([ap.tf.transforms[0].transform.translation.x,ap.tf.transforms[0].transform.translation.y,ap.tf.transforms[0].transform.translation.z,1])
    M_1_rotation = np.array([ap.tf.transforms[0].transform.rotation.x,ap.tf.transforms[0].transform.rotation.y,ap.tf.transforms[0].transform.rotation.z,ap.tf.transforms[0].transform.rotation.w])

    R_M1 = R.from_quat(M_1_rotation)
    M_1 = np.zeros([4,4])
    M_1[0:3,0:3] = R_M1.as_matrix()
    M_1[:,3] = M_1_position.T
    M_1_inv = np.linalg.inv(M_1)

    M_2_position = np.array([para['M2_x'],para['M2_y'],para['M2_z'],1])
    M_2_rotation = np.array([para['M2_R_x'],para['M2_R_y'],para['M2_R_z'],para['M2_R_w']])

    R_M2 = R.from_quat(M_2_rotation)
    M_2 = np.zeros([4,4])
    M_2[0:3,0:3] = R_M2.as_matrix()
    M_2[:,3] = M_2_position.T
    M_2_inv = np.linalg.inv(M_2)

    Result = M_2 @ M_1_inv
    Result_position = np.array([Result[0,3],Result[1,3],Result[2,3]])
    Result_rotation = R.from_matrix(Result [0:3,0:3])
    Result_rotation = Result_rotation.as_quat()

    print("\n input Matrix 1(From camera to Tag or Arm_second(right)_base to set position) :\n" )
    print(M_1)
    print("\n input Matrix 2(From EE to Tag or Arm_refer_base to set position) :\n" )
    print(M_2)
    print("\n Output Matrix(From EE to Camera or Arm_refer_base to Arm_second(right)_base) :\n" )
    print(Result)
    print("\n Position Matrix(X,Y,Z) :\n" )
    print(Result_position[0])
    print("\n Rotation Matrix(X,Y,Z,W) :\n" )
    print(-Result_rotation)

    result_data = {
        "Handeye calibration Result(position matrix and rotation matrix)":
            [float(Result_position[0]), float(Result_position[1]), float(Result_position[2]),
             float(Result_rotation[0]), float(Result_rotation[1]), float(Result_rotation[2]), float(Result_rotation[3])]
    }

    with open(os.path.expanduser("~/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/cfg/handeye_result.yaml"), mode='w', encoding='utf8') as file:
        yaml.safe_dump(result_data, file)


    fig = plt.figure(1)
    ax = fig.add_subplot(projection='3d')
    rot_show = R.from_quat(Result_rotation).as_matrix()
    ref_rot_show = R.from_quat(para['Reference_rotation']).as_matrix()
    EE_position = np.array([0,0,0])
    xv = np.array(rot_show @ [1,0,0])/100
    yv = np.array(rot_show @ [0,1,0])/100
    zv = np.array(rot_show @ [0,0,1])/100
    ax.plot3D([0,Result_position[0]],[0,Result_position[1]],[0,Result_position[2]],'-o',color = 'orange')
    ax.quiver(*Result_position.T,*xv.T, color='red')
    ax.quiver(*Result_position.T,*yv.T, color='g')
    ax.quiver(*Result_position.T,*zv.T, color='b')
    ax.quiver(*EE_position.T,*np.array([1,0,0]).T/100, color='red')
    ax.quiver(*EE_position.T,*np.array([0,1,0]).T/100, color='g')
    ax.quiver(*EE_position.T,*np.array([0,0,1]).T/100, color='b')
    set_axes_equal(ax)
    plt.show()

