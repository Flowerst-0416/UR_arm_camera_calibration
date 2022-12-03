#! /usr/bin/env python3

import rospy
import cv2
import copy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys
import numpy as np
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

class ArmPlanner(object):
    def image_cb(self,data):     
        self.img = CvBridge().imgmsg_to_cv2(data, 'bgr8')

    def if_broad_find(self):
        if self.img is None:
            print("No Checkerboard Found")
            return False
        ret, self.corners = cv2.findChessboardCorners(self.img, (7, 10),
                                flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                                    cv2.CALIB_CB_FAST_CHECK +
                                    cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret:
            return True
        else:
            print("No Checkerboard Found")
            return False
            
    def find_distance(self):

        camera_center_x= int(self.img.shape[0]/2)
        camera_center_y= int(self.img.shape[1]/2)
        center_x = int(((self.corners[0][0])[0] +(self.corners[-1][0])[0]) /2)
        center_y = int(((self.corners[0][0])[1] +(self.corners[-1][0])[1]) /2)
        self.distance = np.sqrt((camera_center_y-center_x)**2+(camera_center_x-center_y)**2)
        self.x_distance = camera_center_x-center_x
        self.y_distance = camera_center_y-center_y
        print(self.distance)
        # fnl = cv2.drawChessboardCorners(self.img, (7, 10), corners, ret)
        # fnl = cv2.circle(fnl, (center_x,center_y), radius=5, color=(0, 0, 255), thickness=-1)
        # fnl = cv2.circle(fnl, (camera_center_y,camera_center_x), radius=5, color=(255, 0, 0), thickness=-1)
        # imS = cv2.resize(fnl, (int(self.img.shape[1]/2), int(self.img.shape[0]/2))) 
        # cv2.imshow("fnl", imS)
        # cmd = cv2.waitKey(0)
        # save image if pressed 's' or space key
    
    def __init__(self):
        super(ArmPlanner, self).__init__()

        # initialize node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_planner', anonymous=True)


        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

    
        rospy.Subscriber("/ximea_cam/image_raw", Image, self.image_cb)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.group_names = group_names
        self.quat = None
        self.distance = 0
        self.img = None

    def go_to_joint_new(self):
        mg = self.move_group
        joint_goal = [-pi/2, -pi/2, -pi/2, 0, pi/2, 0]
        # joint_goal= [0,0,0,0,0,0]
        mg.go(joint_goal, wait=True)
        
        mg.stop()

    def move(self,axis): 
        mg = self.move_group
        wpose = mg.get_current_pose().pose
        
        waypoints = []

        wpose = mg.get_current_pose().pose
        if axis == 'y':
            wpose.position.z -= 0.005
        elif axis == 'x':
            wpose.position.x -= 0.005
        elif axis == '-y':
            wpose.position.z += 0.005
        elif axis == '-x':
            wpose.position.x += 0.005    
        elif axis == '-z':
            wpose.position.y += 0.005
        elif axis == 'z':
            wpose.position.y -= 0.005  

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = mg.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold



        mg.execute(plan, wait=True)
        return 0

if __name__ == '__main__':

    ap = ArmPlanner()
    
    while(1):
        if ap.if_broad_find():
                ap.find_distance()
                break
    cal_1 = ap.distance # initial value
    ap.move('y')
    if ap.if_broad_find():
            ap.find_distance()
    cal_2 = ap.distance
    ap.move('x')
    if ap.if_broad_find():
            ap.find_distance()
    cal_3 = ap.distance

    while(ap.distance>100 or ap.distance==0):
        if cal_1-cal_2 <0:
            ap.move('-y')
        else:
            ap.move('y')
        if cal_2-cal_3<0:
            ap.move('-x')
        else:
            ap.move('x')
        ap.if_broad_find()
        ap.find_distance()
    ap.move('z')
    ap.move('z')
    print("done")



    


    

