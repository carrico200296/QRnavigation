#!/usr/bin/env python
 
import rospy
import actionlib
import math
import time
# import second_qr
import numpy as np
from transform_matrix import transform_matrix
# from next_qr_position import next_qr_position 
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String as My_string
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import *


# [w, x, y, z] = [cos(a/2), sin(a/2) * nx, sin(a/2)* ny, sin(a/2) * nz]
# Where a is the angle of rotation and {nx,ny,nz} is the axis of rotation. (From section 2e of Quaternion Spells )
tmp_radius = 2

# initial positions
pa0 = [(2.0, -0.5, 0.0), (0.0, 0.0, 0, 0)]
pa1 = [(2.0, -0.5, 0.0), (0.0, 0.0, 0, math.sqrt(0.5))]

random_waypoints = [  
    [(2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    pa0,
    pa1,
    [(3.0, 4.0, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]
]

#start time
start_time = time.time()

#List with qr codes
qr_list = []
qr_important_list = [0, 0, 0, 0, 0]
qr_stored_list = []

#Robot current pose and orientation 
cur_robot_pose_x = 0.0
cur_robot_pose_y = 0.0
cur_robot_pose_z = 0.0

robot_orientation = euler_from_quaternion((0,0,0,0))
robot_orientation_w = 0.0
robot_orientation_z = 0.0

#Last qr read position in camera frame
last_qr_camera_pose = [0.0, 0.0, 0.0]

# Transformation
tf_matrix

class QR:
  def __init__(self, data1):
    data = str(data1)

    split_data = data.split("\\r\\n")
    counter = 0
    for i in split_data:
        i = i.split("=")[1]
        split_data[counter] = i
        counter += 1
    self.split_data = split_data
    self.x=float(split_data[0])
    self.y=float(split_data[1])
    self.nx=float(split_data[2])
    self.ny=float(split_data[3])
    self.number=int(split_data[4])
    self.letter=split_data[5][0]
    self.read_time = time.time() 

    self.camera_frame_xyz = [0.0, 0.0, 0.0]
    self.robot_frame_x = 0.0
    self.robot_frame_y = 0.0
    self.robot_frame_z = 0.0

def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def second_qr(qr1):
    # PTO 1
    x1 = qr1.x # x_info_qr_code_1_world_frame
    y1 = qr1.y # y_info_qr_code_1_world_frame

    # PTO 2
    x2 = qr1.nx # x_info_qr_code_2_world_frame
    y2 = qr1.ny # y_info_qr_code_2_world_frame

    # Distance
    dist = math.sqrt(math.pow(x1-x2,2) + math.pow(y1-y2,2))

    x_robot_qr1 = qr1.robot_frame_x
    y_robot_qr1 = qr1.robot_frame_y

    x = x2
    y = math.sqrt(math.pow(dist,2) - math.pow(x-x_robot_qr1,2)) + y_robot_qr1
    # for i in range(1,4)
    #     x = x2 + i
    #     y = sqrt(math.pow(dist,2) - math.pow(x-x1,2)) + y1

    # for i in range(1,10)
    #     x = 1 + i
    #     y = sqrt(dist^2 - (x-x1)^2) + y1

    qr2_position = []
    qr2_position = [x,y]
    return qr2_position


def check_qr(t_interval):
    if len(qr_list) == 0:
        return False
    if (time.time() - qr_list[-1].read_time < t_interval):
        return True
    return False


def update_qr_important_list():
    if (last_qr_camera_pose[0] == 0.0):
        return False
    tmp = qr_list[-1]
    if (qr_important_list[tmp.number-1] != 0):
        return False

    # vec takes two arguments first as x and second as y
    vec = [last_qr_camera_pose[2], -last_qr_camera_pose[0], 0, 0]
    tmp.camera_frame_xyz = vec
    

    # vec_rotated = quaternion_multiply(quaternion_multiply(robot_orientation, vec),quaternion_conjugate(robot_orientation))
    global robot_orientation
    tmp.robot_frame_x = cur_robot_pose_x
    # rospy.loginfo("I heard %f, %f, %f \n", tmp.robot_frame_x, tmp.robot_frame_y, tmp.robot_frame_z)

    tmp.robot_frame_x += (abs(vec[0]))*math.cos(robot_orientation[2])
    tmp.robot_frame_y = cur_robot_pose_y + vec[0]*math.sin(robot_orientation[2])
    tmp.robot_frame_z = cur_robot_pose_z + 0

    #assign qr to a right position in the list
    qr_important_list[tmp.number-1] = tmp
    qr_stored_list.append(tmp)

    rospy.loginfo("##################  QR_FOUND %f   ################# \n", tmp.number)
    rospy.loginfo("qr_in_robot_frame %f, %f, %f \n", tmp.robot_frame_x, tmp.robot_frame_y, vec[0]*math.cos(robot_orientation[2]))
    # rospy.loginfo("curr_robot_pos %f, %f, %f \n", cur_robot_pose_x, cur_robot_pose_y, cur_robot_pose_z)

    # rospy.loginfo("I heard vec %f, %f, %f \n", vec[0], vec[1], robot_orientation[2])

    # rospy.loginfo("I heard vec_rotated %f, %f, %f \n", vec_rotated[0], vec_rotated[1], vec_rotated[2])
    # rospy.loginfo("I heard curr_robot_pos %f, %f, %f \n", cur_robot_pose_x, cur_robot_pose_y, cur_robot_pose_z)

    return True

def print_final_word():
    print('########### Printing final word: \n')
    for qr in qr_important_list:
        if qr != 0:
            print(qr.letter)
        continue

def qr_callback(data):
    qr_list.append(QR(data))

def qr_camera_pose_callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    z = abs(data.pose.position.z)
    if (z != 0.0):
        last_qr_camera_pose[0] = x
        last_qr_camera_pose[1] = y
        last_qr_camera_pose[2] = z

#Update position of last qr in camera frame
def update_robot_pose_callback(pose_with_cov):
    global cur_robot_pose_x
    cur_robot_pose_x = pose_with_cov.pose.pose.position.x
    global cur_robot_pose_y
    cur_robot_pose_y = pose_with_cov.pose.pose.position.y
    global cur_robot_pose_z
    cur_robot_pose_z = pose_with_cov.pose.pose.position.z
    # print(cur_robot_pose_x)
    global robot_orientation
    
    # getting robot orientation
    robot_orientation_w = pose_with_cov.pose.pose.orientation.w
    robot_orientation_z = pose_with_cov.pose.pose.orientation.z
    q = (0, 0, robot_orientation_z, robot_orientation_w)
    euler = euler_from_quaternion(q, 'sxyz')
    robot_orientation = euler

# ROS SUBSCRIBERS
rospy.Subscriber("qr_codes", My_string, qr_callback)
rospy.Subscriber("visp_auto_tracker/object_position", PoseStamped, qr_camera_pose_callback)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, update_robot_pose_callback)

if __name__ == '__main__':
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    flag = True
    tf_doesnt_exists = True
    global tf_matrix

    rospy.sleep(0.1)
    # tf_matrix = transform_matrix()
    # if check_qr(1):
    #     if update_qr_important_list():
    #         # print(tf_matrix)
    #         # qr_next = next_qr_position(tf_matrix,qr_stored_list[-1])
    #         rospy.loginfo("number: %f , x: %f, y: %f , next_x: %f , next_y: %f  \n", qr_list[-1].number, qr_list[-1].x, qr_list[-1].y, qr_list[-1].nx, qr_list[-1].ny)
    #         # print(last_qr_camera_pose)
    #     # break

    
    for pose in random_waypoints:
        # rospy.loginfo("msg %s \n", "new waypoint")
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()
        # rospy.loginfo("msg %s \n", "reached waypoint")
        rospy.sleep(0.5)
        if check_qr(1):
            if update_qr_important_list():
                break
                # rospy.loginfo("number: %f , x: %f, y: %f , next_x: %f , next_y: %f  \n", qr_list[-1].number, qr_list[-1].x, qr_list[-1].y, qr_list[-1].nx, qr_list[-1].ny)

    while tf_doesnt_exists:
        goal = goal_pose(get_pos_within_rad(qr_stored_list[-1]))
        goal = offset(pose)
        client.send_goal(goal)
        client.wait_for_result()
        rospy.sleep(0.5)

        if len(qr_stored_list) == 2 and tf_doesnt_exists:
            tf_matrix = transform_matrix()
            print('--------  Transformation matrix ----------- \n')
            print(tf_matrix)
            tf_doesnt_exists = False
                
    while(len(qr_stored_list)<5):
        last_qr = qr_stored_list[-1]
        pose = next_qr_position(tf_matrix, last_qr)
        pose = offset(pose)
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()
        rospy.sleep(0.5)
        if check_qr(1):
            update_qr_important_list()
        #givin 200s time for task execution
        if (time.time() - start_time > 200):
            break

    # print(qr_important_list[1])
    # qr2 = second_qr(qr_list[-1])
    # qr2_waypoints = [[(qr2[0], qr2[1], 0.0), (0.0, 0.0, 0, 1.0)],
    # [(qr2[0], qr2[1], 0.0), (0.0, 0.0, -0.1673, 0.9859)],
    # [(qr2[0], qr2[1], 0.0), (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5))]
    
    # for pose in qr2_waypoints:
    #     rospy.loginfo("msg %s \n", "new waypoint")
    #     goal = goal_pose(pose)
    #     client.send_goal(goal)
    #     client.wait_for_result()
    #     rospy.loginfo("msg %s \n", "reached waypoint")
    #     rospy.sleep(0.5)
    #     if check_qr(1):
    #         if update_qr_important_list():
    #             break
    
    # if (time.time() - start_time > 10):
    #     flag = False
    
    flag = False
    if len(qr_stored_list)<5:
        print("DIDNT FIND ALL THE QRS")
    print_final_word()


    