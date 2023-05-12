#!/usr/bin/env python
import rospy
from aruco_opencv_msgs.msg import ArucoDetection
import actionlib
from actionlib_msgs.msg import GoalStatus,GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs
import numpy as np
from geometry_msgs.msg import Quaternion,PoseStamped
from geometry_msgs.msg import Twist
import cv2 as cv
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler
import math 

class FollowerRobot:
    def __init__(self):
        self.leader_pos = None
        self.follower_pos = None
        self.sub_aruco= rospy.Subscriber("/aruco_detections",ArucoDetection, self.aruco_detections_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def aruco_detections_callback(self, data):
        #print(data)
        if(len(data.markers) != 0 ):
            #print(data.markers)
            self.leader_pos = data.markers[0]
  
            

    def get_follower_pose(self):
        robot_pose = rospy.wait_for_message('/odom', Odometry)
        self.follower_pos = PoseStamped()
        self.follower_pos.header.frame_id = '/odom'
        self.follower_pos.header.stamp = rospy.Time.now()
        self.follower_pos.pose.position = robot_pose.pose.pose.position
        self.follower_pos.pose.orientation =robot_pose.pose.pose.orientation
   
    def calculatePositionWithOffset(self, position, robot_pose):
        # Calculate the vector from the robot position to the object position
        vector_to_object = Point(x=position.x - robot_pose.x,
                                y=position.y - robot_pose.y,
                                z=position.z - robot_pose.z)
        # Normalize the vector and multiply by the desired offset (30cm)
        norm = (vector_to_object.x**2 + vector_to_object.y**2 + vector_to_object.z**2)**0.5
        vector_to_stop = Point(x=vector_to_object.x/norm * 0.35,
                                y=vector_to_object.y/norm * 0.35,
                                z=vector_to_object.z/norm * 0.35)
        # Create a new position 30cm in front of the object
        stop_pos = Point(x=position.x - vector_to_stop.x,
                        y=position.y - vector_to_stop.y,
                        z=position.z - vector_to_stop.z)
        return stop_pos
    
    def calculateOrientationFaceToObject(self,position, robot_pose):
        # Calculate the vector from the robot position to the object position
        vector_to_object = Point(x=position.x - robot_pose.x,
                                y=position.y - robot_pose.y,
                                z=position.z - robot_pose.z)
        # Calculate the orientation to face the object
        yaw = math.atan2(vector_to_object.y, vector_to_object.x)
        quaternion = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*quaternion)

    def follow_leader(self):
        if self.leader_pos is not None and self.follower_pos is not None:
            distance = math.sqrt((self.follower_pos.pose.position.x-self.leader_pos.pose.position.x)**2 + (self.follower_pos.pose.position.y -self.leader_pos.pose.position.y)**2)
            print(distance)
            if distance < 0.35:
                return 
                
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = self.calculatePositionWithOffset(self.leader_pos.pose.position,self.follower_pos.pose.position)
            goal.target_pose.pose.orientation = self.calculateOrientationFaceToObject(self.leader_pos.pose.position,self.follower_pos.pose.position)
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result()

            
    def run(self):
        rospy.init_node('follower_robot')
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_follower_pose()
            self.follow_leader()
            rate.sleep()

if __name__ == '__main__':
    follower = FollowerRobot()
    follower.run()