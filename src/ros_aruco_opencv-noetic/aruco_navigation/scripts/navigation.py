#!/usr/bin/env python
import rospy
from aruco_opencv_msgs.msg import ArucoDetection
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import cv2 as cv
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import math 

class ArucoNavigator:
    def __init__(self):
        self.status = False
        self.goal_marker = 3
        self.isRobotMoving = False
        self.current_marker = 0
        self.already_detected = []
        
        self.detected_marker = None
        self.rate = rospy.Rate(10)
        
        self.image_sub = rospy.Subscriber("/aruco_detections",ArucoDetection, self.aruco_detections_callback)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def rotate_robot(self):
        # Create a Twist message to rotate the robot
        rotate_msg = Twist()
        rotate_msg.angular.z = 0.65
        # Publish Twist messages to the cmd_vel topic to rotate the robot
        self.twist_pub.publish(rotate_msg)

        
    def stop_robot(self):
        # Stop the robot by publishing a Twist message with zero velocity
        stop_msg = Twist()
        self.twist_pub.publish(stop_msg)
    

    def aruco_detections_callback(self, data):
        if self.goal_marker in self.already_detected: 
            rospy.loginfo('Finish goal reached')
            quit()
            
        if(len(data.markers) == 0 and self.status == False):
            rospy.loginfo('Rotating: finding new ArUco code')
            self.isRobotMoving = False
            self.rotate_robot()
            return
        
        elif(self.status == False and self.isRobotMoving == False):
            rospy.loginfo('Stop: preparing for navigate')
            self.isRobotMoving = True
            self.stop_robot()
            return
        
        else:  
            for marker in data.markers:
                self.detected_marker = marker.marker_id
                if (self.detected_marker in self.already_detected):
                    rospy.loginfo('Rotating: finding new ArUco code')
                    self.isRobotMoving = False
                    self.rotate_robot()
                    return
         
                elif (self.status == False and self.current_marker == self.detected_marker):
                    detected_position = marker.pose.position
                    detected_orientation = marker.pose.orientation
                    self.goToGoal(detected_position, detected_orientation, self.detected_marker)
                else :
                    rospy.loginfo('Rotating: finding new ArUco code')
                    self.rotate_robot()
                    return
         
    def calculateOrientationFaceToObject(self,position, robot_pose):
        # Calculate the vector from the robot position to the object position
        vector_to_object = Point(x=position.x - robot_pose.x,
                                y=position.y - robot_pose.y,
                                z=position.z - robot_pose.z)
        # Calculate the orientation to face the object
        yaw = math.atan2(vector_to_object.y, vector_to_object.x)
        quaternion = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*quaternion)
                
    def calculatePositionWithOffset(self, position, robot_pose):
        # Calculate the vector from the robot position to the object position
        vector_to_object = Point(x=position.x - robot_pose.x,
                                y=position.y - robot_pose.y,
                                z=position.z - robot_pose.z)
        # Normalize the vector and multiply by the desired offset (30cm)
        norm = (vector_to_object.x**2 + vector_to_object.y**2 + vector_to_object.z**2)**0.5
        vector_to_stop = Point(x=vector_to_object.x/norm * 0.30,
                                y=vector_to_object.y/norm * 0.30,
                                z=vector_to_object.z/norm * 0.30)
        # Create a new position 30cm in front of the object
        stop_pos = Point(x=position.x - vector_to_stop.x,
                        y=position.y - vector_to_stop.y,
                        z=position.z - vector_to_stop.z)
        return stop_pos
        
    
    def goToGoal(self, position, quaternion, marker_id):
        # Obtain the position of the object that the robot needs to stop in front of
        robot_pose = rospy.wait_for_message('/odom', Odometry)
        robot_pose = robot_pose.pose.pose.position

        rospy.loginfo('Waiting for the action server to start')
        
        self.client.wait_for_server()

        rospy.loginfo('Action server started, sending the goal')

        pose = geometry_msgs.msg.Pose()
        stop_pos = self.calculatePositionWithOffset(position, robot_pose)
        pose.position.x = stop_pos.x
        pose.position.y = stop_pos.y 
        pose.position.z = 0.0

        quat_position = self.calculateOrientationFaceToObject(position, robot_pose)
        pose.orientation = quat_position

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose = pose
        

        self.client.send_goal(goal)
        self.status = True
        rospy.loginfo('Waiting for the result')
        self.client.wait_for_result()
        
        # Wait for the goal to be reached
        while not rospy.is_shutdown():
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Succeeded')
                self.status = False
                self.current_marker = marker_id + 1 
                self.isRobotMoving = False
                self.detected_marker = None
                self.already_detected.append(marker_id)
                break
                
            else:
                rospy.loginfo('Failed')
                self.status = False
                self.isRobotMoving = False
                self.detected_marker = None
                break





if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    ArucoNavigator = ArucoNavigator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()