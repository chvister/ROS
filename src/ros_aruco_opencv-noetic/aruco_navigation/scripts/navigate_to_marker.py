#!/usr/bin/env python
import rospy
#aruco_opencv_msgs::ArucoDetection
from aruco_opencv_msgs.msg import ArucoDetection
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs
import tf.transformations
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

current_marker = 0
status = False
goal_marker = 2
isRobotMoving = False


# Set the maximum angular velocity of the robot
max_angular_vel = 0.5  # rad/s

def goToGoal(position,quaternion, marker_id):
    global status, current_marker

    #rospy.init_node('send_goal_python')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rospy.loginfo('Waiting for the action server to start')
    client.wait_for_server()

    rospy.loginfo('Action server started, sending the goal')
    
    # Define a Quaternion object
    quaternion1 = Quaternion()
    quaternion1.x = quaternion.x
    quaternion1.y = quaternion.y
    quaternion1.z = quaternion.z
    quaternion1.w = quaternion.w

    # Convert the Quaternion object to a list
    quaternion_list = [quaternion1.x, quaternion1.y, quaternion1.z, quaternion1.w]


    # Convert the quaternion to Euler angles
    euler_angles = tf.transformations.euler_from_quaternion((quaternion_list))
    # Extract the individual Euler angles
    # roll = euler_angles[0]
    # pitch = euler_angles[1]
    yaw = euler_angles[2]
    
    # Convert the angles from radians to degrees
    # roll_degrees = roll * 180 / 3.14159
    # pitch_degrees = pitch * 180 / 3.14159
    yaw_degrees = yaw * 180 / 3.14159
    # Creates a goal to send to the action server.
    pose = geometry_msgs.msg.Pose()
    pose.position.x = position.x - 0.2 
    pose.position.y = position.y
    pose.position.z = 0.0
    
    #print(yaw_degrees)
    
    q = tf.transformations.quaternion_from_euler(0, 0, yaw_degrees)
    pose.orientation = geometry_msgs.msg.Quaternion(*q)
    
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'

    # set position
    goal.target_pose.pose = pose

    print(goal)
    client.send_goal(goal)
    status = True
    rospy.loginfo('Waiting for the result')
    client.wait_for_result()
    print( client.get_state())
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo('Succeeded')
        status = False
        current_marker = marker_id
        
    else:
        rospy.loginfo('Failed')
        status = False

def rotate_robot():
    # Create a Twist message to rotate the robot
    rotate_msg = Twist()
    rotate_msg.angular.z = max_angular_vel
    

    # Publish Twist messages to the cmd_vel topic to rotate the robot
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    cmd_vel_pub.publish(rotate_msg)
    
def stop_robot():
    # Stop the robot by publishing a Twist message with zero velocity
    stop_msg = Twist()
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd_vel_pub.publish(stop_msg)
    

def aruco_detections_callback(data):
    global status, current_marker, isRobotMoving
    if(len(data.markers) == 0 and status == False):
        # todo if final marker you are in a finish line
        rospy.loginfo('dpc')

        if current_marker == goal_marker: 
            print("finish")
            rospy.loginfo('Finish')
            exit(0)
         
        print("rotate")

        # rotate robot to find code 
        isRobotMoving = False
        rotate_robot()
        
        #return
    elif (status == False and isRobotMoving == False): 
        isRobotMoving = True

        print("stop robot",isRobotMoving)
        stop_robot()
        #return

    for marker in data.markers:
        print("current_marker: ", current_marker)
        detected_marker = marker.marker_id
        if (status == False and current_marker == detected_marker):
            detected_position = marker.pose.position
            print( detected_position)

            detected_orientation = marker.pose.orientation
            goToGoal(detected_position, detected_orientation, detected_marker)
            print(detected_position)
        else :
            print("el;se")



if __name__ == '__main__':
    #init new a node and give it a name
    rospy.init_node('aruco_detecitions', anonymous=True)
    
    #subscribe to the topic /aruco_detections.
    image_sub = rospy.Subscriber("/aruco_detections",ArucoDetection, aruco_detections_callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
