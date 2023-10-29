#!/usr/bin/env python3

import rospy
import roslib
import math
import cv2 as cv # OpenCV2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest
import tf
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
import actionlib
import random
import copy
from threading import Lock

# kong work
import matplotlib.pyplot as plt
from matplotlib import colors

from mpl_toolkits.mplot3d import Axes3D  # noqa
from matplotlib.colors import hsv_to_rgb
from visualization_msgs.msg import Marker


#Ben work
import roslaunch
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import LaserScan

def wrap_angle(angle):
    # Function to wrap an angle between 0 and 2*Pi
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose

class BrickSearch:
    def __init__(self):

        # Variables/Flags
        self.localised_ = False
        self.brick_found_ = False
        self.image_msg_count_ = 0

        # Kong work 
        self.brick_angle = 0  # angle of brick wrt robot
        self.brick_distance = 0
        self.brick_scan = False
        self.brick_img = False
        
        
        self.scan_data = None
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }
        
        # Get the map via a ROS service call
        rospy.loginfo("Waiting for static_map service...")
        rospy.wait_for_service('static_map')
        get_map_service = rospy.ServiceProxy('static_map', GetMap)
        try:
            #get_map = GetMapRequest()
            resp = get_map_service()
            self.map_ = resp.map
        except rospy.ServiceException as exc:
            rospy.logerror('Service did not process request: ' + str(exc))
            rospy.signal_shutdown('Service did not process request')
        rospy.loginfo("Map received")

        # Convert map into a CV image
        self.cv_bridge_ = CvBridge()
        self.map_image_ = np.reshape(self.map_.data, (self.map_.info.height, self.map_.info.width)).astype(np.int32)

        # Wait for the transform to become available
        rospy.loginfo("Waiting for transform from map to base_link")
        self.tf_listener_ = tf.TransformListener()

        while not rospy.is_shutdown() and not self.tf_listener_.canTransform("map", "base_link", rospy.Time(0.)):
            rospy.sleep(0.1)

        # Subscribe to the camera
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

        # Advertise "cmd_vel" publisher to control TurtleBot manually
        self.cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Action client for move_base
        self.move_base_action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client_.wait_for_server()
        rospy.loginfo("move_base action available")

        # Reinitialise AMCL
        global_localization_service_client = rospy.ServiceProxy('global_localization', Empty)
        # empty = global_localization_service_client()
        rospy.sleep(0.5)
        rospy.loginfo("After AMCL")
        # Subscribe to "amcl_pose" to get pose covariance
        self.amcl_pose_sub_ = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback, queue_size=1)
        rospy.loginfo("After AMCL POSE")
        
        # Create a publisher for the "/image/test" topic with a message type of Image
        self.test_image_pub = rospy.Publisher("/image/test", Image, queue_size=1)

        # Create a publisher for the "/brick_found" topic with a message type of Bool
        # self.brick_found_pub = rospy.Publisher("/brick_found", Bool, queue_size=10)
        
        # Subscribe to laser scan
        self.laser_sub_ = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        # Create a Marker publisher
        self.marker_publisher = rospy.Publisher('pose_marker', Marker, queue_size=1)

        
        self.pub_ = None
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state_ = 0
        self.state_dict_ = {
            -1: 'brick found',
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }
    # ====================================================== # 
    
    def get_pose_2d(self):

        # Lookup the latest transform
        (trans,rot) = self.tf_listener_.lookupTransform('map', 'base_link', rospy.Time(0))

        print(trans)
        print(rot)

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = trans[0]
        pose.y = trans[1]

        qw = rot[3]
        qz = rot[2]

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        return pose

    def amcl_pose_callback(self, pose_msg):

        # Check the covariance
        frobenius_norm = 0.0

        for e in pose_msg.pose.covariance:
            frobenius_norm += e**2

        if frobenius_norm < 0.05:
            self.localised_ = True

            # Unsubscribe from "amcl_pose" because we should only need to localise once at start up
            self.amcl_pose_sub_.unregister()


    def image_callback(self, image_msg):
        # Use this method to identify when the brick is visible

        # The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
        if self.image_msg_count_ < 15:
            self.image_msg_count_ += 1
            return
        else:
            self.image_msg_count_ = 0

        # Copy the image message to a cv_bridge image
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg)

        # You can set "brick_found_" to true to signal to "mainLoop" that you have found a brick
        # You may want to communicate more information
        # Since the "image_callback" and "main_loop" methods can run at the same time you should protect any shared variables
        # with a mutex
        # "brick_found_" doesn't need a mutex because it's an atomic
        
        hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)

        # Define the lower and upper HSV range for red color
        lower_red = np.array([0, 100, 20])
        upper_red = np.array([10, 255, 255])
        
        # Create a mask to isolate red color
        mask = cv.inRange(hsv, lower_red, upper_red)
        
        image = cv.bitwise_and(image, image, mask=mask)

        # Apply morphological operations to enhance the shape
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        # Find contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Filter contours based on size and shape (adjust these criteria)
        min_area = 65000
        max_area = 250000

        cubes = []

        for contour in contours:
            area = cv.contourArea(contour)
            if min_area < area < max_area:
                perimeter = cv.arcLength(contour, True)
                approx = cv.approxPolyDP(contour, 0.04 * perimeter, True)
                if len(approx) >= 4 and len(approx) <= 6:
                    cubes.append(approx)

        # Create a variable to track detection
        self.brick_img = len(cubes) > 0

        # Draw rectangles around detected cubes
        for cube in cubes:
            cv.drawContours(image, [cube], -1, (0, 255, 0), 3)

        # Display the result
        # cv.imshow('Detected Cubes', image)
        # cv.destroyAllWindows()

        # Print detection status

        rospy.loginfo('image_callback')
        rospy.loginfo('brick_found_: ' + str(self.brick_found_))
        
        # Convert the OpenCV image to a ROS image message
        try:
            msg = self.cv_bridge_.cv2_to_imgmsg(image, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Publish the image
        self.test_image_pub.publish(msg)
        
        if (self.brick_img & self.brick_scan):
            self.brick_found_ = True
            
            # self.state_ = -1
            # self.marker_publisher.publish(marker)
            # Stop turning
            # twist = Twist()
            # twist.angular.z = 0.0
            # twist.linear.x = 0.0
            # self.cmd_vel_pub_.publish(twist)
            
        
        
    
    def laser_callback(self,scan_data):
        self.scan_data = scan_data
        self.regions_ = {
            'right':  min(min(scan_data.ranges[-90:-89]), 10),
            'fright': min(min(scan_data.ranges[-60:-30]), 10),
            'front':  min(min(np.concatenate((scan_data.ranges[-18:], scan_data.ranges[:18]))), 10),
            'fleft':  min(min(scan_data.ranges[30:50]), 10),
            'left':   min(min(scan_data.ranges[80:90]), 10),
        }
        
        ### lidar scan data gives distane for each degree in 360 
        
        rospy.loginfo('laser_callback')
        # print("LEFT", self.regions_['left'])
        # print("RIGHT", self.regions_['right'])
        # print("FRONT", self.regions_['front'])

        beam_angle = 25
        distance_threshold = 0.4
        self.brick_angle = 0
        point_count = 0
        for i in np.array(range(-beam_angle, beam_angle)):
            if np.abs(scan_data.ranges[i] - scan_data.ranges[i-1]) > distance_threshold:
            # if scan_data.ranges[i] - scan_data.ranges[i-1] > distance_threshold:
                point_count += 1
                print(" =====  OBSTACLE ANGLE =====", i)
                self.brick_angle += i/2                 # get the average sum for the angle
                if (point_count == 2):
                    self.brick_scan = True
                    if (self.brick_img & self.brick_scan):
                        self.brick_found_ = True
        
        print(" ===== brick_angle =====", self.brick_angle)
        self.brick_distance = scan_data.ranges[int(self.brick_angle)]
        print(" ===== brick_distance =====", self.brick_distance)
        self.take_action()
    
    def change_state(self,state):
        
        if state is not self.state_:
            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def take_action(self):
        
        regions = self.regions_
        # msg = Twist()
        # linear_x = 0
        # angular_z = 0
        
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub_.publish(twist)
        
        d = 0.5
        d_min = 0.3
        
        if self.brick_found_:
            self.change_state(-1)
        elif regions['front'] > d and regions['fright'] > d:      # outter turn
            self.change_state(0)
        elif regions['front'] <= d and regions['fleft'] > d and regions['fright'] > d:
            # diagnolly left
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] <= d and regions['fright'] > d_min: #regions['right']
            # state_description = 'case 3 - fright'
            # following
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] <= d and regions['fright'] < d_min: #regions['right']
            # state_description = 'case 3 - fright'
            # following
            self.change_state(1)
        
        # elif regions['front'] > d and regions['fleft'] <= d and regions['fright'] > d:  #regions['right']
        #     # state_description = 'case 4 - fleft'
        #     self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] <= d:
            # state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] <= d and regions['fleft'] <= d:       # inner turn 
            
            self.change_state(1)
        # elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        #     # state_description = 'case 8 - fleft and fright'
        #     self.change_state(0)
        else:
            # state_description = 'unknown case'
            rospy.loginfo(regions)
            
        # rospy.sleep(0.2)
        
    def main_loop(self):

        # Wait for the TurtleBot to localise
        rospy.loginfo('Localising...')
        while not rospy.is_shutdown():

            # Turn slowly
            twist = Twist()
            twist.angular.z = 1.
            self.cmd_vel_pub_.publish(twist)

            if self.localised_:
                rospy.loginfo('Localised')
                break

            rospy.sleep(0.1)

        # Stop turning
        twist = Twist()
        twist.angular.z = 0.
        self.cmd_vel_pub_.publish(twist)
        
        global brick_distance
        global delta_theta 
        
        rate = rospy.Rate(20)
        stop_finding = False
        while not stop_finding:
            twist = Twist()
            if self.state_ == 0:
                twist.linear.x = 0.25
                twist.angular.z = -0.7
            elif self.state_ == 1:   
                # turn left       
                twist.linear.x = 0
                twist.angular.z = 0.5
            elif self.state_ == 2:
                # follow wall
                twist.linear.x = 0.3
                twist.angular.z = 0
            elif self.state_ == -1: 
                # detected brick
                twist.linear.x = 0
                twist.angular.z = 0
                stop_finding = True
                
                brick_distance = self.brick_distance
                delta_theta = self.brick_angle
            else:
                rospy.logerr('Unknown state!')
            
            self.cmd_vel_pub_.publish(twist)

        rate.sleep()
        
        return
        # self.move_toward_brick(delta_theta, brick_distance)

    def move_toward_brick(self, delta_theta, brick_distance):
        # The map is stored in "map_"
        # You will probably need the data stored in "map_.info"
        # You can also access the map data as an OpenCV image with "map_image_"

        # Here's an example of getting the current pose and sending a goal to "move_base":
        pose_2d = self.get_pose_2d()

        rospy.loginfo('========   Current pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # Move toward the brick a distance = 1/3 
        
        delta_theta = math.radians(delta_theta)
        
        # target move forward in base_link
        target = Pose2D()
        target.x = brick_distance * math.cos(delta_theta) / 3 
        target.y = brick_distance * math.sin(delta_theta) / 3
        target.theta = delta_theta
        # target.theta = pose_2d.theta
        
        # pose2d of brick in base_link
        pose_2d.x = brick_distance * math.cos(delta_theta)
        pose_2d.y = brick_distance * math.sin(delta_theta)
        pose_2d.theta = delta_theta
        
        print("=== Brick Pose base_link: ", pose_2d.x, pose_2d.y, pose_2d.theta)
        
        # pose2d of brick in 'map' 
        pose_2d = self.transform_pose_to_map_frame(pose_2d)
        
        target = self.transform_pose_to_map_frame(target)
        
        # print('========   Current pose: ', pose_2d.theta + delta_theta)
        # pose_2d.x += 0.5 * math.cos(pose_2d.theta + delta_theta)
        # pose_2d.y += 0.5 * math.sin(pose_2d.theta + delta_theta)
        rospy.loginfo('======== Target Pose (map): ' + str(target.x) + ' ' + str(target.y) + ' ' + str(target.theta))
        
        
        rospy.loginfo('======== Brick Pose (map): ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))
        
        self.publish_pose_marker(pose2d_to_pose(pose_2d))
        
        return
        # Send a goal to "move_base" with "self.move_base_action_client_"
        action_goal = MoveBaseActionGoal()
        action_goal.goal.target_pose.header.frame_id = 'map'
        action_goal.goal.target_pose.pose = pose2d_to_pose(target)

        rospy.loginfo('Sending goal...')
        self.move_base_action_client_.send_goal(action_goal.goal)

        # self.move_base_action_client_.wait_for_result()
        # This loop repeats until ROS is shutdown
        # You probably want to put all your code in here
        
        while not rospy.is_shutdown():

            # rospy.loginfo('main_loop')

            # Get the state of the goal
            state = self.move_base_action_client_.get_state()

            # rospy.loginfo('action state: ' + self.move_base_action_client_.get_goal_status_text())

            if state == actionlib.GoalStatus.SUCCEEDED:

                rospy.loginfo('Action succeeded!')

                # Shutdown when done
                rospy.signal_shutdown('Action succeeded!')

            # Delay so the loop doesn't run too fast
            rospy.sleep(0.2)

    
    def transform_pose_to_map_frame(self,pose_in_robot_frame):
        # Create a PoseStamped message for the 2D pose in the 'base_link' (robot frame)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'  # Set the frame ID to 'base_link'
        pose_stamped.header.stamp = rospy.Time(0)  # Set the timestamp to 0

        # Fill in the 2D pose data
        pose_stamped.pose.position.x = pose_in_robot_frame.x
        pose_stamped.pose.position.y = pose_in_robot_frame.y

        try:
            # Perform the coordinate transformation
            # Transform the PoseStamped from 'base_link' to 'map' frame
            pose_in_map_frame = self.tf_listener_.transformPose('map', pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Exception: Unable to transform pose coordinates.")
            return None

        # Extract the robot's orientation (yaw) from the transform
        _, _, yaw = tf.transformations.euler_from_quaternion([pose_in_map_frame.pose.orientation.x,
                                                            pose_in_map_frame.pose.orientation.y,
                                                            pose_in_map_frame.pose.orientation.z,
                                                            pose_in_map_frame.pose.orientation.w])

        # Convert the transformed PoseStamped back to Pose2D (2D map)
        pose_2d = Pose2D()
        pose_2d.x = pose_in_map_frame.pose.position.x
        pose_2d.y = pose_in_map_frame.pose.position.y

        # Adjust the orientation (theta) by adding the robot's orientation (yaw)
        pose_2d.theta = pose_in_robot_frame.theta + yaw

        return pose_2d

    def publish_pose_marker(self, pose):
    # Initialize a ROS node

        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame ID to your desired frame
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.2  # Adjust the scale as needed
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0  # Set red to 0
        marker.color.g = 1.0  # Set green to 1 (green color)
        marker.color.b = 0.0  # Set blue to 0
        marker.color.a = 1.0  # Fully opaque

        rate = rospy.Rate(1)  # Publish at 1 Hz
        
        self.marker_publisher.publish(marker)


if __name__ == '__main__':

    # Create the ROS node
    rospy.init_node('brick_search')

    # Create the brick search
    brick_search = BrickSearch()

    # Loop forever while processing callbacks
    brick_search.main_loop()

    brick_search.move_toward_brick(delta_theta, brick_distance)


