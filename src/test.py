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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
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
from geometry_msgs.msg import PointStamped

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
        self.parent = None
        self.scan_data = None
        self.front_distance = None
        
        # Cong
        self.brick_img = False
        self.brick_scan = False

        explore_mode = rospy.get_param("~explore_mode", False)

        if not explore_mode:
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
                print("wait--")
                rospy.sleep(0.1)
        self.cv_bridge_ = CvBridge()
        self.tf_listener_ = tf.TransformListener()
        # Subscribe to the camera
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)

        # # Subscribe to laser scan
        # self.laser_sub_ = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

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
        
        self.marker_publisher = rospy.Publisher('/point_marker', Marker, queue_size=1)


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
        global X, Y
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
        # Initializing x & y lists every nth frame
        if self.image_msg_count_%4 == 0: X, Y = [], []
        # Initializing x & y lists every nth frame
        
	    
        for contour in contours:
            area = cv.contourArea(contour)
            print("--------------- AREA", area)
            if min_area < area < max_area:
                perimeter = cv.arcLength(contour, True)
                approx = cv.approxPolyDP(contour, 0.04 * perimeter, True)
                # print("----------------- AREA")
                if len(approx) >= 3 and len(approx) <= 6:
                    cubes.append(approx)
            
                # c = max(contour, key=cv.contourArea)
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                X.append(cX)
                Y.append(cY)
        # Create a variable to track detection
        self.brick_img = len(cubes) > 0
        
        # Draw rectangles around detected cubes
        for cube in cubes:
            cv.drawContours(image, [cube], -1, (0, 255, 0), 3)

        # Display the result
        # cv.imshow('Detected Cubes', image)
        # cv.destroyAllWindows()
        
        if X and Y:
            cX = int(sum(X)/len(X))
            cY = int(sum(Y)/len(Y))
        else: 
            cX = cY = 0
        point = Point()
        point.x = cX
        point.y = cY
        point.z = 0	# width of the frame
        marker = self.create_marker()
        marker.points.append(point)
        
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
            
            # self.marker_publisher.publish(marker)
            # Stop turning
            twist = Twist()
            twist.angular.z = 0.
            self.cmd_vel_pub_.publish(twist)
            self.stop_explore_launch()
            self.move_forward()
            
            
            
            
    def laser_callback(self,scan_data):
        #Laser scan here
        self.scan_data = scan_data
        center_index = 0
        print("====================",len(scan_data.ranges))
        # Get the distance value for the front-facing direction
        self.front_distance = scan_data.ranges[center_index]
        print("DIST == ", self.front_distance)
        
        distance_threshold = 10
        point_count_threshold  = 0.3
        
        # Define the angular range in radians for object detection (e.g., -45 to 45 degrees).
        min_angle = -0.785398  # -45 degrees in radians
        max_angle = 0.785398   # 45 degrees in radians

        # Get the angular resolution of the LIDAR scan.
        angular_resolution = scan_data.angle_increment

        # Calculate the indices corresponding to the specified angular range.
        # min_index = int((min_angle - scan_data.angle_min) / angular_resolution)
        # max_index = int((max_angle - scan_data.angle_min) / angular_resolution)
        min_index = len(scan_data.ranges)// 2 - len(scan_data.ranges)// 6
        max_index = len(scan_data.ranges) // 2 + len(scan_data.ranges)// 6
        
        area = len(scan_data.ranges)// 12
        # print("FIRST", scan_data.ranges[0])
        # print("LAST", scan_data.ranges[-1])
        
        # Process only the LIDAR data within the defined angular range.
        lidar_ranges = np.concatenate((scan_data.ranges[:area], scan_data.ranges[-area:]))
        
        # print("========lidar==========")
        # print(lidar_ranges)
        # Segment objects based on distance
        object_distances = np.where(lidar_ranges < distance_threshold)[0]   # output indices
        
        for i in range(1, len(lidar_ranges)):
            if lidar_ranges[i] - lidar_ranges[i-1] > 0.3:
                self.brick_scan = True
                print("========   OBSTACLE")
        
        return
        # Filter small objects based on points count
        object_indices = []
        object_start = object_distances[0]
        for i in range(1, len(object_distances)):
            if object_distances[i] - object_distances[i-1] > point_count_threshold:
                object_end = object_distances[i-1]
                object_indices.append((object_start, object_end))
                object_start = object_distances[i]
        
        # Publish objects as markers
        # marker_publisher = rospy.Publisher('objects_markers', Marker, queue_size=10)
        for start, end in object_indices:
            object_marker = Marker()
            object_marker.header.frame_id = scan_data.header.frame_id
            object_marker.type = Marker.POINTS
            object_marker.action = Marker.ADD
            object_marker.pose.orientation.w = 1.0
            object_marker.scale.x = 0.3  # Point size

            object_points = []
            for i in range(start, end):
                angle = scan_data.angle_min + i * scan_data.angle_increment
                x = lidar_ranges[i] * np.cos(angle)
                y = lidar_ranges[i] * np.sin(angle)
                object_points.append(Point(x, y, 0.0))

            object_marker.points = object_points

            object_marker.color.g = 1.0  # Object color
            object_marker.color.a = 1.0  # Object transparency
            self.marker_publisher.publish(object_marker)
            # print("++++++ marker objects")
        for point in object_marker.points:
            # Create a PointStamped message to hold the position
            point_stamped = PointStamped()
            point_stamped.header.frame_id = scan_data.header.frame_id
            point_stamped.point = point
            print("PPPPPCCCC ",point)
            try:
                # Use TF to transform the point to the map frame
                transformed_point = self.tf_listener_.transformPoint("map", point_stamped)
                # Use transformed_point.point for further processing
                print("Transformed Point (map frame):", transformed_point.point)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF Exception: Unable to transform point.")
            
        # Get the distance value for the front-facing direction
        # self.front_distance = scan_data.ranges[center_index]

        # Print or use the front_distance value
        # rospy.loginfo(f"Front-facing distance: {self.front_distance} meters")

    def start_explore_launch(self):
        rospy.loginfo("Starting explore mode...")
        # Start roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(["explore_lite", "explore.launch"])
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
        self.parent.start()

    def stop_explore_launch(self):
        # Stop roslaunch
        rospy.loginfo("Stopping explore mode...")

        self.move_base_action_client_.cancel_all_goals()
        rospy.loginfo("Cancelled move_base goals.")
        
        self.move_base_action_client_.wait_for_result()
        twist = Twist()
        twist.angular.z = 0
        self.cmd_vel_pub_.publish(twist)
        self.parent.shutdown()


    def main_loop(self):

        # Wait for the TurtleBot to localise
        explore_mode = rospy.get_param("~explore_mode", False)
        if not explore_mode:
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
        # twist = Twist()
        # twist.angular.z = 0.
        # self.cmd_vel_pub_.publish(twist)

        # The map is stored in "map_"
        # You will probably need the data stored in "map_.info"
        # You can also access the map data as an OpenCV image with "map_image_"
        
        # self.turn_around()
        self.start_explore_launch()  
        # # Here is where we would have the camera loop running searching for the cube
        rospy.sleep(200.0)
        self.stop_explore_launch()
        print("KEPPP")
        # # Here's an example of getting the current pose and sending a goal to "move_base":
        # pose_2d = self.get_pose_2d()

        # rospy.loginfo('Current pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # # Move forward 0.5 m
        # pose_2d.x += (self.front_distance // 2) * math.cos(pose_2d.theta)
        # pose_2d.y += (self.front_distance // 2) * math.sin(pose_2d.theta)

        # rospy.loginfo('Target pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # # Send a goal to "move_base" with "self.move_base_action_client_"
        # action_goal = MoveBaseActionGoal()
        # action_goal.goal.target_pose.header.frame_id = "map"
        # action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

        # rospy.loginfo('Sending goal...')
        # self.move_base_action_client_.send_goal(action_goal.goal)

        # # This loop repeats until ROS is shutdown
        # # You probably want to put all your code in here
        # while not rospy.is_shutdown():

        #     rospy.loginfo('main_loop')

        #     # Get the state of the goal
        #     state = self.move_base_action_client_.get_state()

        #     rospy.loginfo('action state: ' + self.move_base_action_client_.get_goal_status_text())

        #     if state == actionlib.GoalStatus.SUCCEEDED:

        #         rospy.loginfo('Action succeeded!')

        #         # Shutdown when done
        #         rospy.signal_shutdown('Action succeeded!')

        #     # Delay so the loop doesn't run too fast
        #     rospy.sleep(0.2)
    
    def move_forward(self):
  
        # Here's an example of getting the current pose and sending a goal to "move_base":
        pose_2d = self.get_pose_2d()

        rospy.loginfo('Current pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # Move forward 0.5 m
        pose_2d.x += (self.front_distance // 2) * math.cos(pose_2d.theta)
        pose_2d.y += (self.front_distance // 2) * math.sin(pose_2d.theta)

        rospy.loginfo('Target pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # Send a goal to "move_base" with "self.move_base_action_client_"
        action_goal = MoveBaseActionGoal()
        action_goal.goal.target_pose.header.frame_id = "map"
        action_goal.goal.target_pose.pose = pose2d_to_pose(pose_2d)

        rospy.loginfo('Sending goal...')
        self.move_base_action_client_.send_goal(action_goal.goal)

        # This loop repeats until ROS is shutdown
        # You probably want to put all your code in here
        while not rospy.is_shutdown():

            rospy.loginfo('move_forward')

            # Get the state of the goal
            state = self.move_base_action_client_.get_state()

            rospy.loginfo('action state: ' + self.move_base_action_client_.get_goal_status_text())

            if state == actionlib.GoalStatus.SUCCEEDED:

                rospy.loginfo('Action succeeded!')

                # Shutdown when done
                rospy.signal_shutdown('Action succeeded!')

            # Delay so the loop doesn't run too fast
            rospy.sleep(0.2)

            # Control loop rate
            # rate.sleep()

    
    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust the frame_id as needed
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.4  # Point size
        marker.scale.y = 0.4
        marker.color.a = 1.0  # Alpha (1.0 for fully visible)
        marker.color.r = 0.0  # Color (red)
        marker.color.g = 1.0
        marker.color.b = 0.0    
        return marker

if __name__ == '__main__':

    # Create the ROS node
    rospy.init_node('brick_search')

    # Create the brick search
    brick_search = BrickSearch()

    # Loop forever while processing callbacks
    brick_search.main_loop()


