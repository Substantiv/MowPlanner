#!/usr/bin/env python3
# -*-coding:utf-8-*
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
from MPC import MPC # Import the MPC class from MPC.py
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import SetBool
import math
from gazebo_msgs.msg import ModelStates

class Local_Planner():
    def __init__(self):
        self.replan_period = 0.020 # Frequency of calling the MPC solver, should match the discrete time of MPC
        self.curr_state = np.zeros(5)   # Current state of the robot
        self.z = 0.0                    # z-position of the robot
        self.N = 50                     # Number of steps in the MPC prediction horizon
        self.goal_state = np.zeros([self.N,4])  # Goal state, the nearest point on the desired path to the robot
        self.desired_global_path = [np.zeros([300,4]), 0]  # Desired global path provided by the upper-level planner
        self.have_plan = False          # Flag indicating if the MPC has successfully solved
        self.robot_state_set = False    # Flag indicating if the robot's state has been received
        self.ref_path_set = False       # Flag indicating if the reference path has been received
        self.is_end = 0                 # Flag to check if the robot has reached the goal

        # Timer that triggers MPC optimization periodically
        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)                     
        self.__pub_local_path = rospy.Publisher('/mpc/local_path', Path, queue_size=10)  # Publisher for the local path calculated by MPC
        self.__pub_rtc_cmd = rospy.Publisher('/willand/drive_controller/cmd_vel', Twist, queue_size=10)  # Publisher for the cmd_vel control commands calculated by MPC
        self._model_states = rospy.Subscriber('/willand/gazebo_odom', Odometry, self._model_states_cb)  # Subscriber for robot state from Gazebo (Odometry)
        self._sub_traj_waypts = rospy.Subscriber('/kino_astar/path', Path, self._vomp_path_callback)  # Subscriber for the reference path from the global planner
        
        self.control_cmd = Twist()  # Create an instance of Twist message for control commands

    # @Function: Callback function for receiving robot state (odometry)
    def _model_states_cb(self, data):
        # Extract the robot's position and orientation
        self.robot_state_set = True
        self.curr_state[0] = data.pose.pose.position.x  # Current x position
        self.curr_state[1] = data.pose.pose.position.y  # Current y position
        # Convert quaternion orientation to roll, pitch, yaw (rpy)
        roll, pitch, self.curr_state[2] = self.quart_to_rpy(
            data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
            data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.curr_state[3] = 0.0  # These two states are not used in this implementation
        self.curr_state[4] = 0.0
        self.z = data.pose.pose.position.z  # Robot's z position (used for visualization)

    # @Function: Convert quaternion to roll, pitch, yaw
    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        p = math.asin(2 * (w * y - z * x))
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return r, p, y

    # @Function: Timer callback for periodic MPC optimization
    def __replan_cb(self, event):
        if self.robot_state_set and self.ref_path_set:
            self.choose_goal_state()
            start_time = rospy.Time.now()
            states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0), self.goal_state)  # Call MPC to get control commands
            end_time = rospy.Time.now()
            rospy.loginfo('[pHRI Planner] MPC solved in {} sec'.format((end_time - start_time).to_sec()))

            if self.is_end == 0:
                self.__publish_local_plan(input_sol, states_sol)  # Publish the local path from MPC
                self.cmd(input_sol)  # Publish the first control command
            self.have_plan = True
        elif not self.robot_state_set and self.ref_path_set:
            print("No pose")
        elif self.robot_state_set and not self.ref_path_set:
            print("No path")
        else:
            print("No path and no pose")

    # @Function: Publish the local path calculated by MPC
    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "world"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z + 0.2  # Add offset for better visualization
            this_pose_stamped.header.seq = sequ
            sequ += 1
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "world"
            local_path.poses.append(this_pose_stamped)

        self.__pub_local_path.publish(local_path)

    # @Function: Calculate the distance between two points
    def distance_global(self, c1, c2):
        distance = np.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)
        return distance

    # @Function: Find the closest point on the global path to the current robot state
    def find_min_distance(self, c1):
        number = np.argmin(np.array([self.distance_global(c1, self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]))
        return number

    # @Function: Choose the nearest goal state from the global path for MPC
    def choose_goal_state(self):
        num = self.find_min_distance(self.curr_state)  # Find the closest point in the global path
        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(self.desired_global_path[1] - 1, int(num + i * scale))  # Choose the nearest N points from the global path
            num_list.append(num_path)
        if num >= self.desired_global_path[1]:
            self.is_end = 1  # Mark the end flag if the robot has reached the end of the path
        for k in range(self.N):
            self.goal_state[k] = self.desired_global_path[0][num_list[k]]

    # @Function: Callback function for receiving the desired global path
    def _vomp_path_callback(self, data):
        if len(data.poses) != 0:
            self.ref_path_set = True

            # Get the number of waypoints in the path
            size = len(data.poses)
            self.desired_global_path[1] = size

            # Get the current robot's yaw angle
            car_yaw = self.curr_state[2]

            # Traverse the path and fill the desired global path
            for i in range(size):
                pose = data.poses[i].pose
                x = pose.position.x
                y = pose.position.y
                yaw = self._get_yaw_from_quaternion(pose.orientation)

                # Store x, y, and yaw in the desired path
                self.desired_global_path[0][i, 0] = x
                self.desired_global_path[0][i, 1] = y

                # Adjust the yaw difference between the current robot and the desired path
                yaw_diff = yaw - car_yaw
                if yaw_diff > np.pi:
                    self.desired_global_path[0][i, 2] = yaw - 2.0 * np.pi
                elif yaw_diff < -np.pi:
                    self.desired_global_path[0][i, 2] = yaw + 2.0 * np.pi
                else:
                    self.desired_global_path[0][i, 2] = yaw

                # Assume speed is zero at each waypoint
                self.desired_global_path[0][i, 3] = 0.0

    # @Function: Convert quaternion to yaw angle
    def _get_yaw_from_quaternion(self, quat):
        """
        Extract the yaw angle from a quaternion.
        :param quat: Quaternion, geometry_msgs/Quaternion type
        :return: yaw angle
        """
        orientation = quat
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)  # Return yaw in the range [-pi, pi]
        return yaw

    # @Function: Publish the first control command from the MPC solution as cmd_vel to drive the robot
    def cmd(self, data):
        self.control_cmd.linear.x = data[0][0]  # Linear velocity
        self.control_cmd.angular.z = data[0][1]  # Angular velocity
        self.__pub_rtc_cmd.publish(self.control_cmd)

if __name__ == '__main__':
    rospy.init_node("MPC_Traj_follower")  # Initialize the ROS node
    phri_planner = Local_Planner()  # Create an instance of the Local_Planner class
    rospy.spin()  # Keep the node running and processing callbacks
