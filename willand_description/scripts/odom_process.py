#!/usr/bin/env python3
# -*-coding:utf-8-*
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped, PoseStamped

# Path message to store the trajectory
history_path_msg = Path()

# Variable to store the current model state
car_curr_state = None

def model_states_cb(data):
    # Get the Gazebo model state for the four-wheel differential drive car
    global car_curr_state
    car_curr_state = data.pose[-1]  # The state of ski4wd in Gazebo

if __name__ == '__main__':
    rospy.init_node('odom_process')

    # Publisher to publish the Gazebo odometry
    gazebo_odom_pub = rospy.Publisher('/willand/gazebo_odom', Odometry, queue_size=10)
    
    # Publisher to publish the historical path
    history_path_pub = rospy.Publisher('/willand/history_path', Path, queue_size=10)
    
    # Subscriber to the Gazebo model state, subscribing to get the model position
    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb)

    # Wait for model state data initialization
    rospy.sleep(1)

    while not rospy.is_shutdown():
        if car_curr_state is None:
            rospy.logwarn("Waiting for model state data...")
            rospy.sleep(0.1)
            continue
        
        # Construct the odometry message and fill in the header and pose
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "world"
        odom_msg.pose.pose.position.x = car_curr_state.position.x
        odom_msg.pose.pose.position.y = car_curr_state.position.y
        odom_msg.pose.pose.position.z = car_curr_state.position.z
        odom_msg.pose.pose.orientation.w = car_curr_state.orientation.w
        odom_msg.pose.pose.orientation.x = car_curr_state.orientation.x
        odom_msg.pose.pose.orientation.y = car_curr_state.orientation.y
        odom_msg.pose.pose.orientation.z = car_curr_state.orientation.z
        # Publish the odometry message
        gazebo_odom_pub.publish(odom_msg)

        # Create tf broadcaster and TransformStamped message
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"  # World coordinate frame
        t.child_frame_id = "base_link"  # Robot's base link
        t.transform.translation.x = car_curr_state.position.x
        t.transform.translation.y = car_curr_state.position.y
        t.transform.translation.z = car_curr_state.position.z
        t.transform.rotation.x = car_curr_state.orientation.x
        t.transform.rotation.y = car_curr_state.orientation.y
        t.transform.rotation.z = car_curr_state.orientation.z
        t.transform.rotation.w = car_curr_state.orientation.w
        # Publish the transform
        br.sendTransform(t)

        # Add the current position to the Path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"  # Use the world coordinate frame
        pose_stamped.pose.position.x = car_curr_state.position.x
        pose_stamped.pose.position.y = car_curr_state.position.y
        pose_stamped.pose.position.z = car_curr_state.position.z
        pose_stamped.pose.orientation.w = car_curr_state.orientation.w
        pose_stamped.pose.orientation.x = car_curr_state.orientation.x
        pose_stamped.pose.orientation.y = car_curr_state.orientation.y
        pose_stamped.pose.orientation.z = car_curr_state.orientation.z
        
        # Add the current pose to the path and publish
        history_path_msg.header.stamp = rospy.Time.now()
        history_path_msg.header.frame_id = "world"  # Use the world coordinate frame
        history_path_msg.poses.append(pose_stamped)
        history_path_pub.publish(history_path_msg)
        
        # Control the loop frequency
        rospy.sleep(0.1)  # Set loop frequency to 10Hz
    
    rospy.spin()
