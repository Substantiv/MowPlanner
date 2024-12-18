#!/usr/bin/env python3
# -*-coding:utf-8-*
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped, PoseStamped

# 用于保存轨迹的Path消息
history_path_msg = Path()

# 保存当前的模型状态
car_curr_state = None

def model_states_cb(data):
    # 获取四轮差速车模型的gazebo状态
    global car_curr_state
    car_curr_state = data.pose[-1]  # ski4wd在gazebo中的状态

if __name__ == '__main__':
    rospy.init_node('odom_process')

    # 根据gazebo的绝对位置获取里程计
    gazebo_odom_pub = rospy.Publisher('/willand/gazebo_odom', Odometry, queue_size=10)
    
    # 发布历史路径
    history_path_pub = rospy.Publisher('/willand/history_path', Path, queue_size=10)
    
    # gazebo 模型状态的订阅者，订阅里程计获得gazebo模型的位置
    model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_cb)

    # 等待模型状态数据初始化
    rospy.sleep(1)

    while not rospy.is_shutdown():
        if car_curr_state is None:
            rospy.logwarn("等待模型状态数据...")
            rospy.sleep(0.1)
            continue
        
        # 构建里程计消息并填充里程计消息头部和位姿
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
        # 发布里程计消息
        gazebo_odom_pub.publish(odom_msg)

        # 创建tf发布器和TransformStamped消息
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"  # 世界坐标系
        t.child_frame_id = "base_link"  # 机器人的基础链接
        t.transform.translation.x = car_curr_state.position.x
        t.transform.translation.y = car_curr_state.position.y
        t.transform.translation.z = car_curr_state.position.z
        t.transform.rotation.x = car_curr_state.orientation.x
        t.transform.rotation.y = car_curr_state.orientation.y
        t.transform.rotation.z = car_curr_state.orientation.z
        t.transform.rotation.w = car_curr_state.orientation.w
        # 发布变换
        br.sendTransform(t)

        # 将当前位置添加到Path中
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"  # 使用世界坐标系
        pose_stamped.pose.position.x = car_curr_state.position.x
        pose_stamped.pose.position.y = car_curr_state.position.y
        pose_stamped.pose.position.z = car_curr_state.position.z
        pose_stamped.pose.orientation.w = car_curr_state.orientation.w
        pose_stamped.pose.orientation.x = car_curr_state.orientation.x
        pose_stamped.pose.orientation.y = car_curr_state.orientation.y
        pose_stamped.pose.orientation.z = car_curr_state.orientation.z
        
        # 将当前位姿加入到路径中并发布
        history_path_msg.header.stamp = rospy.Time.now()
        history_path_msg.header.frame_id = "world"  # 使用世界坐标系
        history_path_msg.poses.append(pose_stamped)
        history_path_pub.publish(history_path_msg)
        
        # 控制循环频率
        rospy.sleep(0.1)  # 控制循环频率为10Hz
    
    rospy.spin()
