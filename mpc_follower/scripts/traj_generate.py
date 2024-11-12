#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class FigureEightPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('/mpc/traj_path', Path, queue_size=5)
        self.num_points = 100      # 轨迹点数量
        self.total_cycles = 1      # 轨迹的完整周期数
        self.rate = rospy.Rate(1)  # 每秒发布一次路径，1Hz

    def generate_straight_line(self):
        # 生成直线轨迹
        t = np.linspace(0, 20, self.num_points * self.total_cycles)  # 直线的 t 值，假设长度为 10
        y = t  # x 轴沿着 t 变化
        x = np.zeros_like(t)  # y 轴保持为零（直线在 y 轴上不变化）
        yaw = np.zeros_like(t) + 1.57  # 假设没有偏航角，保持朝一个方向

        # 返回轨迹点（x, y, yaw）
        return np.vstack((x, y, yaw)).T  # 返回 (x, y, yaw) 形式的点

    def publish_traj_points(self):
        while not rospy.is_shutdown():
            # 生成完整轨迹
            path_msg = Path()
            path_msg.header.frame_id = "world"          # 设置参考坐标系为world
            path_msg.header.stamp = rospy.Time.now()    # 设置时间戳

            # 获取轨迹点
            points = self.generate_straight_line()

            # 将每个点封装为PoseStamped消息，并添加到Path消息中
            for point in points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = point[0]  # x 坐标
                pose.pose.position.y = point[1]  # y 坐标
                pose.pose.position.z = 0.2       # z坐标设置为0.2 方便显示

                # 偏航角作为方向（方向在2D平面上是绕z轴旋转）
                pose.pose.orientation.z = np.sin(point[2] / 2)  # 偏航角的sin部分
                pose.pose.orientation.w = np.cos(point[2] / 2)  # 偏航角的cos部分

                path_msg.poses.append(pose)

            # 发布路径点
            self.publisher.publish(path_msg)
            rospy.loginfo("Published complete trajectory path")

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('figure_eight_publisher', anonymous=True)
    publisher = FigureEightPublisher()
    publisher.publish_traj_points()

