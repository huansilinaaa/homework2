#! /usr/bin/env python
import rospy
import rosbag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import MagneticField

if __name__ == "__main__":
    #初始化节点 
    rospy.init_node("w_bag_p")

    # 创建 rosbag 对象
    bag = rosbag.Bag("/home/huanlin/homework/jiantu2.bag",'r')
    # 读数据
    bagMessage1 = bag.read_messages("/driver/encoder")
    bagMessage2 = bag.read_messages("/driver/eul")
    bagMessage3 = bag.read_messages("/driver/imu")
    bagMessage4 = bag.read_messages("/driver/mag")
    bagMessage5 = bag.read_messages("/driver/scan")
    for topic, msg, t in bagMessage1:
            rospy.loginfo("%s, 时间戳: %s, 位置: [%.2f, %.2f, %.2f]",
                          topic, t, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

   
    for topic, msg, t in bagMessage2:
            rospy.loginfo(" %s, 时间戳: %s, 姿态: [%.2f, %.2f, %.2f, %.2f]",
                          topic, t, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            rospy.loginfo("加速度: [%.2f, %.2f, %.2f],角速度: [%.2f, %.2f, %.2f]",
                          msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                          msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

   
    for topic, msg, t in bagMessage3:
            rospy.loginfo(" %s, 时间戳: %s, 姿态: [%.2f, %.2f, %.2f, %.2f]",
                          topic, t, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            rospy.loginfo("加速度: [%.2f, %.2f, %.2f], 角速度: [%.2f, %.2f, %.2f]",
                          msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                          msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

    
    for topic, msg, t in bagMessage5:
            rospy.loginfo(" %s, 时间戳: %s, min: %.2f, max: %.2f",
                          topic, t, msg.angle_min, msg.angle_max)

    for topic, msg, t in bagMessage4:
            rospy.loginfo("%s, 时间戳: %s, x,y,z: [%.2f, %.2f, %.2f]",
                          topic, t, msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    # 关闭流
    bag.close()