#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MagneticField.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

int main(int argc,char * argv[]){
      setlocale(LC_ALL,"");
    ros::init(argc,argv,"bag");
    ros::NodeHandle nh;

    rosbag::Bag bag;

    bag.open("/home/huanlin/homework/jiantu2.bag",rosbag::BagMode::Read);

    for(auto &&m : rosbag::View(bag)){
        std::string topic = m.getTopic();
        ros::Time time = m.getTime();
        nav_msgs::Odometry::ConstPtr p1 = m.instantiate<nav_msgs::Odometry>();
        sensor_msgs::Imu::ConstPtr p2 = m.instantiate<sensor_msgs::Imu>();
        sensor_msgs::LaserScan::ConstPtr p3 = m.instantiate<sensor_msgs::LaserScan>();
        sensor_msgs::MagneticField::ConstPtr p4 = m.instantiate<sensor_msgs::MagneticField>();
        if(p1!=nullptr){
            ROS_INFO("encoder读取的数据:%s,时间戳:%.2f,坐标:(%f,%f,%f)",
            topic.c_str(),
            time.toSec(),
            p1->pose.pose.position.x,
            p1->pose.pose.position.y,
            p1->pose.pose.position.z
            );
        }
        else if(p2!=nullptr){
            ROS_INFO("eul/imu读取的数据:%s,时间戳:%.2f,姿态:(%f,%f,%f)加速度:(%f,%f,%f)角速度:(%f,%f,%f) ",
            topic.c_str(),
            time.toSec(),
            p2->orientation.x, p2->orientation.y, p2->orientation.z, p2->orientation.w,
               p2->linear_acceleration.x, p2->linear_acceleration.y, p2->linear_acceleration.z,  
                p2->angular_velocity.x, p2->angular_velocity.y, p2->angular_velocity.z
                ); 
        }
        else if(p3!=nullptr){
            ROS_INFO("mag读取的数据:%s,时间戳:%.2f,max:%f,min:%f",
            topic.c_str(),
            time.toSec(),
            p3->angle_min,
                p3->angle_max
            );
        }
        else if(p4!=nullptr){
            ROS_INFO("scan读取的数据:%s,时间戳:%.2f,x:%f,y:%f,z:%f",
            topic.c_str(),
            time.toSec(),
            p4->magnetic_field.x,
            p4->magnetic_field.y,
            p4->magnetic_field.z
            );
        }

    }

        bag.close();

    return 0;

}
